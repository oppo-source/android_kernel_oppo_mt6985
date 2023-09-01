// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: Tiffany Lin <tiffany.lin@mediatek.com>
 */

#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <soc/mediatek/smi.h>

#include "mtk_vcodec_enc_pm.h"
#include "mtk_vcodec_enc_pm_plat.h"
#include "mtk_vcodec_util.h"
#include "mtk_vcu.h"

#define USE_GCE 0
#if ENC_DVFS
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>
#include "vcodec_dvfs.h"
#define STD_VENC_FREQ 250000000
#endif

#if ENC_EMI_BW
//#include <linux/interconnect-provider.h>
#include "mtk-interconnect.h"
#include "vcodec_bw.h"
#include "mtk-smi-dbg.h"
#endif

//#define VENC_PRINT_DTS_INFO
int venc_pmqos_monitor = 1;
module_param(venc_pmqos_monitor, int , 0644);

static bool mtk_enc_tput_init(struct mtk_vcodec_dev *dev)
{
	const int tp_item_num = 6;
	const int cfg_item_num = 4;
	const int bw_item_num = 3;
	int i, larb_cnt, ret;
	struct platform_device *pdev;
	u32 nmin, nmax;
	s32 offset;

	pdev = dev->plat_dev;
	larb_cnt = 0;

	ret = of_property_read_s32(pdev->dev.of_node, "throughput-op-rate-thresh", &nmax);
	if (ret)
		mtk_v4l2_debug(0, "[VENC] Cannot get op rate thresh, default 0");

	dev->venc_dvfs_params.per_frame_adjust_op_rate = nmax;
	dev->venc_dvfs_params.per_frame_adjust = 1;

	ret = of_property_read_u32(pdev->dev.of_node, "throughput-min", &nmin);
	if (ret) {
		nmin = STD_VENC_FREQ;
		mtk_v4l2_debug(0, "[VENC] Cannot get min, default %u", nmin);
	}

	ret = of_property_read_u32(pdev->dev.of_node, "throughput-normal-max", &nmax);
	if (ret) {
		nmax = STD_VENC_FREQ;
		mtk_v4l2_debug(0, "[VENC] Cannot get normal max, default %u", nmax);
	}
	dev->venc_dvfs_params.codec_type = MTK_INST_ENCODER;
	dev->venc_dvfs_params.min_freq = nmin;
	dev->venc_dvfs_params.normal_max_freq = nmax;
	dev->venc_dvfs_params.allow_oc = 0;

	/* throughput */
	dev->venc_tput_cnt = of_property_count_u32_elems(pdev->dev.of_node,
				"throughput-table") / tp_item_num;

	if (!dev->venc_tput_cnt) {
		mtk_v4l2_debug(0, "[VENC] throughput table not exist");
		return false;
	}

	dev->venc_tput = vzalloc(sizeof(struct vcodec_perf) * dev->venc_tput_cnt);
	if (!dev->venc_tput) {
		/* mtk_v4l2_debug(0, "[VENC] vzalloc venc_tput table failed"); */
		return false;
	}

	ret = of_property_read_s32(pdev->dev.of_node, "throughput-config-offset", &offset);
	if (ret)
		mtk_v4l2_debug(0, "[VENC] Cannot get config-offset, default 0");

	for (i = 0; i < dev->venc_tput_cnt; i++) {
		ret = of_property_read_u32_index(pdev->dev.of_node, "throughput-table",
				i * tp_item_num, &dev->venc_tput[i].codec_fmt);
		if (ret) {
			mtk_v4l2_debug(0, "[VENC] Cannot get codec_fmt");
			return false;
		}

		ret = of_property_read_u32_index(pdev->dev.of_node, "throughput-table",
				i * tp_item_num + 1, (u32 *)&dev->venc_tput[i].config);
		if (ret) {
			mtk_v4l2_debug(0, "[VENC] Cannot get config");
			return false;
		}
		dev->venc_tput[i].config -= offset;

		ret = of_property_read_u32_index(pdev->dev.of_node, "throughput-table",
				i * tp_item_num + 2, &dev->venc_tput[i].cy_per_mb_1);
		if (ret) {
			mtk_v4l2_debug(0, "[VENC] Cannot get cycle per mb 1");
			return false;
		}

		ret = of_property_read_u32_index(pdev->dev.of_node, "throughput-table",
				i * tp_item_num + 3, &dev->venc_tput[i].cy_per_mb_2);
		if (ret) {
			mtk_v4l2_debug(0, "[VENC] Cannot get cycle per mb 2");
			return false;
		}
		dev->venc_tput[i].codec_type = 1;

		ret = of_property_read_u32_index(pdev->dev.of_node, "throughput-table",
				i * tp_item_num + 4, &dev->venc_tput[i].base_freq);
		mtk_v4l2_debug(0, "[VENC][tput] get base_freq: %d", dev->venc_tput[i].base_freq);
		if (ret) {
			mtk_v4l2_debug(0, "[VENC] Cannot get base_freq");
			return false;
		}

		ret = of_property_read_u32_index(pdev->dev.of_node, "throughput-table",
				i * tp_item_num + 5, &dev->venc_tput[i].bw_factor);
		mtk_v4l2_debug(0, "[VENC][tput] get bw_factor: %d", dev->venc_tput[i].bw_factor);
		if (ret) {
			mtk_v4l2_debug(0, "[VENC] Cannot get bw_factor");
			return false;
		}
	}

	/* config */
	dev->venc_cfg_cnt = of_property_count_u32_elems(pdev->dev.of_node,
				"config-table") / cfg_item_num;

	if (!dev->venc_cfg_cnt) {
		mtk_v4l2_debug(0, "[VENC] config table not exist");
		return false;
	}

	dev->venc_cfg = vzalloc(sizeof(struct vcodec_config) * dev->venc_cfg_cnt);
	if (!dev->venc_cfg) {
		/* mtk_v4l2_debug(0, "[VENC] vzalloc venc_cfg table failed"); */
		return false;
	}

	ret = of_property_read_s32(pdev->dev.of_node, "throughput-config-offset", &offset);
	if (ret)
		mtk_v4l2_debug(0, "[VENC] Cannot get config-offset, default 0");

	for (i = 0; i < dev->venc_cfg_cnt; i++) {
		ret = of_property_read_u32_index(pdev->dev.of_node, "config-table",
				i * cfg_item_num, &dev->venc_cfg[i].codec_fmt);
		if (ret) {
			mtk_v4l2_debug(0, "[VENC] Cannot get cfg codec_fmt");
			return false;
		}

		ret = of_property_read_u32_index(pdev->dev.of_node, "config-table",
				i * cfg_item_num + 1, (u32 *)&dev->venc_cfg[i].mb_thresh);
		if (ret) {
			mtk_v4l2_debug(0, "[VENC] Cannot get mb_thresh");
			return false;
		}

		ret = of_property_read_u32_index(pdev->dev.of_node, "config-table",
				i * cfg_item_num + 2, &dev->venc_cfg[i].config_1);
		if (ret) {
			mtk_v4l2_debug(0, "[VENC] Cannot get config 1");
			return false;
		}
		dev->venc_cfg[i].config_1 -= offset;

		ret = of_property_read_u32_index(pdev->dev.of_node, "config-table",
				i * cfg_item_num + 3, &dev->venc_cfg[i].config_2);
		if (ret) {
			mtk_v4l2_debug(0, "[VENC] Cannot get config 2");
			return false;
		}
		dev->venc_cfg[i].config_2 -= offset;
		dev->venc_cfg[i].codec_type = 1;
	}

	/* bw */
	dev->venc_larb_cnt = of_property_count_u32_elems(pdev->dev.of_node,
				"bandwidth-table") / bw_item_num;

	if (dev->venc_larb_cnt > MTK_VENC_LARB_NUM) {
		mtk_v4l2_debug(0, "[VENC] venc larb over limit %d > %d",
				dev->venc_larb_cnt, MTK_VENC_LARB_NUM);
		dev->venc_larb_cnt = MTK_VENC_LARB_NUM;
	}

	if (!dev->venc_larb_cnt) {
		mtk_v4l2_debug(0, "[VENC] bandwidth table not exist");
		return false;
	}

	dev->venc_larb_bw = vzalloc(sizeof(struct vcodec_larb_bw) * dev->venc_larb_cnt);
	if (!dev->venc_larb_bw) {
		/* mtk_v4l2_debug(0, "[VENC] vzalloc venc_larb_bw table failed"); */
		return false;
	}

	for (i = 0; i < dev->venc_larb_cnt; i++) {
		ret = of_property_read_u32_index(pdev->dev.of_node, "bandwidth-table",
				i * bw_item_num, (u32 *)&dev->venc_larb_bw[i].larb_id);
		if (ret) {
			mtk_v4l2_debug(0, "[VENC] Cannot get bw port type");
			return false;
		}

		ret = of_property_read_u32_index(pdev->dev.of_node, "bandwidth-table",
				i * bw_item_num + 1, &dev->venc_larb_bw[i].larb_type);
		if (ret) {
			mtk_v4l2_debug(0, "[VENC] Cannot get base bw");
			return false;
		}

		ret = of_property_read_u32_index(pdev->dev.of_node, "bandwidth-table",
				i * bw_item_num + 2, &dev->venc_larb_bw[i].larb_base_bw);
		if (ret) {
			mtk_v4l2_debug(0, "[VENC] Cannot get base bw");
			return false;
		}
	}
   mtk_venc_pmqos_monitor_init(dev);
#ifdef VENC_PRINT_DTS_INFO
	mtk_v4l2_debug(0, "[VENC] tput_cnt %d, cfg_cnt %d, larb_cnt %d\n",
		dev->venc_tput_cnt, dev->venc_cfg_cnt, dev->venc_larb_cnt);

	for (i = 0; i < dev->venc_tput_cnt; i++) {
		mtk_v4l2_debug(0, "[VENC] tput fmt %u, cfg %d, cy1 %u, cy2 %u",
			dev->venc_tput[i].codec_fmt,
			dev->venc_tput[i].config,
			dev->venc_tput[i].cy_per_mb_1,
			dev->venc_tput[i].cy_per_mb_2);
	}

	for (i = 0; i < dev->venc_cfg_cnt; i++) {
		mtk_v4l2_debug(0, "[VENC] config fmt %u, mb_thresh %u, cfg1 %d, cfg2 %d",
			dev->venc_cfg[i].codec_fmt,
			dev->venc_cfg[i].mb_thresh,
			dev->venc_cfg[i].config_1,
			dev->venc_cfg[i].config_2);
	}

	for (i = 0; i < dev->venc_larb_cnt; i++) {
		mtk_v4l2_debug(0, "[VENC] larb %u type %d, bw %u",
			dev->venc_larb_bw[i].larb_id
			dev->venc_larb_bw[i].larb_type,
			dev->venc_larb_bw[i].larb_base_bw);
	}
#endif
	return true;
}

static void mtk_enc_tput_deinit(struct mtk_vcodec_dev *dev)
{
	if (dev->venc_tput) {
		vfree(dev->venc_tput);
		dev->venc_tput = 0;
	}

	if (dev->venc_cfg) {
		vfree(dev->venc_cfg);
		dev->venc_cfg = 0;
	}

	if (dev->venc_larb_bw) {
		vfree(dev->venc_larb_bw);
		dev->venc_larb_bw = 0;
	}
	mtk_venc_pmqos_monitor_deinit(dev);
}

void mtk_prepare_venc_dvfs(struct mtk_vcodec_dev *dev)
{
#if ENC_DVFS
	int ret;
	struct dev_pm_opp *opp = 0;
	unsigned long freq = 0;
	int i = 0;
	bool tput_ret;

	INIT_LIST_HEAD(&dev->venc_dvfs_inst);

	ret = dev_pm_opp_of_add_table(&dev->plat_dev->dev);
	if (ret < 0) {
		dev->venc_reg = 0;
		mtk_v4l2_debug(0, "[VENC] Failed to get opp table (%d)", ret);
		return;
	}

	dev->venc_reg = devm_regulator_get_optional(&dev->plat_dev->dev,
						"mmdvfs-dvfsrc-vcore");
	if (!dev->venc_reg) {
		mtk_v4l2_debug(0, "[VENC] Failed to get regulator");
		dev->venc_reg = 0;
		dev->venc_mmdvfs_clk = devm_clk_get(&dev->plat_dev->dev, "mmdvfs_clk");
		if (!dev->venc_mmdvfs_clk) {
			mtk_v4l2_debug(0, "[VENC] Failed to mmdvfs_clk");
			dev->venc_mmdvfs_clk = 0;
		}
		mtk_v4l2_debug(0, "[VENC] get venc_mmdvfs_clk successfully");
	} else {
		mtk_v4l2_debug(0, "[VENC] get regulator successfully");
	}

	dev->venc_freq_cnt = dev_pm_opp_get_opp_count(&dev->plat_dev->dev);
	freq = 0;
	while (!IS_ERR(opp =
		dev_pm_opp_find_freq_ceil(&dev->plat_dev->dev, &freq))) {
		dev->venc_freqs[i] = freq;
		freq++;
		i++;
		dev_pm_opp_put(opp);
	}

	tput_ret = mtk_enc_tput_init(dev);
#endif
}

void mtk_unprepare_venc_dvfs(struct mtk_vcodec_dev *dev)
{
#if ENC_DVFS
	mtk_enc_tput_deinit(dev);
#endif
}

void mtk_prepare_venc_emi_bw(struct mtk_vcodec_dev *dev)
{
#if ENC_EMI_BW
	int i, ret;
	struct platform_device *pdev = 0;
	u32 larb_num = 0;
	const char *path_strs[MTK_VENC_LARB_NUM];

	pdev = dev->plat_dev;
	for (i = 0; i < MTK_VENC_LARB_NUM; i++)
		dev->venc_qos_req[i] = 0;

	ret = of_property_read_u32(pdev->dev.of_node, "interconnect-num", &larb_num);
	if (ret) {
		mtk_v4l2_debug(0, "[VENC] Cannot get interconnect num, skip");
		return;
	}

	ret = of_property_read_string_array(pdev->dev.of_node, "interconnect-names",
		path_strs, larb_num);

	if (ret < 0) {
		mtk_v4l2_debug(0, "[VENC] Cannot get interconnect names, skip");
		return;
	} else if (ret != (int)larb_num) {
		mtk_v4l2_debug(0, "[VENC] Interconnect name count not match %u %d", larb_num, ret);
	}

	if (larb_num > MTK_VENC_LARB_NUM) {
		mtk_v4l2_debug(0, "[VENC] venc larb over limit %u > %d",
				larb_num, MTK_VENC_LARB_NUM);
		larb_num = MTK_VENC_LARB_NUM;
	}

	for (i = 0; i < larb_num; i++) {
		dev->venc_qos_req[i] = of_mtk_icc_get(&pdev->dev, path_strs[i]);
		mtk_v4l2_debug(0, "[VENC] %d %p %s", i, dev->venc_qos_req[i], path_strs[i]);
	}
#endif
}

void mtk_unprepare_venc_emi_bw(struct mtk_vcodec_dev *dev)
{
#if ENC_EMI_BW
#endif
}

void set_venc_opp(struct mtk_vcodec_dev *dev, u32 freq)
{
	struct dev_pm_opp *opp = 0;
	int volt = 0;
	int ret = 0;
	unsigned long freq_64 = (unsigned long)freq;

	if (dev->venc_reg || dev->venc_mmdvfs_clk) {
		opp = dev_pm_opp_find_freq_ceil(&dev->plat_dev->dev, &freq_64);
		volt = dev_pm_opp_get_voltage(opp);
		dev_pm_opp_put(opp);

		if (dev->venc_mmdvfs_clk) {
			ret = clk_set_rate(dev->venc_mmdvfs_clk, freq_64);
			if (ret) {
				mtk_v4l2_err("[VENC] Failed to set mmdvfs rate %lu\n",
						freq_64);
			}
			mtk_v4l2_debug(8, "[VENC] freq %lu, find_freq %lu", freq, freq_64);
		} else if (dev->venc_reg) {
			ret = regulator_set_voltage(dev->venc_reg, volt, INT_MAX);
			if (ret) {
				mtk_v4l2_err("[VENC] Failed to set regulator voltage %d\n",
						volt);
			}
			mtk_v4l2_debug(8, "[VENC] freq %lu, voltage %lu", freq, volt);
		}
	}
}

void mtk_venc_dvfs_begin_inst(struct mtk_vcodec_ctx *ctx)
{
	mtk_v4l2_debug(8, "[VENC] ctx = %p",  ctx);

	if (need_update(ctx)) {
		update_freq(ctx->dev, MTK_INST_ENCODER);
		mtk_v4l2_debug(4, "[VENC] freq %u", ctx->dev->venc_dvfs_params.target_freq);
		set_venc_opp(ctx->dev, ctx->dev->venc_dvfs_params.target_freq);
	}
}

void mtk_venc_dvfs_end_inst(struct mtk_vcodec_ctx *ctx)
{
	mtk_v4l2_debug(8, "[VENC] ctx = %p",  ctx);

	if (remove_update(ctx)) {
		update_freq(ctx->dev, MTK_INST_ENCODER);
		mtk_v4l2_debug(4, "[VENC] freq %u", ctx->dev->venc_dvfs_params.target_freq);
		set_venc_opp(ctx->dev, ctx->dev->venc_dvfs_params.target_freq);
	}
}

void mtk_venc_pmqos_begin_inst(struct mtk_vcodec_ctx *ctx)
{
	int i;
	struct mtk_vcodec_dev *dev = 0;
	u32 target_bw = 0;

	dev = ctx->dev;
	mtk_v4l2_debug(4, "[VENC] ctx:%d");

	for (i = 0; i < dev->venc_larb_cnt; i++) {
		target_bw = (dev->venc_larb_bw[i].larb_base_bw / 100)
			* (dev->venc_dvfs_params.target_bw_factor / (100 * 100));
		if (dev->venc_larb_bw[i].larb_type < VCODEC_LARB_SUM) {
			mtk_icc_set_bw(dev->venc_qos_req[i],
					MBps_to_icc((u32)target_bw), 0);
			mtk_v4l2_debug(8, "[VENC] set larb%d: %dMB/s",
				dev->venc_larb_bw[i].larb_id, target_bw);
		} else {
			mtk_v4l2_debug(8, "[VENC] unknown port type %d\n",
					dev->venc_larb_bw[i].larb_type);
		}
	}
}

void mtk_venc_pmqos_end_inst(struct mtk_vcodec_ctx *ctx)
{
	int i;
	struct mtk_vcodec_dev *dev = 0;
	u32 target_bw = 0;

	dev = ctx->dev;

	for (i = 0; i < dev->venc_larb_cnt; i++) {
		target_bw = (dev->venc_larb_bw[i].larb_base_bw / 100)
			* (dev->venc_dvfs_params.target_bw_factor / (100 * 100));

		if (list_empty(&dev->venc_dvfs_inst)) /* no more instances */
			target_bw = 0;

		if (dev->venc_larb_bw[i].larb_type < VCODEC_LARB_SUM) {
			mtk_icc_set_bw(dev->venc_qos_req[i],
					MBps_to_icc((u32)target_bw), 0);
			mtk_v4l2_debug(8, "[VENC] set larb %d: %dMB/s",
					dev->venc_larb_bw[i].larb_id, target_bw);
		} else {
			mtk_v4l2_debug(8, "[VENC] unknown port type %d\n",
					dev->venc_larb_bw[i].larb_type);
		}
	}
}

void mtk_venc_pmqos_monitor_init(struct mtk_vcodec_dev *dev)
{
	struct vcodec_dev_qos *dev_qos = &dev->venc_dev_qos;

	mtk_v4l2_debug(4, "[VQOS] init pmqos monitor\n");
	memset((void *) dev_qos, 0, sizeof(struct vcodec_dev_qos));
	dev_qos->dev = dev->v4l2_dev.dev;
	dev_qos->comm_monitor = 2;
	dev_qos->monitor_ring_frame_cnt = 0;

	dev_qos->commlarb_id[SMI_COMMON_ID_0][0] = 6;
	dev_qos->commlarb_id[SMI_COMMON_ID_0][1] = 6;
	dev_qos->commlarb_id[SMI_COMMON_ID_0][2] = 6;
	dev_qos->commlarb_id[SMI_COMMON_ID_0][3] = 6;

	dev_qos->commlarb_id[SMI_COMMON_ID_17][0] = 1;
	dev_qos->commlarb_id[SMI_COMMON_ID_17][1] = 1;
	dev_qos->commlarb_id[SMI_COMMON_ID_17][2] = 2;
	dev_qos->commlarb_id[SMI_COMMON_ID_17][3] = 2;

	dev_qos->flag[0] = 1;
	dev_qos->flag[1] = 2;
	dev_qos->flag[2] = 1;
	dev_qos->flag[3] = 2;
}


void mtk_venc_pmqos_monitor_deinit(struct mtk_vcodec_dev *dev)
{
	struct vcodec_dev_qos *dev_qos = &dev->venc_dev_qos;

	dev_qos->monitor_ring_frame_cnt = 0;
	mtk_v4l2_debug(4, "[VQOS] deinit pmqos monitor");

}

void mtk_venc_pmqos_monitor_reset(struct mtk_vcodec_dev *dev)
{
	struct vcodec_dev_qos *dev_qos = &dev->venc_dev_qos;

	if (venc_pmqos_monitor <= 0) {
		mtk_v4l2_debug(0, "[VQOS] no pmqos_monitor");
		return;
	} else {
		mtk_v4l2_debug(0, "[VQOS] pmqos_monitor is enable");
	}

	mtk_v4l2_debug(4, "[VQOS] reset pmqos monitor");
	dev_qos->monitor_ring_frame_cnt = 0;
	dev_qos->apply_monitor_config = false;
}

void mtk_venc_pmqos_monitor(struct mtk_vcodec_dev *dev, u32 state)
{
	struct vcodec_dev_qos *dev_qos = &dev->venc_dev_qos;

	u32 data_comm0[MTK_SMI_MAX_MON_REQ];
	u32 data_comm1[MTK_SMI_MAX_MON_REQ];

	if (venc_pmqos_monitor <= 0)
		return;

	if (unlikely(!dev_qos->comm_monitor)) {
		mtk_v4l2_debug(0, "[VQOS] no common port to monitor");
		return;
	}

	switch (state) {
	case VCODEC_SMI_MONITOR_START:
		mtk_v4l2_debug(8, "[VQOS] start to monitor BW...");
		smi_monitor_start(dev_qos->dev, 0, dev_qos->commlarb_id[SMI_COMMON_ID_0], dev_qos->flag, 3);
		smi_monitor_start(dev_qos->dev, 17, dev_qos->commlarb_id[SMI_COMMON_ID_17], dev_qos->flag, 0);

		break;
	case VCODEC_SMI_MONITOR_STOP:
		smi_monitor_stop(NULL, 0, data_comm0, 3); // common 0, will update SMI_MONITOR ID
		smi_monitor_stop(NULL, 17, data_comm1, 0); // common 1, will update SMI_MONITOR ID

		dev_qos->data_total[SMI_COMMON_ID_0][SMI_READ] += data_comm0[0]; // MB
		dev_qos->data_total[SMI_COMMON_ID_0][SMI_WRITE] += data_comm0[1];

		dev_qos->data_total[SMI_COMMON_ID_17][SMI_READ] += data_comm1[0];
		dev_qos->data_total[SMI_COMMON_ID_17][SMI_WRITE] += data_comm1[1];
		dev_qos->data_total[SMI_COMMON_ID_17][SMI_READ] += data_comm1[2];
		dev_qos->data_total[SMI_COMMON_ID_17][SMI_WRITE] += data_comm1[3];

		mtk_v4l2_debug(8, "[VQOS] frame_cnt %d: Acquire bytes: (%d, %d, %d, %d, %d, %d), Total bytes: (%d, %d, %d, %d)\n",
		dev_qos->monitor_ring_frame_cnt,
		data_comm0[0], data_comm0[1],
		data_comm1[0], data_comm1[1], data_comm1[2], data_comm1[3],
		dev_qos->data_total[SMI_COMMON_ID_0][SMI_READ],
		dev_qos->data_total[SMI_COMMON_ID_17][SMI_WRITE],
		dev_qos->data_total[SMI_COMMON_ID_17][SMI_READ],
		dev_qos->data_total[SMI_COMMON_ID_0][SMI_WRITE]);

		if(++dev_qos->monitor_ring_frame_cnt >= MTK_SMI_MAX_MON_FRM)
			dev_qos->apply_monitor_config = true;

		mtk_v4l2_debug(8, "[VQOS] frame_cnt (%d): stop to monitor BW...",
			dev_qos->monitor_ring_frame_cnt);

		break;
	default:
		break;
	}
}


void mtk_venc_pmqos_frame_req(struct mtk_vcodec_dev *dev)
{
	struct vcodec_dev_qos *dev_qos = &dev->venc_dev_qos;
	int i;
	u32 common_bw[4] = {0};
	u32 cur_fps = dev->venc_dvfs_params.oprate_sum;

	if (venc_pmqos_monitor <= 0)
		return;

	// already req init value in streamOn/Off
	if (!dev_qos->apply_monitor_config) {
		mtk_v4l2_debug(8, "[VQOS] not use monitor value: %d",
			dev_qos->monitor_ring_frame_cnt);
		return;
	}


	common_bw[0] = (u32)((dev_qos->data_total[SMI_COMMON_ID_0][SMI_READ] * cur_fps / MTK_SMI_MAX_MON_FRM)>>20);
	common_bw[1] = (u32)((dev_qos->data_total[SMI_COMMON_ID_17][SMI_WRITE] * cur_fps / MTK_SMI_MAX_MON_FRM)>>20);
	common_bw[2] = (u32)((dev_qos->data_total[SMI_COMMON_ID_17][SMI_READ] * cur_fps / MTK_SMI_MAX_MON_FRM)>>20);
	common_bw[3] = (u32)((dev_qos->data_total[SMI_COMMON_ID_0][SMI_WRITE] * cur_fps / MTK_SMI_MAX_MON_FRM)>>20);

	mtk_v4l2_debug(8, "[VQOS] AVG BW (%d %d %d %d)",
		common_bw[0],common_bw[1],common_bw[2],common_bw[3]);

	for (i = 0; i < dev->venc_larb_cnt; i++) {
		common_bw[i] = (common_bw[i] >> 2) * 5;
		mtk_icc_set_bw(dev->venc_qos_req[i], MBps_to_icc((u32)common_bw[i]), 0);
		mtk_v4l2_debug(4, "[VQOS] set larb%d: %dMB/s", dev->venc_larb_bw[i].larb_id, common_bw[i]);
	}
	dev_qos->monitor_ring_frame_cnt = 0;
	dev_qos->apply_monitor_config = false;

	memset(dev_qos->data_total, 0,
		sizeof(unsigned long long) * MTK_VCODEC_QOS_GROUP * MTK_VCODEC_QOS_TYPE);

}

