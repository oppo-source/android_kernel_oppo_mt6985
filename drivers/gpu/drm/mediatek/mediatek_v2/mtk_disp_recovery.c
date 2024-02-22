// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/sched/clock.h>
#include <linux/delay.h>
#include <uapi/linux/sched/types.h>
#include <linux/pinctrl/consumer.h>

#ifndef DRM_CMDQ_DISABLE
#include <linux/soc/mediatek/mtk-cmdq-ext.h>
#else
#include "mtk-cmdq-ext.h"
#endif

#include "mtk_drm_drv.h"
#include "mtk_drm_ddp_comp.h"
#include "mtk_drm_crtc.h"
#include "mtk_drm_helper.h"
#include "mtk_drm_assert.h"
#include "mtk_drm_mmp.h"
#include "mtk_drm_trace.h"
#include "mtk_dump.h"
#include "mtk_disp_bdg.h"
#include "mtk_dsi.h"

#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
#include "oplus_display_temp_compensation.h"
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */

#define ESD_TRY_CNT 5
#ifdef OPLUS_FEATURE_DISPLAY
#define ESD_CHECK_PERIOD 3000 /* ms */
#else
#define ESD_CHECK_PERIOD 2000 /* ms */
#endif
#define esd_timer_to_mtk_crtc(x) container_of(x, struct mtk_drm_crtc, esd_timer)

#ifdef OPLUS_FEATURE_DISPLAY
#include <mt-plat/mtk_boot_common.h>
#include <soc/oplus/system/oplus_project.h>
extern void ddic_dsi_read_cmd_test(unsigned int case_num);
bool read_ddic_once = true;
unsigned long esd_flag = 0;
extern unsigned int esd_mode;
EXPORT_SYMBOL(esd_flag);
static count_irq_sta = 1;
unsigned int dsi0te_err = 1;
unsigned int dsi1te_err = 1;
extern void gpio_dump_regs_range(int start, int end);
extern unsigned int get_project(void);
#endif

static DEFINE_MUTEX(pinctrl_lock);

/* esd check GPIO_TE, after esd recovery, set backlight */
extern int mtkfb_set_backlight_level(unsigned int level, unsigned int panel_ext_param, unsigned int cfg_flag);
static int esd_recovery_flg = 0;
unsigned int backlight_level_esd = 0;
EXPORT_SYMBOL(backlight_level_esd);

/* pinctrl implementation */
long _set_state(struct drm_crtc *crtc, const char *name)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct pinctrl_state *pState = 0;
	long ret = 0;

	mutex_lock(&pinctrl_lock);
	if (!priv->pctrl) {
		DDPPR_ERR("this pctrl is null\n");
		ret = -1;
		goto exit;
	}

	pState = pinctrl_lookup_state(priv->pctrl, name);
	if (IS_ERR(pState)) {
		DDPPR_ERR("lookup state '%s' failed\n", name);
		ret = PTR_ERR(pState);
		goto exit;
	}

	/* select state! */
	pinctrl_select_state(priv->pctrl, pState);

exit:
	mutex_unlock(&pinctrl_lock);
	return ret; /* Good! */
#else
	return 0; /* Good! */
#endif
}

long disp_dts_gpio_init(struct device *dev, struct mtk_drm_private *private)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
	long ret = 0;
	struct pinctrl *pctrl;

	/* retrieve */
	pctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pctrl)) {
		DDPPR_ERR("Cannot find disp pinctrl!\n");
		ret = PTR_ERR(pctrl);
		goto exit;
	}

	private->pctrl = pctrl;

exit:
	return ret;
#else
	return 0;
#endif
}

static inline int _can_switch_check_mode(struct drm_crtc *crtc,
					 struct mtk_panel_ext *panel_ext)
{
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	int ret = 0;

	if (panel_ext->params->cust_esd_check == 0 &&
	    panel_ext->params->lcm_esd_check_table[0].cmd != 0 &&
	    mtk_drm_helper_get_opt(priv->helper_opt,
				   MTK_DRM_OPT_ESD_CHECK_SWITCH))
		ret = 1;

	return ret;
}

static inline int _lcm_need_esd_check(struct mtk_panel_ext *panel_ext)
{
	int ret = 0;

	if (panel_ext->params->esd_check_enable == 1)
		ret = 1;

	return ret;
}

static inline int need_wait_esd_eof(struct drm_crtc *crtc,
				    struct mtk_panel_ext *panel_ext)
{
	int ret = 1;

	/*
	 * 1.vdo mode
	 * 2.cmd mode te
	 */
	if (!mtk_crtc_is_frame_trigger_mode(crtc))
		ret = 0;

	if (panel_ext->params->cust_esd_check == 0)
		ret = 0;

	return ret;
}

static void esd_cmdq_timeout_cb(struct cmdq_cb_data data)
{
	struct drm_crtc *crtc = data.data;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_esd_ctx *esd_ctx = mtk_crtc->esd_ctx;
	struct mtk_ddp_comp *output_comp = NULL;

	if (!crtc) {
		DDPMSG("%s find crtc fail\n", __func__);
		return;
	}

	DDPMSG("read flush fail\n");
	esd_ctx->chk_sta = 0xff;

	if (is_bdg_supported()) {
		if (mtk_crtc) {
			output_comp = mtk_ddp_comp_request_output(mtk_crtc);
			if (output_comp) {
				mtk_dump_analysis(output_comp);
				mtk_dump_reg(output_comp);
			}
		}
		bdg_dsi_dump_reg(DISP_BDG_DSI0);
	} else {
		mtk_drm_crtc_analysis(crtc);
		mtk_drm_crtc_dump(crtc);
	}
}


int _mtk_esd_check_read(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *output_comp;
	struct mtk_panel_ext *panel_ext;
	struct cmdq_pkt *cmdq_handle, *cmdq_handle2;
	struct mtk_drm_esd_ctx *esd_ctx;
	int ret = 0;

	DDPINFO("[ESD%u]ESD read panel\n", drm_crtc_index(crtc));


	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (unlikely(!output_comp)) {
		DDPPR_ERR("%s:invalid output comp\n", __func__);
		return -EINVAL;
	}

	if (mtk_drm_is_idle(crtc) && mtk_dsi_is_cmd_mode(output_comp))
		#ifdef OPLUS_FEATURE_DISPLAY
			mtk_drm_idlemgr_kick(__func__, crtc, 0);
		#else
			return 0;
		#endif

	mtk_ddp_comp_io_cmd(output_comp, NULL, REQ_PANEL_EXT, &panel_ext);
	if (unlikely(!(panel_ext && panel_ext->params))) {
		DDPPR_ERR("%s:can't find panel_ext handle\n", __func__);
		return -EINVAL;
	}

	cmdq_handle = cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);
	cmdq_handle->err_cb.cb = esd_cmdq_timeout_cb;
	cmdq_handle->err_cb.data = crtc;

	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 2, 1);

	if (mtk_dsi_is_cmd_mode(output_comp)) {
		if (mtk_crtc_with_sub_path(crtc, mtk_crtc->ddp_mode))
			mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
						 DDP_SECOND_PATH, 0);
		else
			mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
						 DDP_FIRST_PATH, 0);

		cmdq_pkt_wfe(cmdq_handle,
				     mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);

#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
		if (oplus_temp_compensation_is_supported()) {
			oplus_temp_compensation_temp_check(output_comp, cmdq_handle);
		}
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */

		mtk_ddp_comp_io_cmd(output_comp, cmdq_handle, ESD_CHECK_READ,
				    (void *)mtk_crtc);

		cmdq_pkt_set_event(cmdq_handle,
				   mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
	} else { /* VDO mode */
		if (mtk_crtc_with_sub_path(crtc, mtk_crtc->ddp_mode))
			mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
						 DDP_SECOND_PATH, mtk_crtc->is_mml?0:1);
		else
			mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
						 DDP_FIRST_PATH, mtk_crtc->is_mml?0:1);

		if (mtk_crtc->msync2.msync_on) {
			u32 vfp_early_stop = 1;

			mtk_ddp_comp_io_cmd(output_comp, cmdq_handle, DSI_VFP_EARLYSTOP,
							&vfp_early_stop);
		}

		CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 2, 2);

		mtk_ddp_comp_io_cmd(output_comp, cmdq_handle, DSI_STOP_VDO_MODE,
				    NULL);

		CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 2, 3);

		mtk_ddp_comp_io_cmd(output_comp, cmdq_handle, ESD_CHECK_READ,
				    (void *)mtk_crtc);

		mtk_ddp_comp_io_cmd(output_comp, cmdq_handle,
				    DSI_START_VDO_MODE, NULL);

		mtk_disp_mutex_trigger(mtk_crtc->mutex[0], cmdq_handle);
		mtk_ddp_comp_io_cmd(output_comp, cmdq_handle, COMP_REG_START,
				    NULL);
	}
	esd_ctx = mtk_crtc->esd_ctx;
	esd_ctx->chk_sta = 0;
	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 2, 4);
	cmdq_pkt_flush(cmdq_handle);

	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 2, 5);


	mtk_ddp_comp_io_cmd(output_comp, NULL, CONNECTOR_READ_EPILOG,
				    NULL);
	if (esd_ctx->chk_sta == 0xff) {
		ret = -1;
		if (need_wait_esd_eof(crtc, panel_ext)) {
			/* TODO: set ESD_EOF event through CPU is better */
			mtk_crtc_pkt_create(&cmdq_handle2, crtc,
				mtk_crtc->gce_obj.client[CLIENT_CFG]);

			cmdq_pkt_set_event(
				cmdq_handle2,
				mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
			cmdq_pkt_flush(cmdq_handle2);
			cmdq_pkt_destroy(cmdq_handle2);
		}
		goto done;
	}

	ret = mtk_ddp_comp_io_cmd(output_comp, NULL, ESD_CHECK_CMP,
				  (void *)mtk_crtc);
	#ifdef OPLUS_FEATURE_DISPLAY
	if (ret != 0) {
		if (output_comp->id == DDP_COMPONENT_DSI1) {
			dsi1te_err = 0;
		} else if (output_comp->id == DDP_COMPONENT_DSI0) {
			dsi0te_err = 0;
		}
	}
	#endif
done:
	cmdq_pkt_destroy(cmdq_handle);
	return ret;
}

#ifdef OPLUS_FEATURE_DISPLAY
static void esd_read_cpu(struct mtk_drm_crtc *mtk_crtc)
{
	struct mtk_ddp_comp *output_comp;
	int enable_readic = 1;
	output_comp = mtk_ddp_comp_request_output(mtk_crtc);

	if (unlikely(!output_comp)) {
		DDPPR_ERR("%s:invalid output comp\n", __func__);
		return;
	}
	if (output_comp->id == DDP_COMPONENT_DSI1) {
		gpio_dump_regs_range(67, 69);
		gpio_dump_regs_range(218, 220);
	} else {
		gpio_dump_regs_range(165, 167);
	}
	/* only work at frame trigger mode */
	if (!mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base))
		return;

	mtk_drm_idlemgr_kick(__func__, &mtk_crtc->base, 0);

	DDPMSG("%s, READ_ESD fail stop cancel all gce jobs\n", __func__);
	if (mtk_crtc->gce_obj.client[CLIENT_CFG])
		cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_CFG]);
	if (mtk_crtc->gce_obj.client[CLIENT_DSI_CFG])
		cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_DSI_CFG]);
	if (mtk_crtc->gce_obj.client[CLIENT_SUB_CFG])
		cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_SUB_CFG]);
	if (mtk_crtc->gce_obj.client[CLIENT_TRIG_LOOP])
		cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_TRIG_LOOP]);
	if (mtk_crtc->gce_obj.client[CLIENT_SODI_LOOP])
		cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_SODI_LOOP]);

	if (output_comp->id == DDP_COMPONENT_DSI1) {
		if (dsi1te_err == 0) {
			enable_readic = 0;
		}
		mtk_ddp_comp_io_cmd(output_comp, NULL, OPLUS_GET_INFO, &enable_readic);
		dsi1te_err = 1;
	} else if (output_comp->id == DDP_COMPONENT_DSI0) {
		if (dsi0te_err == 0) {
			enable_readic = 0;
		}
		mtk_ddp_comp_io_cmd(output_comp, NULL, OPLUS_GET_INFO, &enable_readic);
		dsi0te_err = 1;
	}
}
#endif

static irqreturn_t _esd_check_ext_te_irq_handler(int irq, void *data)
{
	struct mtk_drm_esd_ctx *esd_ctx = (struct mtk_drm_esd_ctx *)data;

	atomic_set(&esd_ctx->ext_te_event, 1);
	wake_up_interruptible(&esd_ctx->ext_te_wq);

	return IRQ_HANDLED;
}

static int _mtk_esd_check_eint(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_esd_ctx *esd_ctx = mtk_crtc->esd_ctx;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	int ret = 1;

	DDPINFO("[ESD%u]ESD check eint\n", drm_crtc_index(crtc));

	if (unlikely(!esd_ctx)) {
		DDPPR_ERR("%s:invalid ESD context\n", __func__);
		return -EINVAL;
	}

	if (mtk_drm_helper_get_opt(priv->helper_opt,
			MTK_DRM_OPT_DUAL_TE) &&
			(atomic_read(&mtk_crtc->d_te.te_switched) == 1))
		atomic_set(&mtk_crtc->d_te.esd_te1_en, 1);
	else
		enable_irq(esd_ctx->eint_irq);

	/* check if there is TE in the last 2s, if so ESD check is pass */
	if (wait_event_interruptible_timeout(
		    esd_ctx->ext_te_wq,
		    atomic_read(&esd_ctx->ext_te_event),
		    HZ / 2) > 0)
		ret = 0;

	if (mtk_drm_helper_get_opt(priv->helper_opt,
			MTK_DRM_OPT_DUAL_TE) &&
			(atomic_read(&mtk_crtc->d_te.te_switched) == 1))
		atomic_set(&mtk_crtc->d_te.esd_te1_en, 0);
	else
		disable_irq(esd_ctx->eint_irq);
	atomic_set(&esd_ctx->ext_te_event, 0);

	return ret;
}

static int mtk_drm_request_eint(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_esd_ctx *esd_ctx = mtk_crtc->esd_ctx;
	struct mtk_ddp_comp *output_comp;
	struct device_node *node;
	u32 ints[2] = {0, 0};
	char *compat_str = "";
	int ret = 0;

	if (unlikely(!esd_ctx)) {
		DDPPR_ERR("%s:invalid ESD context\n", __func__);
		return -EINVAL;
	}

	if (unlikely(esd_ctx->eint_irq != -1)) {
		DDPPR_ERR("%s: reentry with inited eint_irq %d\n", __func__, esd_ctx->eint_irq);
		return -EINVAL;
	}

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);

	if (unlikely(!output_comp)) {
		DDPPR_ERR("%s:invalid output comp\n", __func__);
		return -EINVAL;
	}

	mtk_ddp_comp_io_cmd(output_comp, NULL, REQ_ESD_EINT_COMPAT,
			    &compat_str);
	if (unlikely(!compat_str)) {
		DDPPR_ERR("%s: invalid compat string\n", __func__);
		return -EINVAL;
	}
	node = of_find_compatible_node(NULL, NULL, compat_str);
	if (unlikely(!node)) {
		DDPPR_ERR("can't find ESD TE eint compatible node %s\n", compat_str);
		return -EINVAL;
	}

	of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
	esd_ctx->eint_irq = irq_of_parse_and_map(node, 0);

	ret = request_irq(esd_ctx->eint_irq, _esd_check_ext_te_irq_handler,
			  IRQF_TRIGGER_RISING, "ESD_TE-eint", esd_ctx);
	if (ret) {
		DDPPR_ERR("eint irq line %u not available! %d\n", esd_ctx->eint_irq, ret);
		return ret;
	}

	disable_irq(esd_ctx->eint_irq);
	DDPPR_ERR("[OPLUS]request count_irq_sta = %d, esd_ctx->eint_irq= %u for debug\n", ++count_irq_sta, esd_ctx->eint_irq);

	/* mode_te_te1 mapping to non-primary display's TE */
	if (output_comp->id == DDP_COMPONENT_DSI0)
		_set_state(crtc, "mode_te_te");
	else
		_set_state(crtc, "mode_te_te1");


	return ret;
}

static int mtk_drm_esd_check(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_panel_ext *panel_ext;
	struct mtk_drm_esd_ctx *esd_ctx = mtk_crtc->esd_ctx;
	int ret = 0;
	#ifdef OPLUS_FEATURE_DISPLAY
	unsigned int prj_id = get_project();
	#endif

	CRTC_MMP_EVENT_START(drm_crtc_index(crtc), esd_check, 0, 0);

	if (mtk_crtc->enabled == 0) {
		CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 0, 99);
		DDPINFO("[ESD] CRTC %d disable. skip esd check\n",
			drm_crtc_index(crtc));
		goto done;
	}

	panel_ext = mtk_crtc->panel_ext;
	if (unlikely(!(panel_ext && panel_ext->params))) {
		DDPPR_ERR("can't find panel_ext handle\n");
		ret = -EINVAL;
		goto done;
	}

	/* Check panel EINT */
	if (panel_ext->params->cust_esd_check == 0 &&
	    esd_ctx->chk_mode == READ_EINT) {
		CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 1, 0);
		ret = _mtk_esd_check_eint(crtc);
	} else { /* READ LCM CMD  */
		CRTC_MMP_MARK(drm_crtc_index(crtc), esd_check, 2, 0);
		ret = _mtk_esd_check_read(crtc);
	}

	/* switch ESD check mode */
	if (_can_switch_check_mode(crtc, panel_ext) ||
	    !mtk_crtc_is_frame_trigger_mode(crtc))
		esd_ctx->chk_mode =
			(esd_ctx->chk_mode == READ_EINT) ? READ_LCM : READ_EINT;

#ifdef OPLUS_FEATURE_DISPLAY
	if ((prj_id == 22023 || prj_id == 22223) && (get_eng_version() == AGING)) {
		esd_ctx->chk_mode = READ_EINT;
	}
#endif
done:
	CRTC_MMP_EVENT_END(drm_crtc_index(crtc), esd_check, 0, ret);
	return ret;
}

static int mtk_drm_esd_recover(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *output_comp;
	struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;
	int ret = 0;
	struct mtk_dsi *dsi = NULL;
	struct cmdq_pkt *cmdq_handle = NULL;

	CRTC_MMP_EVENT_START(drm_crtc_index(crtc), esd_recovery, 0, 0);
	if (crtc->state && !crtc->state->active) {
		DDPMSG("%s: crtc is inactive\n", __func__);
		return 0;
	}
	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_recovery, 0, 1);
	output_comp = mtk_ddp_comp_request_output(mtk_crtc);

	if (unlikely(!output_comp)) {
		DDPPR_ERR("%s: invalid output comp\n", __func__);
		ret = -EINVAL;
		goto done;
	}
	mtk_drm_idlemgr_kick(__func__, &mtk_crtc->base, 0);

	if (mtk_crtc->is_mml) {
		mtk_crtc_pkt_create(&cmdq_handle, crtc, mtk_crtc->gce_obj.client[CLIENT_CFG]);
		mtk_crtc_mml_racing_stop_sync(crtc, cmdq_handle,
					      mtk_crtc_is_frame_trigger_mode(crtc) ? true : false);
		/* flush cmdq with stop_vdo_mode before it set DSI_START to 0 */
	}
#ifdef OPLUS_FEATURE_DISPLAY
	if (mtk_crtc_is_frame_trigger_mode(crtc)) {
		cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_CFG]);
		DDPMSG("%s: cmdq_mbox_stop for run recover fast\n", __func__);
	}
#endif
	mtk_ddp_comp_io_cmd(output_comp, cmdq_handle, CONNECTOR_PANEL_DISABLE, NULL);
	if (is_bdg_supported()) {
		dsi = container_of(output_comp, struct mtk_dsi, ddp_comp);
		bdg_common_deinit(DISP_BDG_DSI0, NULL, dsi);
	}

	mtk_drm_crtc_disable(crtc, true);
	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_recovery, 0, 2);

	if (mtk_drm_helper_get_opt(priv->helper_opt,
		MTK_DRM_OPT_MMQOS_SUPPORT)) {
		if (drm_crtc_index(crtc) == 0)
			mtk_disp_set_hrt_bw(mtk_crtc,
				mtk_crtc->qos_ctx->last_hrt_req);
	}

	mtk_drm_crtc_enable(crtc);
	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_recovery, 0, 3);

	if (mtk_crtc->is_mml)
		mtk_crtc_mml_racing_resubmit(crtc, NULL);
	if (is_bdg_supported()) {
		dsi = container_of(output_comp, struct mtk_dsi, ddp_comp);
		mtk_output_bdg_enable(dsi, false);
	}
	mtk_ddp_comp_io_cmd(output_comp, NULL, CONNECTOR_PANEL_ENABLE, NULL);

	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_recovery, 0, 4);

	mtk_crtc_hw_block_ready(crtc);
	if (mtk_crtc_is_frame_trigger_mode(crtc)) {
		struct cmdq_pkt *cmdq_handle;

		mtk_crtc_pkt_create(&cmdq_handle, &mtk_crtc->base,
			mtk_crtc->gce_obj.client[CLIENT_CFG]);

		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_DIRTY]);
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_ESD_EOF]);

		cmdq_pkt_flush(cmdq_handle);
		cmdq_pkt_destroy(cmdq_handle);
	}
	mtk_drm_idlemgr_kick(__func__, &mtk_crtc->base, 0);
	CRTC_MMP_MARK(drm_crtc_index(crtc), esd_recovery, 0, 5);

done:
	CRTC_MMP_EVENT_END(drm_crtc_index(crtc), esd_recovery, 0, ret);

	return 0;
}

int mtk_drm_esd_testing_process(struct mtk_drm_esd_ctx *esd_ctx, bool need_lock)
{
		struct mtk_drm_private *private = NULL;
		struct drm_crtc *crtc = NULL;
		struct mtk_drm_crtc *mtk_crtc = NULL;
		struct mtk_ddp_comp *output_comp;
		int ret = 0;
		int i = 0;
		int recovery_flg = 0;
		unsigned int crtc_idx;
		unsigned int prj_id = get_project();

		if (!esd_ctx) {
			DDPPR_ERR("%s invalid ESD context, stop thread\n", __func__);
			return -EINVAL;
		}

		if (!esd_ctx->chk_active)
			return 0;

		crtc = esd_ctx->crtc;
		if (!crtc) {
			DDPPR_ERR("%s invalid CRTC context, stop thread\n", __func__);
			return -EINVAL;
		}
		mtk_crtc = to_mtk_crtc(crtc);
		if (!mtk_crtc) {
			DDPPR_ERR("%s invalid mtk_crtc stop thread\n", __func__);
			return -EINVAL;
		}

#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
		if (oplus_temp_compensation_is_supported()) {
			oplus_temp_compensation_get_ntc_temp();
		}
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */

		private = crtc->dev->dev_private;
		if (need_lock) {
			mutex_lock(&private->commit.lock);
			DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);
		}
		if (!mtk_drm_is_idle(crtc))
			atomic_set(&esd_ctx->target_time, 0);

		crtc_idx = drm_crtc_index(crtc);
		#ifdef OPLUS_FEATURE_DISPLAY
		output_comp = mtk_ddp_comp_request_output(mtk_crtc);
		if (unlikely(!output_comp)) {
			DDPPR_ERR("%s:invalid output comp\n", __func__);
			return -EINVAL;
		}
		#endif
		i = 0; /* repeat */
		do {
			ret = mtk_drm_esd_check(crtc);
			if (!ret && !esd_mode) /* success */
				break;

			#ifdef OPLUS_FEATURE_DISPLAY
			esd_flag = 1;
			#endif
			DDPPR_ERR("[ESD%u]esd check fail, will do esd recovery. try=%d\n", crtc_idx, i);

			#ifdef OPLUS_FEATURE_DISPLAY
			if ((prj_id == 22023 || prj_id == 22223)) {
				gpio_dump_regs_range(121, 123);
				gpio_dump_regs_range(84, 86);
				if (get_eng_version() != AGING) {
					if (get_eng_version() == PREVERSION) {
						esd_read_cpu(mtk_crtc);
					}
					mtk_drm_esd_recover(crtc);
				} else {
					esd_read_cpu(mtk_crtc);
					DDPMSG("%s, READ_ESD TE fail cancel esd recover in aging\n", __func__);
				}
			} else {
				mtk_drm_esd_recover(crtc);
			}
			#endif
			recovery_flg = 1;
			#ifdef OPLUS_FEATURE_DISPLAY
			esd_flag = 0;
			esd_mode = 0;
			#endif
		} while (++i < ESD_TRY_CNT);

		if (ret != 0) {
			DDPPR_ERR(
				"[ESD%u]after esd recovery %d times, still fail, disable esd check\n",
				crtc_idx, ESD_TRY_CNT);
			mtk_disp_esd_check_switch(crtc, false);

			if (need_lock) {
				DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
				mutex_unlock(&private->commit.lock);
			}
			return 0;
		} else if (recovery_flg && ret == 0) {
			DDPPR_ERR("[ESD%u] esd recovery success\n", crtc_idx);
			recovery_flg = 0;
			esd_recovery_flg = 1;
		}
		mtk_drm_trace_end("esd");

		if (need_lock) {
			DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
			mutex_unlock(&private->commit.lock);
		}

		/* esd check GPIO_TE, after esd recovery, set backlight */
		if (is_bdg_supported()) {
			if (esd_recovery_flg) {
				esd_recovery_flg = 0;
				mtkfb_set_backlight_level(backlight_level_esd, 0, 0x1);
			}
		}

		return 0;

}

static void mtk_esd_timer_do(struct timer_list *esd_timer)
{
	//wake up interrupt
	struct mtk_drm_esd_ctx *esd_ctx =
		container_of(esd_timer, struct mtk_drm_esd_ctx, esd_timer);

	if (!esd_ctx) {
		DDPPR_ERR("%s invalid ESD_CTX\n", __func__);
		return;
	}

	atomic_set(&esd_ctx->target_time, 1);
	wake_up_interruptible(&esd_ctx->check_task_wq);
}

static void init_esd_timer(struct mtk_drm_esd_ctx *esd_ctx)
{
	if (unlikely(!esd_ctx)) {
		DDPPR_ERR("%s invalid ESD context\n");
		return;
	}

	timer_setup(&esd_ctx->esd_timer, mtk_esd_timer_do, 0);
	mod_timer(&esd_ctx->esd_timer, jiffies + (1*HZ));
}

static int mtk_drm_esd_check_worker_kthread(void *data)
{
	struct sched_param param = {.sched_priority = 87};
	struct mtk_drm_esd_ctx *esd_ctx = (struct mtk_drm_esd_ctx *)data;
	int ret = 0;
#ifdef OPLUS_FEATURE_DISPLAY
	struct drm_crtc *crtc = NULL;
	struct mtk_drm_crtc *mtk_crtc = NULL;
	struct mtk_crtc_state *mtk_state = NULL;
#endif

	sched_setscheduler(current, SCHED_RR, &param);

	if (!esd_ctx) {
		DDPPR_ERR("%s invalid ESD context, stop thread\n", __func__);
		return -EINVAL;
	}

#ifdef OPLUS_FEATURE_DISPLAY
	crtc = esd_ctx->crtc;
	if (!crtc) {
		DDPPR_ERR("%s invalid CRTC context, stop thread\n", __func__);
		return -EINVAL;
	}
	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc) {
		DDPPR_ERR("%s invalid mtk_crtc stop thread\n", __func__);
		return -EINVAL;
	}
#endif

	while (1) {
		msleep(ESD_CHECK_PERIOD);
		if (esd_ctx->chk_en == 0)
			continue;

		init_esd_timer(esd_ctx);

		ret = wait_event_interruptible(
			esd_ctx->check_task_wq,
			atomic_read(&esd_ctx->check_wakeup) &&
			(atomic_read(&esd_ctx->target_time) ||
				esd_ctx->chk_mode == READ_EINT));
		if (ret < 0) {
			DDPINFO("[ESD]check thread waked up accidently\n");
			continue;
		}

#ifdef OPLUS_FEATURE_DISPLAY
		if (read_ddic_once) {
			DDPMSG("[ESD] get_boot_mode() is %d\n", get_boot_mode());
			DDPMSG("[ESD] Read DDIC lcm id1 0xDA valve\n");
			if (is_bdg_supported() == false) {
				ddic_dsi_read_cmd_test(8);
			}
			DDPMSG("[ESD] Read DDIC lcm id2 0xDB valve\n");
			if (is_bdg_supported() == false) {
				ddic_dsi_read_cmd_test(7);
			}
			read_ddic_once = false;
		}

		if ((get_boot_mode() != RECOVERY_BOOT) && (get_boot_mode() != FACTORY_BOOT)) {

			DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);
			if (crtc->state && crtc->state->enable) {
				mtk_state = to_mtk_crtc_state(crtc->state);
				if (!mtk_state) {
					DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
					DDPINFO("[ESD] mtk_state is null\n");
					continue;
				}

				if (mtk_state->prop_val[CRTC_PROP_DOZE_ACTIVE]) {
					DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
					DDPINFO("[ESD] is in aod doze mode, skip esd check!\n");
					continue;
				}
			}
			DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		}
#endif

#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
		if (oplus_temp_compensation_is_supported()) {
			oplus_temp_compensation_data_update();
		}
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */

		del_timer_sync(&esd_ctx->esd_timer);
		#ifdef OPLUS_FEATURE_DISPLAY
			mtk_drm_idlemgr_kick(__func__, crtc, 1);
		#endif
		mtk_drm_esd_testing_process(esd_ctx, true);
		/* 2. other check & recovery */
		if (kthread_should_stop())
			break;
	}
	return 0;
}

void mtk_disp_esd_attach_crtc(struct drm_crtc *crtc, struct mtk_ddp_comp *output_comp)
{
	struct mtk_drm_esd_ctx *esd_ctx = NULL;
	struct mtk_dsi *mtk_dsi;

	if (unlikely(!output_comp)) {
		DDPPR_ERR("%s invalid output_comp\n", __func__);
		return;
	}

	if (unlikely(mtk_ddp_comp_get_type(output_comp->id) != MTK_DSI)) {
		DDPPR_ERR("%s invalid output_comp\n", __func__);
		return;
	}

	mtk_dsi = container_of(output_comp, struct mtk_dsi, ddp_comp);
	esd_ctx = mtk_dsi->esd_ctx;

	if (unlikely(!esd_ctx)) {
		DDPPR_ERR("%s invalid esd_ctx\n", __func__);
		return;
	}

	esd_ctx->crtc = crtc;
}
void mtk_disp_esd_check_switch(struct drm_crtc *crtc, bool enable)
{
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_esd_ctx *esd_ctx = NULL;
	struct mtk_ddp_comp *output_comp;
	struct mtk_panel_ext *panel_ext;

	output_comp = (mtk_crtc) ? mtk_ddp_comp_request_output(mtk_crtc) : NULL;

	if (!mtk_crtc) {
		DDPMSG("can't find crtc handle\n");
		return;
	}
	panel_ext = mtk_crtc->panel_ext;
	if (!mtk_drm_helper_get_opt(priv->helper_opt,
					   MTK_DRM_OPT_ESD_CHECK_RECOVERY))
		return;

	DDPINFO("%s %u, esd chk active: %d\n", __func__, drm_crtc_index(crtc), enable);
	if (!(panel_ext && panel_ext->params)) {
		DDPMSG("can't find panel_ext handle\n");
		return;
	}
	if (!_lcm_need_esd_check(panel_ext)) {
				DDPPR_ERR("%s:no ESD check\n", __func__);
               return;
	}
	if (output_comp && mtk_ddp_comp_get_type(output_comp->id) == MTK_DSI) {
		struct mtk_dsi *mtk_dsi = container_of(output_comp, struct mtk_dsi, ddp_comp);

		if (mtk_dsi)
			esd_ctx = mtk_dsi->esd_ctx;
	}

	if (esd_ctx) {
		mtk_crtc->esd_ctx = esd_ctx;
	} else {
		DDPINFO("%s:%d invalid ESD context, crtc id:%d\n",
				__func__, __LINE__, drm_crtc_index(crtc));
		return;
	}
	esd_ctx->chk_active = enable;
	if (enable)
		esd_ctx->crtc = crtc;
	else
		esd_ctx->crtc = NULL;

	DDPPR_ERR("[OPLUS]%s %u, esd chk active: %d for debug\n", __func__, drm_crtc_index(crtc), enable);

	atomic_set(&esd_ctx->check_wakeup, enable);
	if (enable)
		wake_up_interruptible(&esd_ctx->check_task_wq);
}

static void mtk_disp_esd_chk_deinit(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_esd_ctx *esd_ctx = mtk_crtc->esd_ctx;

	if (unlikely(!esd_ctx)) {
		DDPPR_ERR("%s:invalid ESD context\n", __func__);
		return;
	}

	/* Stop ESD task */
	mtk_disp_esd_check_switch(crtc, false);

	/* Stop ESD kthread */
	kthread_stop(esd_ctx->disp_esd_chk_task);

	kfree(esd_ctx);
}

static void mtk_disp_esd_chk_init(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_panel_ext *panel_ext;
	struct mtk_drm_esd_ctx *esd_ctx;
	struct mtk_ddp_comp *output_comp;

	if (IS_ERR_OR_NULL(mtk_crtc)) {
		DDPPR_ERR("%s invalid crtc\n", __func__);
		return;
	}

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	panel_ext = mtk_crtc->panel_ext;
	if (!(panel_ext && panel_ext->params)) {
		DDPMSG("can't find panel_ext handle\n");
		return;
	}

	if (_lcm_need_esd_check(panel_ext) == 0)
		return;

	DDPINFO("create ESD thread\n");
	/* primary display check thread init */
	esd_ctx = kzalloc(sizeof(*esd_ctx), GFP_KERNEL);
	if (!esd_ctx) {
		DDPPR_ERR("allocate ESD context failed!\n");
		return;
	}
	mutex_init(&esd_ctx->lock);
	mtk_crtc->esd_ctx = esd_ctx;
	if (output_comp && mtk_ddp_comp_get_type(output_comp->id) == MTK_DSI) {
		struct mtk_dsi *dsi = container_of(output_comp, struct mtk_dsi, ddp_comp);

		if (dsi)
			dsi->esd_ctx = esd_ctx;
	}

	esd_ctx->eint_irq = -1;
	esd_ctx->chk_en = 1;
	esd_ctx->crtc = crtc;
	esd_ctx->disp_esd_chk_task = kthread_create(
		mtk_drm_esd_check_worker_kthread, esd_ctx, "disp_echk");

	init_waitqueue_head(&esd_ctx->check_task_wq);
	init_waitqueue_head(&esd_ctx->ext_te_wq);
	atomic_set(&esd_ctx->check_wakeup, 0);
	atomic_set(&esd_ctx->ext_te_event, 0);
	atomic_set(&esd_ctx->target_time, 0);
	if (panel_ext->params->cust_esd_check == 1)
		esd_ctx->chk_mode = READ_LCM;
	else
		esd_ctx->chk_mode = READ_EINT;
	mtk_drm_request_eint(crtc);

	wake_up_process(esd_ctx->disp_esd_chk_task);
}

void mtk_disp_chk_recover_deinit(struct drm_crtc *crtc)
{
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *output_comp;

	output_comp = (mtk_crtc) ? mtk_ddp_comp_request_output(mtk_crtc) : NULL;

	/* only support ESD check for DSI output interface */
	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_ESD_CHECK_RECOVERY) &&
			output_comp && mtk_ddp_comp_get_type(output_comp->id) == MTK_DSI)
		mtk_disp_esd_chk_deinit(crtc);
}

void mtk_disp_chk_recover_init(struct drm_crtc *crtc)
{
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *output_comp;

	output_comp = (mtk_crtc) ? mtk_ddp_comp_request_output(mtk_crtc) : NULL;

	/* only support ESD check for DSI output interface */
	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_ESD_CHECK_RECOVERY) &&
			output_comp && mtk_ddp_comp_get_type(output_comp->id) == MTK_DSI)
		mtk_disp_esd_chk_init(crtc);
}
