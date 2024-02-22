// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 MediaTek Inc.
 * Author: Chong-ming Wei <chong-ming.wei@mediatek.com>
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <dt-bindings/power/mt6886-power.h>

#include "mtk-pd-chk.h"
#include "clkchk-mt6886.h"

#define TAG				"[pdchk] "
#define BUG_ON_CHK_ENABLE		0

/*
 * The clk names in Mediatek CCF.
 */

static unsigned int suspend_cnt;

/* afe */
struct pd_check_swcg afe_swcgs[] = {
	SWCG("afe_afe"),
	SWCG("afe_22m"),
	SWCG("afe_24m"),
	SWCG("afe_apll2_tuner"),
	SWCG("afe_apll_tuner"),
	SWCG("afe_tdm_ck"),
	SWCG("afe_adc"),
	SWCG("afe_dac"),
	SWCG("afe_dac_predis"),
	SWCG("afe_tml"),
	SWCG("afe_nle"),
	SWCG("afe_general3_asrc"),
	SWCG("afe_connsys_i2s_asrc"),
	SWCG("afe_general1_asrc"),
	SWCG("afe_general2_asrc"),
	SWCG("afe_dac_hires"),
	SWCG("afe_adc_hires"),
	SWCG("afe_adc_hires_tml"),
	SWCG("afe_adda6_adc"),
	SWCG("afe_adda6_adc_hires"),
	SWCG("afe_3rd_dac"),
	SWCG("afe_3rd_dac_predis"),
	SWCG("afe_3rd_dac_tml"),
	SWCG("afe_3rd_dac_hires"),
	SWCG("afe_i2s5_bclk"),
	SWCG("afe_i2s6_bclk"),
	SWCG("afe_i2s7_bclk"),
	SWCG("afe_i2s8_bclk"),
	SWCG("afe_i2s9_bclk"),
	SWCG("afe_etdm_in0_bclk"),
	SWCG("afe_etdm_out0_bclk"),
	SWCG("afe_i2s1_bclk"),
	SWCG("afe_i2s2_bclk"),
	SWCG("afe_i2s3_bclk"),
	SWCG("afe_i2s4_bclk"),
	SWCG("afe_etdm_in1_bclk"),
	SWCG("afe_etdm_out1_bclk"),
	SWCG(NULL),
};
/* camsys_main */
struct pd_check_swcg camsys_main_swcgs[] = {
	SWCG("cam_m_larb13_con_0"),
	SWCG("cam_m_larb14_con_0"),
	SWCG("cam_m_larb27_con_0"),
	SWCG("cam_m_larb29_con_0"),
	SWCG("cam_m_cam_con_0"),
	SWCG("cam_m_cam_suba_con_0"),
	SWCG("cam_m_cam_subb_con_0"),
	SWCG("cam_m_cam_subc_con_0"),
	SWCG("cam_m_cam_mraw_con_0"),
	SWCG("cam_m_camtg_con_0"),
	SWCG("cam_m_seninf_con_0"),
	SWCG("cam_m_camsv_con_0"),
	SWCG("cam_m_adlrd_con_0"),
	SWCG("cam_m_adlwr_con_0"),
	SWCG("cam_m_uisp_con_0"),
	SWCG("cam_m_fake_eng_con_0"),
	SWCG("cam_m_cam2mm0_gcon_0"),
	SWCG("cam_m_cam2mm1_gcon_0"),
	SWCG("cam_m_cam2sys_gcon_0"),
	SWCG("cam_m_cam2mm2_gcon_0"),
	SWCG("cam_m_ccusys_con_0"),
	SWCG("cam_m_ips_con_0"),
	SWCG("cam_m_camsv_a_con_1"),
	SWCG("cam_m_camsv_b_con_1"),
	SWCG("cam_m_camsv_c_con_1"),
	SWCG("cam_m_camsv_d_con_1"),
	SWCG("cam_m_camsv_e_con_1"),
	SWCG("cam_m_camsv_con_1"),
	SWCG(NULL),
};
/* camsys_mraw */
struct pd_check_swcg camsys_mraw_swcgs[] = {
	SWCG("cam_mr_larbx"),
	SWCG("cam_mr_camtg"),
	SWCG("cam_mr_mraw0"),
	SWCG("cam_mr_mraw1"),
	SWCG("cam_mr_mraw2"),
	SWCG("cam_mr_mraw3"),
	SWCG("cam_mr_pda0"),
	SWCG("cam_mr_pda1"),
	SWCG(NULL),
};
/* camsys_rawa */
struct pd_check_swcg camsys_rawa_swcgs[] = {
	SWCG("cam_ra_larbx"),
	SWCG("cam_ra_cam"),
	SWCG("cam_ra_camtg"),
	SWCG(NULL),
};
/* camsys_rawb */
struct pd_check_swcg camsys_rawb_swcgs[] = {
	SWCG("cam_rb_larbx"),
	SWCG("cam_rb_cam"),
	SWCG("cam_rb_camtg"),
	SWCG(NULL),
};
/* camsys_yuva */
struct pd_check_swcg camsys_yuva_swcgs[] = {
	SWCG("cam_ya_larbx"),
	SWCG("cam_ya_cam"),
	SWCG("cam_ya_camtg"),
	SWCG(NULL),
};
/* camsys_yuvb */
struct pd_check_swcg camsys_yuvb_swcgs[] = {
	SWCG("cam_yb_larbx"),
	SWCG("cam_yb_cam"),
	SWCG("cam_yb_camtg"),
	SWCG(NULL),
};
/* ccu_main */
struct pd_check_swcg ccu_main_swcgs[] = {
	SWCG("ccu_larb19"),
	SWCG("ccu_ahb"),
	SWCG("ccusys_ccu0"),
	SWCG("ccusys_dpe"),
	SWCG(NULL),
};
/* dip_nr1_dip1 */
struct pd_check_swcg dip_nr1_dip1_swcgs[] = {
	SWCG("dip_nr1_dip1_larb"),
	SWCG("dip_nr1_dip1_dip_nr1"),
	SWCG(NULL),
};
/* dip_nr2_dip1 */
struct pd_check_swcg dip_nr2_dip1_swcgs[] = {
	SWCG("dip_nr2_dip1_larb15"),
	SWCG("dip_nr2_dip1_dip_nr"),
	SWCG(NULL),
};
/* dip_top_dip1 */
struct pd_check_swcg dip_top_dip1_swcgs[] = {
	SWCG("dip_dip1_larb10"),
	SWCG("dip_dip1_dip_top"),
	SWCG(NULL),
};
/* dispsys_config */
struct pd_check_swcg dispsys_config_swcgs[] = {
	SWCG("mm_disp_mutex0"),
	SWCG("mm_disp_ovl0"),
	SWCG("mm_disp_merge0"),
	SWCG("mm_disp_fake_eng0"),
	SWCG("mm_inlinerot0"),
	SWCG("mm_disp_wdma0"),
	SWCG("mm_disp_fake_eng1"),
	SWCG("mm_disp_ovl0_2l_nw"),
	SWCG("mm_disp_rdma0"),
	SWCG("mm_disp_rdma1"),
	SWCG("mm_disp_rsz0"),
	SWCG("mm_disp_color0"),
	SWCG("mm_disp_ccorr0"),
	SWCG("mm_disp_ccorr1"),
	SWCG("mm_disp_aal0"),
	SWCG("mm_disp_gamma0"),
	SWCG("mm_disp_postmask0"),
	SWCG("mm_disp_dither0"),
	SWCG("mm_disp_cm0"),
	SWCG("mm_disp_spr0"),
	SWCG("mm_disp_dsc_wrap0"),
	SWCG("mm_fmm_clk0"),
	SWCG("mm_disp_ufbc_wdma0"),
	SWCG("mm_disp_wdma1"),
	SWCG("mm_apb_bus"),
	SWCG("mm_disp_c3d0"),
	SWCG("mm_disp_y2r0"),
	SWCG("mm_disp_chist0"),
	SWCG("mm_disp_chist1"),
	SWCG("mm_disp_ovl0_2l"),
	SWCG("mm_disp_dli_async3"),
	SWCG("mm_disp_dlo_async3"),
	SWCG("mm_disp_ovl1_2l"),
	SWCG("mm_disp_ovl1_2l_nw"),
	SWCG("mm_smi_common"),
	SWCG("mm_clk0"),
	SWCG("mm_sig_emi"),
	SWCG(NULL),
};
/* imgsys_main */
struct pd_check_swcg imgsys_main_swcgs[] = {
	SWCG("img_fdvt"),
	SWCG("img_me"),
	SWCG("img_mmg"),
	SWCG("img_larb12"),
	SWCG("img_larb9"),
	SWCG("img_traw0"),
	SWCG("img_traw1"),
	SWCG("img_vcore_gals"),
	SWCG("img_dip0"),
	SWCG("img_wpe0"),
	SWCG("img_ipe"),
	SWCG("img_wpe1"),
	SWCG("img_wpe2"),
	SWCG("img_avs"),
	SWCG("img_gals"),
	SWCG(NULL),
};
/* mdpsys_config */
struct pd_check_swcg mdpsys_config_swcgs[] = {
	SWCG("mdp_mutex0"),
	SWCG("mdp_apb_bus"),
	SWCG("mdp_smi0"),
	SWCG("mdp_rdma0"),
	SWCG("mdp_hdr0"),
	SWCG("mdp_aal0"),
	SWCG("mdp_rsz0"),
	SWCG("mdp_tdshp0"),
	SWCG("mdp_color0"),
	SWCG("mdp_wrot0"),
	SWCG("mdp_fake_eng0"),
	SWCG("mdp_dl_relay0"),
	SWCG("mdp_dl_relay1"),
	SWCG("mdp_rdma1"),
	SWCG("mdp_hdr1"),
	SWCG("mdp_aal1"),
	SWCG("mdp_rsz1"),
	SWCG("mdp_tdshp1"),
	SWCG("mdp_color1"),
	SWCG("mdp_wrot1"),
	SWCG("mdp_dlo_async0"),
	SWCG("mdp_dlo_async1"),
	SWCG("mdp_hre_mdpsys"),
	SWCG(NULL),
};
/* mminfra_config */
struct pd_check_swcg mminfra_config_swcgs[] = {
	SWCG("mminfra_gce_d"),
	SWCG("mminfra_gce_m"),
	SWCG("mminfra_smi"),
	SWCG("mminfra_gce_26m"),
	SWCG(NULL),
};
/* traw_dip1 */
struct pd_check_swcg traw_dip1_swcgs[] = {
	SWCG("traw_dip1_larb28"),
	SWCG("traw_dip1_traw"),
	SWCG(NULL),
};
/* vdec_gcon_base */
struct pd_check_swcg vdec_gcon_base_swcgs[] = {
	SWCG("vde2_larb1_cken"),
	SWCG("vde2_lat_cken"),
	SWCG("vde2_lat_active"),
	SWCG("vde2_mini_mdp_en"),
	SWCG("vde2_vdec_cken"),
	SWCG("vde2_vdec_active"),
	SWCG(NULL),
};
/* venc_gcon */
struct pd_check_swcg venc_gcon_swcgs[] = {
	SWCG("ven_larb"),
	SWCG("ven_venc"),
	SWCG("ven_jpgenc"),
	SWCG("ven_gals"),
	SWCG("ven_gals_sram"),
	SWCG(NULL),
};
/* wpe1_dip1 */
struct pd_check_swcg wpe1_dip1_swcgs[] = {
	SWCG("wpe1_dip1_larb11"),
	SWCG("wpe1_dip1_wpe"),
	SWCG(NULL),
};
/* wpe2_dip1 */
struct pd_check_swcg wpe2_dip1_swcgs[] = {
	SWCG("wpe2_dip1_larb11"),
	SWCG("wpe2_dip1_wpe"),
	SWCG(NULL),
};
/* wpe3_dip1 */
struct pd_check_swcg wpe3_dip1_swcgs[] = {
	SWCG("wpe3_dip1_larb11"),
	SWCG("wpe3_dip1_wpe"),
	SWCG(NULL),
};

struct subsys_cgs_check {
	unsigned int pd_id;		/* power domain id */
	int pd_parent;		/* power domain parent id */
	struct pd_check_swcg *swcgs;	/* those CGs that would be checked */
	enum chk_sys_id chk_id;		/*
					 * chk_id is used in
					 * print_subsys_reg() and can be NULL
					 * if not porting ready yet.
					 */
};

struct subsys_cgs_check mtk_subsys_check[] = {
	{MT6886_CHK_PD_AUDIO, PD_NULL, afe_swcgs, afe},
	{MT6886_CHK_PD_CAM_MAIN, MT6886_CHK_PD_CAM_VCORE, camsys_main_swcgs, cam_m},
	{MT6886_CHK_PD_CAM_MRAW, MT6886_CHK_PD_CAM_MAIN, camsys_mraw_swcgs, cam_mr},
	{MT6886_CHK_PD_CAM_SUBA, MT6886_CHK_PD_CAM_MAIN, camsys_rawa_swcgs, cam_ra},
	{MT6886_CHK_PD_CAM_SUBB, MT6886_CHK_PD_CAM_MAIN, camsys_rawb_swcgs, cam_rb},
	{MT6886_CHK_PD_CAM_SUBA, MT6886_CHK_PD_CAM_MAIN, camsys_yuva_swcgs, cam_ya},
	{MT6886_CHK_PD_CAM_SUBB, MT6886_CHK_PD_CAM_MAIN, camsys_yuvb_swcgs, cam_yb},
	{MT6886_CHK_PD_CAM_MAIN, MT6886_CHK_PD_CAM_VCORE, ccu_main_swcgs, ccu},
	{MT6886_CHK_PD_ISP_DIP1, MT6886_CHK_PD_ISP_MAIN, dip_nr1_dip1_swcgs, dip_nr1_dip1},
	{MT6886_CHK_PD_ISP_DIP1, MT6886_CHK_PD_ISP_MAIN, dip_nr2_dip1_swcgs, dip_nr2_dip1},
	{MT6886_CHK_PD_ISP_DIP1, MT6886_CHK_PD_ISP_MAIN, dip_top_dip1_swcgs, dip_top_dip1},
	{MT6886_CHK_PD_DIS0, MT6886_CHK_PD_MM_INFRA, dispsys_config_swcgs, mm},
	{MT6886_CHK_PD_ISP_MAIN, MT6886_CHK_PD_ISP_VCORE, imgsys_main_swcgs, img},
	{MT6886_CHK_PD_MDP0, MT6886_CHK_PD_MM_INFRA, mdpsys_config_swcgs, mdp},
	{MT6886_CHK_PD_MM_INFRA, PD_NULL, mminfra_config_swcgs, mminfra_config},
	{MT6886_CHK_PD_ISP_DIP1, MT6886_CHK_PD_ISP_MAIN, traw_dip1_swcgs, traw_dip1},
	{MT6886_CHK_PD_VDE0, MT6886_CHK_PD_MM_PROC, vdec_gcon_base_swcgs, vde2},
	{MT6886_CHK_PD_VEN0, MT6886_CHK_PD_MM_INFRA, venc_gcon_swcgs, ven},
	{MT6886_CHK_PD_ISP_DIP1, MT6886_CHK_PD_ISP_MAIN, wpe1_dip1_swcgs, wpe1_dip1},
	{MT6886_CHK_PD_ISP_DIP1, MT6886_CHK_PD_ISP_MAIN, wpe2_dip1_swcgs, wpe2_dip1},
	{MT6886_CHK_PD_ISP_DIP1, MT6886_CHK_PD_ISP_MAIN, wpe3_dip1_swcgs, wpe3_dip1},
};

static struct pd_check_swcg *get_subsys_cg(unsigned int id)
{
	int i;

	if (id >= MT6886_CHK_PD_NUM)
		return NULL;

	for (i = 0; i < ARRAY_SIZE(mtk_subsys_check); i++) {
		if (mtk_subsys_check[i].pd_id == id)
			return mtk_subsys_check[i].swcgs;
	}

	return NULL;
}

static void dump_subsys_reg(unsigned int id)
{
	int i;

	if (id >= MT6886_CHK_PD_NUM)
		return;

	for (i = 0; i < ARRAY_SIZE(mtk_subsys_check); i++) {
		if (mtk_subsys_check[i].pd_id == id)
			print_subsys_reg_mt6886(mtk_subsys_check[i].chk_id);
	}
}

unsigned int pd_list[] = {
	MT6886_CHK_PD_MFG1,
	MT6886_CHK_PD_MFG2,
	MT6886_CHK_PD_MFG9,
	MT6886_CHK_PD_MFG10,
	MT6886_CHK_PD_MFG11,
	MT6886_CHK_PD_MFG12,
	MT6886_CHK_PD_MD1,
	MT6886_CHK_PD_CONN,
	MT6886_CHK_PD_UFS0,
	MT6886_CHK_PD_UFS0_PHY,
	MT6886_CHK_PD_AUDIO,
	MT6886_CHK_PD_ADSP_TOP,
	MT6886_CHK_PD_ADSP_INFRA,
	MT6886_CHK_PD_ISP_MAIN,
	MT6886_CHK_PD_ISP_DIP1,
	MT6886_CHK_PD_ISP_VCORE,
	MT6886_CHK_PD_VDE0,
	MT6886_CHK_PD_VEN0,
	MT6886_CHK_PD_CAM_MAIN,
	MT6886_CHK_PD_CAM_MRAW,
	MT6886_CHK_PD_CAM_SUBA,
	MT6886_CHK_PD_CAM_SUBB,
	MT6886_CHK_PD_CAM_VCORE,
	MT6886_CHK_PD_MDP0,
	MT6886_CHK_PD_DIS0,
	MT6886_CHK_PD_MM_INFRA,
	MT6886_CHK_PD_MM_PROC,
};

static bool is_in_pd_list(unsigned int id)
{
	int i;

	if (id >= MT6886_CHK_PD_NUM)
		return false;

	for (i = 0; i < ARRAY_SIZE(pd_list); i++) {
		if (id == pd_list[i])
			return true;
	}

	return false;
}

static enum chk_sys_id debug_dump_id[] = {
	ifrao,
	perao,
	spm,
	top,
	apmixed,
	ufscfg_ao_bus,
	gpu_eb_rpc,
	mfg_ao,
	mfgsc_ao,
	vlpcfg,
	vlp_ck,
	cci,
	cpu_ll,
	cpu_bl,
	ptp,
	chk_sys_num,
};

static void debug_dump(unsigned int id, unsigned int pwr_sta)
{
	int i, parent_id = PD_NULL;

	if (id >= MT6886_CHK_PD_NUM)
		return;

	set_subsys_reg_dump_mt6886(debug_dump_id);

	get_subsys_reg_dump_mt6886();

	for (i = 0; i < ARRAY_SIZE(mtk_subsys_check); i++) {
		if (mtk_subsys_check[i].pd_id == id) {
			print_subsys_reg_mt6886(mtk_subsys_check[i].chk_id);
			parent_id = mtk_subsys_check[i].pd_parent;
			break;
		}
	}

	for (i = 0; i < ARRAY_SIZE(mtk_subsys_check); i++) {
		if (parent_id == PD_NULL)
			break;

		if (mtk_subsys_check[i].pd_id == parent_id)
			print_subsys_reg_mt6886(mtk_subsys_check[i].chk_id);
	}

	BUG_ON(1);
}

static enum chk_sys_id log_dump_id[] = {
	ufscfg_ao_bus,
	gpu_eb_rpc,
	spm,
	vlpcfg,
	chk_sys_num,
};

static void log_dump(unsigned int id, unsigned int pwr_sta)
{
	if (id >= MT6886_CHK_PD_NUM)
		return;

	if (id == MT6886_CHK_PD_MD1) {
		set_subsys_reg_dump_mt6886(log_dump_id);
		get_subsys_reg_dump_mt6886();
	}
}

static struct pd_sta pd_pwr_sta[] = {
	{MT6886_CHK_PD_MFG1, gpu_eb_rpc, 0x0070, GENMASK(31, 30)},
	{MT6886_CHK_PD_MFG2, gpu_eb_rpc, 0x00A0, GENMASK(31, 30)},
	{MT6886_CHK_PD_MFG9, gpu_eb_rpc, 0x00BC, GENMASK(31, 30)},
	{MT6886_CHK_PD_MFG10, gpu_eb_rpc, 0x00C0, GENMASK(31, 30)},
	{MT6886_CHK_PD_MFG11, gpu_eb_rpc, 0x00C4, GENMASK(31, 30)},
	{MT6886_CHK_PD_MFG12, gpu_eb_rpc, 0x00C8, GENMASK(31, 30)},
	{MT6886_CHK_PD_MD1, spm, 0x0E00, GENMASK(31, 30)},
	{MT6886_CHK_PD_CONN, spm, 0x0E04, GENMASK(31, 30)},
	{MT6886_CHK_PD_UFS0, spm, 0x0E10, GENMASK(31, 30)},
	{MT6886_CHK_PD_UFS0_PHY, spm, 0x0E14, GENMASK(31, 30)},
	{MT6886_CHK_PD_AUDIO, spm, 0x0E2C, GENMASK(31, 30)},
	{MT6886_CHK_PD_ADSP_TOP, spm, 0x0E30, GENMASK(31, 30)},
	{MT6886_CHK_PD_ADSP_INFRA, spm, 0x0E34, GENMASK(31, 30)},
	{MT6886_CHK_PD_ISP_MAIN, spm, 0x0E3C, GENMASK(31, 30)},
	{MT6886_CHK_PD_ISP_DIP1, spm, 0x0E40, GENMASK(31, 30)},
	{MT6886_CHK_PD_ISP_VCORE, spm, 0x0E48, GENMASK(31, 30)},
	{MT6886_CHK_PD_VDE0, spm, 0x0E4C, GENMASK(31, 30)},
	{MT6886_CHK_PD_VEN0, spm, 0x0E54, GENMASK(31, 30)},
	{MT6886_CHK_PD_CAM_MAIN, spm, 0x0E60, GENMASK(31, 30)},
	{MT6886_CHK_PD_CAM_MRAW, spm, 0x0E64, GENMASK(31, 30)},
	{MT6886_CHK_PD_CAM_SUBA, spm, 0x0E68, GENMASK(31, 30)},
	{MT6886_CHK_PD_CAM_SUBB, spm, 0x0E6C, GENMASK(31, 30)},
	{MT6886_CHK_PD_CAM_VCORE, spm, 0x0E74, GENMASK(31, 30)},
	{MT6886_CHK_PD_MDP0, spm, 0x0E78, GENMASK(31, 30)},
	{MT6886_CHK_PD_DIS0, spm, 0x0E80, GENMASK(31, 30)},
	{MT6886_CHK_PD_MM_INFRA, spm, 0x0E90, GENMASK(31, 30)},
	{MT6886_CHK_PD_MM_PROC, spm, 0x0E94, GENMASK(31, 30)},
};

static u32 get_pd_pwr_status(int pd_id)
{
	u32 val;
	int i;

	if (pd_id == PD_NULL || pd_id > ARRAY_SIZE(pd_pwr_sta))
		return 0;

	for (i = 0; i < ARRAY_SIZE(pd_pwr_sta); i++) {
		if (pd_id == pd_pwr_sta[i].pd_id) {
			val = get_mt6886_reg_value(pd_pwr_sta[i].base, pd_pwr_sta[i].ofs);
			if ((val & pd_pwr_sta[i].msk) == pd_pwr_sta[i].msk)
				return 1;
			else
				return 0;
		}
	}

	return 0;
}

static int off_mtcmos_id[] = {
	MT6886_CHK_PD_MFG1,
	MT6886_CHK_PD_MFG2,
	MT6886_CHK_PD_MFG9,
	MT6886_CHK_PD_MFG10,
	MT6886_CHK_PD_MFG11,
	MT6886_CHK_PD_MFG12,
	MT6886_CHK_PD_UFS0,
	MT6886_CHK_PD_ADSP_INFRA,
	MT6886_CHK_PD_ISP_MAIN,
	MT6886_CHK_PD_ISP_DIP1,
	MT6886_CHK_PD_ISP_VCORE,
	MT6886_CHK_PD_VDE0,
	MT6886_CHK_PD_VEN0,
	MT6886_CHK_PD_CAM_MAIN,
	MT6886_CHK_PD_CAM_MRAW,
	MT6886_CHK_PD_CAM_SUBA,
	MT6886_CHK_PD_CAM_SUBB,
	MT6886_CHK_PD_CAM_VCORE,
	MT6886_CHK_PD_MDP0,
	MT6886_CHK_PD_DIS0,
	MT6886_CHK_PD_MM_INFRA,
	MT6886_CHK_PD_MM_PROC,
	PD_NULL,
};

static int notice_mtcmos_id[] = {
	MT6886_CHK_PD_UFS0_PHY,
	MT6886_CHK_PD_MD1,
	MT6886_CHK_PD_CONN,
	MT6886_CHK_PD_AUDIO,
	MT6886_CHK_PD_ADSP_TOP,
	PD_NULL,
};

static int *get_off_mtcmos_id(void)
{
	return off_mtcmos_id;
}

static int *get_notice_mtcmos_id(void)
{
	return notice_mtcmos_id;
}

static bool is_mtcmos_chk_bug_on(void)
{
#if (BUG_ON_CHK_ENABLE) || (IS_ENABLED(CONFIG_MTK_CLKMGR_DEBUG))
	return true;
#endif
	return false;
}

static int suspend_allow_id[] = {
	MT6886_CHK_PD_UFS0,
	PD_NULL,
};

static int *get_suspend_allow_id(void)
{
	return suspend_allow_id;
}

static bool pdchk_suspend_retry(bool reset_cnt)
{
	if (reset_cnt == true) {
		suspend_cnt = 0;
		return true;
	}

	suspend_cnt++;
	pr_notice("%s: suspend cnt: %d\n", __func__, suspend_cnt);

	if (suspend_cnt < 2)
		return false;

	return true;
}

/*
 * init functions
 */

static struct pdchk_ops pdchk_mt6886_ops = {
	.get_subsys_cg = get_subsys_cg,
	.dump_subsys_reg = dump_subsys_reg,
	.is_in_pd_list = is_in_pd_list,
	.debug_dump = debug_dump,
	.log_dump = log_dump,
	.get_pd_pwr_status = get_pd_pwr_status,
	.get_off_mtcmos_id = get_off_mtcmos_id,
	.get_notice_mtcmos_id = get_notice_mtcmos_id,
	.is_mtcmos_chk_bug_on = is_mtcmos_chk_bug_on,
	.get_suspend_allow_id = get_suspend_allow_id,
	.pdchk_suspend_retry = pdchk_suspend_retry,
};

static int pd_chk_mt6886_probe(struct platform_device *pdev)
{
	suspend_cnt = 0;

	pdchk_common_init(&pdchk_mt6886_ops);

	return 0;
}

static const struct of_device_id of_match_pdchk_mt6886[] = {
	{
		.compatible = "mediatek,mt6886-pdchk",
	}, {
		/* sentinel */
	}
};

static struct platform_driver pd_chk_mt6886_drv = {
	.probe = pd_chk_mt6886_probe,
	.driver = {
		.name = "pd-chk-mt6886",
		.owner = THIS_MODULE,
		.pm = &pdchk_dev_pm_ops,
		.of_match_table = of_match_pdchk_mt6886,
	},
};

/*
 * init functions
 */

static int __init pd_chk_init(void)
{
	return platform_driver_register(&pd_chk_mt6886_drv);
}

static void __exit pd_chk_exit(void)
{
	platform_driver_unregister(&pd_chk_mt6886_drv);
}

subsys_initcall(pd_chk_init);
module_exit(pd_chk_exit);
MODULE_LICENSE("GPL");
