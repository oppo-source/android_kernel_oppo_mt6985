/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2022 MediaTek Inc.
 * Author: Owen Chen <owen.chen@mediatek.com>
 */

#ifndef __DRV_CLKCHK_MT6985_H
#define __DRV_CLKCHK_MT6985_H

enum chk_sys_id {
	top = 0,
	ifrao = 1,
	apmixed = 2,
	ifr_bus = 3,
	emi_reg = 4,
	semi_reg = 5,
	ssr_top = 6,
	perao = 7,
	afe = 8,
	impc = 9,
	ufscfg_ao_bus = 10,
	ufsao = 11,
	ufspdn = 12,
	pext = 13,
	imps = 14,
	impn = 15,
	gpu_eb_rpc = 16,
	mfg_ao = 17,
	mfgsc_ao = 18,
	mm = 19,
	mm1 = 20,
	ovl = 21,
	ovl1 = 22,
	img = 23,
	img_sub0_bus = 24,
	img_sub1_bus = 25,
	dip_top_dip1 = 26,
	dip_nr1_dip1 = 27,
	dip_nr2_dip1 = 28,
	wpe1_dip1 = 29,
	wpe2_dip1 = 30,
	wpe3_dip1 = 31,
	traw_dip1 = 32,
	vde1 = 33,
	vde2 = 34,
	ven = 35,
	ven_c1 = 36,
	ven_c2 = 37,
	cam_sub0_bus = 38,
	cam_sub2_bus = 39,
	cam_sub1_bus = 40,
	spm = 41,
	vlpcfg = 42,
	vlp_ck = 43,
	scp = 44,
	scp_iic = 45,
	cam_m = 46,
	cam_ra = 47,
	cam_ya = 48,
	cam_rb = 49,
	cam_yb = 50,
	cam_rc = 51,
	cam_yc = 52,
	cam_mr = 53,
	ccu = 54,
	dvfsrc_apb = 55,
	sramrc_apb = 56,
	mminfra_config = 57,
	mdp = 58,
	mdp1 = 59,
	cci = 60,
	cpu_ll = 61,
	cpu_bl = 62,
	cpu_b = 63,
	ptp = 64,
	ifr_mem = 65,
	semi = 66,
	hwv_wrt = 67,
	hwv = 68,
	hwv_ext = 69,
	pmsr = 70,
	smi_larb12 = 71,
	smi_subcomm = 72,
	chk_sys_num = 73,
};

enum chk_pd_id {
	MT6985_CHK_PD_MFG1 = 0,
	MT6985_CHK_PD_MFG2 = 1,
	MT6985_CHK_PD_MFG3 = 2,
	MT6985_CHK_PD_MFG4 = 3,
	MT6985_CHK_PD_MFG5 = 4,
	MT6985_CHK_PD_MFG6 = 5,
	MT6985_CHK_PD_MFG7 = 6,
	MT6985_CHK_PD_MFG8 = 7,
	MT6985_CHK_PD_MFG9 = 8,
	MT6985_CHK_PD_MFG10 = 9,
	MT6985_CHK_PD_MFG11 = 10,
	MT6985_CHK_PD_MFG12 = 11,
	MT6985_CHK_PD_MFG13 = 12,
	MT6985_CHK_PD_MFG14 = 13,
	MT6985_CHK_PD_MFG15 = 14,
	MT6985_CHK_PD_MFG16 = 15,
	MT6985_CHK_PD_MFG17 = 16,
	MT6985_CHK_PD_MFG18 = 17,
	MT6985_CHK_PD_MFG19 = 18,
	MT6985_CHK_PD_MD1 = 19,
	MT6985_CHK_PD_CONN = 20,
	MT6985_CHK_PD_UFS0 = 21,
	MT6985_CHK_PD_UFS0_PHY = 22,
	MT6985_CHK_PD_PEXTP_MAC0 = 23,
	MT6985_CHK_PD_PEXTP_MAC1 = 24,
	MT6985_CHK_PD_PEXTP_PHY0 = 25,
	MT6985_CHK_PD_PEXTP_PHY1 = 26,
	MT6985_CHK_PD_AUDIO = 27,
	MT6985_CHK_PD_ADSP_TOP = 28,
	MT6985_CHK_PD_MM_INFRA = 29,
	MT6985_CHK_PD_MM_PROC = 30,
	MT6985_CHK_PD_ISP_VCORE = 31,
	MT6985_CHK_PD_ISP_MAIN = 32,
	MT6985_CHK_PD_ISP_DIP1 = 33,
	MT6985_CHK_PD_VDE_VCORE0 = 34,
	MT6985_CHK_PD_VDE0 = 35,
	MT6985_CHK_PD_VDE_VCORE1 = 36,
	MT6985_CHK_PD_VDE1 = 37,
	MT6985_CHK_PD_VEN0 = 38,
	MT6985_CHK_PD_VEN1 = 39,
	MT6985_CHK_PD_VEN2 = 40,
	MT6985_CHK_PD_CAM_VCORE = 41,
	MT6985_CHK_PD_CAM_MAIN = 42,
	MT6985_CHK_PD_CAM_MRAW = 43,
	MT6985_CHK_PD_CAM_SUBA = 44,
	MT6985_CHK_PD_CAM_SUBB = 45,
	MT6985_CHK_PD_CAM_SUBC = 46,
	MT6985_CHK_PD_MDP0 = 47,
	MT6985_CHK_PD_MDP1 = 48,
	MT6985_CHK_PD_DIS0 = 49,
	MT6985_CHK_PD_DIS1 = 50,
	MT6985_CHK_PD_OVLSYS = 51,
	MT6985_CHK_PD_OVLSYS1 = 52,
	MT6985_CHK_PD_DP_TX = 53,
	MT6985_CHK_PD_CSI_RX = 54,
	MT6985_CHK_PD_APU = 55,
	MT6985_CHK_PD_NUM,
};

#ifdef CONFIG_MTK_DVFSRC_HELPER
extern int get_sw_req_vcore_opp(void);
#endif

extern void print_subsys_reg_mt6985(enum chk_sys_id id);
extern void set_subsys_reg_dump_mt6985(enum chk_sys_id id[]);
extern void get_subsys_reg_dump_mt6985(void);
extern u32 get_mt6985_reg_value(u32 id, u32 ofs);
extern void release_mt6985_hwv_secure(void);
#endif	/* __DRV_CLKCHK_MT6985_H */
