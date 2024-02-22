// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <linux/of_graph.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <soc/oplus/device_info.h>
#include <linux/regulator/consumer.h>
#include <mt-plat/mtk_boot_common.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

/*#ifdef OPLUS_BUG_STABILITY*/
#include "../oplus/oplus_display_panel_power.h"
#include "../mediatek/mediatek_v2/mtk_log.h"

#include "AC068_P_3_A0001_fhd_cmd.h"
#include "oplus_bl.h"
#ifdef OPLUS_FEATURE_DISPLAY_ADFR
#include "../oplus/oplus_adfr_ext.h"
#endif /* OPLUS_FEATURE_DISPLAY_ADFR  */
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
/* add for ofp */
#include "../oplus/oplus_display_onscreenfingerprint.h"
/* add for cmdq_pkt_sleep */
#include "../mediatek/mediatek_v2/mtk-cmdq-ext.h"
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
/* add for dips_drv log  */
#include "../oplus/oplus_display_mtk_debug.h"
#include "../oplus/oplus_drm_disp_panel.h"
#include "../oplus/oplus_display_high_frequency_pwm.h"
#define BRIGHTNESS_MAX    4095
#define BRIGHTNESS_HALF   2047
#define MAX_NORMAL_BRIGHTNESS   3515
#define SILKY_MAX_NORMAL_BRIGHTNESS   8191
#define LCM_BRIGHTNESS_TYPE 2
#define LCM_DV_HPWM 0x24
#define LCM_DV_HPWM_70 0x96
#define LCM_DV_HPWM_DVT 0x25
#define DRM_PANEL_EVENT_PWM_TURBO  0x14
#define PCB_VERSION_DVT 7

#define FHD_LCM_WIDTH  1080
#define FHD_LCM_HEIGHT 2520

#define lcm_dcs_write_seq(ctx, seq...)				\
	({												\
		const u8 d[] = { seq };						\
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 128,		\
				 "DCS sequence too big for stack");	\
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));		\
	})

#define lcm_dcs_write_seq_static(ctx, seq...)		\
	({												\
		static const u8 d[] = { seq };				\
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));		\
	})

extern int dsi_panel_parse_panel_power_cfg(struct panel_voltage_bak *panel_vol);
static PANEL_VOLTAGE_BAK panel_vol_bak[PANEL_VOLTAGE_ID_MAX] = {
						{0, 1800000, 1800000, 1800000, "vddi"},
						{1, 1800000, 1104000, 1800000, "vddr"},
						{2, 3000000, 1104000, 3000000, "vci"},
						{3, 0, 1, 2, ""}};

extern unsigned int seed_mode;
extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
static int esd_brightness;
extern unsigned int last_backlight;
extern bool pwm_power_on;
extern unsigned int hpwm_bl;
extern unsigned int hpwm_mode;
extern unsigned int lcm_id2;

/* whether enter hbm brightness level or not */
static bool hbm_brightness_flag = false;

extern void lcdinfo_notify(unsigned long val, void *v);
extern unsigned int get_PCB_Version(void);
static unsigned int pcb_version = 0;
extern struct oplus_demura_setting_table demura_setting;
extern bool oplus_display_get_demura_support(void);
extern void oplus_display_get_demura_cfg(struct device *dev);
struct lcm_pmic_info {
	struct regulator *reg_vufs18;
	struct regulator *reg_vmch3p0;
};

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *sm3011_reset_gpio;
	struct gpio_desc *bias_pos, *bias_neg;
	struct gpio_desc *bias_gpio;
	struct gpio_desc *vddr1p5_enable_gpio;
	struct gpio_desc *aod1p3_enable_gpio;
	struct gpio_desc *vddi1p8_enable_gpio;
	struct drm_display_mode *m;
	bool prepared;
	bool enabled;

	int error;
};

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_dcs_write_ext(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return;


	ret = mipi_dsi_dcs_write_buffer(dsi, data, len);

	if (ret < 0) {
		DISP_ERR("error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static void push_table(struct lcm *ctx, struct LCM_setting_table *table,
		unsigned int count)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_DELAY:
			usleep_range(table[i].count*1000, table[i].count*1000 + 100);
			break;
		case REGFLAG_UDELAY:
			usleep_range(table[i].count, table[i].count + 100);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			lcm_dcs_write_ext(ctx, table[i].para_list,
				table[i].count);
			break;
		}
	}
}

#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		DISP_ERR("error %d reading dcs seq:(%#x)\n", ret,
			 cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	DISP_INFO("%s+\n");

	if (ret == 0) {
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		DISP_DEBUG("return %d data(0x%08x) to dsi engine\n",ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		DISP_ERR("error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}
#if 1
static struct lcm_pmic_info *g_pmic;
static unsigned int lcm_get_reg_vufs18(void)
{
	unsigned int volt = 0;

	if (regulator_is_enabled(g_pmic->reg_vufs18))
		/* regulator_get_voltage return volt with uV */
		volt = regulator_get_voltage(g_pmic->reg_vufs18) ;

	return volt;
}

static unsigned int lcm_get_reg_vmch3p0(void)
{
	unsigned int volt = 0;

	if (regulator_is_enabled(g_pmic->reg_vmch3p0))
		/* regulator_get_voltage return volt with uV */
		volt = regulator_get_voltage(g_pmic->reg_vmch3p0);

	return volt;
}
static unsigned int lcm_enable_reg_vufs18(int en)
{
	unsigned int ret = 0,volt = 0;
	if (en) {
		if (!IS_ERR_OR_NULL(g_pmic->reg_vufs18)) {
			ret = regulator_enable(g_pmic->reg_vufs18);
			DISP_INFO("Enable the Regulator VRFIO18 ret=%d.\n",ret);
			volt = lcm_get_reg_vufs18();
			DISP_DEBUG("get the Regulator VRFIO18 = %d.\n",volt);
		}
	} else {
		if (!IS_ERR_OR_NULL(g_pmic->reg_vufs18)) {
			ret = regulator_disable(g_pmic->reg_vufs18);
			volt = lcm_get_reg_vufs18();
			DISP_INFO("disable the Regulator VRFIO18 ret=%d,volt=%d.\n",ret,volt);
		}
	}
	return ret;

}

static unsigned int lcm_enable_reg_vmch3p0(int en)
{
	unsigned int ret=0,volt = 0;
	if(en) {
		if (!IS_ERR_OR_NULL(g_pmic->reg_vmch3p0)) {
			ret = regulator_enable(g_pmic->reg_vmch3p0);
			DISP_INFO("Enable the Regulator vmc3p0 ret=%d.\n",ret);
			volt = lcm_get_reg_vmch3p0();
			DISP_DEBUG("get the Regulator vmc3p0 =%d.\n",volt);
		}
	}else {
		if (!IS_ERR_OR_NULL(g_pmic->reg_vmch3p0)) {
			ret = regulator_disable(g_pmic->reg_vmch3p0);
			volt = lcm_get_reg_vmch3p0();
			DISP_INFO("disable the Regulator vmc3p0 ret=%d,volt=%d.\n",ret,volt);
		}
	}
	return ret;

}
#endif

static struct drm_display_mode *get_mode_by_id(struct drm_connector *connector,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}

	return NULL;
}

static int get_mode_enum(struct drm_display_mode *m)
{
	int ret = 0;
	int m_vrefresh = 0;

	if (m == NULL) {
		return -EINVAL;
	}

	m_vrefresh = drm_mode_vrefresh(m);

	if (m_vrefresh == 60) {
		ret = FHD_SDC60;
	} else if (m_vrefresh == 120) {
		ret = FHD_SDC120;
	} else {
		ret = FHD_SDC60;
	}
	return ret;
}


static void lcm_panel_init(struct lcm *ctx)
{
    int mode_id = -1;
	struct drm_display_mode *m = ctx->m;

	mode_id = get_mode_enum(m);
	switch (mode_id) {
	case FHD_SDC60:
		if ((lcm_id2 > 0) && (lcm_id2 < LCM_DV_HPWM)) {
			push_table(ctx, fhd_dsi_on_cmd_sdc60, sizeof(fhd_dsi_on_cmd_sdc60)/sizeof(struct LCM_setting_table));
			DISP_INFO("fhd_dsi_on_cmd_sdc60\n");
		} else {
			if ((lcm_id2 == LCM_DV_HPWM) || (lcm_id2 == LCM_DV_HPWM_70)) {
				push_table(ctx, fhd_dsi_on_high_pwm_cmd_sdc60_evt, sizeof(fhd_dsi_on_high_pwm_cmd_sdc60_evt)/sizeof(struct LCM_setting_table));
				DISP_INFO("fhd_dsi_on_high_pwm_cmd_sdc60_evt.\n");
			} else if (lcm_id2 == LCM_DV_HPWM_DVT) {
				push_table(ctx, fhd_dsi_on_high_pwm_cmd_sdc60_dvt, sizeof(fhd_dsi_on_high_pwm_cmd_sdc60_dvt)/sizeof(struct LCM_setting_table));
				DISP_INFO("fhd_dsi_on_high_pwm_cmd_sdc60_dvt.\n");
			} else {
				if (oplus_display_get_demura_support()) {
					push_table(ctx, fhd_dsi_on_demura_pwm_cmd_sdc60, sizeof(fhd_dsi_on_demura_pwm_cmd_sdc60)/sizeof(struct LCM_setting_table));
					DISP_INFO("fhd_dsi_on_demura_pwm_cmd_sdc60.\n");
				} else {
					push_table(ctx, fhd_dsi_on_high_pwm_cmd_sdc60, sizeof(fhd_dsi_on_high_pwm_cmd_sdc60)/sizeof(struct LCM_setting_table));
					DISP_INFO("fhd_dsi_on_high_pwm_cmd_sdc60.\n");
				}
			}
		}
		break;
	case FHD_SDC120:
		if ((lcm_id2 > 0) && (lcm_id2 < LCM_DV_HPWM)) {
			push_table(ctx, fhd_dsi_on_cmd_sdc120, sizeof(fhd_dsi_on_cmd_sdc120)/sizeof(struct LCM_setting_table));
			DISP_INFO("fhd_dsi_on_cmd_sdc120\n");
		} else {
			if ((lcm_id2 == LCM_DV_HPWM) || (lcm_id2 == LCM_DV_HPWM_70)) {
				push_table(ctx, fhd_dsi_on_high_pwm_cmd_sdc120_evt, sizeof(fhd_dsi_on_high_pwm_cmd_sdc120_evt)/sizeof(struct LCM_setting_table));
				DISP_INFO("fhd_dsi_on_high_pwm_cmd_sdc120_evt.\n");
			} else if (lcm_id2 == LCM_DV_HPWM_DVT) {
				push_table(ctx, fhd_dsi_on_high_pwm_cmd_sdc120_dvt, sizeof(fhd_dsi_on_high_pwm_cmd_sdc120_dvt)/sizeof(struct LCM_setting_table));
				DISP_INFO("fhd_dsi_on_high_pwm_cmd_sdc120_dvt.\n");
			} else {
				if (oplus_display_get_demura_support()) {
					push_table(ctx, fhd_dsi_on_demura_pwm_cmd_sdc120, sizeof(fhd_dsi_on_demura_pwm_cmd_sdc120)/sizeof(struct LCM_setting_table));
					DISP_INFO("fhd_dsi_on_demura_pwm_cmd_sdc120.\n");
				} else {
					push_table(ctx, fhd_dsi_on_high_pwm_cmd_sdc120, sizeof(fhd_dsi_on_high_pwm_cmd_sdc120)/sizeof(struct LCM_setting_table));
					DISP_INFO("fhd_dsi_on_high_pwm_cmd_sdc120.\n");
				}
			}
		}
		break;
	default:
		if ((lcm_id2 > 0) && (lcm_id2 < LCM_DV_HPWM)) {
			push_table(ctx, fhd_dsi_on_cmd_sdc60, sizeof(fhd_dsi_on_cmd_sdc60)/sizeof(struct LCM_setting_table));
			DISP_INFO("default mode_id\n");
		} else {
			if ((lcm_id2 == LCM_DV_HPWM) || (lcm_id2 == LCM_DV_HPWM_70)) {
				push_table(ctx, fhd_dsi_on_high_pwm_cmd_sdc60_evt, sizeof(fhd_dsi_on_high_pwm_cmd_sdc60_evt)/sizeof(struct LCM_setting_table));
				DISP_INFO("default high_pwm_mode_id_evt mode_id=%d\n", mode_id);
			} else if (lcm_id2 == LCM_DV_HPWM_DVT) {
				push_table(ctx, fhd_dsi_on_high_pwm_cmd_sdc60_dvt, sizeof(fhd_dsi_on_high_pwm_cmd_sdc60_dvt)/sizeof(struct LCM_setting_table));
				DISP_INFO("default high_pwm_mode_id_dvt mode_id=%d\n", mode_id);
			} else {
				if (oplus_display_get_demura_support()) {
					push_table(ctx, fhd_dsi_on_demura_pwm_cmd_sdc60, sizeof(fhd_dsi_on_demura_pwm_cmd_sdc60)/sizeof(struct LCM_setting_table));
					DISP_INFO("default fhd_dsi_on_demura_pwm_cmd_sdc60.\n");
				} else {
					push_table(ctx, fhd_dsi_on_high_pwm_cmd_sdc60, sizeof(fhd_dsi_on_high_pwm_cmd_sdc60)/sizeof(struct LCM_setting_table));
					DISP_INFO("default fhd_dsi_on_high_pwm_cmd_sdc60.\n");
				}
			}
		}
		break;
	}

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	if (oplus_adfr_is_support()) {
		// reset adfr auto mode status as auto mode will be change after power on
		oplus_adfr_status_reset(NULL, m);
	}
#endif /* OPLUS_FEATURE_DISPLAY_ADFR  */

	pwm_power_on = true;
	DISP_DEBUG("debug for successful! mode_id=%d\n", mode_id);
}

static void lcm_panel_seed(struct lcm *ctx){
    if (seed_mode == 100) {
        push_table(ctx, lcm_seed_mode0, sizeof(lcm_seed_mode0)/sizeof(struct LCM_setting_table));
    } else if (seed_mode == 101){
        push_table(ctx, lcm_seed_mode1, sizeof(lcm_seed_mode1)/sizeof(struct LCM_setting_table));
    } else if (seed_mode == 102){
        push_table(ctx, lcm_seed_mode2, sizeof(lcm_seed_mode2)/sizeof(struct LCM_setting_table));
    }
	DISP_INFO("seed_mode=%d successful!\n", seed_mode);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int vrefresh_rate = 0;
	OFP_INFO("prepared=%d\n",ctx->prepared);

	if (!ctx->prepared) {
		return 0;
	}

	if (!ctx->m) {
		vrefresh_rate = 120;
		OFP_INFO("default refresh rate is 120hz\n");
	} else {
		vrefresh_rate = drm_mode_vrefresh(ctx->m);
	}

	if (oplus_ofp_get_aod_state() == true) {
		if (vrefresh_rate == 60) {
			if ((lcm_id2 > 0) && (lcm_id2 < LCM_DV_HPWM)) {
				push_table(ctx, aod_off_cmd_lpwm_60hz, sizeof(aod_off_cmd_lpwm_60hz) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, aod_off_cmd_hpwm_60hz, sizeof(aod_off_cmd_hpwm_60hz) / sizeof(struct LCM_setting_table));
			}
		} else {
			if ((lcm_id2 > 0) && (lcm_id2 < LCM_DV_HPWM)) {
				push_table(ctx, aod_off_cmd_lpwm_120hz, sizeof(aod_off_cmd_lpwm_120hz) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, aod_off_cmd_hpwm_120hz, sizeof(aod_off_cmd_hpwm_120hz) / sizeof(struct LCM_setting_table));
			}
		}
		usleep_range(9000, 9100);
		DISP_INFO("send aod off cmd\n");
	}

	lcm_dcs_write_seq_static(ctx, 0x28);
	usleep_range(25000, 25100);
	lcm_dcs_write_seq_static(ctx, 0x10);
	usleep_range(125000, 125100);
	ctx->error = 0;
	ctx->prepared = false;
	DISP_INFO("lcm_unprepare Successful\n");

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared) {
		return 0;
	}

	lcm_panel_init(ctx);
	usleep_range(2000, 2100);
	lcm_panel_seed(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	DISP_INFO("lcm_prepare Successful\n");
	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

#define HFP (12)
#define HSA (10)
#define HBP (13)
#define VFP (20)
#define VSA (2)
#define VBP (8)

#define VAC_FHD (2520)
#define HAC_FHD (1080)

static const struct drm_display_mode display_mode[MODE_NUM] = {
	//fhd_sdc_60_mode
	{
    	.clock = 170595,
    	.hdisplay = HAC_FHD,
    	.hsync_start = HAC_FHD + HFP,
    	.hsync_end = HAC_FHD + HFP + HSA,
    	.htotal = HAC_FHD + HFP + HSA + HBP,
    	.vdisplay = VAC_FHD,
    	.vsync_start = VAC_FHD + VFP,
    	.vsync_end = VAC_FHD + VFP + VSA,
    	.vtotal = VAC_FHD + VFP + VSA + VBP,
    	.hskew = SDC_MFR,
	},
	//fhd_sdc_120_mode
	{
    	.clock = 341190,
    	.hdisplay = HAC_FHD,
    	.hsync_start = HAC_FHD + HFP,
    	.hsync_end = HAC_FHD + HFP + HSA,
    	.htotal = HAC_FHD + HFP + HSA + HBP,
    	.vdisplay = VAC_FHD,
    	.vsync_start = VAC_FHD + VFP,
    	.vsync_end = VAC_FHD + VFP + VSA,
    	.vtotal = VAC_FHD + VFP + VSA + VBP,
    	.hskew = SDC_ADFR,
	},
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params[MODE_NUM] = {
	//fhd_sdc_60_mode
	{
		.pll_clk = 413,
		.data_rate = 826,
		.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
		.cmd_null_pkt_en = 1,
		.cmd_null_pkt_len = 95,
		.merge_trig_offset = 660,
		.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
		.dsc_params = {
			.enable                =  DSC_ENABLE,
			.ver                   =  DSC_VER,
			.dsc_scr_version       =  DSC_SCR_VER,
			.slice_mode            =  DSC_SLICE_MODE,
			.rgb_swap              =  DSC_RGB_SWAP,
			.dsc_cfg               =  DSC_DSC_CFG,
			.rct_on                =  DSC_RCT_ON,
			.bit_per_channel       =  DSC_BIT_PER_CHANNEL,
			.dsc_line_buf_depth    =  DSC_DSC_LINE_BUF_DEPTH,
			.bp_enable             =  DSC_BP_ENABLE,
			.bit_per_pixel         =  DSC_BIT_PER_PIXEL,
			.pic_height            =  FRAME_HEIGHT,
			.pic_width             =  FRAME_WIDTH,
			.slice_height          =  DSC_SLICE_HEIGHT,
			.slice_width           =  DSC_SLICE_WIDTH,
			.chunk_size            =  DSC_CHUNK_SIZE,
			.xmit_delay            =  DSC_XMIT_DELAY,
			.dec_delay             =  DSC_DEC_DELAY,
			.scale_value           =  DSC_SCALE_VALUE,
			.increment_interval    =  DSC_INCREMENT_INTERVAL,
			.decrement_interval    =  DSC_DECREMENT_INTERVAL,
			.line_bpg_offset       =  DSC_LINE_BPG_OFFSET,
			.nfl_bpg_offset        =  DSC_NFL_BPG_OFFSET,
			.slice_bpg_offset      =  DSC_SLICE_BPG_OFFSET,
			.initial_offset        =  DSC_INITIAL_OFFSET,
			.final_offset          =  DSC_FINAL_OFFSET,
			.flatness_minqp        =  DSC_FLATNESS_MINQP,
			.flatness_maxqp        =  DSC_FLATNESS_MAXQP,
			.rc_model_size         =  DSC_RC_MODEL_SIZE,
			.rc_edge_factor        =  DSC_RC_EDGE_FACTOR,
			.rc_quant_incr_limit0  =  DSC_RC_QUANT_INCR_LIMIT0,
			.rc_quant_incr_limit1  =  DSC_RC_QUANT_INCR_LIMIT1,
			.rc_tgt_offset_hi      =  DSC_RC_TGT_OFFSET_HI,
			.rc_tgt_offset_lo      =  DSC_RC_TGT_OFFSET_LO,
		},
		.oplus_mode_switch_hs = 1,
#ifdef OPLUS_FEATURE_DISPLAY_ADFR
		.oplus_fakeframe_cfg = 0,
		.oplus_fakeframe_deferred_time = 0,
		.oplus_autoon_cfg = 0,
		.oplus_autooff_cfg = 0,
		.oplus_minfps0_cfg = 1,
		.oplus_minfps1_cfg = 0,
#endif /* OPLUS_FEATURE_DISPLAY_ADFR  */
		.color_vivid_status = true,
		.color_srgb_status = true,
		.color_softiris_status = false,
		.color_dual_panel_status = false,
		.color_dual_brightness_status = true,
		.color_oplus_calibrate_status = true,
		.color_loading_status = false,
		.color_2nit_status = true,
		.vendor = "AC068_P_3_A0001",
		.manufacture = "P_3",
		.cust_esd_check = 0,
		.esd_check_enable = 1,
		.lcm_esd_check_table[0] = {
			.cmd = 0x0A,
			.count = 1,
			.para_list[0] = 0x9C,
			.mask_list[0] = 0xDC,
		},
		.lcm_esd_check_table[1] = {
			.cmd = 0x91,
			.count = 4,
			.para_list[0] = 0xAB,
			.mask_list[0] = 0xAB,
			.para_list[1] = 0xA8,
			.mask_list[1] = 0xA8,
			.para_list[2] = 0x00,
			.mask_list[2] = 0x00,
			.para_list[3] = 0x28,
			.mask_list[3] = 0x28,
		},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
		.round_corner_en = 1,
		.corner_pattern_height = ROUND_CORNER_1K_H_TOP,
		.corner_pattern_height_bot = ROUND_CORNER_1K_H_BOT,
		.corner_pattern_tp_size_l = sizeof(top_rc_1k_pattern_l),
		.corner_pattern_lt_addr_l = (void *)top_rc_1k_pattern_l,
		.corner_pattern_tp_size_r =  sizeof(top_rc_1k_pattern_r),
		.corner_pattern_lt_addr_r = (void *)top_rc_1k_pattern_r,
#endif
		.phy_timcon = {
			.hs_trail = 14,
			.clk_trail = 13,
			.oplus_lpx_config = 50,
		},
		.dyn_fps = {
			.switch_en = 1,
			.vact_timing_fps = 60,
			.apollo_limit_superior_us = 10540,
			.apollo_limit_inferior_us = 14200,
			.apollo_transfer_time_us = 8200,
		},
		.skip_unnecessary_switch = false,
		.oplus_hpwm_config = 1,
		.panel_bpp = DSC_BIT_PER_CHANNEL,
	},

	//fhd_sdc_120_mode
	{
		.pll_clk = 413,
		.data_rate = 826,
		.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
		.cmd_null_pkt_en = 1,
		.cmd_null_pkt_len = 95,
		.merge_trig_offset = 150,
		.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
		.dsc_params = {
			.enable                =  DSC_ENABLE,
			.ver                   =  DSC_VER,
			.dsc_scr_version       =  DSC_SCR_VER,
			.slice_mode            =  DSC_SLICE_MODE,
			.rgb_swap              =  DSC_RGB_SWAP,
			.dsc_cfg               =  DSC_DSC_CFG,
			.rct_on                =  DSC_RCT_ON,
			.bit_per_channel       =  DSC_BIT_PER_CHANNEL,
			.dsc_line_buf_depth    =  DSC_DSC_LINE_BUF_DEPTH,
			.bp_enable             =  DSC_BP_ENABLE,
			.bit_per_pixel         =  DSC_BIT_PER_PIXEL,
			.pic_height            =  FRAME_HEIGHT,
			.pic_width             =  FRAME_WIDTH,
			.slice_height          =  DSC_SLICE_HEIGHT,
			.slice_width           =  DSC_SLICE_WIDTH,
			.chunk_size            =  DSC_CHUNK_SIZE,
			.xmit_delay            =  DSC_XMIT_DELAY,
			.dec_delay             =  DSC_DEC_DELAY,
			.scale_value           =  DSC_SCALE_VALUE,
			.increment_interval    =  DSC_INCREMENT_INTERVAL,
			.decrement_interval    =  DSC_DECREMENT_INTERVAL,
			.line_bpg_offset       =  DSC_LINE_BPG_OFFSET,
			.nfl_bpg_offset        =  DSC_NFL_BPG_OFFSET,
			.slice_bpg_offset      =  DSC_SLICE_BPG_OFFSET,
			.initial_offset        =  DSC_INITIAL_OFFSET,
			.final_offset          =  DSC_FINAL_OFFSET,
			.flatness_minqp        =  DSC_FLATNESS_MINQP,
			.flatness_maxqp        =  DSC_FLATNESS_MAXQP,
			.rc_model_size         =  DSC_RC_MODEL_SIZE,
			.rc_edge_factor        =  DSC_RC_EDGE_FACTOR,
			.rc_quant_incr_limit0  =  DSC_RC_QUANT_INCR_LIMIT0,
			.rc_quant_incr_limit1  =  DSC_RC_QUANT_INCR_LIMIT1,
			.rc_tgt_offset_hi      =  DSC_RC_TGT_OFFSET_HI,
			.rc_tgt_offset_lo      =  DSC_RC_TGT_OFFSET_LO,
		},
		.oplus_mode_switch_hs = 1,
#ifdef OPLUS_FEATURE_DISPLAY_ADFR
		.oplus_fakeframe_cfg = 0,
		.oplus_fakeframe_deferred_time = 0,
		.oplus_autoon_cfg = 0,
		.oplus_autooff_cfg = 0,
		.oplus_minfps0_cfg = 1,
		.oplus_minfps1_cfg = 0,
#endif /* OPLUS_FEATURE_DISPLAY_ADFR  */
		.color_vivid_status = true,
		.color_srgb_status = true,
		.color_softiris_status = false,
		.color_dual_panel_status = false,
		.color_dual_brightness_status = true,
		.color_oplus_calibrate_status = true,
		.color_loading_status = false,
		.color_2nit_status = true,
		.vendor = "AC068_P_3_A0001",
		.manufacture = "P_3",
		.cust_esd_check = 0,
		.esd_check_enable = 1,
		.lcm_esd_check_table[0] = {
			.cmd = 0x0A,
			.count = 1,
			.para_list[0] = 0x9C,
			.mask_list[0] = 0xDC,
		},
		.lcm_esd_check_table[1] = {
			.cmd = 0x91,
			.count = 4,
			.para_list[0] = 0xAB,
			.mask_list[0] = 0xAB,
			.para_list[1] = 0xA8,
			.mask_list[1] = 0xA8,
			.para_list[2] = 0x00,
			.mask_list[2] = 0x00,
			.para_list[3] = 0x28,
			.mask_list[3] = 0x28,
		},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
		.round_corner_en = 1,
		.corner_pattern_height = ROUND_CORNER_1K_H_TOP,
		.corner_pattern_height_bot = ROUND_CORNER_1K_H_BOT,
		.corner_pattern_tp_size_l = sizeof(top_rc_1k_pattern_l),
		.corner_pattern_lt_addr_l = (void *)top_rc_1k_pattern_l,
		.corner_pattern_tp_size_r =  sizeof(top_rc_1k_pattern_r),
		.corner_pattern_lt_addr_r = (void *)top_rc_1k_pattern_r,
#endif
		.phy_timcon = {
			.hs_trail = 14,
			.clk_trail = 13,
			.oplus_lpx_config = 50,
		},
		.dyn_fps = {
			.switch_en = 1,
			.vact_timing_fps = 120,
			.apollo_limit_superior_us = 1000,
			.apollo_limit_inferior_us = 7200,
			.apollo_transfer_time_us = 6200,
		},
		.skip_unnecessary_switch = false,
		.oplus_hpwm_config = 1,
		.panel_bpp = DSC_BIT_PER_CHANNEL,
	},
};

static int mtk_panel_ext_param_get(struct drm_panel *panel,
		struct drm_connector *connector,
		struct mtk_panel_params **ext_param,
		unsigned int id)
{
	int ret = 0;
    int mode_id = -1;
	struct drm_display_mode *m = get_mode_by_id(connector, id);

	mode_id = get_mode_enum(m);

	DISP_INFO("mode:%d,mode_id:%d\n",id,mode_id);

	if (mode_id == FHD_SDC60) {
		*ext_param = &ext_params[0];
	} else if (mode_id == FHD_SDC120) {
		*ext_param = &ext_params[1];
	} else {
		*ext_param = &ext_params[0];
	}

	if (*ext_param)
		DISP_DEBUG("data_rate:%d\n", (*ext_param)->data_rate);
	else
		DISP_ERR("ext_param is NULL;\n");

	return ret;
}

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			 struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
    int mode_id = -1;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);

	mode_id = get_mode_enum(m);

	DISP_INFO("mode:%d,mode_id:%d\n",mode,mode_id);

	if (mode_id == FHD_SDC60) {
		ext->params = &ext_params[0];
	} else if (mode_id == FHD_SDC120) {
		ext->params = &ext_params[1];
	} else {
		ext->params = &ext_params[0];
	}

	return ret;
}


static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static int panel_send_pack_hs_cmd(void *dsi, struct LCM_setting_table *table, unsigned int lcm_cmd_count, dcs_write_gce_pack cb, void *handle)
{
	unsigned int i = 0;
	struct mtk_ddic_dsi_cmd send_cmd_to_ddic;

	if(lcm_cmd_count > MAX_TX_CMD_NUM_PACK) {
		DISP_ERR("out of mtk_ddic_dsi_cmd\n");
		return 0;
	}

	for (i = 0; i < lcm_cmd_count; i++) {
		send_cmd_to_ddic.mtk_ddic_cmd_table[i].cmd_num = table[i].count;
		send_cmd_to_ddic.mtk_ddic_cmd_table[i].para_list = table[i].para_list;
	}
	send_cmd_to_ddic.is_hs = 1;
	send_cmd_to_ddic.is_package = 1;
	send_cmd_to_ddic.cmd_count = lcm_cmd_count;
	cb(dsi, handle, &send_cmd_to_ddic);

	return 0;
}

static int oplus_display_panel_set_demura_bl(void *dsi, dcs_write_gce_pack cb, void *handle, int bl_demura_mode)
{
	int index = 12;

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) {
		if (pwm_power_on)
			bl_demura_mode = OPLUS_DEMURA_DBV_MODE1;
		else
			return 0;
	}
	switch (bl_demura_mode) {
		case OPLUS_DEMURA_DBV_MODE0:
			index = sizeof(dsi_demura0_bl) / sizeof(struct LCM_setting_table);
			panel_send_pack_hs_cmd(dsi, dsi_demura0_bl, index, cb, handle);
			break;
		case OPLUS_DEMURA_DBV_MODE1:
			index = sizeof(dsi_demura1_bl) / sizeof(struct LCM_setting_table);
			panel_send_pack_hs_cmd(dsi, dsi_demura1_bl, index, cb, handle);
			break;
		case OPLUS_DEMURA_DBV_MODE2:
			index = sizeof(dsi_demura2_bl) / sizeof(struct LCM_setting_table);
			panel_send_pack_hs_cmd(dsi, dsi_demura2_bl, index, cb, handle);
			break;
		default:
			DISP_ERR("Unsupported mode\n");
			break;
	}
	DISP_INFO("bl_demura_mode=%d, demura_bl_1=%d demura_bl_2=%d\n", bl_demura_mode,demura_setting.demura_switch_dvb1,demura_setting.demura_switch_dvb2);
	return 0;
}

static int oplus_display_panel_set_pwm_plus_bl(void *dsi, dcs_write_gce_pack cb, void *handle, unsigned int bl_lvl)
{
	int index = 4;
	int pulse = 0;

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (bl_lvl == 1) {
		DISP_INFO("enter aod\n");
		return 0;
	} else if (bl_lvl > 4094) {
		bl_lvl = 4094;
	}
	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && bl_lvl > 0) {
		bl_lvl = 2047;
	}
	if (bl_lvl > 1) {
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &bl_lvl);
	}
	if (bl_lvl <= hpwm_bl) {
		pulse = 1;
		index = sizeof(dsi_high_12plus_bl) / sizeof(struct LCM_setting_table);
		dsi_high_12plus_bl[index-1].para_list[1] = bl_lvl >> 8;
		dsi_high_12plus_bl[index-1].para_list[2] = bl_lvl & 0xFF;
		panel_send_pack_hs_cmd(dsi, dsi_high_12plus_bl, index, cb, handle);
		lcdinfo_notify(DRM_PANEL_EVENT_PWM_TURBO, &pulse);
		DISP_INFO("pwm_turbo dsi_high_12plus_bl backlight level=%d, last_backlight=%d\n", bl_lvl, last_backlight);
	} else if (bl_lvl > hpwm_bl) {
		pulse = 0;
		index = sizeof(dsi_low_3plus_bl) / sizeof(struct LCM_setting_table);
		dsi_low_3plus_bl[index-1].para_list[1]  = bl_lvl >> 8;
		dsi_low_3plus_bl[index-1].para_list[2]  = bl_lvl & 0xFF;
		panel_send_pack_hs_cmd(dsi, dsi_low_3plus_bl, index, cb, handle);
		lcdinfo_notify(DRM_PANEL_EVENT_PWM_TURBO, &pulse);
		DISP_INFO("pwm_turbo dsi_low_3plus_bl backlight level=%d, last_backlight=%d\n", bl_lvl, last_backlight);
	}
	esd_brightness = bl_lvl;
	return 0;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	unsigned int mapped_level = 0;
	char bl_tb0[] = {0x51, 0x07, 0xFF};

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (level == 1) {
		DISP_INFO("enter aod\n");
		return 0;
	} else if (level > 4094) {
		level = 4094;
	}

	if (last_backlight == 0 || level == 0) {
		DISP_INFO("[INFO][%s:%d]backlight lvl:%u\n", __func__, __LINE__, level);
	} else {
		DISP_BACKLIGHT("[INFO][%s:%d]backlight lvl:%u\n", __func__, __LINE__, level);
	}

    if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0) {
        level = 2047;
    }
	last_backlight = level;
	mapped_level = level;
	if (mapped_level > 1) {
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);
	}

	bl_tb0[1] = level >> 8;
	bl_tb0[2] = level & 0xFF;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	esd_brightness = level;
	oplus_display_brightness = level;
	return 0;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb,
                void *handle)
{
	char bl_tb0[] = {0x51, 0x03, 0xff};
	int bl_demura_mode = OPLUS_DEMURA_DBV_MODE_MAX;
	int index = 12;
	int i = 0;

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (oplus_display_get_demura_support()) {
		if ((esd_brightness > 1) && (esd_brightness <= demura_setting.demura_switch_dvb1)) {
			bl_demura_mode = OPLUS_DEMURA_DBV_MODE0;
			index = sizeof(dsi_demura0_bl) / sizeof(struct LCM_setting_table);
			for (i = 0; i < index; i++){
				cb(dsi, handle, dsi_demura0_bl[i].para_list, dsi_demura0_bl[i].count);
			}
		} else if ((esd_brightness > demura_setting.demura_switch_dvb1) && (esd_brightness <= demura_setting.demura_switch_dvb2)) {
			bl_demura_mode = OPLUS_DEMURA_DBV_MODE1;
			index = sizeof(dsi_demura1_bl) / sizeof(struct LCM_setting_table);
			for (i = 0; i < index; i++){
				cb(dsi, handle, dsi_demura1_bl[i].para_list, dsi_demura1_bl[i].count);
			}
		} else if (esd_brightness > demura_setting.demura_switch_dvb2) {
			bl_demura_mode = OPLUS_DEMURA_DBV_MODE2;
			index = sizeof(dsi_demura2_bl) / sizeof(struct LCM_setting_table);
			for (i = 0; i < index; i++){
				cb(dsi, handle, dsi_demura2_bl[i].para_list, dsi_demura2_bl[i].count);
			}
		}
		if (bl_demura_mode != OPLUS_DEMURA_DBV_MODE_MAX) {
			pwm_power_on = false;
		}
	}
	bl_tb0[1] = esd_brightness >> 8;
	bl_tb0[2] = esd_brightness & 0xFF;
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	DISP_INFO("esd_brightness=%d ,bl_tb0[1]=%x, bl_tb0[2]=%x, bl_demura_mode=%d\n", esd_brightness,bl_tb0[1], bl_tb0[2], bl_demura_mode);
	return 1;
}

static int panel_set_seed(void *dsi, dcs_write_gce cb, void *handle, unsigned int seed_mode)
{
	int i = 0;

	DISP_INFO("seed_mode=%d\n", seed_mode);

	if (seed_mode == 100) {
		for (i = 0; i < sizeof(lcm_seed_mode0)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_seed_mode0[i].para_list, lcm_seed_mode0[i].count);
		}
	} else if (seed_mode == 101){
		for (i = 0; i < sizeof(lcm_seed_mode1)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_seed_mode1[i].para_list, lcm_seed_mode1[i].count);
		}
	} else if (seed_mode == 102){
		for (i = 0; i < sizeof(lcm_seed_mode2)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_seed_mode2[i].para_list, lcm_seed_mode2[i].count);
		}
	}

	return 0;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	DISP_INFO("reset on=%d\n",on);

	gpiod_set_value(ctx->reset_gpio, on);

	return 0;
}

static int lcm_panel_reset(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->prepared) {
		DISP_DEBUG("ctx->prepared:%d return! \n",ctx->prepared);
		return 0;
	}
	usleep_range(12000,12100);
	gpiod_set_value(ctx->reset_gpio,1);
	usleep_range(2000,2100);
	gpiod_set_value(ctx->reset_gpio,0);
	usleep_range(2000,2100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(25000,25100);
	DISP_INFO("Successful\n");

	return 0;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared){
		return 0;
	}

	if ((pcb_version > 0) && (pcb_version < PCB_VERSION_DVT)) {
		//set vddi 1.8v
		ret = regulator_set_voltage(g_pmic->reg_vufs18,
					1800000, 1800000);
		ret=lcm_enable_reg_vufs18(1);
	} else {
		gpiod_set_value(ctx->vddi1p8_enable_gpio, 1);
	}
	usleep_range(5000, 5100);

	//enable 1.5V gpio165
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 1);
	//enable aod_en gpio 11 to 1.2V
	gpiod_set_value(ctx->aod1p3_enable_gpio, 0);
	usleep_range(1000, 1100);

	//set vddi 3.0v
	ret = regulator_set_voltage(g_pmic->reg_vmch3p0,
				3000000, 3000000);
	ret=lcm_enable_reg_vmch3p0(1);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);
	gpiod_set_value(ctx->sm3011_reset_gpio, 1);
	DISP_INFO("Successful. PCB is %u.\n",pcb_version);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared) {
		return 0;
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(2000, 2100);

	//set vci 3.0v
	ret = regulator_set_voltage(g_pmic->reg_vmch3p0,
				0, 0);
	ret=lcm_enable_reg_vmch3p0(0);
	usleep_range(5000, 5100);
	//disable vddr 1.5V
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 0);
	usleep_range(33000, 33100);
	// gpiod_set_value(ctx->aod1p3_enable_gpio, 0);
	// usleep_range(5000, 5100);
	if ((pcb_version > 0) && (pcb_version < PCB_VERSION_DVT)) {
		//disable 1.8V
		ret = regulator_set_voltage(g_pmic->reg_vufs18,
					0, 0);
		ret=lcm_enable_reg_vufs18(0);
	} else {
		gpiod_set_value(ctx->vddi1p8_enable_gpio, 0);
	}

	hbm_brightness_flag = false;
	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);
	gpiod_set_value(ctx->sm3011_reset_gpio, 0);
	usleep_range(70000, 70100);
	DISP_INFO("Successful. PCB is %u.\n",pcb_version);
	return 0;
}

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
static int panel_set_minfps(void *dsi, struct drm_panel *panel, dcs_write_gce_pack cb, void *handle,
	void *minfps, struct drm_display_mode *m)
{
	struct oplus_minfps *min_fps = (struct oplus_minfps *)minfps;
	unsigned int lcm_cmd_count = 0;
	int mode_id = 0;

	if ((m == NULL) || (minfps == NULL)) {
		ADFR_ERR("panel_set_minfps, mode or minfps is NULL\n");
		return -EINVAL;
	}

	mode_id = get_mode_enum(m);
	ADFR_INFO("minfps_flag:%d,extern_frame:%d,mode_id:%d\n",
		min_fps->minfps_flag, min_fps->extend_frame, mode_id);

	/*update the sdc min fps cmds */
	if (min_fps->minfps_flag == 0) {
		if (mode_id == FHD_SDC60 || mode_id == FHD_SDC120) {
			/* update manual min fps */
			auto_off_minfps_cmd[1].para_list[1] = min_fps->extend_frame;
			lcm_cmd_count = sizeof(auto_off_minfps_cmd) / sizeof(struct LCM_setting_table);
			ADFR_INFO("auto_off_minfps_cmd,extend_frame=%d\n", min_fps->extend_frame);
			panel_send_pack_hs_cmd(dsi, auto_off_minfps_cmd, lcm_cmd_count, cb, handle);
		}
	} else if (min_fps->minfps_flag == 1) {
		if (mode_id == FHD_SDC120) {
			/* update auto min fps */
			auto_on_minfps_cmd[1].para_list[1] = min_fps->extend_frame;
			lcm_cmd_count = sizeof(auto_on_minfps_cmd) / sizeof(struct LCM_setting_table);
			ADFR_INFO("auto_on_minfps_cmd,extend_frame=%d\n", min_fps->extend_frame);
			panel_send_pack_hs_cmd(dsi, auto_on_minfps_cmd, lcm_cmd_count, cb, handle);
		}
	}

	return 0;
}
#endif /* OPLUS_FEATURE_DISPLAY_ADFR  */

static int mode_switch_hs(struct drm_panel *panel, struct drm_connector *connector,
		void *dsi_drv, unsigned int cur_mode, unsigned int dst_mode,
			enum MTK_PANEL_MODE_SWITCH_STAGE stage, dcs_write_gce_pack cb)
{
	int ret = 0;
	int mode_id = -1;
	unsigned int lcm_cmd_count = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, dst_mode);
	struct drm_display_mode *src_m = get_mode_by_id(connector, cur_mode);
	struct lcm *ctx = panel_to_lcm(panel);

	DISP_INFO("lcm cur_mode = %d dst_mode %d\n", cur_mode, dst_mode);
	if (cur_mode == dst_mode)
		return ret;

	if (stage == BEFORE_DSI_POWERDOWN) {
		DISP_INFO("cur_mode = %d dst_mode %d\n", cur_mode, dst_mode);
		mode_id = get_mode_enum(m);
		if (mode_id == FHD_SDC60) {
			if ((lcm_id2 > 0) && (lcm_id2 < LCM_DV_HPWM)) {
				lcm_cmd_count = sizeof(fhd_timing_switch_cmd_sdc60) / sizeof(struct LCM_setting_table);
				panel_send_pack_hs_cmd(dsi_drv, fhd_timing_switch_cmd_sdc60, lcm_cmd_count, cb, NULL);
				DISP_INFO("timing switch to FHD_SDC60\n");
			} else {
				if (oplus_display_get_demura_support()) {
					lcm_cmd_count = sizeof(fhd_timing_switch_demura_pwm_cmd_sdc60) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, fhd_timing_switch_demura_pwm_cmd_sdc60, lcm_cmd_count, cb, NULL);
					DISP_INFO("timing switch to demura_pwm_FHD_SDC60\n");
				} else {
					lcm_cmd_count = sizeof(fhd_timing_switch_high_pwm_cmd_sdc60) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, fhd_timing_switch_high_pwm_cmd_sdc60, lcm_cmd_count, cb, NULL);
					DISP_INFO("timing switch to high_pwm_FHD_SDC60\n");
				}
			}
		} else if (mode_id == FHD_SDC120) {
			if ((lcm_id2 > 0) && (lcm_id2 < LCM_DV_HPWM)) {
				lcm_cmd_count = sizeof(fhd_timing_switch_cmd_sdc120) / sizeof(struct LCM_setting_table);
				panel_send_pack_hs_cmd(dsi_drv, fhd_timing_switch_cmd_sdc120, lcm_cmd_count, cb, NULL);
				DISP_INFO("timing switch to FHD_SDC120\n");
			} else {
				if (oplus_display_get_demura_support()) {
					lcm_cmd_count = sizeof(fhd_timing_switch_demura_pwm_cmd_sdc120) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, fhd_timing_switch_demura_pwm_cmd_sdc120, lcm_cmd_count, cb, NULL);
					DISP_INFO("timing switch to demura_pwm_FHD_SDC120\n");
				} else {
					lcm_cmd_count = sizeof(fhd_timing_switch_high_pwm_cmd_sdc120) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, fhd_timing_switch_high_pwm_cmd_sdc120, lcm_cmd_count, cb, NULL);
					DISP_INFO("timing switch to high_pwm_FHD_SDC120\n");
				}
			}
		} else {
			if ((lcm_id2 > 0) && (lcm_id2 < LCM_DV_HPWM)) {
				lcm_cmd_count = sizeof(fhd_timing_switch_cmd_sdc60) / sizeof(struct LCM_setting_table);
				panel_send_pack_hs_cmd(dsi_drv, fhd_timing_switch_cmd_sdc60, lcm_cmd_count, cb, NULL);
			} else {
				if (oplus_display_get_demura_support()) {
					lcm_cmd_count = sizeof(fhd_timing_switch_demura_pwm_cmd_sdc60) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, fhd_timing_switch_demura_pwm_cmd_sdc60, lcm_cmd_count, cb, NULL);
					DISP_INFO("default timing switch to demura_pwm_FHD_SDC60\n");
				} else {
					lcm_cmd_count = sizeof(fhd_timing_switch_high_pwm_cmd_sdc60) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, fhd_timing_switch_high_pwm_cmd_sdc60, lcm_cmd_count, cb, NULL);
					DISP_INFO("default timing switch to high_pwm_FHD_SDC60\n");
				}
			}
		}

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
		if (oplus_adfr_is_support()) {
			// reset adfr auto mode status as panel mode will be change after timing switch
			oplus_adfr_status_reset(src_m, m);
		}
#endif /* OPLUS_FEATURE_DISPLAY_ADFR  */
	} else if (stage == AFTER_DSI_POWERON) {
		ctx->m = m;
	}
	return ret;
}
static int lcm_get_modes(struct drm_panel *panel,
                    struct drm_connector *connector) {
    struct drm_display_mode *mode[MODE_NUM];
    int i = 0;

    mode[0] = drm_mode_duplicate(connector->dev, &display_mode[0]);
    if (!mode[0]) {
        DISP_ERR("failed to add mode %ux%ux@%u\n",
            display_mode[0].hdisplay, display_mode[0].vdisplay,
             drm_mode_vrefresh(&display_mode[0]));
        return -ENOMEM;
    }

    drm_mode_set_name(mode[0]);
    mode[0]->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
    drm_mode_probed_add(connector, mode[0]);
    DISP_DEBUG("en=%u, clock=%d,htotal=%d,vtotal=%d,hskew=%d,vrefresh=%d\n",mode[0], mode[0]->clock, mode[0]->htotal,
        mode[0]->vtotal, mode[0]->hskew, drm_mode_vrefresh(mode[0]));

    for (i = 1; i < MODE_NUM; i++) {
        mode[i] = drm_mode_duplicate(connector->dev, &display_mode[i]);
        DISP_DEBUG(" en=%u\n",mode[i]);
        if (!mode[i]) {
            DISP_ERR("not enough memory\n");
            return -ENOMEM;
        }

        drm_mode_set_name(mode[i]);
        mode[i]->type = DRM_MODE_TYPE_DRIVER;
        drm_mode_probed_add(connector, mode[i]);
    }
    connector->display_info.width_mm = 68;   //align x3 panel physical w/h
    connector->display_info.height_mm = 159;

    return 1;
}

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
	unsigned int reg_count = 0;
	int vrefresh_rate = 0;
	struct lcm *ctx = NULL;
	struct LCM_setting_table *aod_off_cmd = NULL;

	if (!panel || !dsi) {
		OFP_ERR("Invalid dsi params\n");
	}

	ctx = panel_to_lcm(panel);
	if (!ctx) {
		OFP_ERR("Invalid lcm params\n");
	}

	if (!ctx->m) {
		vrefresh_rate = 120;
		OFP_INFO("default refresh rate is 120hz\n");
	} else {
		vrefresh_rate = drm_mode_vrefresh(ctx->m);
	}

	DISP_ERR("vrefresh_rate=%d\n", vrefresh_rate);
	if (vrefresh_rate == 60) {
		if ((lcm_id2 > 0) && (lcm_id2 < LCM_DV_HPWM)) {
			aod_off_cmd = aod_off_cmd_lpwm_60hz;
			reg_count = sizeof(aod_off_cmd_lpwm_60hz) / sizeof(struct LCM_setting_table);
		} else {
			aod_off_cmd = aod_off_cmd_hpwm_60hz;
			reg_count = sizeof(aod_off_cmd_hpwm_60hz) / sizeof(struct LCM_setting_table);
		}
	} else {
		if ((lcm_id2 > 0) && (lcm_id2 < LCM_DV_HPWM)) {
			aod_off_cmd = aod_off_cmd_lpwm_120hz;
			reg_count = sizeof(aod_off_cmd_lpwm_120hz) / sizeof(struct LCM_setting_table);
		} else {
			aod_off_cmd = aod_off_cmd_hpwm_120hz;
			reg_count = sizeof(aod_off_cmd_hpwm_120hz) / sizeof(struct LCM_setting_table);
		}
	}

	for (i = 0; i < reg_count; i++) {
		unsigned int cmd;
		cmd = aod_off_cmd[i].cmd;

		switch (cmd) {
			case REGFLAG_DELAY:
				if (handle == NULL) {
					usleep_range(aod_off_cmd[i].count * 1000, aod_off_cmd[i].count * 1000 + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(aod_off_cmd[i].count * 1000), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_UDELAY:
				if (handle == NULL) {
					usleep_range(aod_off_cmd[i].count, aod_off_cmd[i].count + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(aod_off_cmd[i].count), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, aod_off_cmd[i].para_list, aod_off_cmd[i].count);
		}
	}

	lcm_setbacklight_cmdq(dsi, cb, handle, last_backlight);
	pwm_power_on = true;
	OFP_INFO("send aod off cmd\n");

	return 0;
}

static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
	unsigned int reg_count = 0;
	struct LCM_setting_table *aod_on_cmd = NULL;

	if ((lcm_id2 > 0) && (lcm_id2 < LCM_DV_HPWM)) {
		aod_on_cmd = aod_on_cmd_lpwm;
		reg_count = sizeof(aod_on_cmd_lpwm) / sizeof(struct LCM_setting_table);
	} else {
		aod_on_cmd = aod_on_cmd_hpwm;
		reg_count = sizeof(aod_on_cmd_hpwm) / sizeof(struct LCM_setting_table);
	}

	for (i = 0; i < reg_count; i++) {
		unsigned int cmd;
		cmd = aod_on_cmd[i].cmd;

		switch (cmd) {
			case REGFLAG_DELAY:
				usleep_range(aod_on_cmd[i].count * 1000, aod_on_cmd[i].count * 1000 + 100);
				break;
			case REGFLAG_UDELAY:
				usleep_range(aod_on_cmd[i].count, aod_on_cmd[i].count + 100);
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, aod_on_cmd[i].para_list, aod_on_cmd[i].count);
		}
	}

	OFP_INFO("send aod on cmd\n");

	return 0;
}

static int panel_set_aod_light_mode(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	int i = 0;

	if (level == 0) {
		for (i = 0; i < sizeof(aod_high_mode)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_high_mode[i].para_list, aod_high_mode[i].count);
		}
	} else {
		for (i = 0; i < sizeof(aod_low_mode)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_low_mode[i].para_list, aod_low_mode[i].count);
		}
	}
	OFP_INFO("level = %d\n", level);

	return 0;
}

static int oplus_lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		DDPPR_ERR("error %ld reading dcs seq:(%#x)\n", ret,
			 cmd);
		ctx->error = ret;
	}

	return ret;
}

static void oplus_lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	ret = oplus_lcm_dcs_read(ctx, 0xAB, buffer, 2);
	DISP_INFO("0xAB return %d data(0x%08x, 0x%08x) to dsi engine\n",ret, buffer[1], buffer[0]);

	ret = oplus_lcm_dcs_read(ctx, 0x0A, buffer, 1);
	DISP_INFO("0x0A return %d data(0x%08x) to dsi engine\n",ret, buffer[0] | (buffer[1] << 8));

	ret = oplus_lcm_dcs_read(ctx, 0x0E, buffer, 1);
	DISP_INFO("return 0x0E %d data(0x%08x) to dsi engine\n",ret, buffer[0] | (buffer[1] << 8));
}

void lcm_read_info(struct drm_panel *panel, int read_ic)
{
	unsigned int ret = 0, volt = 0;
	struct lcm *ctx = panel_to_lcm(panel);

	if ((pcb_version > 0) && (pcb_version < PCB_VERSION_DVT)) {
		/*read the 1.8v*/
		if (!IS_ERR_OR_NULL(g_pmic->reg_vufs18)) {
			volt = lcm_get_reg_vufs18();
			DISP_INFO("get the Regulator vufs1p8 =%d.\n",volt);
		}
	} else {
		if (IS_ERR_OR_NULL(ctx->vddi1p8_enable_gpio))
			DDPPR_ERR("get the enable gpio failed\n");
		else {
			ret = gpiod_get_value(ctx->vddi1p8_enable_gpio);
			DISP_INFO("vddi1p8_enable_gpio = %d\n", ret);
		}
	}

	/*read the 3.3v*/
	if (!IS_ERR_OR_NULL(g_pmic->reg_vmch3p0)) {
		volt = lcm_get_reg_vmch3p0();
		DISP_INFO("get the Regulator vmch3p0 =%d.\n",volt);
	}

	if (IS_ERR_OR_NULL(ctx->aod1p3_enable_gpio))
		DDPPR_ERR("get the enable gpio failed\n");
	else {
		ret = gpiod_get_value(ctx->aod1p3_enable_gpio);
		DISP_INFO("aod1p3_enable_gpio = %d\n", ret);
	}
	DISP_INFO("read_ic = %d\n", read_ic);
	if (read_ic > 0) {
		oplus_lcm_panel_get_data(ctx);
	}

	return;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.panel_poweron = lcm_panel_poweron,
	.panel_reset = lcm_panel_reset,
	.panel_poweroff = lcm_panel_poweroff,
	.ata_check = panel_ata_check,
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
	.mode_switch_hs = mode_switch_hs,
#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	.set_minfps = panel_set_minfps,
#endif /* OPLUS_FEATURE_DISPLAY_ADFR  */
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
	.set_seed = panel_set_seed,
	.doze_disable = panel_doze_disable,
	.doze_enable = panel_doze_enable,
	//.doze_post_disp_on = panel_doze_post_disp_on,
	.set_aod_light_mode = panel_set_aod_light_mode,
	.lcm_high_pwm_set_plus_bl = oplus_display_panel_set_pwm_plus_bl,
	.lcm_demura_set_bl = oplus_display_panel_set_demura_bl,
	.oplus_get_info = lcm_read_info,
};
#endif

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};


static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;
	int rc = 0;
	u32 config = 0;

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);

		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				DISP_ERR("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			DISP_INFO("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		DISP_ERR("skip probe due to not current lcm\n");
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	g_pmic = kzalloc(sizeof(struct lcm_pmic_info), GFP_KERNEL);
	if (!g_pmic) {
		DISP_ERR("fail to alloc lcm_pmic_info (ENOMEM)\n");
		return -ENOMEM;
	}
	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		DISP_ERR("cannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}

	ctx->sm3011_reset_gpio = devm_gpiod_get(dev, "sm3011_reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->sm3011_reset_gpio)) {
		DISP_ERR("cannot get sm3011_reset-gpios %ld\n",
			 PTR_ERR(ctx->sm3011_reset_gpio));
		return PTR_ERR(ctx->sm3011_reset_gpio);
	}

	//enable aod_en 1.2V
	ctx->aod1p3_enable_gpio = devm_gpiod_get(ctx->dev, "aod_en", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->aod1p3_enable_gpio)) {
		DISP_ERR("cannot get aod_en_enable_gpio %ld\n",
			PTR_ERR(ctx->aod1p3_enable_gpio));
		return PTR_ERR(ctx->aod1p3_enable_gpio);
	}

	pcb_version = get_PCB_Version();
	DISP_INFO("get PCB Version = %u\n", pcb_version);
	if ((pcb_version > 0) && (pcb_version < PCB_VERSION_DVT)) {
		g_pmic->reg_vufs18= regulator_get(dev, "1p8_evt");
		if (IS_ERR(g_pmic->reg_vufs18)) {
			DISP_ERR("cannot get reg_vufs18 %ld\n",
				PTR_ERR(g_pmic->reg_vufs18));
		}
		ret = regulator_set_voltage(g_pmic->reg_vufs18,
					1800000, 1800000);
		ret = lcm_enable_reg_vufs18(1);
	} else {
		ctx->vddi1p8_enable_gpio = devm_gpiod_get(ctx->dev, "1p8_dvt", GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->vddi1p8_enable_gpio)) {
			DISP_ERR("cannot get vddi1p8_enable_gpio %ld\n",
				PTR_ERR(ctx->vddi1p8_enable_gpio));
			return PTR_ERR(ctx->vddi1p8_enable_gpio);
		}
		gpiod_set_value(ctx->vddi1p8_enable_gpio, 1);
	}
	usleep_range(5000, 5100);

	ctx->vddr1p5_enable_gpio = devm_gpiod_get(dev, "vddr1p5-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p5_enable_gpio)) {
		DISP_ERR(" cannot get vddr1p5_enable_gpio %ld\n",
			PTR_ERR(ctx->vddr1p5_enable_gpio));
		return PTR_ERR(ctx->vddr1p5_enable_gpio);
	}
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 1);
	gpiod_set_value(ctx->aod1p3_enable_gpio, 0);
	usleep_range(5000, 5100);

	g_pmic->reg_vmch3p0= regulator_get(dev, "3p0");
	if (IS_ERR(g_pmic->reg_vmch3p0)) {
		DISP_ERR("cannot get reg_vmch3p0 %ld\n",
			PTR_ERR(g_pmic->reg_vmch3p0));
	}
	ret = regulator_set_voltage(g_pmic->reg_vmch3p0,
				3000000, 3000000);
	ret = lcm_enable_reg_vmch3p0(1);

	ctx->prepared = true;
	ctx->enabled = true;
	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params[0], &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;

#endif
	dsi_panel_parse_panel_power_cfg(panel_vol_bak);

	rc = of_property_read_u32(dev->of_node, "oplus-adfr-config", &config);
	if (rc == 0) {
		oplus_adfr_config = config;
		DISP_INFO("config=%d, adfrconfig=%d\n", config, oplus_adfr_config);
	} else {
		oplus_adfr_config = 0;
		DISP_INFO("adfrconfig=%d\n", oplus_adfr_config);
	}

	oplus_display_get_demura_cfg(dev);

	register_device_proc("lcd", "AC068_P_3_A0001", "P_3");
	hpwm_bl = 0x58D;
	pwm_power_on = true;
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	flag_silky_panel = BL_SETTING_DELAY_60HZ;
	DISP_INFO("Successful, adfrconfig=%d\n", oplus_adfr_config);
	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif

	return 0;
}

static const struct of_device_id lcm_of_match[] = {
	{
	    .compatible = "AC068_P_3_A0001_fhd_cmd",
	},
	{}
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "AC068_P_3_A0001_fhd_cmd",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

static int __init lcm_drv_init(void)
{
	int ret = 0;

	pr_notice("%s+\n", __func__);
	mtk_panel_lock();
	ret = mipi_dsi_driver_register(&lcm_driver);
	if (ret < 0)
		pr_notice("%s, Failed to register lcm driver: %d\n",
			__func__, ret);

	mtk_panel_unlock();
	pr_notice("%s- ret:%d\n", __func__, ret);
	return 0;
}

static void __exit lcm_drv_exit(void)
{
	pr_notice("%s+\n", __func__);
	mtk_panel_lock();
	mipi_dsi_driver_unregister(&lcm_driver);
	mtk_panel_unlock();
	pr_notice("%s-\n", __func__);
}
module_init(lcm_drv_init);
module_exit(lcm_drv_exit);

MODULE_AUTHOR("liuwenqi");
MODULE_DESCRIPTION("lcm AC068_P_3_A0001 Panel Driver");
MODULE_LICENSE("GPL v2");
