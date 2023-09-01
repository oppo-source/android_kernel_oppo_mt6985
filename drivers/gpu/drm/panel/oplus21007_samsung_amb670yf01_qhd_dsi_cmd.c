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
#include <linux/regulator/consumer.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif


#include "oplus21007_samsung_amb670yf01_qhd_dsi_cmd.h"
#include "oplus_bl.h"
//#ifdef 2
//#include "../oplus/2_ext.h"
//#endif


#define BRIGHTNESS_MAX    4095
#define BRIGHTNESS_HALF   2047
#define MAX_NORMAL_BRIGHTNESS   2047
#define SILKY_MAX_NORMAL_BRIGHTNESS   8191
#define LCM_BRIGHTNESS_TYPE 2

#define FHD_LCM_WIDTH  1080
#define FHD_LCM_HEIGHT 2412

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

/*extern int dsi_panel_parse_panel_power_cfg(struct panel_voltage_bak *panel_vol);
static PANEL_VOLTAGE_BAK panel_vol_bak[PANEL_VOLTAGE_ID_MAX] = {
						{0, 1800000, 1800000, 1800000, "vddi"},
						{1, 1800000, 1104000, 1800000, "vddr"},
						{2, 3000000, 1104000, 3000000, "vci"},
						{3, 0, 1, 2, ""}};
*/
static int esd_brightness;
/* whether enter hbm brightness level or not */
static bool hbm_brightness_flag = false;

//extern void lcdinfo_notify(unsigned long val, void *v);

struct lcm_pmic_info {
	struct regulator *reg_vufs18;
	struct regulator *reg_vmch3p0;
};

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos, *bias_neg;
	struct gpio_desc *bias_gpio;
	struct gpio_desc *vddr1p5_enable_gpio;
	struct gpio_desc *aod1p3_enable_gpio;
    struct gpio_desc *te_switch_gpio, *te_out_gpio;
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
		printk("debug forerror %zd writing seq: %ph\n", ret, data);
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
			msleep(table[i].count);
			break;
		case REGFLAG_UDELAY:
			usleep_range(table[i].count, table[i].count + 1000);
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
		printk("debug forerror %d reading dcs seq:(%#x)\n", ret,
			 cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	printk("debug for%s+\n");

	if (ret == 0) {
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		printk("debug forreturn %d data(0x%08x) to dsi engine\n",ret, buffer[0] | (buffer[1] << 8));
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
		printk("debug forerror %zd writing seq: %ph\n", ret, data);
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
	if(en) {
		if (!IS_ERR_OR_NULL(g_pmic->reg_vufs18)) {
			ret = regulator_enable(g_pmic->reg_vufs18);
			printk("debug forEnable the Regulator vufs1p8ret=%d.\n",ret);
			volt = lcm_get_reg_vufs18();
			printk("debug forget the Regulator vufs1p8 =%d.\n",volt);
		}
	}else {
		if (!IS_ERR_OR_NULL(g_pmic->reg_vufs18)) {
			ret = regulator_disable(g_pmic->reg_vufs18);
			volt = lcm_get_reg_vufs18();
			printk("debug fordisable the Regulator vufs1p8 ret=%d,volt=%d.\n",ret,volt);
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
			printk("debug forEnable the Regulator vmch3p0 ret=%d.\n",ret);
			volt = lcm_get_reg_vmch3p0();
			printk("debug forget the Regulator vmch3p0 =%d.\n",volt);
		}
	}else {
		if (!IS_ERR_OR_NULL(g_pmic->reg_vmch3p0)) {
			ret = regulator_disable(g_pmic->reg_vmch3p0);
			volt = lcm_get_reg_vmch3p0();
			printk("debug fordisable the Regulator vmch3p0 ret=%d,volt=%d.\n",ret,volt);
		}
	}
	return ret;

}
#endif

static void lcm_panel_init(struct lcm *ctx)
{

//reset begin
	usleep_range(3000,3100);
	gpiod_set_value(ctx->reset_gpio,1);
	usleep_range(10000,10100);
	gpiod_set_value(ctx->reset_gpio,0);
	usleep_range(5000,5100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10000,10100);
//reset

		printk("debug forfhd_dsi_on_cmd_sdc60\n");
		push_table(ctx, fhd_dsi_on_cmd_sdc60, sizeof(fhd_dsi_on_cmd_sdc60)/sizeof(struct LCM_setting_table));

}

/*static void lcm_panel_seed(struct lcm *ctx){


	if (seed_mode == 100) {
			push_table(ctx, lcm_seed_mode0, sizeof(lcm_seed_mode0)/sizeof(struct LCM_setting_table));
	} else if (seed_mode == 101){
			push_table(ctx, lcm_seed_mode1, sizeof(lcm_seed_mode1)/sizeof(struct LCM_setting_table));
	} else if (seed_mode == 102){
			push_table(ctx, lcm_seed_mode2, sizeof(lcm_seed_mode2)/sizeof(struct LCM_setting_table));
	}
	printk("debug forseed_mode=%d successful!\n", seed_mode);

}*/

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


	if (!ctx->prepared) {
		return 0;
	}

	lcm_dcs_write_seq_static(ctx, 0x28);
	usleep_range(20000, 20100);
	lcm_dcs_write_seq_static(ctx, 0x10);
	usleep_range(110000, 110100);
	ctx->error = 0;
	ctx->prepared = false;

	//poweroff
	{
			struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared) {
		return 0;
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(2000, 2100);

	ret = regulator_set_voltage(g_pmic->reg_vufs18,
				0, 0);
	//disable vddi 1.5V
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 0);
	usleep_range(2000, 2100);
	//gpiod_set_value(ctx->aod1p3_enable_gpio, 0);
	usleep_range(5000, 5100);
	//disable 1.8V
	ret=lcm_enable_reg_vufs18(0);

	usleep_range(5000, 5100);
	//set vddi 3.0v
	ret = regulator_set_voltage(g_pmic->reg_vmch3p0,
				0, 0);
	ret=lcm_enable_reg_vmch3p0(0);

	hbm_brightness_flag = false;
	}
	//poweroff end

	printk("debug lcm_unprepare for Successful\n");

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared) {
		return 0;
	}

//poweron
	//set vddi 1.8v
	ret = regulator_set_voltage(g_pmic->reg_vufs18,
				1800000, 1800000);
	ret=lcm_enable_reg_vufs18(1);

	usleep_range(5000, 5100);
	//enable aod_en gpio 3 to 1.3V
	//gpiod_set_value(ctx->aod1p3_enable_gpio, 0);
	//enable 1.5V gpio148
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 1);
	usleep_range(1000, 1100);

	//set vddi 3.0v
	ret = regulator_set_voltage(g_pmic->reg_vmch3p0,
				3000000, 3000000);
	ret=lcm_enable_reg_vmch3p0(1);
//poweron end

	lcm_panel_init(ctx);
	usleep_range(2000, 2100);
	//lcm_panel_seed(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	printk("debug forSuccessful\n");
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

#define HFP (40)
#define HSA (10)
#define HBP (20)
#define VFP (8)
#define VSA (8)
#define VBP (8)
//#define VAC_WQHD (3216)
//#define HAC_WQHD (1440)

#define VAC_FHD (2412)
#define HAC_FHD (1080)

//static u32 fake_heigh = 3216;
//static u32 fake_width = 1440;
//static bool need_fake_resolution;
static const struct drm_display_mode display_mode[MODE_NUM] = {
	//fhd_sdc_60_mode
	{
		.clock = 168084,
		.hdisplay = HAC_FHD,
		.hsync_start = HAC_FHD + HFP,
		.hsync_end = HAC_FHD + HFP + HSA,
		.htotal = HAC_FHD + HFP + HSA + HBP,
		.vdisplay = VAC_FHD,
		.vsync_start = VAC_FHD + VFP,
		.vsync_end = VAC_FHD + VFP + VSA,
		.vtotal = VAC_FHD + VFP + VSA + VBP,
		.hskew = 1,
	},
};

/*static const struct drm_display_mode default_mode = {
	.clock = 307152,
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,
	.hsync_end = HAC + HFP + HSA,
	.htotal = HAC + HFP + HSA + HBP,
	.vdisplay = VAC,
	.vsync_start = VAC + VFP,
	.vsync_end = VAC + VFP + VSA,
	.vtotal = VAC + VFP + VSA + VBP,
};*/

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params[MODE_NUM] = {
	//fhd_sdc_60_mode
		{
			.data_rate = 826,
			.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,

			.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
			.dsc_params = {
				.enable = 1,
				.ver = 18,
				.slice_mode = 1,
				.rgb_swap = 0,
				.dsc_cfg = 2088,
				.rct_on = 1,
				.bit_per_channel = 10,
				.dsc_line_buf_depth = 11,
				.bp_enable = 1,
				.bit_per_pixel = 128,
				.pic_height = 2412,
				.pic_width = 1080,
				.slice_height = 36,
				.slice_width = 540,
				.chunk_size = 540,
				.xmit_delay = 512,
				.dec_delay = 571,
				.scale_value = 32,
				.increment_interval = 821,
				.decrement_interval = 7,
				.line_bpg_offset = 14,
				.nfl_bpg_offset = 820,
				.slice_bpg_offset = 724,
				.initial_offset = 6144,
				.final_offset = 4336,
				.flatness_minqp = 7,
				.flatness_maxqp = 16,
				.rc_model_size = 8192,
				.rc_edge_factor = 6,
				.rc_quant_incr_limit0 = 15,
				.rc_quant_incr_limit1 = 15,
				.rc_tgt_offset_hi = 3,
				.rc_tgt_offset_lo = 3,
			},

			.dyn_fps = {
				.switch_en = 0, .vact_timing_fps = 60,
			},

			/*.color_vivid_status = true,
			.color_srgb_status = true,
			.color_softiris_status = true,
			.color_dual_panel_status = false,
			.color_dual_brightness_status = true,*/
			.cust_esd_check = 0,
			.esd_check_enable = 0,
			.lcm_esd_check_table[0] = {
				.cmd = 0x0a,
				.count = 1,
				.para_list[0] = 0x9F,
				.para_list[1] = 0x9D,
			},

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
			.round_corner_en = 0,
			.corner_pattern_height = ROUND_CORNER_1K_H_TOP,
			.corner_pattern_height_bot = ROUND_CORNER_1K_H_BOT,
			.corner_pattern_tp_size_l = sizeof(top_rc_1k_pattern_l),
			.corner_pattern_lt_addr_l = (void *)top_rc_1k_pattern_l,
			.corner_pattern_tp_size_r =  sizeof(top_rc_1k_pattern_r),
			.corner_pattern_lt_addr_r = (void *)top_rc_1k_pattern_r,
#endif
		.phy_timcon = {
			.clk_trail = 14,
			.hs_trail = 11,
		},
	},
};

/*static struct drm_display_mode *get_mode_by_id(struct drm_connector *connector,
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

static int mtk_panel_ext_param_get(struct drm_panel *panel,
		struct drm_connector *connector,
		struct mtk_panel_params **ext_param,
		unsigned int id)
{
	int ret = 0;
	int m_vrefresh = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, id);

	m_vrefresh = drm_mode_vrefresh(m);
	printk("debug formode:%d,m_vrefresh:%d\n",id,m_vrefresh);

	if (m->hdisplay == FHD_LCM_WIDTH && m->vdisplay == FHD_LCM_HEIGHT) {
		if (m_vrefresh == 60 && m->hskew == 1) {
			*ext_param = &ext_params[0];
		}
		else if (m_vrefresh == 120 && m->hskew == 0) {
			*ext_param = &ext_params[1];
		} else if (m_vrefresh == 90 && m->hskew == 0) {
			*ext_param = &ext_params[2];
		} else if (m_vrefresh == 118 && m->hskew == 2) {
			*ext_param = &ext_params[3];
		} else {
			*ext_param = &ext_params[0];
		}
	} else if (m->hdisplay == WQHD_LCM_WIDTH && m->vdisplay == WQHD_LCM_HEIGHT) {
		if (m_vrefresh == 60 && m->hskew == 1) {
			*ext_param = &ext_params[4];
		}
		else if (m_vrefresh == 120 && m->hskew == 0) {
			*ext_param = &ext_params[5];
		} else if (m_vrefresh == 90 && m->hskew == 0) {
			*ext_param = &ext_params[6];
		} else if (m_vrefresh == 118 && m->hskew == 2) {
			*ext_param = &ext_params[7];
		} else {
			*ext_param = &ext_params[4];
		}
	}

	if (*ext_param)
		printk("debug fordata_rate:%d\n", (*ext_param)->data_rate);
	else
		printk("debug forext_param is NULL;\n");

	return ret;
}

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			 struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	int m_vrefresh = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);

	m_vrefresh = drm_mode_vrefresh(m);

	printk("debug formode:%d,m_vrefresh:%d\n",mode,m_vrefresh);

	if (m->hdisplay == FHD_LCM_WIDTH && m->vdisplay == FHD_LCM_HEIGHT) {
		if (m_vrefresh == 60 && m->hskew == 1) {
			ext->params = &ext_params[0];
		}
		else if (m_vrefresh == 120 && m->hskew == 0) {
			ext->params = &ext_params[1];
		} else if (m_vrefresh == 90 && m->hskew == 0) {
			ext->params = &ext_params[2];
		} else if (m_vrefresh == 118 && m->hskew == 2) {
			ext->params = &ext_params[3];
		} else {
			ext->params = &ext_params[0];
		}
	} else if (m->hdisplay == WQHD_LCM_WIDTH && m->vdisplay == WQHD_LCM_HEIGHT) {
		if (m_vrefresh == 60 && m->hskew == 1) {
			ext->params = &ext_params[4];
		}
		else if (m_vrefresh == 120 && m->hskew == 0) {
			ext->params = &ext_params[5];
		} else if (m_vrefresh == 90 && m->hskew == 0) {
			ext->params = &ext_params[6];
		} else if (m_vrefresh == 118 && m->hskew == 2) {
			ext->params = &ext_params[7];
		} else {
			ext->params = &ext_params[4];
		}
	}
	return ret;
}*/

//#ifdef 2
/*static int panel_send_pack_hs_cmd(void *dsi, struct LCM_setting_table *table, unsigned int lcm_cmd_count, dcs_write_gce_pack cb, void *handle)
{
	unsigned int i = 0;
	struct mtk_ddic_dsi_cmd send_cmd_to_ddic;

	if(lcm_cmd_count > MAX_TX_CMD_NUM_PACK) {
		printk("debug forout of mtk_ddic_dsi_cmd \n");
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
}*/








/*static int panel_set_seed(void *dsi, dcs_write_gce cb, void *handle, unsigned int seed_mode)
{
	int i = 0;

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
	printk("debug forseed_mode=%d successful \n", seed_mode);

	return 0;
}*/
static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
	unsigned int level)
{
	unsigned int mapped_level = 0;
	//unsigned int notify_level = 0;
	char bl_tb0[] = {0x51, 0x03, 0xFF};
	char bl_tb1[] = {0x53, 0x28};

	if (!cb)
		return -EINVAL;

	mapped_level = level;
	esd_brightness = mapped_level;

	/*if (mapped_level > 1) {
		notify_level = mapped_level * 2047 / MAX_NORMAL_BRIGHTNESS;
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &notify_level);
	}*/

	if(mapped_level <= PANEL_MAX_NOMAL_BRIGHTNESS) { //2047
		if(hbm_brightness_flag == true) {
			bl_tb1[1] = 0x28;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			hbm_brightness_flag = false;
		}
		mapped_level = backlight_buf[mapped_level];
	} else if(mapped_level > HBM_BASE_600NIT) { //2749
		if(hbm_brightness_flag == false) {
			bl_tb1[1] = 0xE0;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			hbm_brightness_flag = true;
		}
		mapped_level = backlight_600_800nit_buf[mapped_level - HBM_BASE_600NIT];
	}else if (mapped_level > PANEL_MAX_NOMAL_BRIGHTNESS) { //2047
		if(hbm_brightness_flag == true) {
			bl_tb1[1] = 0x28;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			hbm_brightness_flag = false;
		}
		mapped_level = backlight_500_600nit_buf[mapped_level - PANEL_MAX_NOMAL_BRIGHTNESS];
	}

	bl_tb0[1] = mapped_level >> 8;
	bl_tb0[2] = mapped_level & 0xFF;

	printk("debug forlevel:%d, backlight = %d,bl_tb0[1]=0x%x,bl_tb0[2]=0x%x\n", level, mapped_level,bl_tb0[1],bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 0;
}


static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	printk("debug foron=%d\n",on);

	gpiod_set_value(ctx->reset_gpio, on);


	return 0;
}

/*static int lcm_panel_reset(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->prepared) {
		printk("debug forctx->prepared:%d return! \n",ctx->prepared);
		return 0;
	}
	usleep_range(3000,3100);
	gpiod_set_value(ctx->reset_gpio,1);
	usleep_range(10000,10100);
	gpiod_set_value(ctx->reset_gpio,0);
	usleep_range(5000,5100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10000,10100);
	printk("debug forSuccessful\n");

	return 0;
}*/

/*static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared){
		return 0;
	}
	//set vddi 1.8v
	ret = regulator_set_voltage(g_pmic->reg_vufs18,
				1800000, 1800000);
	ret=lcm_enable_reg_vufs18(1);

	usleep_range(5000, 5100);
	//enable aod_en gpio 3 to 1.3V
	gpiod_set_value(ctx->aod1p3_enable_gpio, 0);
	//enable 1.5V gpio148
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 1);
	usleep_range(1000, 1100);

	//set vddi 3.0v
	ret = regulator_set_voltage(g_pmic->reg_vmch3p0,
				3000000, 3000000);
	ret=lcm_enable_reg_vmch3p0(1);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	printk("debug forSuccessful\n");
	return 0;
}*/

/*static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared) {
		return 0;
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(2000, 2100);

	ret = regulator_set_voltage(g_pmic->reg_vufs18,
				0, 0);
	//disable vddi 1.5V
	gpiod_set_value(ctx->vddr1p5_enable_gpio, 0);
	usleep_range(2000, 2100);
	gpiod_set_value(ctx->aod1p3_enable_gpio, 0);
	usleep_range(5000, 5100);
	//disable 1.8V
	ret=lcm_enable_reg_vufs18(0);

	usleep_range(5000, 5100);
	//set vddi 3.0v
	ret = regulator_set_voltage(g_pmic->reg_vmch3p0,
				0, 0);
	ret=lcm_enable_reg_vmch3p0(0);

	hbm_brightness_flag = false;
	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	usleep_range(70000, 70100);
	printk("debug forSuccessful\n");
	return 0;
}*/

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	//.panel_poweron = lcm_panel_poweron,
	//.panel_reset = lcm_panel_reset,
	//.panel_poweroff = lcm_panel_poweroff,
	.ata_check = panel_ata_check,
};
#endif

static int lcm_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode[MODE_NUM];
	int i = 0;

	mode[0] = drm_mode_duplicate(connector->dev, &display_mode[0]);
	if (!mode[0]) {
		printk("debug forfailed to add mode %ux%ux@%u\n",
			display_mode[0].hdisplay, display_mode[0].vdisplay,
			 drm_mode_vrefresh(&display_mode[0]));
		return -ENOMEM;
	}

	drm_mode_set_name(mode[0]);
	mode[0]->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode[0]);
	printk("debug foren=%u, clock=%d,htotal=%d,vtotal=%d,hskew=%d,vrefresh=%d\n",mode[0], mode[0]->clock, mode[0]->htotal,
		mode[0]->vtotal, mode[0]->hskew, drm_mode_vrefresh(mode[0]));

	for (i = 1; i < MODE_NUM; i++) {
		mode[i] = drm_mode_duplicate(connector->dev, &display_mode[i]);
		printk(" en=%u\n",mode[i]);
		if (!mode[i]) {
			printk("not enough memory\n");
			return -ENOMEM;
		}

		drm_mode_set_name(mode[i]);
		mode[i]->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode[i]);
	}
	connector->display_info.width_mm = 70;   //align x3 panel physical w/h
	connector->display_info.height_mm = 156;

	return 1;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

extern bool g_is_silky_panel;

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;
	//int rc = 0;
	//u32 config = 0;

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);

		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				printk("debug forNo panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			printk("debug fordevice node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		printk("debug forskip probe due to not current lcm\n");
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	g_pmic = kzalloc(sizeof(struct lcm_pmic_info), GFP_KERNEL);
	if (!g_pmic) {
		printk("debug forfail to alloc lcm_pmic_info (ENOMEM)\n");
		return -ENOMEM;
	}
	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_LPM |  MIPI_DSI_MODE_NO_EOT_PACKET
			 | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		printk("debug forcannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	/*devm_gpiod_put(dev, ctx->reset_gpio);*/

	ctx->te_switch_gpio = devm_gpiod_get(dev, "te_switch", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->te_switch_gpio)) {
		dev_info(dev, "%s: cannot get te_switch_gpio %ld\n",
			__func__, PTR_ERR(ctx->te_switch_gpio));
		return PTR_ERR(ctx->te_switch_gpio);
	}
	gpiod_set_value(ctx->te_switch_gpio, 1);
	//devm_gpiod_put(dev, ctx->te_switch_gpio);


	ctx->te_out_gpio = devm_gpiod_get(dev, "te_out", GPIOD_IN);
	if (IS_ERR(ctx->te_out_gpio)) {
		dev_info(dev, "%s: cannot get te_out_gpio %ld\n",
			__func__, PTR_ERR(ctx->te_out_gpio));
		//return PTR_ERR(ctx->te_out_gpio);
	}
    //devm_gpiod_put(dev, ctx->te_out_gpio);

	//enable aod_en 1.3V
	ctx->aod1p3_enable_gpio = devm_gpiod_get(ctx->dev, "aod_en", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->aod1p3_enable_gpio)) {
		printk("debug cannot get aod_en_enable_gpio %ld\n",
			PTR_ERR(ctx->aod1p3_enable_gpio));
		//return PTR_ERR(ctx->aod1p3_enable_gpio);
	}

	//gpiod_set_value(ctx->aod1p3_enable_gpio, 0);
	//devm_gpiod_put(ctx->dev, ctx->aod1p3_enable_gpio);

	ctx->vddr1p5_enable_gpio = devm_gpiod_get(dev, "vddr1p5-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p5_enable_gpio)) {
		printk("debug for cannot get vddr1p5_enable_gpio %ld\n",
			PTR_ERR(ctx->vddr1p5_enable_gpio));
		//return PTR_ERR(ctx->vddr1p5_enable_gpio);
	}
    //gpiod_set_value(ctx->vddr1p5_enable_gpio, 1);
	//devm_gpiod_put(dev, ctx->vddr1p5_enable_gpio);

	g_pmic->reg_vufs18= regulator_get(dev, "1p8");
	if (IS_ERR(g_pmic->reg_vufs18)) {
		printk("debug forcannot get reg_vufs18 %ld\n",
			PTR_ERR(g_pmic->reg_vufs18));
		//return PTR_ERR(g_pmic->reg_vufs18);
	}
	ret=lcm_enable_reg_vufs18(1);

	g_pmic->reg_vmch3p0= regulator_get(dev, "3p0");
	if (IS_ERR(g_pmic->reg_vmch3p0)) {
		printk("debug forcannot get reg_vmch3p0 %ld\n",
			PTR_ERR(g_pmic->reg_vmch3p0));
		//return PTR_ERR(g_pmic->reg_vmch3p0);
	}
	msleep(10);
	ret=lcm_enable_reg_vmch3p0(1);

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
	//dsi_panel_parse_panel_power_cfg(panel_vol_bak);


	printk("debug for Successful\n");

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
	    .compatible = "oplus21007_samsung_amb670yf01_qhd_dsi_cmd",
	},
	{}
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "oplus21007_samsung_amb670yf01_qhd_dsi_cmd",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("lianghao <lianghao1@oppo.com>");
MODULE_DESCRIPTION("lcm amb670 yf01 Panel Driver");
MODULE_LICENSE("GPL v2");
