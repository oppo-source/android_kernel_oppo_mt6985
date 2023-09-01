// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 imx581mipiraw_Sensor_22023.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include "imx581mipiraw_Sensor_22023.h"

#define IMX581_EEPROM_READ_ID_22023	0xA0
#define IMX581_EEPROM_WRITE_ID_22023	0xA1
#define IMX581_MAX_OFFSET_22023		0x8000
#define OPLUS_CAMERA_COMMON_DATA_LENGTH 40
#define PFX "imx581_camera_sensor_22023"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)
#define OTP_SIZE    0x8000
#define OTP_QSC_VALID_ADDR 0x2200
#define IMX581_UNIQUE_SENSOR_ID_22023 0x0A1F
#define IMX581_UNIQUE_SENSOR_ID_LENGHT_22023 11

static kal_uint8 otp_data_checksum[OTP_SIZE] = {0};
static void set_sensor_cali(void *arg);
static int get_sensor_temperature(void *arg);
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static void imx581_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void imx581_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void imx581_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void imx581_get_sensor_sn(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void imx581_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void imx581_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void imx581_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void imx581_get_unique_sensorid(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void imx581_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void imx581_get_cust_pixel_rate(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);
static void imx581_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void imx581_esd_reset_by_user(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void get_sensor_cali(void* arg);
extern unsigned int get_PCB_Version(void);
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size);

enum PCB_VERSION{
    PCB_UNKNOWN,
    PCB_VERSION_EVB1,
    PCB_VERSION_T0,
    PCB_VERSION_T1,
    PCB_VERSION_EVT1,
    PCB_VERSION_EVT2,
    PCB_VERSION_EVT3,
    PCB_VERSION_DVT1,
    PCB_VERSION_DVT2,
    PCB_VERSION_DVT3,
    PCB_VERSION_PVT1,
    PCB_VERSION_MP1,
    PCB_VERSION_MP2,
    PCB_VERSION_MP3,
    PCB_VERSION_MAX,
};

/* STRUCT */

static BYTE imx581_common_data[OPLUS_CAMERA_COMMON_DATA_LENGTH] = { 0 };
static BYTE imx581_unique_id[IMX581_UNIQUE_SENSOR_ID_LENGHT_22023] = { 0 };

/* Normal(Qbin) to Normal(Qbin) */
/* Normal(Qbin) to 2DOL(Qbin) */
static void comp_mode_tran_time_cal1(struct subdrv_ctx *ctx, u32 pre_scenario_id, u32* prsh);
typedef void (*cal_comp_mode_tran_time)(struct subdrv_ctx *ctx, u32 pre_scenario_id, u32* prsh);
struct comp_mode_tran_time_params {
	u8 enable;
	u32 clock_vtpxck;
	cal_comp_mode_tran_time cal_fn;
};
static struct comp_mode_tran_time_params imx581_comp_params[SENSOR_SCENARIO_ID_MAX] = {
	{ .enable = 0, }, /*pre*/
	{ .enable = 1, .clock_vtpxck = 1884, .cal_fn = comp_mode_tran_time_cal1, }, /*cap*/
	{ .enable = 0, }, /*vid*/
	{ .enable = 0, }, /*hvid*/
	{ .enable = 0, }, /*svid*/
	{ .enable = 0, }, /*cus1*/
	{ .enable = 0, }, /*cus2*/
	{ .enable = 0, }, /*csu3*/
	{ .enable = 0, }, /*cus4*/
	{ .enable = 0, }, /*cus5*/
	{ .enable = 0, }, /*cus6*/
	{ .enable = 1, .clock_vtpxck = 1404, .cal_fn = comp_mode_tran_time_cal1, }, /*cus7*/
	{ .enable = 0, }, /*cus8*/
	{ .enable = 0, }, /*cus9*/
	{ .enable = 0, }, /*cus10*/
	{ .enable = 0, }, /*cus11*/
	{ .enable = 0, }, /*cus12*/
	{ .enable = 0, }, /*cus13*/
};

static struct eeprom_map_info imx581_eeprom_info[] = {
	{ EEPROM_META_MODULE_ID, 0x0000, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_SENSOR_ID, 0x0006, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_LENS_ID, 0x0008,0x0010, 0x0011, 2, true },
	{ EEPROM_META_VCM_ID, 0x000A, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_MIRROR_FLIP, 0x000E, 0x0010, 0x0011, 1, true },
	{ EEPROM_META_MODULE_SN, 0x00B0, 0x00C1, 0x00C2, 17, true },
	{ EEPROM_META_AF_CODE, 0x0092, 0x0098, 0x0099, 6, true },
	{ EEPROM_META_AF_FLAG, 0x0098, 0x0098, 0x0099, 1, true },
	{ EEPROM_META_STEREO_DATA, 0x3150, 0x37E9, 0x37EA, CALI_DATA_SLAVE_LENGTH, false },
	{ EEPROM_META_STEREO_MW_MAIN_DATA, 0x2B00, 0x3199, 0x319A, CALI_DATA_MASTER_LENGTH, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA, 0x31C0, 0x3859, 0x385A, CALI_DATA_MASTER_LENGTH, false },
};

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, imx581_set_test_pattern},
	{SENSOR_FEATURE_SEAMLESS_SWITCH, imx581_seamless_switch},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, imx581_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, imx581_get_sensor_sn},
	{SENSOR_FEATURE_SET_SENSOR_OTP, imx581_set_eeprom_calibration},
	{SENSOR_FEATURE_GET_EEPROM_STEREODATA, imx581_get_eeprom_calibration},
	{SENSOR_FEATURE_GET_SENSOR_OTP_ALL, imx581_get_otp_checksum_data},
	{SENSOR_FEATURE_GET_UNIQUE_SENSORID, imx581_get_unique_sensorid},
	{SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO, imx581_get_min_shutter_by_scenario_adapter},
	{SENSOR_FEATURE_GET_CUST_PIXEL_RATE, imx581_get_cust_pixel_rate},
	{SENSOR_FEATURE_SET_AWB_GAIN, imx581_set_awb_gain},
	{SENSOR_FEATURE_ESD_RESET_BY_USER, imx581_esd_reset_by_user},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x0158000A,
		.addr_header_id = 0x00000006,
		.i2c_write_id = 0xA0,

		.lrc_support = TRUE,
		.lrc_size = 384,
		.addr_lrc = 0x2220,
		.sensor_reg_addr_lrc = 0xC800, // useless

		.qsc_support = TRUE,
		.qsc_size = 2304,
		.addr_qsc = 0x1900,//QSC_EEPROM_ADDR
		.sensor_reg_addr_qsc = 0x7F00,//QSC_OTP_ADDR
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX =  17,
	.i4OffsetY =  12,
	.i4PitchX  =  8,
	.i4PitchY  =  16,
	.i4PairNum  = 8,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 2,
	.i4PosL = {{20, 13}, {18, 15}, {22, 17}, {24, 19},
				{20, 21}, {18, 23}, {22, 25}, {24, 27} },
	.i4PosR = {{19, 13}, {17, 15}, {21, 17}, {23, 19},
				{19, 21}, {17, 23}, {21, 25}, {23, 27} },
	.i4BlockNumX = 496, // Tail_X / (i4PitchX / i4SubBlkW)
	.i4BlockNumY = 186, // Tail_Y / 2 / (i4PitchY / i4SubBlkH)
	.iMirrorFlip = 3,
	// i4Crop = (fullRaw - imgSz) / 2 / Bin
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 372}, {0, 372}, {0, 0},
		{368, 276}, {0, 0}, {0, 0}, {0, 0}
	},
	.i4FullRawW = 4000,
	.i4FullRawH = 3000,
	.i4VCPackNum = 1,
	//.i4ModeIndex = 0,
	.sPDMapInfo[0] = {
		.i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 2,
		.i4PDRepetition = 4,
		.i4PDOrder = {1},
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_video = {
	.i4OffsetX =  17,
	.i4OffsetY =  12,
	.i4PitchX  =  8,
	.i4PitchY  =  16,
	.i4PairNum  = 8,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 2,
	.i4PosL = {{20, 13}, {18, 15}, {22, 17}, {24, 19},
				{20, 21}, {18, 23}, {22, 25}, {24, 27} },
	.i4PosR = {{19, 13}, {17, 15}, {21, 17}, {23, 19},
				{19, 21}, {17, 23}, {21, 25}, {23, 27} },
	.i4BlockNumX = 496,
	.i4BlockNumY = 140,
	.iMirrorFlip = 3,
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 372}, {0, 372}, {0, 0},
		{0, 0}, {0, 0}, {0, 372}, {0, 0}
	},
	.i4FullRawW = 4000, // out_size = 4080x2296
	.i4FullRawH = 3000,
	.i4VCPackNum = 1,
	//.i4ModeIndex = 0,
	.sPDMapInfo[0] = {
		.i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 2,
		.i4PDRepetition = 4,
		.i4PDOrder = {1},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4000,
			.vsize = 3000,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 992,
			.vsize = 1488,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4000, //0xff0
			.vsize = 3000, //0xc00
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 992,
			.vsize = 1488,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4000,
			.vsize = 2256,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 992,
			.vsize = 1120,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4000,
			.vsize = 2256,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 992,
			.vsize = 1120,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2000,
			.vsize = 1128,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 8000,
			.vsize = 6000,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2c,
			.hsize = 4000, //0xff0
			.vsize = 2256, //0xc00
			.user_data_desc = VC_RAW_PROCESSED_DATA,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4000,
			.vsize = 2256,
			.user_data_desc = VC_RAW_DATA,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x12,
			.hsize = 1024,	// 1024 pixels data width (embedded data by byte)
			.vsize = 568,	// 568 data height
			.user_data_desc = VC_GENERAL_EMBEDDED,  // 3a_meta
		},
	},
	{
		.bus.csi2 = {
			.channel = 2,
			.data_type = 0x2e,
			.hsize = 992,
			.vsize = 1120,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2c,
			.hsize = 4000, //0xff0
			.vsize = 2256, //0xc00
			.user_data_desc = VC_RAW_PROCESSED_DATA,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x12,
			.hsize = 1024,	// 1024 pixels data width (embedded data by byte)
			.vsize = 568,	// 568 data height
			.user_data_desc = VC_GENERAL_EMBEDDED,  // 3a_meta
		},
	},
	{
		.bus.csi2 = {
			.channel = 2,
			.data_type = 0x2e,
			.hsize = 992,
			.vsize = 1120,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600, //0xff0
			.vsize = 1200, //0xc00
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2560, //0xff0
			.vsize = 1440, //0xc00
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {
	{/* Reg_M_DX1_4000x3000 @30fps -> sensor mode0*/
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = imx581_preview_setting,
		.mode_setting_len = ARRAY_SIZE(imx581_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 739200000,
		.linelength = 7872,
		.framelength = 3130,
		.max_framerate = 300,
		.mipi_pixel_rate = 489600000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8000,
			.full_h = 6000,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8000,
			.h0_size = 6000,
			.scale_w = 4000,
			.scale_h = 3000,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4000,
			.h1_size = 3000,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4000,
			.h2_tg_size = 3000,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{/* Reg_M_DX1_4000x3000 @30fps -> sensor mode1*/
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = imx581_capture_setting,
		.mode_setting_len = ARRAY_SIZE(imx581_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 739200000,
		.linelength = 7872,
		.framelength = 3130,
		.max_framerate = 300,
		.mipi_pixel_rate = 489600000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8000,
			.full_h = 6000,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8000,
			.h0_size = 6000,
			.scale_w = 4000,
			.scale_h = 3000,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4000,
			.h1_size = 3000,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4000,
			.h2_tg_size = 3000,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{/* Reg_M_DX2_4000*2256@30fps -> sensor mode2*/
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = imx581_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(imx581_normal_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 559200000,
		.linelength = 7872,
		.framelength = 2366,
		.max_framerate = 300,
		.mipi_pixel_rate = 364680000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8000,
			.full_h = 6000,
			.x0_offset = 0,
			.y0_offset = 744,
			.w0_size = 8000,
			.h0_size = 4512,
			.scale_w = 4000,
			.scale_h = 2256,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4000,
			.h1_size = 2256,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4000,
			.h2_tg_size = 2256,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_video,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{/* reg_M_DX4 4000x2256 @60fps  -> sensor mode3*/
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = imx581_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(imx581_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 1128000000,
		.linelength = 7872,
		.framelength = 2388,
		.max_framerate = 600,
		.mipi_pixel_rate = 760460000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8000,
			.full_h = 6000,
			.x0_offset = 0,
			.y0_offset = 744,
			.w0_size = 8000,
			.h0_size = 4512,
			.scale_w = 4000,
			.scale_h = 2256,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4000,
			.h1_size = 2256,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4000,
			.h2_tg_size = 2256,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_video,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{/* reg_M_DX6 2000x1128 @120fps -> sensor mode4*/
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = imx581_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(imx581_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 782400000,
		.linelength = 5376,
		.framelength = 1212,
		.max_framerate = 1200,
		.mipi_pixel_rate = 354420000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8000,
			.full_h = 6000,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8000,
			.h0_size = 6000,
			.scale_w = 2000,
			.scale_h = 1500,
			.x1_offset = 0,
			.y1_offset = 186,
			.w1_size = 2000,
			.h1_size = 1128,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2000,
			.h2_tg_size = 1128,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{/* reg_M_DX8 8000x6000 @15fps -> sensor mode5*/
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = imx581_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(imx581_custom1_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 883200000,
		.linelength = 9440,
		.framelength = 6237,
		.max_framerate = 150,
		.mipi_pixel_rate = 824910000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8000,
			.full_h = 6000,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8000,
			.h0_size = 6000,
			.scale_w = 8000,
			.scale_h = 6000,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 8000,
			.h1_size = 6000,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 8000,
			.h2_tg_size = 6000,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.coarse_integ_step = 1,
		.min_exposure_line = 5,
		.ana_gain_max = BASEGAIN * 16,
	},
	{/* Reg_M_DX2_4000*2256@30fps -> preisp for pure raw dump*/
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = imx581_custom2_setting,
		.mode_setting_len = ARRAY_SIZE(imx581_custom2_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 559200000,
		.linelength = 7872,
		.framelength = 2366,
		.max_framerate = 300,
		.mipi_pixel_rate = 364680000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8000,
			.full_h = 6000,
			.x0_offset = 0,
			.y0_offset = 744,
			.w0_size = 8000,
			.h0_size = 4512,
			.scale_w = 4000,
			.scale_h = 2256,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4000,
			.h1_size = 2256,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4000,
			.h2_tg_size = 2256,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{/* Reg_M_DX2_4000*2256@30fps -> preisp*/
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = imx581_custom3_setting,
		.mode_setting_len = ARRAY_SIZE(imx581_custom3_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 559200000,
		.linelength = 7872,
		.framelength = 2366,
		.max_framerate = 300,
		.mipi_pixel_rate = 364680000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8000,
			.full_h = 6000,
			.x0_offset = 0,
			.y0_offset = 744,
			.w0_size = 8000,
			.h0_size = 4512,
			.scale_w = 4000,
			.scale_h = 2256,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4000,
			.h1_size = 2256,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4000,
			.h2_tg_size = 2256,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_video,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{/* Reg_M_DX7_1 1600x1200,24fps,Qbin_V2H2,w/o PDAF,Tline=11.42us */
		.frame_desc = frame_desc_cus4,
		.num_entries = ARRAY_SIZE(frame_desc_cus4),
		.mode_setting_table = imx581_custom4_setting,
		.mode_setting_len = ARRAY_SIZE(imx581_custom4_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 470400000,
		.linelength = 5376,
		.framelength = 3644,
		.max_framerate = 240,
		.mipi_pixel_rate = 345600000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8000,
			.full_h = 6000,
			.x0_offset = 0,
			.y0_offset = 600,
			.w0_size = 8000,
			.h0_size = 4800,
			.scale_w = 2000,
			.scale_h = 1200,
			.x1_offset = 200,
			.y1_offset = 0,
			.w1_size = 1600,
			.h1_size = 1200,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1600,
			.h2_tg_size = 1200,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{/* Reg_M_DX11 2560x1440,30fps,Qbin,w/PDAF Type2,Tline>=4.94us&Specify MIPI */
		.frame_desc = frame_desc_cus5,
		.num_entries = ARRAY_SIZE(frame_desc_cus5),
		.mode_setting_table = imx581_custom5_setting,
		.mode_setting_len = ARRAY_SIZE(imx581_custom5_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 1588800000,
		.linelength = 7872,
		.framelength = 6726,
		.max_framerate = 300,
		.mipi_pixel_rate = 785140000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8000,
			.full_h = 6000,
			.x0_offset = 0,
			.y0_offset = 1560,
			.w0_size = 8000,
			.h0_size = 2880,
			.scale_w = 4000,
			.scale_h = 1440,
			.x1_offset = 720,
			.y1_offset = 0,
			.w1_size = 2560,
			.h1_size = 1440,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2560,
			.h2_tg_size = 1440,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = IMX581_SENSOR_ID_22023,
	.reg_addr_sensor_id = {0x0016, 0x0017},
	.i2c_addr_table = {0x34, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {8000, 6000},
	.mirror = IMAGE_HV_MIRROR,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY,
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1.12,
	.ana_gain_max = BASEGAIN * 64,
	.ana_gain_type = 0,
	.ana_gain_step = 1,
	.ana_gain_table = imx581_ana_gain_table,
	.ana_gain_table_size = sizeof(imx581_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 6,
	.exposure_max = 128*(65535 - 48), /* exposure reg is limited to 4x. max = max - margin */
	.exposure_step = 2,
	.exposure_margin = 48,

	.frame_length_max = 0xffff,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 3,
	.start_exposure_offset = 1908400,//cc

	.pdaf_type = PDAF_SUPPORT_CAMSV,
	.hdr_type = HDR_SUPPORT_STAGGER_FDOL,
	.seamless_switch_support = TRUE,
	.temperature_support = TRUE,

	.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.g_cali = get_sensor_cali,
	.s_gph = set_group_hold,
	.s_cali = set_sensor_cali,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {
			{0x0202, 0x0203},//Long exposure
			{0x3FE0, 0x3FE1},//Middle exposure
			{0x0224, 0x0225},//Short exposure
	},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x3100,
	.reg_addr_ana_gain = {
			{0x0204, 0x0205},//Long Gian
			{0x3FE2, 0x3FE3},//Middle Gian
			{0x0216, 0x0217},//Short Gian
	},
	.reg_addr_frame_length = {0x0340, 0x0341},
	.reg_addr_temp_en = 0x0138,
	.reg_addr_temp_read = 0x013A,
	.reg_addr_auto_extend = 0x0350,
	.reg_addr_frame_count = PARAM_UNDEFINED,
	.reg_addr_fast_mode = 0x3010,

	.init_setting_table = imx581_init_setting,
	.init_setting_len = ARRAY_SIZE(imx581_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	.chk_s_off_before_s_on = 1,
	.chk_s_off_before_control = 1,
	.chk_s_off_after_s_off = 0,

	//.checksum_value = 0xAF3E324F,
	.checksum_value = 0xf10e5980,
};

static struct subdrv_ops ops = {
	.get_id = get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
	.close = common_close,
	.get_frame_desc = common_get_frame_desc,
	.get_temp = common_get_temp,
	.get_csi_param = common_get_csi_param,
	.vsync_notify = vsync_notify,
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_AVDD, 2904000, 3},
	{HW_ID_AVDD1, 1800000, 3},
	{HW_ID_AFVDD, 2800000, 3},
	{HW_ID_DVDD, 1136000, 4},
	{HW_ID_DOVDD, 1800000, 3},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 6},
	{HW_ID_RST, 1, 2},
};

const struct subdrv_entry imx581_mipi_raw_22023_entry = {
	.name = "imx581_mipi_raw_22023",
	.id = IMX581_SENSOR_ID_22023,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

static unsigned int read_imx581_eeprom_info(struct subdrv_ctx *ctx, kal_uint16 meta_id,
	BYTE *data, int size)
{
	kal_uint16 addr;
	int readsize;

	if (meta_id != imx581_eeprom_info[meta_id].meta)
		return -1;

	if (size != imx581_eeprom_info[meta_id].size)
		return -1;

	addr = imx581_eeprom_info[meta_id].start;
	readsize = imx581_eeprom_info[meta_id].size;

	if(!read_cmos_eeprom_p8(ctx, addr, data, readsize)) {
		DRV_LOGE(ctx, "read meta_id(%d) failed", meta_id);
	}

	return 0;
}

static void read_module_data(struct subdrv_ctx *ctx)
{
	kal_uint16 idx = 0;

	read_imx581_eeprom_info(ctx, EEPROM_META_MODULE_ID,
		&(imx581_common_data[0]), 2);
	// ctx->s_ctx.module_id = (kal_uint16)(imx581_common_data[1] << 8) |
	// 	imx581_common_data[0];
	read_imx581_eeprom_info(ctx, EEPROM_META_SENSOR_ID,
		&(imx581_common_data[2]), 2);
	read_imx581_eeprom_info(ctx, EEPROM_META_LENS_ID,
		&(imx581_common_data[4]), 2);
	read_imx581_eeprom_info(ctx, EEPROM_META_VCM_ID,
		&(imx581_common_data[6]), 2);
	read_imx581_eeprom_info(ctx, EEPROM_META_MODULE_SN,
		&(imx581_common_data[8]), 17);
	read_imx581_eeprom_info(ctx, EEPROM_META_AF_CODE,
		&(imx581_common_data[25]), 6);
	read_imx581_eeprom_info(ctx, EEPROM_META_AF_FLAG,
		&(imx581_common_data[33]), 1);

	for (idx = 0; idx < 36; idx = idx + 4)
		DRV_LOG(ctx, "otp data1: %02x %02x %02x %02x\n", imx581_common_data[idx],
			imx581_common_data[idx + 1], imx581_common_data[idx + 2],
			imx581_common_data[idx + 3]);
}

static void read_unique_sensorid(struct subdrv_ctx *ctx)
{
	u8 i = 0;
	LOG_INF("read sensor unique sensorid");
	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		subdrv_i2c_wr_u8(ctx, 0x0A02, 0x7F);
		subdrv_i2c_wr_u8(ctx, 0x0A00, 0x01);
		adaptor_i2c_rd_p8(ctx->i2c_client, ctx->i2c_write_id >> 1,
		IMX581_UNIQUE_SENSOR_ID_22023, &(imx581_unique_id[0]), IMX581_UNIQUE_SENSOR_ID_LENGHT_22023);
		i++;
	}
}

static void imx581_get_sensor_sn(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	DRV_LOGE(ctx, "get otp data");
	memcpy(feature_return_para_32, imx581_common_data,
		OPLUS_CAMERA_COMMON_DATA_LENGTH);
	*len = OPLUS_CAMERA_COMMON_DATA_LENGTH;
}

static void imx581_get_unique_sensorid(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	LOG_INF("get unique sensorid");
	memcpy(feature_return_para_32, imx581_unique_id,
		IMX581_UNIQUE_SENSOR_ID_LENGHT_22023);
	LOG_INF("para :%x, get unique sensorid", *para);
}

static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
	kal_uint16 get_byte = 0;

	adaptor_i2c_rd_u8(ctx->i2c_client, IMX581_EEPROM_READ_ID_22023 >> 1, addr, (u8 *)&get_byte);
	return get_byte;
}

#ifdef WRITE_DATA_MAX_LENGTH
#undef WRITE_DATA_MAX_LENGTH
#endif
#define   WRITE_DATA_MAX_LENGTH     (32)
static kal_int32 table_write_eeprom_30Bytes(struct subdrv_ctx *ctx,
        kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
	kal_int32 ret = ERROR_NONE;
    ret = adaptor_i2c_wr_p8(ctx->i2c_client, IMX581_EEPROM_WRITE_ID_22023 >> 1,
            addr, para, len);

	return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
    kal_int32 ret = ERROR_NONE;
    kal_uint16 reg = 0x8000;
    if (enable) {
        adaptor_i2c_wr_u8(ctx->i2c_client, IMX581_EEPROM_READ_ID_22023 >> 1, reg, 0x0E);
    }
    else {
        adaptor_i2c_wr_u8(ctx->i2c_client, IMX581_EEPROM_READ_ID_22023 >> 1, reg, 0x06);
    }

    return ret;
}

static kal_int32 write_Module_data(struct subdrv_ctx *ctx,
    ACDK_SENSOR_ENGMODE_STEREO_STRUCT * pStereodata)
{
    kal_int32  ret = ERROR_NONE;
    kal_uint16 data_base, data_length;
    kal_uint32 idx, idy;
    kal_uint8 *pData;
    UINT32 i = 0;
    kal_uint16 offset = 0;
    if(pStereodata != NULL) {
        LOG_INF("SET_SENSOR_OTP: 0x%x %d 0x%x %d\n",
            pStereodata->uSensorId,
            pStereodata->uDeviceId,
            pStereodata->baseAddr,
            pStereodata->dataLength);

        data_base = pStereodata->baseAddr;
        data_length = pStereodata->dataLength;
        pData = pStereodata->uData;
        offset = ALIGN(data_base, WRITE_DATA_MAX_LENGTH) - data_base;
        if (offset > data_length) {
            offset = data_length;
        }
        if ((pStereodata->uSensorId == IMX581_SENSOR_ID_22023) && (data_length == CALI_DATA_SLAVE_LENGTH)
            && (data_base == IMX581_STEREO_START_ADDR_22023 )) {
            LOG_INF("Write: %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556]);
            write_eeprom_protect(ctx, 0);
            msleep(6);
            if (offset > 0) {
                ret = table_write_eeprom_30Bytes(ctx, data_base, &pData[0], offset);
                if (ret != ERROR_NONE) {
                    LOG_INF("write_eeprom error: offset\n");
                    /* open write protect */
                    write_eeprom_protect(ctx, 1);
                    msleep(6);
                    return -1;
                }
                msleep(6);
                data_base += offset;
                data_length -= offset;
                pData += offset;
            }
            idx = data_length/WRITE_DATA_MAX_LENGTH;
            idy = data_length%WRITE_DATA_MAX_LENGTH;
            for (i = 0; i < idx; i++ ) {
                ret = table_write_eeprom_30Bytes(ctx, (data_base+WRITE_DATA_MAX_LENGTH*i),
                    &pData[WRITE_DATA_MAX_LENGTH*i], WRITE_DATA_MAX_LENGTH);
                if (ret != ERROR_NONE) {
                    LOG_INF("write_eeprom error: i= %d\n", i);
                    write_eeprom_protect(ctx, 1);
                    msleep(6);
                    return -1;
                }
                msleep(6);
            }
            ret = table_write_eeprom_30Bytes(ctx, (data_base+WRITE_DATA_MAX_LENGTH*idx),
                &pData[WRITE_DATA_MAX_LENGTH*idx], idy);
            if (ret != ERROR_NONE) {
                LOG_INF("write_eeprom error: idx= %d idy= %d\n", idx, idy);
                write_eeprom_protect(ctx, 1);
                msleep(6);
                return -1;
            }
            msleep(6);
            write_eeprom_protect(ctx, 1);
            msleep(6);
            LOG_INF("com_0:0x%x\n", read_cmos_eeprom_8(ctx, data_base));
            LOG_INF("com_39:0x%x\n", read_cmos_eeprom_8(ctx, data_base+39));
            LOG_INF("innal_40:0x%x\n", read_cmos_eeprom_8(ctx, data_base+40));
            LOG_INF("innal_1556:0x%x\n", read_cmos_eeprom_8(ctx, data_base+1556));
            LOG_INF("write_Module_data Write end\n");
        } else if ((pStereodata->uSensorId == IMX581_SENSOR_ID_22023) && (data_length < AESYNC_DATA_LENGTH_TOTAL)
            && (data_base == IMX581_AESYNC_START_ADDR_22023)) {
            LOG_INF("write main aesync: %x %x %x %x %x %x %x %x\n", pData[0], pData[1],
                pData[2], pData[3], pData[4], pData[5], pData[6], pData[7]);
            write_eeprom_protect(ctx, 0);
            msleep(6);
            if (offset > 0) {
                ret = table_write_eeprom_30Bytes(ctx, data_base, &pData[0], offset);
                if (ret != ERROR_NONE) {
                    LOG_INF("write_eeprom error: offset\n");
                    /* open write protect */
                    write_eeprom_protect(ctx, 1);
                    msleep(6);
                    return -1;
                }
                msleep(6);
                data_base += offset;
                data_length -= offset;
                pData += offset;
            }
            idx = data_length/WRITE_DATA_MAX_LENGTH;
            idy = data_length%WRITE_DATA_MAX_LENGTH;
            for (i = 0; i < idx; i++ ) {
                ret = table_write_eeprom_30Bytes(ctx, (data_base+WRITE_DATA_MAX_LENGTH*i),
                    &pData[WRITE_DATA_MAX_LENGTH*i], WRITE_DATA_MAX_LENGTH);
                if (ret != ERROR_NONE) {
                    LOG_INF("write_eeprom error: i= %d\n", i);
                    write_eeprom_protect(ctx, 1);
                    msleep(6);
                    return -1;
                }
                msleep(6);
            }
            ret = table_write_eeprom_30Bytes(ctx, (data_base+WRITE_DATA_MAX_LENGTH*idx),
                &pData[WRITE_DATA_MAX_LENGTH*idx], idy);
            if (ret != ERROR_NONE) {
                LOG_INF("write_eeprom error: idx= %d idy= %d\n", idx, idy);
                write_eeprom_protect(ctx, 1);
                msleep(6);
                return -1;
            }
            msleep(6);
            write_eeprom_protect(ctx, 1);
            msleep(6);
            LOG_INF("readback main aesync: %x %x %x %x %x %x %x %x\n",
                read_cmos_eeprom_8(ctx, IMX581_AESYNC_START_ADDR_22023),
                read_cmos_eeprom_8(ctx, IMX581_AESYNC_START_ADDR_22023+1),
                read_cmos_eeprom_8(ctx, IMX581_AESYNC_START_ADDR_22023+2),
                read_cmos_eeprom_8(ctx, IMX581_AESYNC_START_ADDR_22023+3),
                read_cmos_eeprom_8(ctx, IMX581_AESYNC_START_ADDR_22023+4),
                read_cmos_eeprom_8(ctx, IMX581_AESYNC_START_ADDR_22023+5),
                read_cmos_eeprom_8(ctx, IMX581_AESYNC_START_ADDR_22023+6),
                read_cmos_eeprom_8(ctx, IMX581_AESYNC_START_ADDR_22023+7));
            LOG_INF("AESync write_Module_data Write end\n");
        } else {
            LOG_INF("Invalid Sensor id:0x%x write eeprom\n", pStereodata->uSensorId);
            return -1;
        }
    } else {
        LOG_INF("imx581 write_Module_data pStereodata is null\n");
        return -1;
    }
    return ret;
}

static void imx581_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    int ret = ERROR_NONE;
    ret = write_Module_data(ctx, (ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(para));
    if (ret != ERROR_NONE) {
        LOG_INF("ret=%d\n", ret);
    }
}

static void imx581_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    if(*len > CALI_DATA_MASTER_LENGTH)
        *len = CALI_DATA_MASTER_LENGTH;
    read_imx581_eeprom_info(ctx, EEPROM_META_STEREO_DATA,
            (BYTE *)para, *len);
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, IMX581_EEPROM_READ_ID_22023 >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
	DRV_LOGE(ctx, "imx581 read_otp_info begin\n");
	read_cmos_eeprom_p8(ctx, 0, otp_data_checksum, OTP_SIZE);
	DRV_LOGE(ctx, "imx581 read_otp_info end\n");
}

static void imx581_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	DRV_LOGE(ctx, "get otp data");
	if (otp_data_checksum[0] == 0) {
		read_otp_info(ctx);
	} else {
		DRV_LOG(ctx, "otp data has already read");
	}
	memcpy(feature_return_para_32, (UINT32 *)otp_data_checksum, sizeof(otp_data_checksum));
	*len = sizeof(otp_data_checksum);
}

static void imx581_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	get_imgsensor_id(ctx, (u32 *)para);
}

static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
{
	u8 i = 0;
	u8 retry = 2;
	static bool first_read = KAL_TRUE;
	u32 addr_h = ctx->s_ctx.reg_addr_sensor_id.addr[0];
	u32 addr_l = ctx->s_ctx.reg_addr_sensor_id.addr[1];
	u32 addr_ll = ctx->s_ctx.reg_addr_sensor_id.addr[2];

	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		do {
			*sensor_id = (subdrv_i2c_rd_u8(ctx, addr_h) << 8) |
				subdrv_i2c_rd_u8(ctx, addr_l);
			if (addr_ll)
				*sensor_id = ((*sensor_id) << 8) | subdrv_i2c_rd_u8(ctx, addr_ll);
			DRV_LOG(ctx, "i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == IMX581_SENSOR_ID) {
				*sensor_id = ctx->s_ctx.sensor_id;
				if (first_read) {
					read_module_data(ctx);
					read_unique_sensorid(ctx);
					first_read = KAL_FALSE;
				}
				return ERROR_NONE;
			}
			DRV_LOG(ctx, "Read sensor id fail. i2c_write_id: 0x%x\n", ctx->i2c_write_id);
			DRV_LOG(ctx, "sensor_id = 0x%x, ctx->s_ctx.sensor_id = 0x%x\n",
				*sensor_id, ctx->s_ctx.sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != ctx->s_ctx.sensor_id) {
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

static u16 imx581_feedback_awbgain[] = {
	0x0B8E, 0x01,
	0x0B8F, 0x00,
	0x0B90, 0x02,
	0x0B91, 0x28,
	0x0B92, 0x01,
	0x0B93, 0x77,
	0x0B94, 0x01,
	0x0B95, 0x00,
};

/*write AWB gain to sensor*/
static void feedback_awbgain(struct subdrv_ctx *ctx, kal_uint32 r_gain, kal_uint32 b_gain)
{
	UINT32 r_gain_int = 0;
	UINT32 b_gain_int = 0;

	if (ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3) { // || // RMSC
//			ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM5) { // QRMSC
		DRV_LOG(ctx, "feedback_awbgain r_gain: %d, b_gain: %d\n", r_gain, b_gain);
		r_gain_int = r_gain / 512;
		b_gain_int = b_gain / 512;
		imx581_feedback_awbgain[5] = r_gain_int;
		imx581_feedback_awbgain[7] = (r_gain - r_gain_int * 512) / 2;
		imx581_feedback_awbgain[9] = b_gain_int;
		imx581_feedback_awbgain[11] = (b_gain - b_gain_int * 512) / 2;
		subdrv_i2c_wr_regs_u8(ctx, imx581_feedback_awbgain,
			ARRAY_SIZE(imx581_feedback_awbgain));
	}
}

static void imx581_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len) {
	struct SET_SENSOR_AWB_GAIN *awb_gain = (struct SET_SENSOR_AWB_GAIN *)para;
	feedback_awbgain(ctx, awb_gain->ABS_GAIN_R, awb_gain->ABS_GAIN_B);
	return;
}

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	sensor_init(ctx);

	/*QSC setting*/
	if (ctx->s_ctx.s_cali != NULL) {
		ctx->s_ctx.s_cali((void*)ctx);
	} else {
		write_sensor_Cali(ctx);
	}

	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	memset(ctx->ana_gain, 0, sizeof(ctx->gain));
	ctx->exposure[0] = ctx->s_ctx.exposure_def;
	ctx->ana_gain[0] = ctx->s_ctx.ana_gain_def;
	ctx->current_scenario_id = scenario_id;
	ctx->pclk = ctx->s_ctx.mode[scenario_id].pclk;
	ctx->line_length = ctx->s_ctx.mode[scenario_id].linelength;
	ctx->frame_length = ctx->s_ctx.mode[scenario_id].framelength;
	ctx->current_fps = 10 * ctx->pclk / ctx->line_length / ctx->frame_length;
	ctx->readout_length = ctx->s_ctx.mode[scenario_id].readout_length;
	ctx->read_margin = ctx->s_ctx.mode[scenario_id].read_margin;
	ctx->min_frame_length = ctx->frame_length;
	ctx->autoflicker_en = FALSE;
	ctx->test_pattern = 0;
	ctx->ihdr_mode = 0;
	ctx->pdaf_mode = 0;
	ctx->hdr_mode = 0;
	ctx->extend_frame_length_en = 0;
	ctx->is_seamless = 0;
	ctx->fast_mode_on = 0;
	ctx->sof_cnt = 0;
	ctx->ref_sof_cnt = 0;
	ctx->is_streaming = 0;

	return ERROR_NONE;
}

static void set_sensor_cali(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	u16 idx = 0;
	u8 support = FALSE;
	u8 *pbuf = NULL;
	u16 size = 0;
	u16 addr = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	if (!probe_eeprom(ctx))
		return;

	idx = ctx->eeprom_index;

	/* QSC data */
	support = info[idx].qsc_support;
	pbuf = info[idx].preload_qsc_table;
	size = info[idx].qsc_size;
	addr = info[idx].sensor_reg_addr_qsc;
	if (support) {
		if (pbuf != NULL && addr > 0 && size > 0) {
			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, size);
			subdrv_i2c_wr_u8(ctx, 0x3621, 0x01);
			DRV_LOG(ctx, "set QSC calibration data done.");
		} else {
			subdrv_i2c_wr_u8(ctx, 0x32D2, 0x00);
		}
	}
}

static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u8 temperature = 0;
	int temperature_convert = 0;

	temperature = subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_temp_read);

	if (temperature <= 0x60)
		temperature_convert = temperature;
	else if (temperature >= 0x61 && temperature <= 0x7F)
		temperature_convert = 97;
	else if (temperature >= 0x80 && temperature <= 0xE2)
		temperature_convert = -30;
	else
		temperature_convert = (char)temperature | 0xFFFFFF0;

	DRV_LOG(ctx, "temperature: %d degrees\n", temperature_convert);
	return temperature_convert;
}

static void set_group_hold(void *arg, u8 en)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	if (en)
		set_i2c_buffer(ctx, 0x0104, 0x01);
	else
		set_i2c_buffer(ctx, 0x0104, 0x00);
}

static u16 get_gain2reg(u32 gain)
{
	return (1024 - (1024 * BASEGAIN) / gain);
}

void imx581_get_min_shutter_by_scenario(struct subdrv_ctx *ctx,
		enum SENSOR_SCENARIO_ID_ENUM scenario_id,
		u64 *min_shutter, u64 *exposure_step)
{
	check_current_scenario_id_bound(ctx);
	DRV_LOG(ctx, "sensor_mode_num[%d]", ctx->s_ctx.sensor_mode_num);
	if (scenario_id < ctx->s_ctx.sensor_mode_num) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
			case HDR_RAW_STAGGER_2EXP:
				*exposure_step = ctx->s_ctx.exposure_step*2;
				*min_shutter = ctx->s_ctx.exposure_min * 2;
				break;
			case HDR_RAW_STAGGER_3EXP:
				*exposure_step = ctx->s_ctx.exposure_step*3;
				*min_shutter = ctx->s_ctx.exposure_min * 3;
				break;
			case HDR_NONE:
				if (ctx->s_ctx.mode[scenario_id].coarse_integ_step &&
					ctx->s_ctx.mode[scenario_id].min_exposure_line) {
					*exposure_step = ctx->s_ctx.mode[scenario_id].coarse_integ_step;
					*min_shutter = ctx->s_ctx.mode[scenario_id].min_exposure_line;
				} else {
					*exposure_step = ctx->s_ctx.exposure_step;
					*min_shutter = ctx->s_ctx.exposure_min;
				}
				break;
			default:
				*exposure_step = ctx->s_ctx.exposure_step;
				*min_shutter = ctx->s_ctx.exposure_min;
				break;
		}
	} else {
		DRV_LOG(ctx, "over sensor_mode_num[%d], use default", ctx->s_ctx.sensor_mode_num);
		*exposure_step = ctx->s_ctx.exposure_step;
		*min_shutter = ctx->s_ctx.exposure_min;
	}
	DRV_LOG(ctx, "scenario_id[%d] exposure_step[%d] min_shutter[%d]\n", scenario_id, *exposure_step, *min_shutter);
}

void imx581_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;
	imx581_get_min_shutter_by_scenario(ctx,
		(enum SENSOR_SCENARIO_ID_ENUM)*(feature_data),
		feature_data + 1, feature_data + 2);
	return;
}

static void imx581_get_cust_pixel_rate(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;
	switch (*feature_data) {
		case SENSOR_SCENARIO_ID_CUSTOM2:
		case SENSOR_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = (1500*8/10*1000000);
			break;
		default:
			break;
	}
}

static void imx581_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	enum SENSOR_SCENARIO_ID_ENUM scenario_id;
	u32 *ae_ctrl = NULL;
	u64 *feature_data = (u64 *) para;
	u32 fine_integ_line = 0;
	u32 cit_step = 0;
	u32 rg_shutter = 0;
	u32 prsh_length_lines = 0;

	if (feature_data == NULL) {
		DRV_LOGE(ctx, "input scenario is null!");
		return;
	}
	scenario_id = *feature_data;
	if ((feature_data + 1) != NULL)
		ae_ctrl = (u32 *)((uintptr_t)(*(feature_data + 1)));
	else
		DRV_LOGE(ctx, "no ae_ctrl input");

	check_current_scenario_id_bound(ctx);
	DRV_LOG(ctx, "E: set seamless switch %u %u\n", ctx->current_scenario_id, scenario_id);
	if (!ctx->extend_frame_length_en)
		DRV_LOGE(ctx, "please extend_frame_length before seamless_switch!\n");
	ctx->extend_frame_length_en = FALSE;

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOGE(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		return;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_group == 0 ||
		ctx->s_ctx.mode[scenario_id].seamless_switch_group !=
			ctx->s_ctx.mode[ctx->current_scenario_id].seamless_switch_group) {
		DRV_LOGE(ctx, "seamless_switch not supported\n");
		return;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table == NULL) {
		DRV_LOGE(ctx, "Please implement seamless_switch setting\n");
		return;
	}

	if (imx581_comp_params[ctx->current_scenario_id].enable &&
		imx581_comp_params[ctx->current_scenario_id].clock_vtpxck > 0 &&
		imx581_comp_params[ctx->current_scenario_id].cal_fn) {
		imx581_comp_params[ctx->current_scenario_id].cal_fn(ctx,
			(u32)scenario_id, &prsh_length_lines);
	}

	ctx->is_seamless = TRUE;
	update_mode_info(ctx, scenario_id);

	subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x3010, 0x02);
	i2c_table_write(ctx,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_len);
	if (ae_ctrl) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
		case HDR_RAW_STAGGER_2EXP:
			set_multi_shutter_frame_length(ctx, ae_ctrl, 2, 0);
			set_multi_gain(ctx, ae_ctrl + 5, 2);
			break;
		case HDR_RAW_STAGGER_3EXP:
			set_multi_shutter_frame_length(ctx, ae_ctrl, 3, 0);
			set_multi_gain(ctx, ae_ctrl + 5, 3);
			break;
		default:
			set_shutter(ctx, *ae_ctrl);
			set_gain(ctx, *(ae_ctrl + 5));
			break;
		}
	}
	DRV_LOG(ctx, "write seamless switch para done\n");

	if (ae_ctrl && prsh_length_lines > 0) {
		rg_shutter = *ae_ctrl;
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
		case HDR_RAW_STAGGER_2EXP:
			rg_shutter /= 2;
			break;
		case HDR_RAW_STAGGER_3EXP:
			rg_shutter /= 3;
			break;
		default:
			fine_integ_line = ctx->s_ctx.mode[ctx->current_scenario_id].fine_integ_line;
			cit_step = ctx->s_ctx.mode[ctx->current_scenario_id].coarse_integ_step;
			rg_shutter = FINE_INTEG_CONVERT(rg_shutter, fine_integ_line);
			rg_shutter = max(rg_shutter, ctx->s_ctx.exposure_min);
			rg_shutter = min(rg_shutter, ctx->s_ctx.exposure_max);
			if (cit_step)
				rg_shutter = round_up(rg_shutter, cit_step);
			break;
		}

		if (prsh_length_lines > rg_shutter + 1) {
			prsh_length_lines -= rg_shutter;
			if (ctx->s_ctx.mode[scenario_id].hdr_mode == HDR_NONE)
				prsh_length_lines = round_up(prsh_length_lines, 2);
			DRV_LOG(ctx, "rg_shutter(%u) prsh_length_lines(%u)\n", rg_shutter, prsh_length_lines);
			subdrv_i2c_wr_u8(ctx, 0x3036, 0x01);
			subdrv_i2c_wr_u8(ctx, 0x3039, prsh_length_lines >> 16 & 0xFF);
			subdrv_i2c_wr_u8(ctx, 0x303a, prsh_length_lines >> 8  & 0xFF);
			subdrv_i2c_wr_u8(ctx, 0x303b, prsh_length_lines & 0xFF);
		}
	}

	subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);

	ctx->fast_mode_on = TRUE;
	ctx->ref_sof_cnt = ctx->sof_cnt;
	ctx->is_seamless = FALSE;
	DRV_LOG(ctx, "X: set seamless switch done\n");
}

static void imx581_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode) {
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
		switch (mode) {
		case 5:
			subdrv_i2c_wr_u8(ctx, 0x020E, 0x00); /* dig_gain = 0 */
			subdrv_i2c_wr_u8(ctx, 0x0218, 0x00);
			subdrv_i2c_wr_u8(ctx, 0x3033, 0x00);
			break;
		default:
			subdrv_i2c_wr_u8(ctx, 0x0601, mode);
			break;
		}
	} else if (ctx->test_pattern) {
		subdrv_i2c_wr_u8(ctx, 0x0601, 0x00); /*No pattern*/
		subdrv_i2c_wr_u8(ctx, 0x020E, 0x01);
		subdrv_i2c_wr_u8(ctx, 0x0218, 0x01);
		subdrv_i2c_wr_u8(ctx, 0x3033, 0x40);
	}
	ctx->test_pattern = mode;
}

static void imx581_esd_reset_by_user(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;
	unsigned int version = get_PCB_Version();
	DRV_LOG(ctx, "call SENSOR_FEATURE_ESD_RESET_BY_USER\n");
	switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		case SENSOR_SCENARIO_ID_CUSTOM1:
		case SENSOR_SCENARIO_ID_CUSTOM2:
		case SENSOR_SCENARIO_ID_CUSTOM3:
		case SENSOR_SCENARIO_ID_CUSTOM4:
		case SENSOR_SCENARIO_ID_CUSTOM5:
		case SENSOR_SCENARIO_ID_CUSTOM6:
		case SENSOR_SCENARIO_ID_CUSTOM7:
		case SENSOR_SCENARIO_ID_CUSTOM8:
		case SENSOR_SCENARIO_ID_CUSTOM9:
		case SENSOR_SCENARIO_ID_CUSTOM10:
		case SENSOR_SCENARIO_ID_CUSTOM11:
		default:
			if (version < PCB_VERSION_DVT2)
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;
			else
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
		break;
	}
}

static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(&(ctx->s_ctx), &static_ctx, sizeof(struct subdrv_static_ctx));
	subdrv_ctx_init(ctx);
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
	return 0;
}

static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt)
{
	DRV_LOG(ctx, "sof_cnt(%u) ctx->ref_sof_cnt(%u) ctx->fast_mode_on(%d)",
		sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
	ctx->sof_cnt = sof_cnt;
	if (ctx->fast_mode_on && (sof_cnt > ctx->ref_sof_cnt)) {
		ctx->fast_mode_on = FALSE;
		ctx->ref_sof_cnt = 0;
		DRV_LOG(ctx, "seamless_switch disabled.");
		set_i2c_buffer(ctx, 0x3010, 0x00);
		set_i2c_buffer(ctx, 0x3036, 0x00);
		commit_i2c_buffer(ctx);
	}
	return 0;
}

void get_sensor_cali(void* arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	u16 idx = 0;
	u8 support = FALSE;
	u8 *buf = NULL;
	u16 size = 0;
	u16 addr = 0;
	u8 qsc_is_valid = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	/* Probe EEPROM device */
	if (!probe_eeprom(ctx))
		return;

	idx = ctx->eeprom_index;

	/* QSC data */
	support = info[idx].qsc_support;
	size = info[idx].qsc_size;
	addr = info[idx].addr_qsc;
	buf = info[idx].qsc_table;
	if (support && size > 0) {
		// Check QSC validation
		qsc_is_valid = i2c_read_eeprom(ctx, OTP_QSC_VALID_ADDR);
		if (qsc_is_valid != 0x01) {
			DRV_LOGE(ctx, "QSC data is invalid, flag(%02x)", qsc_is_valid);
		} else if (info[idx].preload_qsc_table == NULL) {
			info[idx].preload_qsc_table = kmalloc(size, GFP_KERNEL);
			if (buf == NULL) {
				if (!read_cmos_eeprom_p8(ctx, addr, info[idx].preload_qsc_table, size)) {
					DRV_LOGE(ctx, "preload QSC data failed");
				}
			} else {
				memcpy(info[idx].preload_qsc_table, buf, size);
			}
			DRV_LOG(ctx, "preload QSC data %u bytes", size);
		} else {
			DRV_LOG(ctx, "QSC data is already preloaded %u bytes", size);
		}
	}

	ctx->is_read_preload_eeprom = 1;
}

static void comp_mode_tran_time_cal1(struct subdrv_ctx *ctx, u32 scenario_id, u32* prsh) {
	#define SYSTEM_USED_LINES1 (96UL)
	#define SYSTEM_DELAY1      (189UL)
	u64 frame_duration = 0;
	u64 data_delay = 0;
	u64 system_delay = 0;
	u64 current_tline = 0;
	u64 tline = 0;

	if (!prsh) {
		DRV_LOGE(ctx, "prsh param is NULL");
		return;
	}

	*prsh = 0U;
	if (imx581_comp_params[ctx->current_scenario_id].clock_vtpxck == 0) {
		DRV_LOG(ctx, "invalid params");
		return;
	}

	frame_duration = 1000000000UL / ctx->current_fps * 10;
	current_tline = 1000000000UL * ctx->s_ctx.mode[ctx->current_scenario_id].linelength /
		ctx->s_ctx.mode[ctx->current_scenario_id].pclk;
	tline = 1000000000UL * ctx->s_ctx.mode[scenario_id].linelength /
		ctx->s_ctx.mode[scenario_id].pclk;
	data_delay = (ctx->s_ctx.mode[ctx->current_scenario_id].imgsensor_winsize_info.h2_tg_size +
		SYSTEM_USED_LINES1) * current_tline;
	system_delay = SYSTEM_DELAY1 * 1000 * 1000 * 10 /
		imx581_comp_params[ctx->current_scenario_id].clock_vtpxck;
	if (frame_duration <= data_delay + system_delay) {
		DRV_LOGE(ctx, "invalid parameter");
		return;
	}

	*prsh = (frame_duration - data_delay - system_delay) / tline;
	if (ctx->s_ctx.mode[scenario_id].hdr_mode == HDR_RAW_STAGGER_2EXP) {
		*prsh = *prsh / 2;
	}

	DRV_LOG(ctx, "frame_duration(%lu), current_tline(%lu), tline(%lu), "
		"data_delay(%lu) system_delay(%lu) prsh(%u)\n", frame_duration,
		current_tline, tline, data_delay, system_delay, *prsh);
}
