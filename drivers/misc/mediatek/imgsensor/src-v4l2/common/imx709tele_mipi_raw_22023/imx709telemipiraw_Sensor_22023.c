// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 imx709telemipiraw_Sensor_22023.c
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
#include "imx709telemipiraw_Sensor_22023.h"

#define IMX709TELE_EEPROM_READ_ID_22023	0xA0
#define IMX709TELE_EEPROM_WRITE_ID_22023	0xA1
#define IMX709TELE_MAX_OFFSET_22023		0x8000
#define OPLUS_CAMERA_COMMON_DATA_LENGTH 40
#define PFX "imx709tele_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)
#define OTP_SIZE    0x4000
#define LRC_L_REG   0xCE00
#define LRC_R_REG   0xCF00
#define OTP_LRC_VALID_ADDR 0x2104
#define OTP_QSC_VALID_ADDR 0x1F18
#define IMX709TELE_UNIQUE_SENSOR_ID_22023 0x3CF0
#define IMX709TELE_UNIQUE_SENSOR_ID_LENGHT_22023 11

static kal_uint8 otp_data_checksum[OTP_SIZE] = {0};
static void set_sensor_cali(void *arg);
static int get_sensor_temperature(void *arg);
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static void imx709tele_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void imx709tele_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void imx709tele_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void imx709tele_get_sensor_sn(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void imx709tele_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void imx709tele_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void imx709tele_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void imx709tele_get_unique_sensorid(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx709tele_open(struct subdrv_ctx *ctx);
static int imx709tele_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);
static void set_sensor_lrc(void *arg);
static void get_sensor_cali(void* arg);
static void feedback_awbgain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void imx709tele_esd_reset_by_user(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size);
static void imx709tele_get_cust_pixel_rate(struct subdrv_ctx *ctx, u8 *para, u32 *len);

unsigned int get_PCB_Version(void);

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

static BYTE imx709tele_common_data[OPLUS_CAMERA_COMMON_DATA_LENGTH] = { 0 };
static BYTE imx709tele_unique_id[IMX709TELE_UNIQUE_SENSOR_ID_LENGHT_22023] = { 0 };

static struct eeprom_map_info imx709tele_eeprom_info[] = {
	{ EEPROM_META_MODULE_ID, 0x0000, 0x000F, 0x0010, 2, true },
	{ EEPROM_META_SENSOR_ID, 0x0006, 0x000F, 0x0010, 2, true },
	{ EEPROM_META_LENS_ID, 0x0008, 0x000F, 0x0010, 2, true },
	{ EEPROM_META_VCM_ID, 0x000A, 0x000F, 0x0010, 2, true },
	{ EEPROM_META_MIRROR_FLIP, 0x000E, 0x000F, 0x0010, 1, true },
	{ EEPROM_META_MODULE_SN, 0x00B0, 0x00C1, 0x00C2, 17, true },
	{ EEPROM_META_AF_CODE, 0x0092, 0x0098, 0x0099, 6, true },
	{ EEPROM_META_AF_FLAG, 0x0098, 0x0098, 0x0099, 1, true },
	{ EEPROM_META_STEREO_DATA, 0x0000, 0x0000, 0x0000, 0, false },
	{ EEPROM_META_STEREO_MW_MAIN_DATA, 0x2B00, 0x3199, 0x319A, CALI_DATA_SLAVE_LENGTH, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA, 0x3100, 0x3E19, 0x3E1A, CALI_DATA_SLAVE_LENGTH, true },
};

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, imx709tele_set_test_pattern},
	{SENSOR_FEATURE_SEAMLESS_SWITCH, imx709tele_seamless_switch},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, imx709tele_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, imx709tele_get_sensor_sn},
	{SENSOR_FEATURE_SET_SENSOR_OTP, imx709tele_set_eeprom_calibration},
	{SENSOR_FEATURE_GET_EEPROM_STEREODATA, imx709tele_get_eeprom_calibration},
	{SENSOR_FEATURE_GET_SENSOR_OTP_ALL, imx709tele_get_otp_checksum_data},
	{SENSOR_FEATURE_GET_UNIQUE_SENSORID, imx709tele_get_unique_sensorid},
	{SENSOR_FEATURE_SET_AWB_GAIN, feedback_awbgain},
	{SENSOR_FEATURE_GET_CUST_PIXEL_RATE, imx709tele_get_cust_pixel_rate},
	{SENSOR_FEATURE_ESD_RESET_BY_USER, imx709tele_esd_reset_by_user},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x00660021,
		.addr_header_id = 0x00000006,
		.i2c_write_id = 0xA0,

		.qsc_support = TRUE,
		.qsc_size = 1560,
		.addr_qsc = 0x1900,
		.sensor_reg_addr_qsc = 0x1000,

		.lrc_support = TRUE,
		.lrc_size = 260,
		.addr_lrc = 0x2000,
		.sensor_reg_addr_lrc = 0xC800, // useless
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_pre_cap = {
	.i4OffsetX = 16,
	.i4OffsetY = 16,
	.i4PitchX  = 16,
	.i4PitchY  = 16,
	.i4PairNum  = 16,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 2,
	.i4PosR = {
		{18, 17}, {26, 17}, {16, 19}, {24, 19},
		{20, 21}, {28, 21}, {22, 23}, {30, 23},
		{18, 25}, {26, 25}, {16, 27}, {24, 27},
		{20, 29}, {28, 29}, {22, 31}, {30, 31},
	},
	.i4PosL = {
		{19, 17}, {27, 17}, {17, 19}, {25, 19},
		{21, 21}, {29, 21}, {23, 23}, {31, 23},
		{19, 25}, {27, 25}, {17, 27}, {25, 27},
		{21, 29}, {29, 29}, {23, 31}, {31, 31},
	},
	.i4BlockNumX = 200,
	.i4BlockNumY = 152,
	.iMirrorFlip = 3,
	.i4Crop = {
		{8, 8}, {8, 8}, {8, 304}, {0, 0}, {0, 0},
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		{0, 0}, {8, 8},{0, 0}, {824, 620}, {824, 620},
	},
	.i4FullRawW = 3280,
	.i4FullRawH = 2464,
	.i4VCPackNum = 1,
	.i4ModeIndex = 0,
	.sPDMapInfo[0] = {
		.i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 2,
		.i4PDRepetition = 4,
		.i4PDOrder = { 1 },
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_video = {
	.i4OffsetX = 16,
	.i4OffsetY = 16,
	.i4PitchX  = 16,
	.i4PitchY  = 16,
	.i4PairNum  = 16,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 2,
	.i4PosR = {
		{18, 17}, {26, 17}, {16, 19}, {24, 19},
		{20, 21}, {28, 21}, {22, 23}, {30, 23},
		{18, 25}, {26, 25}, {16, 27}, {24, 27},
		{20, 29}, {28, 29}, {22, 31}, {30, 31},
	},
	.i4PosL = {
		{19, 17}, {27, 17}, {17, 19}, {25, 19},
		{21, 21}, {29, 21}, {23, 23}, {31, 23},
		{19, 25}, {27, 25}, {17, 27}, {25, 27},
		{21, 29}, {29, 29}, {23, 31}, {31, 31},
	},
	.i4BlockNumX = 200,
	.i4BlockNumY = 116,
	.iMirrorFlip = 3,
	.i4Crop = {
		{8, 8}, {8, 8}, {8, 304}, {0, 0}, {0, 0},
		{0, 0}, {0, 0}, {0, 0}, {8, 304}, {8, 304},
		{8, 304}, {0, 0}, {8, 304},  {824, 620}
	},
	.i4FullRawW = 3280,
	.i4FullRawH = 2464,
	.i4VCPackNum = 1,
	.i4ModeIndex = 0,
	.sPDMapInfo[0] = {
		.i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 2,
		.i4PDRepetition = 4,
		.i4PDOrder = { 1 },
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_full = {
	.i4OffsetX = 32,
	.i4OffsetY = 32,
	.i4PitchX  = 32,
	.i4PitchY  = 32,
	.i4PairNum  = 16,
	.i4SubBlkW  = 16,
	.i4SubBlkH  = 4,

	.i4PosL = {{37,34},{53,34},{33,38},{49,38},
				{41,42},{57,42},{45,46},{61,46},
				{37,50},{53,50},{33,54},{49,54},
				{41,58},{57,58},{45,62},{61,62}, },

	.i4PosR = {{38,34},{54,34},{34,38},{50,38},
				{42,42},{58,42},{46,46},{62,46},
				{38,50},{54,50},{34,54},{50,54},
				{42,58},{58,58},{46,62},{62,62}, },
	.i4BlockNumX = 200,
	.i4BlockNumY = 152,
	.iMirrorFlip = 3,
	.i4FullRawW = 6560,
	.i4FullRawH = 4928,
	.i4VCPackNum = 1,
	.i4Crop = { {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		        {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
                {0, 0}, {0, 0}, {0, 0}, {1648, 1240}},

	.sPDMapInfo[0] = {
		.i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 2,
		.i4PDRepetition = 4,
		.i4PDOrder = { 1 },
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_cus2 = {
	.i4OffsetX = 8,
	.i4OffsetY = 8,
	.i4PitchX  = 8,
	.i4PitchY  = 8,
	.i4PairNum  = 8,
	.i4SubBlkW  = 4,
	.i4SubBlkH  = 2,
	.i4PosR = {
		{8, 9},  {12, 9},  {10, 11}, {14, 11},
		{8, 13}, {12, 13}, {10, 15}, {14, 15},
	},
	.i4PosL = {
		{9, 9},  {13, 9},  {11, 11}, {15, 11},
		{9, 13}, {13, 13}, {11, 15}, {15, 15},
	},
	.i4BlockNumX = 200,
	.i4BlockNumY = 152,
	.iMirrorFlip = 3,
	.i4Crop = {
		{8, 8}, {8, 8}, {8, 304}, {0, 0}, {0, 0},
		{0, 0}, {4, 4}, {0, 0}, {8, 304}, {8, 304},
		{8, 304}, {0, 0}, {8, 304}, {824, 620}
	},
	.i4FullRawW = 1640,
	.i4FullRawH = 1232,
	.i4VCPackNum = 1,
	.i4ModeIndex = 0,
	.sPDMapInfo[0] = {
		.i4VCFeature = VC_PDAF_STATS_NE_PIX_1,
		.i4PDPattern = 2,
		.i4PDRepetition = 2,
		.i4PDOrder = { 1 },
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 2448,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x30,
            .hsize = 800, // pixel
            .vsize = 1216,
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
            .hsize = 3264,
            .vsize = 2448,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x30,
            .hsize = 800, // pixel
            .vsize = 1216,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
        },
    },
};
//reg C4-3-1
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x30,
            .hsize = 800, // pixel
            .vsize = 928,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
        },
    }
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_STAGGER_NE,
        },
    }
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_STAGGER_NE,
        },
    }
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 1632,
            .vsize = 1224,
            .user_data_desc = VC_STAGGER_NE,
        },
    }
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 1632,
            .vsize = 1224,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
#if 1
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x30,
            .hsize = 800, // pixel
            .vsize = 608,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
        },
    }
#endif
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 6528,
            .vsize = 4896,
            .user_data_desc = VC_STAGGER_NE,
        },
    }
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
#if 1
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x30,
            .hsize = 800, // pixel
            .vsize = 928,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
        },
    },
#endif
    {
        .bus.csi2 = {
            .channel = 1,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_STAGGER_ME,
        },
    }
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
	{
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
#if 1
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x30,
            .hsize = 800, // pixel
            .vsize = 928,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
        },
    }
#endif
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus6[] = {
	{
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
#if 0
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x30,
            .hsize = 808, // pixel
            .vsize = 928,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
        },
    }
#endif
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus7[] = {
	{
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 2448,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x30,
            .hsize = 800, // pixel
            .vsize = 1216,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
        },
    }
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus8[] = {
	{
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2c,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_RAW_PROCESSED_DATA,
        },
    },
    {
        .bus.csi2 = {
            .channel = 1,
            .data_type = 0x12,
            .hsize = 1024,
            .vsize = 564,
            .user_data_desc = VC_GENERAL_EMBEDDED,  // meta raw

        },
    },
    {
        .bus.csi2 = {
            .channel = 2,
            .data_type = 0x2e,
            .hsize = 800, // pixel
            .vsize = 928,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
        },
    }
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus9[] = {
	{
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 2448,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x30,
            .hsize = 400, // pixel
            .vsize = 608,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
        },
    }
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus10[] = {
	{
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2c,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_RAW_PROCESSED_DATA,
        },
    },
	{
        .bus.csi2 = {
            .channel = 1,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_RAW_DATA,
        },
    },
    {
        .bus.csi2 = {
            .channel = 1,
            .data_type = 0x12,
            .hsize = 1024,
            .vsize = 564,
            .user_data_desc = VC_GENERAL_EMBEDDED,  // meta raw

        },
    },
    {
        .bus.csi2 = {
            .channel = 2,
            .data_type = 0x2e,
            .hsize = 800, // pixel
            .vsize = 928,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
        },
    }
};

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = imx709tele_preview_setting,
		.mode_setting_len = ARRAY_SIZE(imx709tele_preview_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = imx709tele_seamless_preview,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(imx709tele_seamless_preview),
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 864000000,
		.linelength = 11760,
		.framelength = 2528,
		.max_framerate = 293,
		//.mipi_pixel_rate = 303600000,
		.mipi_pixel_rate = 464000000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 16,
			.w0_size = 6560,
			.h0_size = 4896,
			.scale_w = 3280,
			.scale_h = 2448,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_pre_cap,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 500,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x46,
		},
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = imx709tele_capture_setting,
		.mode_setting_len = ARRAY_SIZE(imx709tele_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 864000000,
		.linelength = 11760,
		.framelength = 2528,
		.max_framerate = 293,
		//.mipi_pixel_rate = 303600000,
		.mipi_pixel_rate = 464000000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 16,
			.w0_size = 6560,
			.h0_size = 4896,
			.scale_w = 3280,
			.scale_h = 2448,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_pre_cap,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 500,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x46,
		},
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = imx709tele_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(imx709tele_normal_video_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = imx709tele_seamless_normal_video,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(imx709tele_seamless_normal_video),
		.hdr_group = 1,
		.hdr_mode = HDR_NONE,
		.pclk = 864000000,
		.linelength = 11760,
		.framelength = 2448,
		.max_framerate = 300,
		.mipi_pixel_rate = 447200000,
		//.mipi_pixel_rate = 479000000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 608,
			.w0_size = 6560,
			.h0_size = 3712,
			.scale_w = 3280,
			.scale_h = 1856,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1856,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1856,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_video,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 500,
		.delay_frame = 3,
		.csi_param = {
			.dphy_trail = 0x48,
		},
	},
	{
		.frame_desc = frame_desc_hs,
		.num_entries = ARRAY_SIZE(frame_desc_hs),
		.mode_setting_table = imx709tele_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(imx709tele_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 849600000,
		.linelength = 7400,
		.framelength = 1912,
		.max_framerate = 600,
		.mipi_pixel_rate = 416000000,
		//.mipi_pixel_rate = 446000000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 608,
			.w0_size = 6560,
			.h0_size = 3712,
			.scale_w = 3280,
			.scale_h = 1856,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1856,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1856,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 795,
		.delay_frame = 3,
		.csi_param = {
			.dphy_trail = 0x46,
		},
	},
	{
		.frame_desc = frame_desc_slim,
		.num_entries = ARRAY_SIZE(frame_desc_slim),
		.mode_setting_table = imx709tele_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(imx709tele_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 849600000,
		.linelength = 7400,
		.framelength = 1912,
		.max_framerate = 600,
		.mipi_pixel_rate = 416000000,
		//.mipi_pixel_rate = 446000000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 608,
			.w0_size = 6560,
			.h0_size = 3712,
			.scale_w = 3280,
			.scale_h = 1856,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1856,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1856,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 795,
		.delay_frame = 3,
		.csi_param = {
			.dphy_trail = 0x45,
		},
	},
	{
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = imx709tele_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(imx709tele_custom1_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 240000000,
		.linelength = 5880,
		.framelength = 2720,
		.max_framerate = 150,
		.mipi_pixel_rate = 120000000,
		//.mipi_pixel_rate = 129000000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6560,
			.h0_size = 4928,
			.scale_w = 1640,
			.scale_h = 1232,
			.x1_offset = 4,
			.y1_offset = 4,
			.w1_size = 1632,
			.h1_size = 1224,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1632,
			.h2_tg_size = 1224,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 1000,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x50,
		},
	},
	{
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = imx709tele_custom2_setting,
		.mode_setting_len = ARRAY_SIZE(imx709tele_custom2_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 355200000,
		.linelength = 5880,
		.framelength = 2008,
		.max_framerate = 300,
		.mipi_pixel_rate = 182800000,
		//.mipi_pixel_rate = 196000000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6560,
			.h0_size = 4928,
			.scale_w = 1640,
			.scale_h = 1232,
			.x1_offset = 4,
			.y1_offset = 4,
			.w1_size = 1632,
			.h1_size = 1224,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1632,
			.h2_tg_size = 1224,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_cus2,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 1000,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x58,
		},
	},
	{
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = imx709tele_custom3_setting,
		.mode_setting_len = ARRAY_SIZE(imx709tele_custom3_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 537600000,
		.linelength = 7120,
		.framelength = 5032,
		.max_framerate = 150,
		.mipi_pixel_rate = 521100000,
		//.mipi_pixel_rate = 559000000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 16,
			.w0_size = 6560,
			.h0_size = 4896,
			.scale_w = 6560,
			.scale_h = 4896,
			.x1_offset = 16,
			.y1_offset = 0,
			.w1_size = 6528,
			.h1_size = 4896,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 6528,
			.h2_tg_size = 4896,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 826,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x49,
		},
		.ana_gain_max = BASEGAIN * 16,
	},
	{
		.frame_desc = frame_desc_cus4,
		.num_entries = ARRAY_SIZE(frame_desc_cus4),
		.mode_setting_table = imx709tele_custom4_setting,
		.mode_setting_len = ARRAY_SIZE(imx709tele_custom4_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = imx709tele_seamless_custom4,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(imx709tele_seamless_custom4),
		.hdr_group = 1,
		.hdr_mode = HDR_RAW_STAGGER_2EXP,
		.pclk = 864000000,
		.linelength = 7456,
		.framelength = 1928*2,
		.max_framerate = 300,
		.mipi_pixel_rate = 447200000,
		//.mipi_pixel_rate = 479000000,
		.readout_length = 0*2,
		.read_margin = 10*2,
		.framelength_step = 4*2,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 608,
			.w0_size = 6560,
			.h0_size = 3712,
			.scale_w = 3280,
			.scale_h = 1856,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1856,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1856,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_video,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 789,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x49,
		},
	},
	{
		.frame_desc = frame_desc_cus5,
		.num_entries = ARRAY_SIZE(frame_desc_cus5),
		.mode_setting_table = imx709tele_custom5_setting,
		.mode_setting_len = ARRAY_SIZE(imx709tele_custom5_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 864000000,
		.linelength = 11760,
		.framelength = 2448,
		.max_framerate = 300,
		.mipi_pixel_rate = 447200000,
		//.mipi_pixel_rate = 479000000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 608,
			.w0_size = 6560,
			.h0_size = 3712,
			.scale_w = 3280,
			.scale_h = 1856,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1856,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1856,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_video,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 500,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x48,
		},
	},
	{
		.frame_desc = frame_desc_cus6,
		.num_entries = ARRAY_SIZE(frame_desc_cus6),
		.mode_setting_table = imx709tele_custom6_setting,
		.mode_setting_len = ARRAY_SIZE(imx709tele_custom6_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 864000000,
		.linelength = 7456,
		.framelength = 1928 * 2,
		.max_framerate = 300,
		.mipi_pixel_rate = 447200000,
		//.mipi_pixel_rate = 479000000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 608,
			.w0_size = 6560,
			.h0_size = 3712,
			.scale_w = 3280,
			.scale_h = 1856,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1856,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1856,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 789,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x48,
		},
	},
	{//reg_N-4 3264x2448 @24FPS w/ PD
		.frame_desc = frame_desc_cus7,
		.num_entries = ARRAY_SIZE(frame_desc_cus7),
		.mode_setting_table = imx709tele_custom7_setting,
		.mode_setting_len = ARRAY_SIZE(imx709tele_custom7_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 864000000,
		.linelength = 11760,
		.framelength = 3056,
		.max_framerate = 240,
		//.mipi_pixel_rate = 447200000,
		.mipi_pixel_rate = 464000000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 16,
			.w0_size = 6560,
			.h0_size = 4896,
			.scale_w = 3280,
			.scale_h = 2448,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_pre_cap,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 789,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x46,
		},
	},
	{//reg_N-4 3264x2448
		.frame_desc = frame_desc_cus8,
		.num_entries = ARRAY_SIZE(frame_desc_cus8),
		.mode_setting_table = imx709tele_custom8_setting,
		.mode_setting_len = ARRAY_SIZE(imx709tele_custom8_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 864000000,
		.linelength = 11760,
		.framelength = 2448,
		.max_framerate = 300,
		//.mipi_pixel_rate = 447200000,
		.mipi_pixel_rate = 458000000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 608,
			.w0_size = 6560,
			.h0_size = 3712,
			.scale_w = 3280,
			.scale_h = 1856,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1856,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1856,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_video,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 500,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x34,
		},
	},
	{//reg_N-3 3264x2448 @29FPS Full-Crop Izoom w/ N-2
		.frame_desc = frame_desc_cus9,
		.num_entries = ARRAY_SIZE(frame_desc_cus9),
		.mode_setting_table = imx709tele_custom9_setting,
		.mode_setting_len = ARRAY_SIZE(imx709tele_custom9_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = imx709tele_seamless_custom9,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(imx709tele_seamless_custom9),
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 864000000,
		.linelength = 7120,
		.framelength = 4184,
		.max_framerate = 293,
		//.mipi_pixel_rate = 447200000,
		.mipi_pixel_rate = 464000000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 1648,
			.y0_offset = 1240,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_full,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 789,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x46,
		},
		.ana_gain_max = BASEGAIN * 16,
	},
	{//pre_isp pure raw dump
		.frame_desc = frame_desc_cus10,
		.num_entries = ARRAY_SIZE(frame_desc_cus10),
		.mode_setting_table = imx709tele_custom10_setting,
		.mode_setting_len = ARRAY_SIZE(imx709tele_custom10_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 864000000,
		.linelength = 11760,
		.framelength = 2448,
		.max_framerate = 300,
		//.mipi_pixel_rate = 447200000,
		.mipi_pixel_rate = 458000000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 608,
			.w0_size = 6560,
			.h0_size = 3712,
			.scale_w = 3280,
			.scale_h = 1856,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1856,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1856,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_video,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 500,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x34,
		},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = IMX709TELE_SENSOR_ID_22023,
	.reg_addr_sensor_id = {0x0016, 0x0017},
	.i2c_addr_table = {0x34, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {6560, 4928},
	.mirror = IMAGE_HV_MIRROR,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY,
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 32,
	.ana_gain_type = 0,
	.ana_gain_step = 1,
	.ana_gain_table = imx709tele_ana_gain_table,
	.ana_gain_table_size = sizeof(imx709tele_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 8,
	.exposure_max = 128 * (0xffff - 24),
	.exposure_step = 1,
	.exposure_margin = 48,

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 3,
	.start_exposure_offset = 1836500,

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
			{0x0202, 0x0203}, // Long exposure
			{0x0000, 0x0000}, // Not supported
			{0x0224, 0x0225}, // Short exposure
	},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x3150,
	.reg_addr_ana_gain = {
			{0x0204, 0x0205}, // Long gain
			{0x0000, 0x0000}, // Not supported
			{0x0216, 0x0217}, // Short gain
	},
	.reg_addr_frame_length = {0x0340, 0x0341},
	.reg_addr_temp_en = 0x0138,
	.reg_addr_temp_read = 0x013A,
	.reg_addr_auto_extend = 0x0350,
	.reg_addr_frame_count = PARAM_UNDEFINED,
	.reg_addr_fast_mode = 0x3010,

	.init_setting_table = imx709tele_init_setting,
	.init_setting_len = ARRAY_SIZE(imx709tele_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	.chk_s_off_before_s_on = 1,
	.chk_s_off_before_control = 1,
	.chk_s_off_after_s_off = 0,

	.checksum_value = 0xAF3E324F,
};

static struct subdrv_ops ops = {
	.get_id = imx709tele_get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = imx709tele_open,
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
    //{HW_ID_PDN, 0, 0},
    {HW_ID_RST, 0, 1},
    {HW_ID_AVDD, 2904000, 3},
    {HW_ID_AFVDD, 2800000, 3},
    {HW_ID_DVDD, 816000, 4},
    {HW_ID_DOVDD, 1800000, 3},
    {HW_ID_MCLK_DRIVING_CURRENT, 4, 6},
    //{HW_ID_PDN, 1, 1},
    {HW_ID_RST, 1, 4}
};

const struct subdrv_entry imx709tele_mipi_raw_22023_entry = {
	.name = "imx709tele_mipi_raw_22023",
	.id = IMX709TELE_SENSOR_ID_22023,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

static unsigned int read_imx709tele_eeprom_info(struct subdrv_ctx *ctx, kal_uint16 meta_id,
	BYTE *data, int size)
{
	kal_uint16 addr;
	int readsize;

	if (meta_id != imx709tele_eeprom_info[meta_id].meta)
		return -1;

	if (size != imx709tele_eeprom_info[meta_id].size)
		return -1;

	addr = imx709tele_eeprom_info[meta_id].start;
	readsize = imx709tele_eeprom_info[meta_id].size;

	if (!read_cmos_eeprom_p8(ctx, addr, data, readsize)) {
		DRV_LOGE(ctx, "read meta_id(%d) failed", meta_id);
	}

	return 0;
}

static void read_module_data(struct subdrv_ctx *ctx)
{
	kal_uint16 idx = 0;

	read_imx709tele_eeprom_info(ctx, EEPROM_META_MODULE_ID,
		&(imx709tele_common_data[0]), 2);
	// ctx->s_ctx.module_id = (kal_uint16)(imx709tele_common_data[1] << 8) |
	// 	imx709tele_common_data[0];
	read_imx709tele_eeprom_info(ctx, EEPROM_META_SENSOR_ID,
		&(imx709tele_common_data[2]), 2);
	read_imx709tele_eeprom_info(ctx, EEPROM_META_LENS_ID,
		&(imx709tele_common_data[4]), 2);
	read_imx709tele_eeprom_info(ctx, EEPROM_META_VCM_ID,
		&(imx709tele_common_data[6]), 2);
	read_imx709tele_eeprom_info(ctx, EEPROM_META_MODULE_SN,
		&(imx709tele_common_data[8]), 17);
	read_imx709tele_eeprom_info(ctx, EEPROM_META_AF_CODE,
		&(imx709tele_common_data[25]), 6);
	read_imx709tele_eeprom_info(ctx, EEPROM_META_AF_FLAG,
		&(imx709tele_common_data[33]), 1);

	for (idx = 0; idx < 36; idx = idx + 4)
		DRV_LOG(ctx, "otp data: %02x %02x %02x %02x\n", imx709tele_common_data[idx],
			imx709tele_common_data[idx + 1], imx709tele_common_data[idx + 2],
			imx709tele_common_data[idx + 3]);
}

static void read_unique_sensorid(struct subdrv_ctx *ctx)
{
	u8 i = 0;
	LOG_INF("read sensor unique sensorid");
	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		adaptor_i2c_rd_p8(ctx->i2c_client, ctx->i2c_write_id >> 1,
		IMX709TELE_UNIQUE_SENSOR_ID_22023, &(imx709tele_unique_id[0]), IMX709TELE_UNIQUE_SENSOR_ID_LENGHT_22023);
		i++;
	}
}

static void imx709tele_get_sensor_sn(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	DRV_LOG(ctx, "get otp data");
	memcpy(feature_return_para_32, imx709tele_common_data,
		OPLUS_CAMERA_COMMON_DATA_LENGTH);
	*len = OPLUS_CAMERA_COMMON_DATA_LENGTH;
}

static void imx709tele_get_unique_sensorid(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	LOG_INF("get unique sensorid");
	memcpy(feature_return_para_32, imx709tele_unique_id,
		IMX709TELE_UNIQUE_SENSOR_ID_LENGHT_22023);
	LOG_INF("para :%x, get unique sensorid", *para);
}

static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
    kal_uint16 get_byte = 0;

    adaptor_i2c_rd_u8(ctx->i2c_client, IMX709TELE_EEPROM_READ_ID_22023 >> 1, addr, (u8 *)&get_byte);
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
    ret = adaptor_i2c_wr_p8(ctx->i2c_client, IMX709TELE_EEPROM_WRITE_ID_22023 >> 1,
            addr, para, len);

    return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
    kal_int32 ret = ERROR_NONE;
    kal_uint16 reg = 0xE000;
    if (enable) {
        adaptor_i2c_wr_u8(ctx->i2c_client, IMX709TELE_EEPROM_WRITE_ID_22023 >> 1, reg, 0xA1);
    }
    else {
        adaptor_i2c_wr_u8(ctx->i2c_client, IMX709TELE_EEPROM_WRITE_ID_22023 >> 1, reg, 0xA0);
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
        if ((pStereodata->uSensorId == IMX709TELE_SENSOR_ID_22023) && (data_length == CALI_DATA_SLAVE_LENGTH)
            && (data_base == IMX709TELE_STEREO_START_ADDR_22023)) {
            LOG_INF("Write: %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556]);
            /* close write protect */
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
                    /* open write protect */
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
                /* open write protect */
                write_eeprom_protect(ctx, 1);
                msleep(6);
                return -1;
            }
            msleep(6);
            /* open write protect */
            write_eeprom_protect(ctx, 1);
            msleep(6);
            LOG_INF("com_0:0x%x\n", read_cmos_eeprom_8(ctx, data_base));
            LOG_INF("com_39:0x%x\n", read_cmos_eeprom_8(ctx, data_base+39));
            LOG_INF("innal_40:0x%x\n", read_cmos_eeprom_8(ctx, data_base+40));
            LOG_INF("innal_1556:0x%x\n", read_cmos_eeprom_8(ctx, data_base+1556));
            LOG_INF("write_Module_data Write end\n");
        } else if ((pStereodata->uSensorId == IMX709TELE_SENSOR_ID_22023) && (data_length < AESYNC_DATA_LENGTH_TOTAL)
            && (data_base == IMX709TELE_AESYNC_START_ADDR_22023)) {
            LOG_INF("write main aesync: %x %x %x %x %x %x %x %x\n", pData[0], pData[1],
                pData[2], pData[3], pData[4], pData[5], pData[6], pData[7]);
            /* close write protect */
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
                    /* open write protect */
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
                /* open write protect */
                write_eeprom_protect(ctx, 1);
                msleep(6);
                return -1;
            }
            msleep(6);
            /* open write protect */
            write_eeprom_protect(ctx, 1);
            msleep(6);
            LOG_INF("readback main aesync: %x %x %x %x %x %x %x %x\n",
                read_cmos_eeprom_8(ctx, IMX709TELE_AESYNC_START_ADDR_22023),
                read_cmos_eeprom_8(ctx, IMX709TELE_AESYNC_START_ADDR_22023+1),
                read_cmos_eeprom_8(ctx, IMX709TELE_AESYNC_START_ADDR_22023+2),
                read_cmos_eeprom_8(ctx, IMX709TELE_AESYNC_START_ADDR_22023+3),
                read_cmos_eeprom_8(ctx, IMX709TELE_AESYNC_START_ADDR_22023+4),
                read_cmos_eeprom_8(ctx, IMX709TELE_AESYNC_START_ADDR_22023+5),
                read_cmos_eeprom_8(ctx, IMX709TELE_AESYNC_START_ADDR_22023+6),
                read_cmos_eeprom_8(ctx, IMX709TELE_AESYNC_START_ADDR_22023+7));
            LOG_INF("AESync write_Module_data Write end\n");
        } else {
            LOG_INF("Invalid Sensor id:0x%x write eeprom\n", pStereodata->uSensorId);
            return -1;
        }
    } else {
        LOG_INF("imx709tele write_Module_data pStereodata is null\n");
        return -1;
    }
    return ret;
}

static void imx709tele_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    int ret = ERROR_NONE;
    ret = write_Module_data(ctx, (ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(para));
    if (ret != ERROR_NONE) {
        LOG_INF("ret=%d\n", ret);
    }
}

static void imx709tele_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    if(*len > CALI_DATA_SLAVE_LENGTH) {
        *len = CALI_DATA_SLAVE_LENGTH;
    }
    read_imx709tele_eeprom_info(ctx, EEPROM_META_STEREO_MT_MAIN_DATA,
            (BYTE *)para, *len);
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, IMX709TELE_EEPROM_READ_ID_22023 >> 1,
		addr, data, size) < 0) {
		return false;
	}
	return true;
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
	DRV_LOGE(ctx, "imx709 read_otp_info begin\n");
	read_cmos_eeprom_p8(ctx, 0, otp_data_checksum, OTP_SIZE);
	DRV_LOGE(ctx, "imx709 read_otp_info end\n");
}

static void imx709tele_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	DRV_LOGE(ctx, "get otp data");
	if (otp_data_checksum[0] == 0) {
		read_otp_info(ctx);
	} else {
		DRV_LOG(ctx, "otp data has already read read read");
	}
	memcpy(feature_return_para_32, (UINT32 *)otp_data_checksum, sizeof(otp_data_checksum));
	*len = sizeof(otp_data_checksum);
}

static void imx709tele_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len) {
	imx709tele_get_imgsensor_id(ctx, (u32*)para);
}

static int imx709tele_open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;
	u32 rmsc_ip_setting_len = ARRAY_SIZE(imx709tele_rmsc_ip_setting);

	/* get sensor id */
	if (imx709tele_get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	sensor_init(ctx);

	/*rmsc_ip_setting*/
	DRV_LOG(ctx, "E: rmsc_ip_setting_len size:%u\n", rmsc_ip_setting_len);
	i2c_table_write(ctx, imx709tele_rmsc_ip_setting, rmsc_ip_setting_len);
	DRV_LOG(ctx, "X: rmsc_ip_setting_len size:%u\n", rmsc_ip_setting_len);

	/*QSC setting*/
	if (ctx->s_ctx.s_cali != NULL) {
		ctx->s_ctx.s_cali((void*)ctx);
	} else {
		write_sensor_Cali(ctx);
	}

	/*LRC setting*/
	set_sensor_lrc((void*)ctx);

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

static int imx709tele_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
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
			DRV_LOG(ctx, "i2c_write_id:0x%x sensor_id(cur/exp):0x%x/0x%x\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == IMX709_SENSOR_ID) {
				*sensor_id = ctx->s_ctx.sensor_id;
				if (first_read) {
					read_module_data(ctx);
					read_unique_sensorid(ctx);
					first_read = KAL_FALSE;
				}
				return ERROR_NONE;
			}
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
			subdrv_i2c_wr_u8(ctx, 0x3207, 0x01); // enable QSC
			DRV_LOG(ctx, "set QSC calibration data done.");
		} else {
			subdrv_i2c_wr_u8(ctx, 0x3207, 0x00); // disable QSC
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

	if (en) {
		set_i2c_buffer(ctx, 0x0104, 0x01);
		if (ctx->s_ctx.reg_addr_fast_mode && ctx->fast_mode_on) {
			ctx->fast_mode_on = FALSE;
			set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x00);
		}
	}
	else
		set_i2c_buffer(ctx, 0x0104, 0x00);
}

static u16 get_gain2reg(u32 gain)
{
	return (16384 - (16384 * BASEGAIN) / gain);
}


static void imx709tele_get_cust_pixel_rate(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;
	switch (*feature_data) {
		case SENSOR_SCENARIO_ID_CUSTOM8:
		case SENSOR_SCENARIO_ID_CUSTOM10:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = (1800*8/10*1000000);
		default:
			break;
	}
}

static void imx709tele_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	enum SENSOR_SCENARIO_ID_ENUM scenario_id;
	u32 *ae_ctrl = NULL;
	u32 *feature_data = (u32 *)para;

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
	DRV_LOGE(ctx, "E: set seamless switch %u %u\n", ctx->current_scenario_id, scenario_id);

	if (ctx->current_scenario_id == scenario_id) {
		DRV_LOGE(ctx, "E: set seamless switch %u %u same,return\n", ctx->current_scenario_id, scenario_id);
		return;
	}

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
	subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);

	ctx->fast_mode_on = TRUE;
	ctx->ref_sof_cnt = ctx->sof_cnt;
	ctx->is_seamless = FALSE;
	DRV_LOGE(ctx, "X: set seamless switch done\n");
}

static void imx709tele_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
	switch (mode) {
	case 5:
		subdrv_i2c_wr_u8(ctx, 0x020E, 0x00); /* dig_gain = 0 */
		subdrv_i2c_wr_u8(ctx, 0x0218, 0x00);
		subdrv_i2c_wr_u8(ctx, 0x3021, 0x00);
		break;
	default:
		subdrv_i2c_wr_u8(ctx, 0x0601, mode);
		break;
	}

	if ((ctx->test_pattern) && (mode != ctx->test_pattern)) {
		if (ctx->test_pattern == 5) {
			subdrv_i2c_wr_u8(ctx, 0x020E, 0x01);
			subdrv_i2c_wr_u8(ctx, 0x0218, 0x01);
			subdrv_i2c_wr_u8(ctx, 0x3021, 0x40);
		}
		else if (mode == 0)
			subdrv_i2c_wr_u8(ctx, 0x0601, 0x00); /* No pattern */
	}

	ctx->test_pattern = mode;
}

static void imx709tele_esd_reset_by_user(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
	unsigned int version = get_PCB_Version();
	DRV_LOG(ctx, "PCB VERSION[%u]\n", version);
	if (version >= PCB_VERSION_DVT2) {
		static_ctx.mipi_sensor_type = MIPI_OPHY_NCSI2;
		static_ctx.mipi_lane_num = SENSOR_MIPI_4_LANE;
	}
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
	if (ctx->fast_mode_on && (sof_cnt > ctx->ref_sof_cnt)) {
		ctx->fast_mode_on = FALSE;
		ctx->ref_sof_cnt = 0;
		DRV_LOG(ctx, "seamless_switch disabled.");
		set_i2c_buffer(ctx, 0x3010, 0x00);
		commit_i2c_buffer(ctx);
	}
	return 0;
}

static void set_sensor_lrc(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	u16 idx = 0;
	u8 support = FALSE;
	u8 *pbuf = NULL;
	u16 size = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	if (!probe_eeprom(ctx))
		return;

	idx = ctx->eeprom_index;

	/* LRC data */
	support = info[idx].lrc_support;
	pbuf = info[idx].preload_lrc_table;
	size = info[idx].lrc_size;
	if (support) {
		if (pbuf != NULL && size > 0) {
			subdrv_i2c_wr_seq_p8(ctx, LRC_L_REG, pbuf, size / 2); // L data
			subdrv_i2c_wr_seq_p8(ctx, LRC_R_REG, pbuf + size / 2, size / 2); // R data
			DRV_LOG(ctx, "set LRC calibration data done.");
		} else {
			DRV_LOGE(ctx, "LRC calibration data error");
		}
	}
}

void get_sensor_cali(void* arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	u16 idx = 0;
	u8 support = FALSE;
	u8 *buf = NULL;
	u16 size = 0;
	u16 addr = 0;
	u8 qsc_is_valid = 0, lrc_is_valid = 0;
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

	/* LRC data */
	support = info[idx].lrc_support;
	size = info[idx].lrc_size;
	addr = info[idx].addr_lrc;
	buf = info[idx].lrc_table;
	if (support && size > 0) {
		lrc_is_valid = i2c_read_eeprom(ctx, OTP_LRC_VALID_ADDR);
		if (lrc_is_valid != 0x01) {
			DRV_LOGE(ctx, "LRC data is invalid, flag(%02x)", lrc_is_valid);
		} else if (info[idx].preload_lrc_table == NULL) {
			info[idx].preload_lrc_table = kmalloc(size, GFP_KERNEL);
			if (buf == NULL) {
				if (!read_cmos_eeprom_p8(ctx, addr, info[idx].preload_lrc_table, size)) {
					DRV_LOGE(ctx, "preload LRC data failed");
				}
			} else {
				memcpy(info[idx].preload_lrc_table, buf, size);
			}
			DRV_LOG(ctx, "preload LRC data %u bytes", size);
		} else {
			DRV_LOG(ctx, "LRC data is already preloaded %u bytes", size);
		}
	}

	ctx->is_read_preload_eeprom = 1;
}


/*write AWB gain to sensor*/
static u16 imx709_feedback_awbgain[] = {
	0x0b90, 0x00,
	0x0b91, 0x01,
	0x0b92, 0x00,
	0x0b93, 0x01,
	0x0B8E, 0x01,
	0x0B8F, 0x00,
	0x0B94, 0x01,
	0x0B95, 0x00,
};


static void feedback_awbgain(struct subdrv_ctx *ctx, u8 *para, u32 *len) {
	u32 *feature_data_32 = (u32 *) para;
	u32 r_gain = (u32)*(feature_data_32 + 1);
	u32 b_gain = (u32)*(feature_data_32 + 2);
	/* modify to separate 3hdr and remosaic */
	if (ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3) {
		/*write AWB gain to sensor*/
		u32 r_gain_int = 0;
		u32 b_gain_int = 0;

		DRV_LOG(ctx, "feedback_awbgain r_gain: %d, b_gain: %d\n", r_gain, b_gain);
		r_gain_int = r_gain / 512;
		b_gain_int = b_gain / 512;

		imx709_feedback_awbgain[1] = r_gain_int;
		imx709_feedback_awbgain[3] = (r_gain - r_gain_int * 512) / 2;
		imx709_feedback_awbgain[5] = b_gain_int;
		imx709_feedback_awbgain[7] = (b_gain - b_gain_int * 512) / 2;
		subdrv_i2c_wr_regs_u8(ctx, imx709_feedback_awbgain,
				sizeof(imx709_feedback_awbgain)/sizeof(u16));
	}
}
