/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef _TD2204_WQHD_AMB678ZY01_S6E3HC3_CMD_H_
#define _TD2204_WQHD_AMB678ZY01_S6E3HC3_CMD_H_

#define LCM_DSI_CMD_MODE		1

#define MAX_BRIGHTNESS			(2047)
#define FRAME_WIDTH			(1080)
#define FRAME_HEIGHT			(2400)
#define MIPI_DATA_RATE			(750)

#define PHYSICAL_WIDTH			(69010)
#define PHYSICAL_HEIGHT			(151588)


#define REGFLAG_UDELAY		0xFFFB
#define REGFLAG_DELAY		0xFFFC
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW	0xFFFE
#define REGFLAG_RESET_HIGH	0xFFFF

#define HFP (12)
#define HSA (10)
#define HBP (13)
#define VFP (20)
#define VSA (2)
#define VBP (8)

#define NIT50_OF_AOD_BL				365

#define DSC_ENABLE                  1
#define DSC_VER                     17
#define DSC_SLICE_MODE              1
#define DSC_RGB_SWAP                0
#define DSC_DSC_CFG                 34
#define DSC_RCT_ON                  1
#define DSC_BIT_PER_CHANNEL         8
#define DSC_DSC_LINE_BUF_DEPTH      9
#define DSC_BP_ENABLE               1
#define DSC_BIT_PER_PIXEL           128
#define DSC_SLICE_HEIGHT            32
#define DSC_SLICE_WIDTH             540
#define DSC_CHUNK_SIZE              540
#define DSC_XMIT_DELAY              512
#define DSC_DEC_DELAY               526
#define DSC_SCALE_VALUE             32
#define DSC_INCREMENT_INTERVAL      789
#define DSC_DECREMENT_INTERVAL      7
#define DSC_LINE_BPG_OFFSET         12
#define DSC_NFL_BPG_OFFSET          793
#define DSC_SLICE_BPG_OFFSET        814
#define DSC_INITIAL_OFFSET          6144
#define DSC_FINAL_OFFSET            4336
#define DSC_FLATNESS_MINQP          3
#define DSC_FLATNESS_MAXQP          12
#define DSC_RC_MODEL_SIZE           8192
#define DSC_RC_EDGE_FACTOR          6
#define DSC_RC_QUANT_INCR_LIMIT0    11
#define DSC_RC_QUANT_INCR_LIMIT1    11
#define DSC_RC_TGT_OFFSET_HI        3
#define DSC_RC_TGT_OFFSET_LO        3

struct LCD_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[200];
};

static struct LCD_setting_table init_setting[] =  {
	/* wqhd dsc 10bit pps setting start*/
	{0x9E, 0x79, {0x9E,
	0x11, 0x00,
	0x00,
	0x89, //BIT_PER_CHANNEL
	0x30, 0x80,
	0x09, 0x60,
	0x04, 0x38,
	0x00, 0x20, //slice hight
	0x02, 0x1C, //slice width
	0x02, 0x1C, //check
	0x02, 0x00, //xmit delay
	0x02, 0x0E, //dec delay
	0x00, 0x20, //SCALE_VALUE
	0x03, 0x15, //INCREMENT
	0x00, 0x07, //DECREMENT
	0x00,
	0x0C,//LINE_BPG_OFFSET
	0x03, 0x19, //NFL_BPG_OFFSET
	0x03, 0x2E, //SLICE_BPG_OFFSET
	0x18, 0x00, //
	0x10, 0xF0, //
	0x03, //FLATNESS_MINQP
	0x0C,//FLATNESS_MAXQP
	0x20, 0x00,//RC_MODEL_SIZE
	0x06, //RC_EDGE_FACTOR
	0x0B, //DSC_RC_QUANT_INCR_LIMIT0
	0x0B,
	0x33,//RC_TGT_OFFSET_HI RC_TGT_OFFSET_LO
	0x0E, 0x1C, 0x2A, 0x38,
	0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7B,
	0x7D, 0x7E, 0x01, 0x02, 0x01, 0x00, 0x09, 0x40,
	0x09, 0xBE, 0x19, 0xFC, 0x19, 0xFA, 0x19, 0xF8,
	0x1A, 0x38, 0x1A, 0x78, 0x1A, 0xB6, 0x2A, 0xF6,
	0x2B, 0x34, 0x2B, 0x74, 0x3B, 0x74, 0x6B, 0xF4,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} },
	/*dsc enable*/
	{ 0x9D, 0x02, {0x9D, 0x01} },
	// /* WQHD DSC Setting end*/
	/* Sleep Out(11h) delay 10+3ms*/
	{ 0x11, 0x02, {0x11, 0x00} },
	{ REGFLAG_DELAY, 13, {} },

	/* vlin current limit,delay 110+3ms*/
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x0B, 0xB1} },
	{ 0xB1, 0x04, {0xB1, 0xFF, 0xFF, 0x0F} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x11, 0xB1} },
	{ 0xB1, 0x02, {0xB1, 0x04} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	{ REGFLAG_DELAY, 113, {} },
	/* Common Setting */
	/* TE(Vsync) ON */
	{ 0x35, 0x02, {0x35, 0x00} },
	{ 0x44, 0x03, {0x44, 0x08, 0xFC} },
	/* FIXED TE OFF */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB9, 0x02, {0xB9, 0x00} }, /*0x41:fix te on 0x00:fix te off*/
	{ 0xB0, 0x04, {0xB0, 0x00, 0x06, 0xB9} },
	/*fix te on:0C 9E 0C 9E 00 1D*/
	/*fix te off:00 00 00 00 00 00*/
	{ 0xB9, 0x07, {0xB9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} },
	/*LTPS UPDATE*/
	{ 0xF7, 0x02, {0xF7, 0x0F} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	/* PAGE ADDRESS SET 1080x2400*/
	{ 0x2A, 0x05, {0x2A, 0x00, 0x00, 0x04, 0x37} },
	{ 0x2B, 0x05, {0x2B, 0x00, 0x00, 0x09, 0x5F} },
	/* Scaler Setting */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xC3, 0x02, {0xC3, 0x01} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x1F, 0xC3} },
	{ 0xC3, 0x17, {0xC3, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD2, 0x00, 0xD2, 0x00,
	  0xD2, 0x00, 0xD2, 0x00, 0xD2, 0x00, 0xD2, 0x00, 0xD2, 0x00, 0xD2, 0x00, 0xD2} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x35, 0xC3} },
	{ 0xC3, 0x17, {0xC3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9B, 0x00, 0x9B, 0x00,
	  0x9B, 0x00, 0x9B, 0x00, 0x9B, 0x00, 0x9B, 0x00, 0x9B, 0x00, 0x9B, 0x00, 0x9B} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x4B, 0xC3} },
	{ 0xC3, 0x17, {0xC3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5D, 0x00, 0x5D, 0x00,
	  0x5D, 0x00, 0x5D, 0x00, 0x5D, 0x00, 0x5D, 0x00, 0x5D, 0x00, 0x5D, 0x00, 0x5D} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x61, 0xC3} },
	{ 0xC3, 0x17, {0xC3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x00, 0x28, 0x00,
	  0x28, 0x00, 0x28, 0x00, 0x28, 0x00, 0x28, 0x00, 0x28, 0x00, 0x28, 0x00, 0x28} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	// /*set aod 30hz code*/
	// { 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	// { 0xB0, 0x04, {0xB0, 0x00, 0x61, 0xBD} },
	// { 0xBD, 0x0A, {0xBD, 0x04, 0x00, 0x08, 0x00, 0x0C, 0x00, 0x10, 0x00, 0x74} },
	// { 0xB0, 0x04, {0xB0, 0x00, 0x01, 0x60} },
	// { 0x60, 0x02, {0x60, 0x00} },
	// { 0xF7, 0x02, {0xF7, 0x0F} },
	// { 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	/* Frequency Select 120HZ*/
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x08, 0xCB} },
	{ 0xCB, 0x02, {0xCB, 0x27} },		   /*120hz:0x27 90hz:0xA7*/
	{ 0xB0, 0x04, {0xB0, 0x00, 0x10, 0xBD} },
	{ 0xBD, 0x02, {0xBD, 0x20} },		   /*120hz:0x20 90hz:0x00*/
	{ 0xBD, 0x03, {0xBD, 0x21, 0x22} },	 /*120hz:21 02 90hz:21 82*/
	{ 0x60, 0x02, {0x60, 0x00} },		   /*120hz:00 90hz:08*/
	{ 0xF7, 0x02, {0xF7, 0x0F} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	/*force increase off*/
	{0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{0xBD, 0x03, {0xBD, 0x21, 0x82} },
	{0xB0, 0x04, {0xB0, 0x00, 0x10, 0xBD} },
	{0xBD, 0x02, {0xBD, 0x00} },
	{0xB0, 0x04, {0xB0, 0x00, 0x21, 0xBD} },
	{0xBD, 0x0F, {0xBD, 0x03, 0x00, 0x06, 0x00, 0x09, 0x00, 0x0C,
			0x00, 0x0F, 0x00, 0x15, 0x00, 0x21, 0x00} },
	{0xB0, 0x04, {0xB0, 0x00, 0x61, 0xBD} },
	{0xBD, 0x0A, {0xBD, 0x04, 0x00, 0x08, 0x00, 0x0C, 0x00, 0x10, 0x00, 0x74} },
	{0xB0, 0x04, {0xB0, 0x00, 0x12, 0xBD} },
	{0xBD, 0x02, {0xBD, 0x01} },
	{0xB0, 0x04, {0xB0, 0x00, 0x14, 0xBD} },
	{0xBD, 0x02, {0xBD, 0x00} },
	{0xB0, 0x04, {0xB0, 0x00, 0x17, 0xBD} },
	{0xBD, 0x02, {0xBD, 0x01} },
	{0xBD, 0x02, {0xBD, 0x21} },
	{0xF7, 0x02, {0xF7, 0x0F} },
	{0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	/* HOP 60HZ*/
	{0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{0x60, 0x02, {0x60, 0x01} },
	{0xB0, 0x04, {0xB0, 0x00, 0x01, 0x60} },
	{0x60, 0x02, {0x60, 0x00} },
	{0xBD, 0x02, {0xBD, 0x21} },
	{0xF7, 0x02, {0xF7, 0x0F} },
	{0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	/* Brightness Control */
	/* Dimming Setting */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x0D, 0x63} },
	{ 0x63, 0x02, {0x63, 0x00} },	   /*dimming:0frame*/
	{ 0xB0, 0x04, {0xB0, 0x00, 0x0C, 0x63} },
	{ 0x63, 0x02, {0x63, 0x20} },	   /*0x20:elvss dim off 0x30:elvss dim on*/
	{ 0x53, 0x02, {0x53, 0x28} },
	{ 0x51, 0x03, {0x51, 0x03, 0xFF} },
	{ 0xF7, 0x02, {0xF7, 0x0F} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	/* ACL Mode */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0x55, 0x02, {0x55, 0x00} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	/* TSP SYNC ON*/
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x22, 0xB9} },
	{ 0xB9, 0x03, {0xB9, 0xB1, 0xA1} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x05, 0xF2} },
	{ 0xF2, 0x02, {0xF2, 0x52} },
	{ 0xF7, 0x02, {0xF7, 0x0F} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	/* Display On */
	{ 0x29, 0x02, {0x29, 0x00 } },

	/* seed crc setting begin*/
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xE4, 0x05, {0xE4, 0x2C, 0x2C, 0x21, 0x00} },
	{ 0xF8, 0x02, {0xF8, 0x00} },
	{ 0x1C, 0x02, {0x1C, 0x05} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x01, 0x1D} },
	{ 0x1D, 0x16, {0x1D, 0xE8, 0x05, 0x00, 0x0E, 0xE3, 0x01, 0x0E,
	  0x02, 0xE0, 0x20, 0xFF, 0xDF, 0xFF, 0x08, 0xDF, 0xF8, 0xF6, 0x00, 0xFF, 0xF7, 0xEE} },
	{ 0x1D, 0x02, {0x1D, 0x00} },
	{ 0xE4, 0x05, {0xE4, 0x2C, 0x2C, 0x01, 0x00} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	/* seed crc setting end*/
	// { 0x51, 0x03, {0x51, 0x07, 0xFF} },

};

static struct LCD_setting_table init_setting_1440x3200[] = {
	// /* wqhd dsc 10bit pps setting start*/
	// { 0x9E, 0x79, {0x9E
	//   0x05, 0xA0, 0x00, 0x19, 0x02, 0xD0, 0x02, 0xD0
	//   0x02, 0x00, 0x02, 0x68, 0x00, 0x20, 0x02, 0xBE
	//   0x00, 0x0A, 0x00, 0x0C, 0x04, 0x00, 0x03, 0x0D
	//   0x18, 0x00, 0x10, 0xF0, 0x07, 0x10, 0x20, 0x00
	//   0x06, 0x0F, 0x0F, 0x33, 0x0E, 0x1C, 0x2A, 0x38
	//   0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7B
	//   0x7D, 0x7E, 0x01, 0x02, 0x22, 0x00, 0x2A, 0x40
	//   0x2A, 0xBE, 0x3A, 0xFC, 0x3A, 0xFA, 0x3A, 0xF8
	//   0x3B, 0x38, 0x3B, 0x78, 0x3B, 0xB6, 0x4B, 0xF6
	//   0x4C, 0x34, 0x4C, 0x74, 0x5C, 0x74, 0x8C, 0xF4
	//   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	//   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	//   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	//   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} },
	// /*dsc enable*/
	// { 0x9D, 0x02, {0x9D, 0x01} },
	// /* WQHD DSC Setting end*/
	/* Sleep Out(11h) delay 10+3ms*/
	{ 0x11, 0x02, {0x11, 0x00} },
	{ REGFLAG_DELAY, 13, {} },

	/* vlin current limit,delay 110+3ms*/
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x0B, 0xB1} },
	{ 0xB1, 0x04, {0xB1, 0xFF, 0xFF, 0x0F} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x11, 0xB1} },
	{ 0xB1, 0x02, {0xB1, 0x04} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	{ REGFLAG_DELAY, 113, {} },
	/* Common Setting */
	/* TE(Vsync) ON */
	{ 0x35, 0x02, {0x35, 0x00} },
	/* FIXED TE OFF */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB9, 0x02, {0xB9, 0x00} }, /*0x41:fix te on 0x00:fix te off*/
	{ 0xB0, 0x04, {0xB0, 0x00, 0x06, 0xB9} },
	/*fix te on:0C 9E 0C 9E 00 1D*/
	/*fix te off:00 00 00 00 00 00*/
	{ 0xB9, 0x07, {0xB9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} },
	/*LTPS UPDATE*/
	{ 0xF7, 0x02, {0xF7, 0x0F} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	/* PAGE ADDRESS SET 1439x3199*/
	{ 0x2A, 0x05, {0x2A, 0x00, 0x00, 0x05, 0x9F} },
	{ 0x2B, 0x05, {0x2B, 0x00, 0x00, 0x0C, 0x7F} },
	/* Scaler Setting */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xC3, 0x02, {0xC3, 0x00} },		/* 0x00:WQHD+(1440x3200) 0x89:FHD+(1080x2400)*/
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	/*set aod 30hz code*/
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x61, 0xBD} },
	{ 0xBD, 0x0A, {0xBD, 0x04, 0x00, 0x08, 0x00, 0x0C, 0x00, 0x10, 0x00, 0x74} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x01, 0x60} },
	{ 0x60, 0x02, {0x60, 0x00} },
	{ 0xF7, 0x02, {0xF7, 0x0F} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	/* Frequency Select 90HZ*/
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x08, 0xCB} },
	{ 0xCB, 0x02, {0xCB, 0xA7} },			/*120hz:0x27 90hz:0xA7*/
	{ 0xB0, 0x04, {0xB0, 0x00, 0x10, 0xBD} },
	{ 0xBD, 0x02, {0xBD, 0x00} },			/*120hz:0x20 90hz:0x00*/
	{ 0xBD, 0x03, {0xBD, 0x21, 0x82} },		/*120hz:21 02 90hz:21 82*/
	{ 0x60, 0x02, {0x60, 0x08} },			/*120hz:00 90hz:08*/
	{ 0xF7, 0x02, {0xF7, 0x0F} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	/* Brightness Control */
	/* Dimming Setting */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x0D, 0x63} },
	{ 0x63, 0x02, {0x63, 0x00} },		/*dimming:0frame*/
	{ 0xB0, 0x04, {0xB0, 0x00, 0x0C, 0x63} },
	{ 0x63, 0x02, {0x63, 0x20} },		/*0x20:elvss dim off 0x30:elvss dim on*/
	{ 0x53, 0x02, {0x53, 0x28} },
	{ 0x51, 0x03, {0x51, 0x00, 0x00} },
	{ 0xF7, 0x02, {0xF7, 0x0F} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	/* ACL Mode */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0x55, 0x02, {0x55, 0x00} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	/* TSP SYNC ON*/
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x22, 0xB9} },
	{ 0xB9, 0x03, {0xB9, 0xB1, 0xA1} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x05, 0xF2} },
	{ 0xF2, 0x02, {0xF2, 0x52} },
	{ 0xF7, 0x02, {0xF7, 0x0F} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	/* Display On */
	{ 0x29, 0x02, {0x29, 0x00 } },
	/* seed crc setting begin*/
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xE4, 0x05, {0xE4, 0x2C, 0x2C, 0x21, 0x00} },
	{ 0xF8, 0x02, {0xF8, 0x00} },
	{ 0x1C, 0x02, {0x1C, 0x05} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x01, 0x1D} },
	{ 0x1D, 0x16, {0x1D, 0xE8, 0x05, 0x00, 0x0E, 0xE3, 0x01, 0x0E,
	  0x02, 0xE0, 0x20, 0xFF, 0xDF, 0xFF, 0x08, 0xDF, 0xF8, 0xF6, 0x00, 0xFF, 0xF7, 0xEE} },
	{ 0x1D, 0x02, {0x1D, 0x00} },
	{ 0xE4, 0x05, {0xE4, 0x2C, 0x2C, 0x01, 0x00} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	/* seed crc setting end*/

	{ 0x51, 0x03, {0x51, 0x0F, 0xFF} },
};

//1080x2400 60HZ
static struct LCD_setting_table init_setting_fhd_60hz[] = {
	/* wqhd dsc 10bit pps setting start*/
	{0x9E, 0x79, {0x9E,
	0x11, 0x00,
	0x00,
	0xAB, //BIT_PER_CHANNEL
	0x30, 0x80,
	0x09, 0x60,
	0x04, 0x38,
	0x00, 0x20, //slice hight
	0x02, 0x1C, //slice width
	0x02, 0x1C, //check
	0x02, 0x00, //xmit delay
	0x02, 0x0E, //dec delay
	0x00, 0x20, //SCALE_VALUE
	0x03, 0x15, //INCREMENT
	0x00, 0x07, //DECREMENT
	0x00,
	0x0C,//LINE_BPG_OFFSET
	0x03, 0x19, //NFL_BPG_OFFSET
	0x03, 0x2E, //SLICE_BPG_OFFSET
	0x18, 0x00, //
	0x10, 0xF0, //
	0x07, //FLATNESS_MINQP
	0x10,//FLATNESS_MAXQP
	0x20, 0x00,//RC_MODEL_SIZE
	0x06, //RC_EDGE_FACTOR
	0x0F, //DSC_RC_QUANT_INCR_LIMIT0
	0x0F,
	0x33,//RC_TGT_OFFSET_HI RC_TGT_OFFSET_LO
	0x0E, 0x1C, 0x2A, 0x38,
	0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7B,
	0x7D, 0x7E, 0x01, 0x02, 0x22, 0x00, 0x2A, 0x40,
	0x2A, 0xBE, 0x3A, 0xFC, 0x3A, 0xFA, 0x3A, 0xF8,
	0x3B, 0x38, 0x3B, 0x78, 0x3B, 0xB6, 0x4B, 0xF6,
	0x4C, 0x34, 0x4C, 0x74, 0x5C, 0x74, 0x8C, 0xF4,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} },
	/*dsc enable*/
	{ 0x9D, 0x02, {0x9D, 0x01} },
	// /* WQHD DSC Setting end*/
	/* Sleep Out(11h) delay 10+3ms*/
	{ 0x11, 0x02, {0x11, 0x00} },
	{ REGFLAG_DELAY, 13, {} },

	/* vlin current limit,delay 110+3ms*/
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x0B, 0xB1} },
	{ 0xB1, 0x04, {0xB1, 0xFF, 0xFF, 0x0F} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x11, 0xB1} },
	{ 0xB1, 0x02, {0xB1, 0x04} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	{ REGFLAG_DELAY, 113, {} },
	/* Common Setting */
	/* TE(Vsync) ON */
	{ 0x35, 0x02, {0x35, 0x00} },
	/* FIXED TE OFF */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB9, 0x02, {0xB9, 0x00} }, /*0x41:fix te on 0x00:fix te off*/
	{ 0xB0, 0x04, {0xB0, 0x00, 0x06, 0xB9} },
	/*fix te on:0C 9E 0C 9E 00 1D*/
	/*fix te off:00 00 00 00 00 00*/
	{ 0xB9, 0x07, {0xB9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} },
	/*LTPS UPDATE*/
	{ 0xF7, 0x02, {0xF7, 0x0F} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	/* PAGE ADDRESS SET 1080x2400*/
	{ 0x2A, 0x05, {0x2A, 0x00, 0x00, 0x04, 0x37} },
	{ 0x2B, 0x05, {0x2B, 0x00, 0x00, 0x09, 0x5F} },
	/* Scaler Setting */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xC3, 0x02, {0xC3, 0x89} },		/* 0x00:WQHD+(1440x3200) 0x89:FHD+(1080x2400)*/
	{ 0xB0, 0x04, {0xB0, 0x00, 0x1F, 0xC3} },
	{ 0xC3, 0x17, {0xC3, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD2, 0x00, 0xD2, 0x00,
	  0xD2, 0x00, 0xD2, 0x00, 0xD2, 0x00, 0xD2, 0x00, 0xD2, 0x00, 0xD2, 0x00, 0xD2} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x35, 0xC3} },
	{ 0xC3, 0x17, {0xC3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9B, 0x00, 0x9B, 0x00,
	  0x9B, 0x00, 0x9B, 0x00, 0x9B, 0x00, 0x9B, 0x00, 0x9B, 0x00, 0x9B, 0x00, 0x9B} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x4B, 0xC3} },
	{ 0xC3, 0x17, {0xC3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5D, 0x00, 0x5D, 0x00,
	  0x5D, 0x00, 0x5D, 0x00, 0x5D, 0x00, 0x5D, 0x00, 0x5D, 0x00, 0x5D, 0x00, 0x5D} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x61, 0xC3} },
	{ 0xC3, 0x17, {0xC3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x00, 0x28, 0x00,
	  0x28, 0x00, 0x28, 0x00, 0x28, 0x00, 0x28, 0x00, 0x28, 0x00, 0x28, 0x00, 0x28} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	// /*set aod 30hz code*/
	// { 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	// { 0xB0, 0x04, {0xB0, 0x00, 0x61, 0xBD} },
	// { 0xBD, 0x0A, {0xBD, 0x04, 0x00, 0x08, 0x00, 0x0C, 0x00, 0x10, 0x00, 0x74} },
	// { 0xB0, 0x04, {0xB0, 0x00, 0x01, 0x60} },
	// { 0x60, 0x02, {0x60, 0x00} },
	// { 0xF7, 0x02, {0xF7, 0x0F} },
	// { 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	/* Frequency Select 120HZ*/
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x08, 0xCB} },
	{ 0xCB, 0x02, {0xCB, 0x27} },			/*120hz:0x27 90hz:0xA7*/
	{ 0xB0, 0x04, {0xB0, 0x00, 0x10, 0xBD} },
	{ 0xBD, 0x02, {0xBD, 0x20} },			/*120hz:0x20 90hz:0x00*/
	{ 0xBD, 0x03, {0xBD, 0x21, 0x22} },		/*120hz:21 02 90hz:21 82*/
	{ 0x60, 0x02, {0x60, 0x01} },			/*120hz:00 90hz:08*/
	{ 0xF7, 0x02, {0xF7, 0x0F} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	/*force increase off*/
	{0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{0xBD, 0x03, {0xBD, 0x21, 0x82} },
	{0xB0, 0x04, {0xB0, 0x00, 0x10, 0xBD} },
	{0xBD, 0x02, {0xBD, 0x00} },
	{0xB0, 0x04, {0xB0, 0x00, 0x21, 0xBD} },
	{0xBD, 0x0F, {0xBD, 0x03, 0x00, 0x06, 0x00, 0x09,
			0x00, 0x0C, 0x00, 0x0F, 0x00, 0x15, 0x00, 0x21, 0x00} },
	{0xB0, 0x04, {0xB0, 0x00, 0x61, 0xBD} },
	{0xBD, 0x0A, {0xBD, 0x04, 0x00, 0x08, 0x00, 0x0C, 0x00, 0x10, 0x00, 0x74} },
	{0xB0, 0x04, {0xB0, 0x00, 0x12, 0xBD} },
	{0xBD, 0x02, {0xBD, 0x01} },
	{0xB0, 0x04, {0xB0, 0x00, 0x14, 0xBD} },
	{0xBD, 0x02, {0xBD, 0x00} },
	{0xB0, 0x04, {0xB0, 0x00, 0x17, 0xBD} },
	{0xBD, 0x02, {0xBD, 0x01} },
	{0xBD, 0x02, {0xBD, 0x21} },
	{0xF7, 0x02, {0xF7, 0x0F} },
	{0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	/* HOP 60HZ*/
	{0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{0x60, 0x02, {0x60, 0x01} },
	{0xB0, 0x04, {0xB0, 0x00, 0x01, 0x60} },
	{0x60, 0x02, {0x60, 0x00} },
	{0xBD, 0x02, {0xBD, 0x21} },
	{0xF7, 0x02, {0xF7, 0x0F} },
	{0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	/* Brightness Control */
	/* Dimming Setting */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x0D, 0x63} },
	{ 0x63, 0x02, {0x63, 0x00} },		/*dimming:0frame*/
	{ 0xB0, 0x04, {0xB0, 0x00, 0x0C, 0x63} },
	{ 0x63, 0x02, {0x63, 0x20} },		/*0x20:elvss dim off 0x30:elvss dim on*/
	{ 0x53, 0x02, {0x53, 0x28} },
	{ 0x51, 0x03, {0x51, 0x00, 0x00} },
	{ 0xF7, 0x02, {0xF7, 0x0F} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	/* ACL Mode */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0x55, 0x02, {0x55, 0x00} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	/* TSP SYNC ON*/
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x22, 0xB9} },
	{ 0xB9, 0x03, {0xB9, 0xB1, 0xA1} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x05, 0xF2} },
	{ 0xF2, 0x02, {0xF2, 0x52} },
	{ 0xF7, 0x02, {0xF7, 0x0F} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	/* Display On */
	{ 0x29, 0x02, {0x29, 0x00 } },

	/* seed crc setting begin*/
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xE4, 0x05, {0xE4, 0x2C, 0x2C, 0x21, 0x00} },
	{ 0xF8, 0x02, {0xF8, 0x00} },
	{ 0x1C, 0x02, {0x1C, 0x05} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x01, 0x1D} },
	{ 0x1D, 0x16, {0x1D, 0xE8, 0x05, 0x00, 0x0E, 0xE3, 0x01, 0x0E,
	  0x02, 0xE0, 0x20, 0xFF, 0xDF, 0xFF, 0x08, 0xDF, 0xF8, 0xF6, 0x00, 0xFF, 0xF7, 0xEE} },
	{ 0x1D, 0x02, {0x1D, 0x00} },
	{ 0xE4, 0x05, {0xE4, 0x2C, 0x2C, 0x01, 0x00} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	/* seed crc setting end*/
	{ 0x51, 0x03, {0x51, 0x07, 0xFF} },
};

static struct LCD_setting_table init_setting_fhd_120hz[] = {
	/* wqhd dsc 10bit pps setting start*/
	{0x9E, 0x79, {0x9E,
	0x11, 0x00,
	0x00,
	0xAB, //BIT_PER_CHANNEL
	0x30, 0x80,
	0x08, 0xE8,
	0x04, 0x38,
	0x00, 0x1E, //slice hight
	0x02, 0x1C, //slice width
	0x02, 0x1C, //check
	0x02, 0x00, //xmit delay
	0x02, 0x0E, //dec delay
	0x00, 0x20, //SCALE_VALUE
	0x03, 0x15, //INCREMENT
	0x00, 0x07, //DECREMENT
	0x00,
	0x0C,//LINE_BPG_OFFSET
	0x03, 0x19, //NFL_BPG_OFFSET
	0x03, 0x2E, //SLICE_BPG_OFFSET
	0x18, 0x00, //
	0x10, 0xF0, //
	0x07, //FLATNESS_MINQP
	0x10,//FLATNESS_MAXQP
	0x20, 0x00,//RC_MODEL_SIZE
	0x06, //RC_EDGE_FACTOR
	0x0F, //DSC_RC_QUANT_INCR_LIMIT0
	0x0F,
	0x33,//RC_TGT_OFFSET_HI RC_TGT_OFFSET_LO
	0x0E, 0x1C, 0x2A, 0x38,
	0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7B,
	0x7D, 0x7E, 0x01, 0x02, 0x22, 0x00, 0x2A, 0x40,
	0x2A, 0xBE, 0x3A, 0xFC, 0x3A, 0xFA, 0x3A, 0xF8,
	0x3B, 0x38, 0x3B, 0x78, 0x3B, 0xB6, 0x4B, 0xF6,
	0x4C, 0x34, 0x4C, 0x74, 0x5C, 0x74, 0x8C, 0xF4,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} },
	/*dsc enable*/
	{ 0x9D, 0x02, {0x9D, 0x01} },
	// /* WQHD DSC Setting end*/
	/* Sleep Out(11h) delay 10+3ms*/
	{ 0x11, 0x02, {0x11, 0x00} },
	{ REGFLAG_DELAY, 13, {} },

	/* vlin current limit,delay 110+3ms*/
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x0B, 0xB1} },
	{ 0xB1, 0x04, {0xB1, 0xFF, 0xFF, 0x0F} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x11, 0xB1} },
	{ 0xB1, 0x02, {0xB1, 0x04} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	{ REGFLAG_DELAY, 113, {} },
	/* Common Setting */
	/* TE(Vsync) ON */
	{ 0x35, 0x02, {0x35, 0x00} },

	{ 0x44, 0x03, {0x44, 0x08, 0xFC} },
	/* FIXED TE OFF */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB9, 0x02, {0xB9, 0x00} }, /*0x41:fix te on 0x00:fix te off*/
	{ 0xB0, 0x04, {0xB0, 0x00, 0x06, 0xB9} },
	/*fix te on:0C 9E 0C 9E 00 1D*/
	/*fix te off:00 00 00 00 00 00*/
	{ 0xB9, 0x07, {0xB9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} },
	/*LTPS UPDATE*/
	{ 0xF7, 0x02, {0xF7, 0x0F} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	/* PAGE ADDRESS SET 1080x2400*/
	{ 0x2A, 0x05, {0x2A, 0x00, 0x00, 0x04, 0x37} },
	{ 0x2B, 0x05, {0x2B, 0x00, 0x00, 0x09, 0x5F} },
	/* Scaler Setting */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xC3, 0x02, {0xC3, 0x89} },		/* 0x00:WQHD+(1440x3200) 0x89:FHD+(1080x2400)*/
	{ 0xB0, 0x04, {0xB0, 0x00, 0x1F, 0xC3} },
	{ 0xC3, 0x17, {0xC3, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD2, 0x00, 0xD2, 0x00,
	  0xD2, 0x00, 0xD2, 0x00, 0xD2, 0x00, 0xD2, 0x00, 0xD2, 0x00, 0xD2, 0x00, 0xD2} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x35, 0xC3} },
	{ 0xC3, 0x17, {0xC3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9B, 0x00, 0x9B, 0x00,
	  0x9B, 0x00, 0x9B, 0x00, 0x9B, 0x00, 0x9B, 0x00, 0x9B, 0x00, 0x9B, 0x00, 0x9B} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x4B, 0xC3} },
	{ 0xC3, 0x17, {0xC3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5D, 0x00, 0x5D, 0x00,
	  0x5D, 0x00, 0x5D, 0x00, 0x5D, 0x00, 0x5D, 0x00, 0x5D, 0x00, 0x5D, 0x00, 0x5D} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x61, 0xC3} },
	{ 0xC3, 0x17, {0xC3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x00, 0x28, 0x00,
	  0x28, 0x00, 0x28, 0x00, 0x28, 0x00, 0x28, 0x00, 0x28, 0x00, 0x28, 0x00, 0x28} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	// /*set aod 30hz code*/
	// { 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	// { 0xB0, 0x04, {0xB0, 0x00, 0x61, 0xBD} },
	// { 0xBD, 0x0A, {0xBD, 0x04, 0x00, 0x08, 0x00, 0x0C, 0x00, 0x10, 0x00, 0x74} },
	// { 0xB0, 0x04, {0xB0, 0x00, 0x01, 0x60} },
	// { 0x60, 0x02, {0x60, 0x00} },
	// { 0xF7, 0x02, {0xF7, 0x0F} },
	// { 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	/* Frequency Select 120HZ*/
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x08, 0xCB} },
	{ 0xCB, 0x02, {0xCB, 0x27} },			/*120hz:0x27 90hz:0xA7*/
	{ 0xB0, 0x04, {0xB0, 0x00, 0x10, 0xBD} },
	{ 0xBD, 0x02, {0xBD, 0x20} },			/*120hz:0x20 90hz:0x00*/
	{ 0xBD, 0x03, {0xBD, 0x21, 0x22} },		/*120hz:21 02 90hz:21 82*/
	{ 0x60, 0x02, {0x60, 0x00} },			/*120hz:00 90hz:08*/
	{ 0xF7, 0x02, {0xF7, 0x0F} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	/*force increase off*/
	{0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{0xBD, 0x03, {0xBD, 0x21, 0x82} },
	{0xB0, 0x04, {0xB0, 0x00, 0x10, 0xBD} },
	{0xBD, 0x02, {0xBD, 0x00} },
	{0xB0, 0x04, {0xB0, 0x00, 0x21, 0xBD} },
	{0xBD, 0x0F, {0xBD, 0x03, 0x00, 0x06, 0x00, 0x09,
			0x00, 0x0C, 0x00, 0x0F, 0x00, 0x15, 0x00, 0x21, 0x00} },
	{0xB0, 0x04, {0xB0, 0x00, 0x61, 0xBD} },
	{0xBD, 0x0A, {0xBD, 0x04, 0x00, 0x08, 0x00, 0x0C, 0x00, 0x10, 0x00, 0x74} },
	{0xB0, 0x04, {0xB0, 0x00, 0x12, 0xBD} },
	{0xBD, 0x02, {0xBD, 0x01} },
	{0xB0, 0x04, {0xB0, 0x00, 0x14, 0xBD} },
	{0xBD, 0x02, {0xBD, 0x00} },
	{0xB0, 0x04, {0xB0, 0x00, 0x17, 0xBD} },
	{0xBD, 0x02, {0xBD, 0x01} },
	{0xBD, 0x02, {0xBD, 0x21} },
	{0xF7, 0x02, {0xF7, 0x0F} },
	{0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	/* HOP 60HZ*/
	{0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	/*0x00:120HZ 0x01:60HZ 0x02:40HZ 0x03:30HZ 0x04:24HZ 0x05:20HZ 0x06:15HZ 0x07:10HZ*/
	{0x60, 0x02, {0x60, 0x00} },
	{0xB0, 0x04, {0xB0, 0x00, 0x01, 0x60} },
	{0x60, 0x02, {0x60, 0x00} },
	{0xBD, 0x02, {0xBD, 0x21} },		/*0x21:manual on 0x23:auto on*/
	{0xF7, 0x02, {0xF7, 0x0F} },
	{0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	/* Brightness Control */
	/* Dimming Setting */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x0D, 0x63} },
	{ 0x63, 0x02, {0x63, 0x00} },		/*dimming:0frame*/
	{ 0xB0, 0x04, {0xB0, 0x00, 0x0C, 0x63} },
	{ 0x63, 0x02, {0x63, 0x20} },		/*0x20:elvss dim off 0x30:elvss dim on*/
	{ 0x53, 0x02, {0x53, 0x28} },
	{ 0x51, 0x03, {0x51, 0x00, 0x00} },
	{ 0xF7, 0x02, {0xF7, 0x0F} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	/* ACL Mode */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0x55, 0x02, {0x55, 0x00} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	/* TSP SYNC ON*/
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x22, 0xB9} },
	{ 0xB9, 0x03, {0xB9, 0xB1, 0xA1} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x05, 0xF2} },
	{ 0xF2, 0x02, {0xF2, 0x52} },
	{ 0xF7, 0x02, {0xF7, 0x0F} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	/* Display On */
	{ 0x29, 0x02, {0x29, 0x00 } },

	/* seed crc setting begin*/
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xE4, 0x05, {0xE4, 0x2C, 0x2C, 0x21, 0x00} },
	{ 0xF8, 0x02, {0xF8, 0x00} },
	{ 0x1C, 0x02, {0x1C, 0x05} },
	{ 0xB0, 0x04, {0xB0, 0x00, 0x01, 0x1D} },
	{ 0x1D, 0x16, {0x1D, 0xE8, 0x05, 0x00, 0x0E, 0xE3, 0x01, 0x0E,
	  0x02, 0xE0, 0x20, 0xFF, 0xDF, 0xFF, 0x08, 0xDF, 0xF8, 0xF6, 0x00, 0xFF, 0xF7, 0xEE} },
	{ 0x1D, 0x02, {0x1D, 0x00} },
	{ 0xE4, 0x05, {0xE4, 0x2C, 0x2C, 0x01, 0x00} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	/* seed crc setting end*/
	{ 0x51, 0x03, {0x51, 0x07, 0xFF} },
};
static struct LCD_setting_table cmd_bl_level[] = {
	{ 0x51, 0x03, {0x51, 0x07, 0xFF} },
	{ REGFLAG_END_OF_TABLE, 0x00, { } },
};

static struct LCD_setting_table cmd_test[] = {
	{ 0x9F, 0x03, {0x9F, 0xA5, 0xA5} },
	{ 0x23, 0x01, {0x23}},
	{ 0x51, 0x03, {0x51, 0x07, 0xFF} },
	{ REGFLAG_END_OF_TABLE, 0x00, { } },
};

static struct LCD_setting_table lcm_suspend_setting[] = {
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0x28, 0x02, {0x28, 0x00} },
	{ REGFLAG_DELAY, 20, { } },
	{ 0x10, 0x02, {0x10, 0x00} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	{ REGFLAG_DELAY, 100, { } },
};

static struct LCD_setting_table cmd_brightness_dimming_off[] = {
	{ 0x53, 0x02, {0x53, 0x20} },
};

static struct LCD_setting_table cmd_brightness_dimming_on[] = {
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x02, {0xB0, 0x06} },
	{ 0xB7, 0x02, {0xB7, 0x3C} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	{ 0x53, 0x02, {0x53, 0x28} },
};

/*************hbm mode config begin*************************/
static struct LCD_setting_table cmd_hbm_ud_off[] = {
	{ 0x53, 0x02, {0x53, 0x20} },
	{ 0xB0, 0x02, {0xB0, 0x05} },
	{ 0xB7, 0x02, {0xB7, 0x13} },
	/* delay on */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x02, {0xB0, 0x01} },
	{ 0xB7, 0x02, {0xB7, 0x44} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
};

static struct LCD_setting_table cmd_hbm_ud_on[] = {
	/* dimming off */
	{ 0x53, 0x02, {0x53, 0x20} },
	/* elvss dimming off */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x02, {0xB0, 0x05} },
	{ 0xB7, 0x02, {0xB7, 0x13} },
	/* delay off */
	{ 0xB0, 0x02, {0xB0, 0x01} },
	{ 0xB7, 0x02, {0xB7, 0x4C} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	{ 0x53, 0x02, {0x53, 0xE0} },
	{ 0x51, 0x03, {0x51, 0x07, 0xFF} },
};

static struct LCD_setting_table cmd_hbm_level1[] = {
	/* elvss0x0 dimming on */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x02, {0xB0, 0x05} },
	{ 0xB7, 0x02, {0xB7, 0x93} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	{ 0x53, 0x02, {0x53, 0xE8} },
	{ 0x51, 0x03, {0x51, 0x00, 0xE5} },
};

static struct LCD_setting_table cmd_hbm_level2[] = {
	/* elvss dimming on */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x02, {0xB0, 0x05} },
	{ 0xB7, 0x02, {0xB7, 0x93} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	{ 0x53, 0x02, {0x53, 0xE8} },
	{ 0x51, 0x03, {0x51, 0x01, 0xC5} },
};

static struct LCD_setting_table cmd_hbm_level3[] = {
	/* elvss dimming on */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x02, {0xB0, 0x05} },
	{ 0xB7, 0x02, {0xB7, 0x93} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	{ 0x53, 0x02, {0x53, 0xE8} },
	{ 0x51, 0x03, {0x51, 0x02, 0x86} },
};

static struct LCD_setting_table cmd_hbm_level4[] = {
	/* elvss dimming on */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x02, {0xB0, 0x05} },
	{ 0xB7, 0x02, {0xB7, 0x93} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	{ 0x53, 0x02, {0x53, 0xE8} },
	{ 0x51, 0x03, {0x51, 0x03, 0x43} },
};

static struct LCD_setting_table cmd_hbm_level5[] = {
	/* elvss dimming on */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x02, {0xB0, 0x05} },
	{ 0xB7, 0x02, {0xB7, 0x93} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	{ 0x53, 0x02, {0x53, 0xE8} },
	{ 0x51, 0x03, {0x51, 0x04, 0x0B} },
};

static struct LCD_setting_table cmd_hbm_level6[] = {
	/* elvss dimming on */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x02, {0xB0, 0x05} },
	{ 0xB7, 0x02, {0xB7, 0x93} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	{ 0x53, 0x02, {0x53, 0xE8} },
	{ 0x51, 0x03, {0x51, 0x04, 0xCB} },
};

static struct LCD_setting_table cmd_hbm_hdr10[] = {
	/* elvss dimming on */
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x02, {0xB0, 0x05} },
	{ 0xB7, 0x02, {0xB7, 0x93} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	{ 0x53, 0x02, {0x53, 0xE8} },
};

static struct LCD_setting_table cmd_set_fps_120hz[] = {
	{ 0x60, 0x02, {0x60, 0x10} },
};

static struct LCD_setting_table cmd_set_fps_60hz[] = {
	{ 0x60, 0x02, {0x60, 0x00} },
};

static struct LCD_setting_table cmd_aod_on[] = {
	{ 0x28, 0x02, {0x28, 0x00} },
	{ REGFLAG_DELAY, 17, { } },
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0x60, 0x02, {0x60, 0x00} },
	{ 0xD4, 0x02, {0xD4, 0xCB} },
	{ 0xB0, 0x02, {0xB0, 0x0B} },
	{ 0xD8, 0x03, {0xD8, 0x00, 0x00} },
	{ 0x53, 0x02, {0x53, 0x22} },
	{ 0xE3, 0x02, {0xE3, 0x00} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },

	/* Display On */
	{ 0x29, 0x02, {0x29, 0x00} },
};

static struct LCD_setting_table cmd_aod_off[] = {
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x02, {0xB0, 0x0B} },
	{ 0xD8, 0x03, {0xD8, 0x09, 0x60} },
	{ 0x60, 0x02, {0x60, 0x00} },
	{ 0x53, 0x02, {0x53, 0x28} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
	{ REGFLAG_DELAY, 17, { } },
};

static struct LCD_setting_table cmd_aod_off_pre[] = {
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0x60, 0x02, {0x60, 0x00} },
	{ 0x53, 0x02, {0x53, 0x28} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
};

static struct LCD_setting_table cmd_aod_off_post[] = {
	{ REGFLAG_DELAY, 34, { } },
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0xB0, 0x02, {0xB0, 0x0B} },
	{ 0xD8, 0x03, {0xD8, 0x09, 0x60} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
};

/* OLED CRC mode P3 */
static struct LCD_setting_table seed_crc_on_command_p3[] = {
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0x81, 0x02, {0x81, 0x90} },
	{ 0xB1, 0x17, {0xB1,
	  0x00, 0xE8, 0x05, 0x00, 0x0A, 0xE3, 0x01, 0x0E, 0x02, 0xE0,
	  0x24, 0xFF, 0xE1, 0xFF, 0x0C, 0xE4, 0xF8, 0xF6, 0x01, 0xFF,
	  0xFF, 0xFF} },
	{ 0xB0, 0x02, {0xB0, 0x01} },
	{ 0xC2, 0x02, {0xC2, 0x07} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
};

/* OLED CRC mode SRGB*/
static struct LCD_setting_table seed_crc_on_command_srgb[] = {
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0x81, 0x02, {0x81, 0x90} },
	{ 0xB1, 0x17, {0xB1,
	  0x00, 0xE5, 0x0C, 0x04, 0x35, 0xEF, 0x0E, 0x08, 0x02, 0xE0,
	  0x40, 0xF0, 0xCD, 0xE0, 0x10, 0xD8, 0xFF, 0xFE, 0x13, 0xFF,
	  0xFB, 0xCE} },
	{ 0xB0, 0x02, {0xB0, 0x01} },
	{ 0xC2, 0x02, {0xC2, 0x27} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
};

/* OLED CRC mode off*/
static struct LCD_setting_table seed_crc_off_command[] = {
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0x81, 0x02, {0x81, 0x00} },
	{ 0xB1, 0x02, {0xB1, 0x01} },
	{ 0xB0, 0x02, {0xB0, 0x01} },
	{ 0xC2, 0x02, {0xC2, 0x07} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
};

/* seed crc off + flat mode */
static struct LCD_setting_table seed_crc_off_flat_command[] = {
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0x81, 0x02, {0x81, 0x00} },
	{ 0xB1, 0x02, {0xB1, 0x01} },
	{ 0xB0, 0x02, {0xB0, 0x01} },
	{ 0xC2, 0x02, {0xC2, 0x27} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
};

/* p3 seed crc compenstate default 550~2047 */
static struct LCD_setting_table sm3010a_seed_crc_compensate_p3_level0_cmd[] = {
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0x81, 0x02, {0x81, 0x90} },
	{ 0xB1, 0x06, {0xB1, 0x00, 0xE8, 0x05, 0x00, 0x0A} },
	{ 0xB0, 0x02, {0xB0, 0x05} },
	{ 0xB1, 0x06, {0xB1, 0xE3, 0x01, 0x0E, 0x02, 0xE0} },
	{ 0xB0, 0x02, {0xB0, 0x0A} },
	{ 0xB1, 0x06, {0xB1, 0x24, 0xFF, 0xE1, 0xFF, 0x0C} },
	{ 0xB0, 0x02, {0xB0, 0x0F} },
	{ 0xB1, 0x05, {0xB1, 0xE4, 0xF8, 0xF6, 0x01} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
};

/* p3 seed crc compenstate 100nit 250~550 */
static struct LCD_setting_table sm3010a_seed_crc_compensate_p3_level1_cmd[] = {
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0x81, 0x02, {0x81, 0x90} },

	{ 0xB1, 0x06, {0xB1, 0x00, 0xE8, 0x04, 0x00, 0x08} },
	{ 0xB0, 0x02, {0xB0, 0x05} },
	{ 0xB1, 0x06, {0xB1, 0xE3, 0x01, 0x0B, 0x01, 0xE0} },
	{ 0xB0, 0x02, {0xB0, 0x0A} },
	{ 0xB1, 0x06, {0xB1, 0x1D, 0xFF, 0xDB, 0xFF, 0x09} },
	{ 0xB0, 0x02, {0xB0, 0x0F} },
	{ 0xB1, 0x05, {0xB1, 0xE1, 0xF5, 0xF9, 0x01} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
};

/* p3 seed crc compenstate 20nit 50~250 */
static struct LCD_setting_table sm3010a_seed_crc_compensate_p3_level2_cmd[] = {
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0x81, 0x02, {0x81, 0x90} },
	{ 0xB1, 0x06, {0xB1, 0x00, 0xE8, 0x04, 0x00, 0x07} },
	{ 0xB0, 0x02, {0xB0, 0x05} },
	{ 0xB1, 0x06, {0xB1, 0xE3, 0x01, 0x0B, 0x01, 0xE0} },
	{ 0xB0, 0x02, {0xB0, 0x0A} },
	{ 0xB1, 0x06, {0xB1, 0x19, 0xFF, 0xDA, 0xFF, 0x08} },
	{ 0xB0, 0x02, {0xB0, 0x0F} },
	{ 0xB1, 0x05, {0xB1, 0xDF, 0xF5, 0xF9, 0x01} },
	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
};

/* p3 seed crc compenstate 3nit 4~50 */
static struct LCD_setting_table sm3010a_seed_crc_compensate_p3_level3_cmd[] = {
	{ 0xF0, 0x03, {0xF0, 0x5A, 0x5A} },
	{ 0x81, 0x02, {0x81, 0x90} },

	{ 0xB1, 0x06, {0xB1, 0x00, 0xE8, 0x03, 0x00, 0x06} },
	{ 0xB0, 0x02, {0xB0, 0x05} },
	{ 0xB1, 0x06, {0xB1, 0xE3, 0x01, 0x0B, 0x01, 0xE0} },
	{ 0xB0, 0x02, {0xB0, 0x0A} },
	{ 0xB1, 0x06, {0xB1, 0x17, 0xFF, 0xD9, 0xFF, 0x08} },
	{ 0xB0, 0x02, {0xB0, 0x0F} },
	{ 0xB1, 0x05, {0xB1, 0xDF, 0xF5, 0xF9, 0x01} },

	{ 0xF0, 0x03, {0xF0, 0xA5, 0xA5} },
};

/* srgb seed crc compenstate default 550~2047 */
static struct LCD_setting_table sm3010a_seed_crc_compensate_srgb_level0_cmd[] = {
};

/* srgb seed crc compenstate 100nit 250~550 */
static struct LCD_setting_table sm3010a_seed_crc_compensate_srgb_level1_cmd[] = {
};

/* srgb seed crc compenstate 20nit 50~250 */
static struct LCD_setting_table sm3010a_seed_crc_compensate_srgb_level2_cmd[] = {
};

/* srgb seed crc compenstate 3nit 4~50 */
static struct LCD_setting_table sm3010a_seed_crc_compensate_srgb_level3_cmd[] = {
};

#endif
