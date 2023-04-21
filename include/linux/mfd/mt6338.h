/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef __MT6338_H__
#define __MT6338_H__

#include <linux/regmap.h>

#define MT6338_PMIC_SLAVEID	(0x6B)

struct mt6338_pmic_info {
	struct i2c_client *i2c;
	struct device *dev;
	struct regmap *regmap;
	unsigned int chip_rev;
	struct mutex io_lock;
/* #ifdef OPLUS_ARCH_EXTENDS */
	unsigned int scl_drv;
	unsigned int sda_drv;
/* #endif */
};

/* #ifdef OPLUS_ARCH_EXTENDS */
static const unsigned int i2c_drv[] = {
	0x8,  //1mA
	0x28, //2mA
	0x9,  //3mA
	0x29, //4mA
	0xa,  //5mA
	0x2a, //6mA
	0xb,  //7mA
	0x2b, //8mA
	0xc,  //9mA
	0x2c, //10mA
	0xd,  //11mA
	0x2d, //12mA
	0xe,  //13mA
	0x2e, //14mA
	0xf,  //15mA
	0x2f, //16 mA
};
/* #endif */

#endif /* __MT6338_H__ */
