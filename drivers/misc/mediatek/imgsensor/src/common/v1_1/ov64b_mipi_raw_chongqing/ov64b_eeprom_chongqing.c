// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "ov64b_eeprom_chongqing.h"

#define PFX "ov64b_eeprom_chongqing"
#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)

kal_uint16 Eeprom_1ByteDataRead(kal_uint16 addr, kal_uint16 slaveaddr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 1, slaveaddr);
    return get_byte;
}
