/*
 * Copyright (C) 2015 HUAQIN Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*

Here is WL2866D Interface,
WL2866D Driver is in vnd\kernel-5.10\drivers\misc\mediatek\wl2866d

*/

#include "wl2866d_hw.h"
extern int wl2866d_set_ldo_enable(enum WL2866D_SELECT, uint32_t);
extern int wl2866d_set_ldo_disable(enum WL2866D_SELECT);
extern void wl2866d_set_registers_init(void);

/*****************************************************************************
 * Static Var
 *****************************************************************************/
static const uint32_t pin_state_vol_table[] = {
    WL2866D_EXTLDO_REGULATOR_VOLTAGE_0,
    WL2866D_EXTLDO_REGULATOR_VOLTAGE_1000,
    WL2866D_EXTLDO_REGULATOR_VOLTAGE_1050,
    WL2866D_EXTLDO_REGULATOR_VOLTAGE_1100,
    WL2866D_EXTLDO_REGULATOR_VOLTAGE_1200,
    WL2866D_EXTLDO_REGULATOR_VOLTAGE_1210,
    WL2866D_EXTLDO_REGULATOR_VOLTAGE_1220,
    WL2866D_EXTLDO_REGULATOR_VOLTAGE_1500,
    WL2866D_EXTLDO_REGULATOR_VOLTAGE_1800,
    WL2866D_EXTLDO_REGULATOR_VOLTAGE_2200,
    WL2866D_EXTLDO_REGULATOR_VOLTAGE_2500,
    WL2866D_EXTLDO_REGULATOR_VOLTAGE_2800,
    WL2866D_EXTLDO_REGULATOR_VOLTAGE_2900,
    WL2866D_EXTLDO_REGULATOR_VOLTAGE_HIGH
};

static struct wl2866d_ldomap ldolist[] = {
    //main 50M
    {IMGSENSOR_SENSOR_IDX_MAIN, AVDD, MAIN_AVDD1}, //BackMain AVDD
    {IMGSENSOR_SENSOR_IDX_MAIN, DVDD, MAIN_DVDD1}, //BackMain DVDD
    //{IMGSENSOR_SENSOR_IDX_MAIN, DOVDD, MAIN_DOVDD}, //BackMain DOVDD
    //{IMGSENSOR_SENSOR_IDX_MAIN, AFVDD, MAIN_AFVDD}, //BackMain AFVDD

    //front 16M
    {IMGSENSOR_SENSOR_IDX_SUB, AVDD, FRONT_AVDD2}, //FrontMain AVDD
    {IMGSENSOR_SENSOR_IDX_SUB, DVDD, FRONT_DVDD2}, //FrontMain DVDD
    //{IMGSENSOR_SENSOR_IDX_SUB, DOVDD, MT6377_VTP_DOVDD}, // mt6377 pmu vtp for frontcam dovdd

	 //mono 2M
    //{IMGSENSOR_SENSOR_IDX_MAIN2, AVDD, REAR_2M_AVDD}, //BackMono AVDD
    //{IMGSENSOR_SENSOR_IDX_MAIN2, DOVDD, MT6377_VTP_DOVDD}, // mt6377 pmu vtp for 2m dovdd
};


static char* camera_power_pin_name[IMGSENSOR_HW_PIN_MAX_NUM];
/*****************************************************************************
 * Static Fun
 *****************************************************************************/

static enum IMGSENSOR_RETURN wl2866d_hw_init(void *pinstance, struct IMGSENSOR_HW_DEVICE_COMMON *pcommon)
{
    wl2866d_set_registers_init();
    WL2866D_PRINT("[wl2866d_hw] %s in.\n", __FUNCTION__);
    return IMGSENSOR_RETURN_SUCCESS;
}



static enum IMGSENSOR_RETURN wl2866d_hw_set(
    void *pinstance,
    enum IMGSENSOR_SENSOR_IDX   sensor_idx,
    enum IMGSENSOR_HW_PIN       pin,
    enum IMGSENSOR_HW_PIN_STATE pin_state)
{
    int ret = IMGSENSOR_RETURN_SUCCESS;
    enum WL2866D_SELECT ldonum = WL2866D_LDO_NONE;
    unsigned int i = 0;

    // var init
    camera_power_pin_name[AVDD] = "AVDD";
    camera_power_pin_name[DVDD] = "DVDD";
    camera_power_pin_name[DOVDD] = "DOVDD";
    camera_power_pin_name[AFVDD] = "AFVDD";

    WL2866D_PRINT("[wl2866d_hw] %s stoneadd pin=%s sensor_idx=%d pin_state=%d\n", __FUNCTION__, camera_power_pin_name[pin], sensor_idx, pin_state);

    if (sensor_idx < IMGSENSOR_SENSOR_IDX_MIN_NUM ||
            sensor_idx >= IMGSENSOR_SENSOR_IDX_MAX_NUM){
        WL2866D_PRINT("[wl2866d_hw] error sensor_idx:%d (%d ~ d%)", sensor_idx, IMGSENSOR_SENSOR_IDX_MIN_NUM,IMGSENSOR_SENSOR_IDX_MAX_NUM);
        return IMGSENSOR_RETURN_ERROR;
    } else if (pin < IMGSENSOR_HW_PIN_AVDD ||
            pin > IMGSENSOR_HW_PIN_DOVDD) {
        WL2866D_PRINT("[wl2866d_hw] error pin:%d (%d ~ d%)", pin, IMGSENSOR_HW_PIN_AVDD, IMGSENSOR_HW_PIN_DOVDD);
        return IMGSENSOR_RETURN_ERROR;
    } else if ( pin_state < IMGSENSOR_HW_PIN_STATE_LEVEL_0 ||
            pin_state >= IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH) {
        WL2866D_PRINT("[wl2866d_hw] error pin_state:%d (%d ~ d%)", pin_state, IMGSENSOR_HW_PIN_STATE_LEVEL_0,IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH);
        return IMGSENSOR_RETURN_ERROR;
    }

    for(i = 0; i < (sizeof(ldolist) / sizeof(ldolist[0])); i++) {
        if(sensor_idx == ldolist[i].sensor_index && pin == ldolist[i].seq_type) {
             ldonum = ldolist[i].ldo_selected;
             WL2866D_PRINT("[wl2866d_hw] %s sensor %d, seq_type = %d matched ldo %d\n", __FUNCTION__, sensor_idx, pin, ldonum + 1);
             break;
         }
    }

    if((ldonum < MAIN_DVDD1) || (ldonum > FRONT_AVDD2)) {
        WL2866D_PRINT("[wl2866d_hw] %s ldo setting not found in ldolist!!!\n",__FUNCTION__);
        return IMGSENSOR_RETURN_ERROR;
    }

    if(pin_state > IMGSENSOR_HW_PIN_STATE_LEVEL_0) {
        wl2866d_set_ldo_enable(ldonum, pin_state_vol_table[pin_state]);
        WL2866D_PRINT("[wl2866d_hw] cameraid%d poweron %s, LDO%d\n", sensor_idx,
                camera_power_pin_name[pin], ldonum + 1);
    } else {
        wl2866d_set_ldo_disable(ldonum);
        WL2866D_PRINT("[wl2866d] cameraid%d poweroff %s, LDO%d.\n", sensor_idx,
                camera_power_pin_name[pin], ldonum + 1);
    }

    return ret;
}

static enum IMGSENSOR_RETURN wl2866d_hw_release(void *instance)
{
    return IMGSENSOR_RETURN_SUCCESS;
}

/*
 * IMGSENSOR_HW_DEVICE obj will be called by imgsensor_hw.c
*/
static struct IMGSENSOR_HW_DEVICE wl2866d_device = {
    .init      = wl2866d_hw_init,
    .set       = wl2866d_hw_set,
    .release   = wl2866d_hw_release,
    .id        = IMGSENSOR_HW_ID_WL2866D
};

enum IMGSENSOR_RETURN imgsensor_hw_wl2866d_open(
    struct IMGSENSOR_HW_DEVICE **pdevice)
{
	*pdevice = &wl2866d_device;
    return IMGSENSOR_RETURN_SUCCESS;
}
