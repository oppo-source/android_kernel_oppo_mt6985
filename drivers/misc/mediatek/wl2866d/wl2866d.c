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

Here is WL2866d Driver,
wl2866d Interface is in vnd\kernel-5.10\drivers\misc\mediatek\imgsensor\src\common\v1_1\camera_hw\wl2866d_hw

*/

#include "wl2866d.h"
#include <linux/reboot.h>
/*****************************************************************************
 * Static Var
 *****************************************************************************/
static int ldo_id = 0;
static int power_reference_counts[] = {0,0,0,0};
static struct wl2866d_i2c_device which_ldo_chip[] = {
    //ldo i2c addr,         chip id addr,           chip id,             enable addr,
    // zhulu
	{WL2866D_LDO_I2C_ADDR, WL2866D_CHIP_REV_ADDR, CAMERA_LDO_WL2866D, WL2866D_LDO_EN_ADDR},
	{ET5904_LDO_I2C_ADDR, ET5904_CHIP_REV_ADDR, CAMERA_LDO_ET5904, ET5904_LDO_EN_ADDR},
};
static struct i2c_client *wl2866d_i2c_client=NULL;
static struct pinctrl *wl2866d_pctrl=NULL; /* static pinctrl instance */
struct mutex i2c_control_mutex;
struct notifier_block ldo_shutdown;
static const char *wl2866d_state_name[WL2866D_GPIO_STATE_MAX] = {
    "wl2866d_gpio_en0",
    "wl2866d_gpio_en1",
    "wl2866d_gpio_ext_buck_en0",
    "wl2866d_gpio_ext_buck_en1"
};/* GPIO state mapping name */
static const struct of_device_id gpio_of_match[] = {
    { .compatible = "mediatek,gpio_wl2866d", },
    {},
};
MODULE_DEVICE_TABLE(of, gpio_of_match);
static const struct platform_device_id wl2866d_gpio_id[] = {
    {"WL2866D_GPIO", 0},
    {},
};

static const struct of_device_id i2c_of_match[] = {
    { .compatible = "mediatek,i2c_wl2866d", },
    {},
};
MODULE_DEVICE_TABLE(of, i2c_of_match);
static const struct i2c_device_id wl2866d_i2c_id[] = {
    {"WL2866D_I2C", 0},
    {},
};
/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int wl2866d_gpio_probe(struct platform_device *pdev);
static int wl2866d_gpio_remove(struct platform_device *pdev);
static int wl2866d_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int wl2866d_i2c_remove(struct i2c_client *client);
/*****************************************************************************
 * Sub Function
 *****************************************************************************/
static int wl2866d_write(unsigned short addr, unsigned short value){
    return i2c_smbus_write_byte_data(wl2866d_i2c_client, addr, value);
}

static int wl2866d_read(unsigned short addr){
    return i2c_smbus_read_byte_data(wl2866d_i2c_client, addr);
}

static long wl2866d_set_state(const char *name)
{
    int ret = 0;
    struct pinctrl_state *pState = 0;

    pState = pinctrl_lookup_state(wl2866d_pctrl, name);
    if (IS_ERR(pState)) {
        pr_err("set state '%s' failed\n", name);
        ret = PTR_ERR(pState);
        goto exit;
    }

    /* select state! */
    pinctrl_select_state(wl2866d_pctrl, pState);

exit:
    return ret; /* Good! */
}

static void wl2866d_gpio_select_state(enum WL2866D_GPIO_STATE s)
{
    WL2866D_PRINT("[wl2866d]%s,%d\n",__FUNCTION__,s);

    wl2866d_set_state(wl2866d_state_name[s]);
}

static void wl2866d_set_ext_buck_en(enum WL2866D_SELECT ldonum, unsigned int en){
    //zhulu
    if (ldonum == MAIN_DVDD1 || ldonum == FRONT_DVDD2) {
        switch (en){
            case 0:
                if (power_reference_counts[0] | power_reference_counts[1]);
                else wl2866d_gpio_select_state(WL2866D_GPIO_EXT_BUCK_EN0);
                WL2866D_PRINT("[wl2866d] %s close WL2866D_GPIO_EXT_BUCK_EN for LDO1 & LDO2\n",__FUNCTION__);
                break;
            case 1:
                wl2866d_gpio_select_state(WL2866D_GPIO_EXT_BUCK_EN1);
                WL2866D_PRINT("[wl2866d] %s open WL2866D_GPIO_EXT_BUCK_EN for LDO1 & LDO2\n",__FUNCTION__);
                break;
            default:
                break;
        }
    }
}

static void wl2866d_set_en_ldo(enum WL2866D_SELECT ldonum, unsigned int en)
{
    int ret = 0;
    unsigned int value =0;

    if (NULL == wl2866d_i2c_client) {
            WL2866D_PRINT("[wl2866d] wl2866d_i2c_client is null!!\n");
            return ;
    }

    if(ldonum < MAIN_DVDD1 || ldonum > FRONT_AVDD2) {
        WL2866D_PRINT("[wl2866d] %s error ldonum not support!!!\n",__FUNCTION__);
        return;
    }

    ret = wl2866d_read(which_ldo_chip[ldo_id].enable_addr);
    if (ret < 0)
    {
        WL2866D_PRINT("[wl2866d] wl2866d_set_en_ldo read error!\n");
        return;
    }

    if(en == 0)
    {
        value = (ret & (~(0x01<<ldonum)));
    }
    else
    {
        if(which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_WL2866D) {
            value = (ret|(0x01<<ldonum));
        }
    }

    ret = wl2866d_write(which_ldo_chip[ldo_id].enable_addr, value);
    if (ret < 0)
    {
        WL2866D_PRINT("[wl2866d] %s write error.\n", __FUNCTION__);
        return;
    }
    WL2866D_PRINT("[wl2866d] wl2866d_set_en_ldo enable before:%x after set :%x\n", ret, value);
    return;

}

static void wl2866d_set_ldo_value(enum WL2866D_SELECT ldonum,unsigned int value)
{
    unsigned int   Dvdd_out = 0;
    unsigned int   Avdd_out = 0;
    unsigned short regaddr = 0;
    int ret = 0;

    WL2866D_PRINT("[wl2866d] %s begin\n",__FUNCTION__);

    if (NULL == wl2866d_i2c_client || ldonum > FRONT_AVDD2 || ldonum < MAIN_DVDD1) {
        WL2866D_PRINT("[wl2866d] (wl2866d_i2c_client is null) or ldonum not support! ldonum = %d \n",ldonum);
        return ;
    }

    switch(ldonum)
    {
        case MAIN_DVDD1:
            if (which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_WL2866D||which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_ET5904) {
                if (value < 600) {
                    WL2866D_PRINT("[wl2866d] DVDD1 error vol!!!\n");
                    goto exit;
                } else {
                    Dvdd_out =(value - 600)/6+2;
                }
            }
            break;
        case FRONT_DVDD2:
            if (which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_WL2866D||which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_ET5904) {
                if (value < 600) {
                    WL2866D_PRINT("[wl2866d] error vol!!!\n");
                    goto exit;
                } else {
                    Dvdd_out =(value - 600)/6;
                }
            }
            break;
        case MAIN_AVDD1:
        case FRONT_AVDD2:
            if(which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_WL2866D||which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_ET5904) {
                if(value < 1200)
                {
                    WL2866D_PRINT("[wl2866d] error vol!!!\n");
                    goto exit;
                }
                else
                {
                    Avdd_out = (value - 1200)*10/125;
                }
            }
            break;
        default:
            goto exit;
        break;
    }

    if(which_ldo_chip[ldo_id].ldoId == CAMERA_LDO_WL2866D) {
        WL2866D_PRINT("[wl2866d] CAMERA_LDO_WL2866D ldonum: = %d\n",ldonum);
        regaddr = ldonum + WL2866D_LDO1_OUT_ADDR;
    }

    if ( MAIN_DVDD1 == ldonum || FRONT_DVDD2 == ldonum) {
        ret = wl2866d_write(regaddr, Dvdd_out);
        WL2866D_PRINT("[wl2866d] %s write regaddr = 0x%x, ldo_out = %d\n", __FUNCTION__, regaddr, Dvdd_out);
    } else {
        ret = wl2866d_write(regaddr, Avdd_out);
        WL2866D_PRINT("[wl2866d] %s write regaddr = 0x%x, ldo_out = %d\n", __FUNCTION__, regaddr, Avdd_out);
    }

    if(ret < 0){
        WL2866D_PRINT("[wl2866d] %s write error.\n", __FUNCTION__);
        return;
    }
    WL2866D_PRINT("[wl2866d] %s write regaddr = 0x%x, ldo_out = %d\n", __FUNCTION__, regaddr, Avdd_out);
    ret = wl2866d_read(regaddr);
    WL2866D_PRINT("[wl2866d] %s read validate ret = %d\n", __FUNCTION__, ret);
    return;

exit:
    WL2866D_PRINT("[wl2866d] %s exit err\n",__FUNCTION__);
}

void wl2866d_set_registers_init(void)
{
    int i = 0;
    unsigned short regaddr = 0x00;
    for (i = 0; i < 16; i++){
        wl2866d_write(regaddr, 0);
        regaddr++;
    }
    return;
}
EXPORT_SYMBOL(wl2866d_set_registers_init);
/*
int wl2866d_ldo_test(enum WL2866D_SELECT ldonum, uint32_t voltage){
   wl2866d_set_ext_buck_en(ldonum, 1);   // PMU EN
   wl2866d_set_ldo_value(ldonum, voltage);
   wl2866d_set_en_ldo(ldonum, 1);
   WL2866D_PRINT("[wl2866d] %s wl2866d_ldo_test ldonum:%d , (voltage is vail)voltage:%d\n",__FUNCTION__, ldonum, voltage);
   return 1;
}
*/
/*****************************************************************************
 * Extern Area
 *****************************************************************************/
int wl2866d_set_ldo_enable(enum WL2866D_SELECT ldonum, uint32_t voltage)
{
    unsigned int ldo_vol_value = 0;

    mutex_lock(&i2c_control_mutex);

    if(ldonum < MAIN_DVDD1 || ldonum > WL2866D_LDO_MAX) {
        WL2866D_PRINT("[wl2866d] %s error ldonum %d not support!!!\n",__FUNCTION__, ldonum);
        return -2;
    }

    ldo_vol_value = voltage;
    power_reference_counts[ldonum] += 1;
    if(power_reference_counts[ldonum] > 1){
        WL2866D_PRINT("[wl2866d]: LDO_%d poweron already!\n", ldonum + 1);
    } else {
        wl2866d_set_ext_buck_en(ldonum, 1);
        wl2866d_set_ldo_value(ldonum, ldo_vol_value);
        wl2866d_set_en_ldo(ldonum, 1);
        WL2866D_PRINT("[wl2866d]: LDO_%d %dmV poweron.\n", ldonum + 1, ldo_vol_value);
    }

    WL2866D_PRINT("[wl2866d] power counts array:[%d, %d, %d, %d, %d, %d, %d, %d]\n",
                                                power_reference_counts[0],power_reference_counts[1],
                                                power_reference_counts[2],power_reference_counts[3]);
    mutex_unlock(&i2c_control_mutex);
    return 0;
}
EXPORT_SYMBOL(wl2866d_set_ldo_enable);

int wl2866d_set_ldo_disable(enum WL2866D_SELECT ldonum)
{
    if(ldonum < MAIN_DVDD1 || ldonum > WL2866D_LDO_MAX) {
        WL2866D_PRINT("[wl2866d] %s ldo %d setting not found in ldolist!!!\n",__FUNCTION__, ldonum);
        return -2;
    }

    mutex_lock(&i2c_control_mutex);
    wl2866d_write(0x02, 0x8F);

    power_reference_counts[ldonum] -= 1;
    if(power_reference_counts[ldonum] == 0){
        wl2866d_set_en_ldo(ldonum, 0);
        wl2866d_set_ext_buck_en(ldonum, 0);
        WL2866D_PRINT("[wl2866d]: LDO_%d poweroff.\n", ldonum + 1);
    }else{
        WL2866D_PRINT("[wl2866d]: LDO_%d will not poweroff, still in use!\n", ldonum + 1);
    }
    WL2866D_PRINT("[wl2866d] power counts array:[%d, %d, %d, %d, %d, %d, %d, %d]\n",
                                                power_reference_counts[0],power_reference_counts[1],
                                                power_reference_counts[2],power_reference_counts[3]);
    mutex_unlock(&i2c_control_mutex);
    return 0;
}
EXPORT_SYMBOL(wl2866d_set_ldo_disable);

/*****************************************************************************
 * Driver Structure
 *****************************************************************************/
static struct platform_driver wl2866d_platform_driver = {
    .id_table = wl2866d_gpio_id,
    .probe = wl2866d_gpio_probe,
    .remove = wl2866d_gpio_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "WL2866D_GPIO",
        .of_match_table = gpio_of_match,
    },
};

static struct i2c_driver wl2866d_i2c_driver = {
/************************************************************
Attention:
Althouh i2c_bus do not use .id_table to match, but it must be defined,
otherwise the probe function will not be executed!
************************************************************/
    .id_table = wl2866d_i2c_id,
    .probe = wl2866d_i2c_probe,
    .remove = wl2866d_i2c_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "WL2866D_I2C",
        .of_match_table = i2c_of_match,
    },
};

/*****************************************************************************
 * Driver Member Function
 *****************************************************************************/
static long wl2866d_gpio_init(struct platform_device *pdev)
{
    int ret = 0;
    struct pinctrl *pctrl;

    /* retrieve */
    pctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR(pctrl)) {
        dev_err(&pdev->dev, "Cannot find disp pinctrl!");
        ret = PTR_ERR(pctrl);
        goto exit;
    }

    wl2866d_pctrl = pctrl;
    WL2866D_PRINT("[wl2866d]wl2866d_gpio_init wl2866d_pctrl:%p\n", wl2866d_pctrl);

exit:
    return ret;
}

static int wl2866d_gpio_probe(struct platform_device *pdev)
{
    int ret = 0;

    ret = wl2866d_gpio_init(pdev);
    if (ret) {
        WL2866D_PRINT("[wl2866d]wl2866d_gpio_probe failed\n");
        return ret;
    }
    WL2866D_PRINT("[wl2866d] wl2866d_gpio_probe success\n");

    return 0;
}

static int wl2866d_gpio_remove(struct platform_device *pdev)
{
    platform_driver_unregister(&wl2866d_platform_driver);

    return 0;
}
static int ldo_shutdown_notifier_callback(struct notifier_block *np, unsigned long type, void *_unused)
{
        switch (type) {
        case SYS_DOWN:
                WL2866D_PRINT("[wl2866d]ldo sys_down ");

        case SYS_POWER_OFF:
                WL2866D_PRINT("[wl2866d] reboot_notify: SYS_POWER_OFF!\n");
                wl2866d_write(0x0E,0x00);
                break;

        case SYS_HALT:
                WL2866D_PRINT("[wl2866d] reboot_notify: SYS_HALT !\n");
                break;

        default:
                WL2866D_PRINT("[wl2866d] reboot_notify: default !\n");
                break;
        }
        return NOTIFY_OK;
}
static struct notifier_block ldo_shutdown_notifier = {
        .notifier_call = ldo_shutdown_notifier_callback,
        .priority = 128,
};

static int wl2866d_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i = 0;
    int chipid = 0;
    int read_chipid = 0;

    if (NULL == client) {
        WL2866D_PRINT("[wl2866d] i2c_client is NULL\n");
        return -1;
    }

    wl2866d_gpio_select_state(WL2866D_GPIO_STATE_EN0);

    for(i = 0; i < (sizeof(which_ldo_chip) / sizeof(which_ldo_chip[0])); i++) {
        client->addr = which_ldo_chip[i].i2c_addr;
        wl2866d_i2c_client = client;
        read_chipid = wl2866d_read(which_ldo_chip[i].chip_addr);
        chipid = wl2866d_read(which_ldo_chip[i].chip_addr) & 0xff;
        WL2866D_PRINT("[wl2866d]camera_ldo_i2c_probe i=%d,addr = 0x%x,read_chipid = 0x%x,chipid = 0x%x\n",i, client->addr, chipid,read_chipid);

        if (chipid == which_ldo_chip[i].ldoId) {
             ldo_id = i;
             WL2866D_PRINT("[wl2866d]camera_ldo_i2c_probe, this is %d\n", i);
             break;
        }
    }
    if(i == (sizeof(which_ldo_chip) / sizeof(which_ldo_chip[0]))){
        WL2866D_PRINT("[wl2866d]wl2866d_i2c_probe failed, because of i2c problems, now try to pull ldo_rst down.");
        //wl2866d_gpio_select_state(WL2836D_EN_GPIO_EN0);
    } else WL2866D_PRINT("[wl2866d]wl2866d_i2c_probe success addr = 0x%x\n", client->addr);

    mutex_init(&i2c_control_mutex);
    ldo_shutdown = ldo_shutdown_notifier;
    register_reboot_notifier(&ldo_shutdown);
    return 0;
}

static int wl2866d_i2c_remove(struct i2c_client *client)
{
    wl2866d_i2c_client = NULL;

    unregister_reboot_notifier(&ldo_shutdown);
    i2c_unregister_device(client);

    return 0;
}

/*****************************************************************************
 * wl286c_char_dev
 *****************************************************************************/
int wl2866d_init_module(void)
{
    // char driver init
    if (platform_driver_register(&wl2866d_platform_driver)) {
        WL2866D_PRINT("[wl2866d]Failed to register wl2866d_platform_driver!\n");
        i2c_del_driver(&wl2866d_i2c_driver);
        return -1;
    }
    WL2866D_PRINT("begin wl2866d initialization");
    if (i2c_add_driver(&wl2866d_i2c_driver)) {
        WL2866D_PRINT("[wl2866d]Failed to register wl2866d_i2c_driver!\n");
        return -1;
    }

    return 0;
}

void wl2866d_exit_module(void)
{
    platform_driver_unregister(&wl2866d_platform_driver);
    i2c_del_driver(&wl2866d_i2c_driver);
}

module_init(wl2866d_init_module);
module_exit(wl2866d_exit_module);

MODULE_AUTHOR("Zhu Lu <zhulu@huaqin.com>");
MODULE_DESCRIPTION("CAMERA LDO Driver");
MODULE_LICENSE("GPL");
