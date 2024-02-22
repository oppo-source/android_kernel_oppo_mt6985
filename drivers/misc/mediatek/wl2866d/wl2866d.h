#ifndef _IMGSENSOR_WL2866D_h_
#define _IMGSENSOR_WL2866D_h_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/init.h>
//ldo i2c 8-bit write addr to 7-bit salve addr
#define WL2866D_LDO_I2C_ADDR     0x28//zhulu
#define ET5904_LDO_I2C_ADDR     0X50
//chip id addr
#define WL2866D_CHIP_REV_ADDR    0x00
#define ET5904_CHIP_REV_ADDR     0x00
//chip id
#define CAMERA_LDO_WL2866D  0x00//zhulu
#define CAMERA_LDO_ET5904   0x00
//enable addr
#define WL2866D_LDO_EN_ADDR      0x0E  //bit0:LDO1 ~ bit6:LDO4
#define ET5904_LDO_EN_ADDR       0x0E
//ldo 1 addr, if addr(ldo 1) = 0x03, then addr(ldo 2) = 0x04, addr(ldo 3) = 0x05...
#define WL2866D_LDO1_OUT_ADDR    0x03
//printk
#define WL2866D_PRINT printk

enum WL2866D_SELECT{
	WL2866D_LDO_NONE=-1,
	MAIN_DVDD1,      // maincam 1.05V
	FRONT_DVDD2,     // frontcam 1.1v
	MAIN_AVDD1,      // maincam 2.8V
	FRONT_AVDD2,    // frontcam  2.8V
	WL2866D_LDO_MAX
};

/* GPIO state */
enum WL2866D_GPIO_STATE{
	WL2866D_GPIO_STATE_EN0, //rst low
	WL2866D_GPIO_STATE_EN1, //rst high
	WL2866D_GPIO_EXT_BUCK_EN0, //buck en low
	WL2866D_GPIO_EXT_BUCK_EN1, //buck en high
	WL2866D_GPIO_STATE_MAX /* for array size */
};
struct wl2866d_i2c_device{
    unsigned short i2c_addr;
    unsigned short chip_addr;
    unsigned short ldoId;
    unsigned short enable_addr;
};

#endif
