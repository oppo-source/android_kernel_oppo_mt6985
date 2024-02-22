// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define DRIVER_NAME                  "dw9800s_22023_tele"
#define DW9800S_I2C_SLAVE_ADDR        0x18

#define LOG_INF(format, args...)                                               \
	pr_info(DRIVER_NAME " [%s] " format, __func__, ##args)

#define DW9800S_NAME				 "dw9800s_22023_tele"
#define DW9800S_MAX_FOCUS_POS		  1023
#define DW9800S_ORIGIN_FOCUS_POS	  512
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define DW9800S_FOCUS_STEPS			  1
#define DW9800S_SET_POSITION_ADDR	  0x03
#define DW9800S_STATUS_REG			  0x05

#define DW9800S_CMD_DELAY			  0xff
#define DW9800S_CTRL_DELAY_US		  10000
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */
#define DW9800S_MOVE_STEPS			  50
// base on 0x06 and 0x07 setting
// tVIB = (6.3 + (SACT[5:0]) *0.1)*DIV[2:0] ms
// 0x06 = 0x40 ==> SAC3
// 0x07 = 0x60 ==> tVIB = 9.4ms
// op_time = 9.4 * 0.72 = 6.77ms
// tolerance -+ 19%
#define DW9800S_MOVE_DELAY_US		  1000

static int g_last_pos = DW9800S_ORIGIN_FOCUS_POS;

/* dw9800s device structure */
struct dw9800s_device {
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	struct regulator *vin;
	struct regulator *vdd;
	struct pinctrl *vcamaf_pinctrl;
	struct pinctrl_state *vcamaf_on;
	struct pinctrl_state *vcamaf_off;
	/* active or standby mode */
	bool active;
};

#define VCM_IOC_POWER_ON         _IO('V', BASE_VIDIOC_PRIVATE + 3)
#define VCM_IOC_POWER_OFF        _IO('V', BASE_VIDIOC_PRIVATE + 4)

static inline struct dw9800s_device *to_dw9800s_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct dw9800s_device, ctrls);
}

static inline struct dw9800s_device *sd_to_dw9800s_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct dw9800s_device, sd);
}

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
};


static int dw9800s_set_position(struct dw9800s_device *dw9800s, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dw9800s->sd);
	int loop_time = 0, status = 0;
	while (loop_time < 20) {
		status = i2c_smbus_read_byte_data(client, DW9800S_STATUS_REG);
		status = status & 0x01;//get reg 05 status
		pr_debug("dw9800s 0x05 status:%x", status);
		if(status == 0) {
			break;
		}
		loop_time++;
		usleep_range(DW9800S_MOVE_DELAY_US, DW9800S_MOVE_DELAY_US + 100);
	}
	if (loop_time >= 20) {
		pr_debug("waiting 0x05 flag timeout!");
	}

	return i2c_smbus_write_word_data(client, DW9800S_SET_POSITION_ADDR,
					 swab16(val));
}

static int dw9800s_goto_last_pos(struct dw9800s_device *dw9800s)
{
	int ret, val = DW9800S_ORIGIN_FOCUS_POS, diff_dac, nStep_count, i;

	diff_dac = g_last_pos - DW9800S_ORIGIN_FOCUS_POS;
	if (diff_dac == 0) {
		return 0;
	}
	nStep_count = (diff_dac < 0 ? (diff_dac*(-1)) : diff_dac) /
			DW9800S_MOVE_STEPS;
	for (i = 0; i < nStep_count; ++i) {
		val += (diff_dac < 0 ? (DW9800S_MOVE_STEPS*(-1)) : DW9800S_MOVE_STEPS);
		LOG_INF("val %d", val);
		ret = dw9800s_set_position(dw9800s, val);
		if (ret) {
			LOG_INF("%s I2C failure: %d", __func__, ret);
			return ret;
		}
		usleep_range(DW9800S_MOVE_DELAY_US, DW9800S_MOVE_DELAY_US + 1000);
	}

	return 0;
}

static int dw9800s_release(struct dw9800s_device *dw9800s)
{
	int ret, val;
	int diff_dac = 0;
	int nStep_count = 0;
	int i = 0;
	struct i2c_client *client = v4l2_get_subdevdata(&dw9800s->sd);

	diff_dac = DW9800S_ORIGIN_FOCUS_POS - dw9800s->focus->val;

	nStep_count = (diff_dac < 0 ? (diff_dac*(-1)) : diff_dac) /
		DW9800S_MOVE_STEPS;

	val = dw9800s->focus->val;

	for (i = 0; i < nStep_count; ++i) {
		val += (diff_dac < 0 ? (DW9800S_MOVE_STEPS*(-1)) : DW9800S_MOVE_STEPS);

		ret = dw9800s_set_position(dw9800s, val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
		usleep_range(DW9800S_MOVE_DELAY_US,
			     DW9800S_MOVE_DELAY_US + 1000);
	}

	// last step to origin
	ret = dw9800s_set_position(dw9800s, DW9800S_ORIGIN_FOCUS_POS);
	if (ret) {
		LOG_INF("%s I2C failure: %d",
			__func__, ret);
		return ret;
	}

	i2c_smbus_write_byte_data(client, 0x02, 0x20);
	dw9800s->active = false;
	LOG_INF("-\n");

	return 0;
}

static int dw9800s_init(struct dw9800s_device *dw9800s)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dw9800s->sd);
	int ret = 0;
	char puSendCmdArray[8][2] = {
	{0x02, 0x01}, {0xFE, 0xFE}, {0x02, 0x00}, {0xFE, 0xFE},
	{0x02, 0x02}, {0x06, 0x40}, {0x07, 0x6C}, {0xFE, 0xFE},
	};
	unsigned char cmd_number;

	LOG_INF("+\n");

	client->addr = DW9800S_I2C_SLAVE_ADDR >> 1;
	//ret = i2c_smbus_read_byte_data(client, 0x02);

	LOG_INF("Check HW version: %x\n", ret);

	for (cmd_number = 0; cmd_number < 8; cmd_number++) {
		if (puSendCmdArray[cmd_number][0] != 0xFE) {
			ret = i2c_smbus_write_byte_data(client,
					puSendCmdArray[cmd_number][0],
					puSendCmdArray[cmd_number][1]);

			if (ret < 0)
				return -1;
		} else {
			udelay(100);
		}
	}

	dw9800s_goto_last_pos(dw9800s);

	LOG_INF("-\n");
	dw9800s->active = true;
	return ret;
}

/* Power handling */
static int dw9800s_power_off(struct dw9800s_device *dw9800s)
{
	int ret;

	LOG_INF("+\n");

	ret = dw9800s_release(dw9800s);
	if (ret)
		LOG_INF("dw9800s release failed!\n");

	ret = regulator_disable(dw9800s->vin);
	if (ret)
		return ret;

	ret = regulator_disable(dw9800s->vdd);
	if (ret)
		return ret;

	if (dw9800s->vcamaf_pinctrl && dw9800s->vcamaf_off)
		ret = pinctrl_select_state(dw9800s->vcamaf_pinctrl,
					dw9800s->vcamaf_off);

	LOG_INF("-\n");

	return ret;
}

static int dw9800s_power_on(struct dw9800s_device *dw9800s)
{
	int ret;

	LOG_INF("+\n");

	ret = regulator_enable(dw9800s->vin);
	if (ret < 0)
		return ret;

	ret = regulator_enable(dw9800s->vdd);
	if (ret < 0)
		return ret;

	if (dw9800s->vcamaf_pinctrl && dw9800s->vcamaf_on)
		ret = pinctrl_select_state(dw9800s->vcamaf_pinctrl,
					dw9800s->vcamaf_on);

	if (ret < 0)
		return ret;

	/*
	 * TODO(b/139784289): Confirm hardware requirements and adjust/remove
	 * the delay.
	 */
	usleep_range(DW9800S_CTRL_DELAY_US, DW9800S_CTRL_DELAY_US + 100);

	ret = dw9800s_init(dw9800s);
	if (ret < 0)
		goto fail;

	LOG_INF("-\n");

	return 0;

fail:
	regulator_disable(dw9800s->vin);
	regulator_disable(dw9800s->vdd);
	if (dw9800s->vcamaf_pinctrl && dw9800s->vcamaf_off) {
		pinctrl_select_state(dw9800s->vcamaf_pinctrl,
				dw9800s->vcamaf_off);
	}

	return ret;
}

static int dw9800s_set_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
	struct dw9800s_device *dw9800s = to_dw9800s_vcm(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		LOG_INF("pos(%d)\n", ctrl->val);
		ret = dw9800s_set_position(dw9800s, ctrl->val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
		g_last_pos = ctrl->val;
	}
	return 0;
}

static const struct v4l2_ctrl_ops dw9800s_vcm_ctrl_ops = {
	.s_ctrl = dw9800s_set_ctrl,
};

static int dw9800s_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;
	struct dw9800s_device *dw9800s = sd_to_dw9800s_vcm(sd);

	ret = dw9800s_power_on(dw9800s);
	if (ret < 0) {
		LOG_INF("power on fail, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static int dw9800s_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct dw9800s_device *dw9800s = sd_to_dw9800s_vcm(sd);

	dw9800s_power_off(dw9800s);

	return 0;
}
static long dw9800s_ops_core_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
    int ret = 0;
    struct dw9800s_device *dw9800s = sd_to_dw9800s_vcm(sd);
    struct i2c_client *client = v4l2_get_subdevdata(&dw9800s->sd);
    char puSendCmdArray[8][2] = {
        {0x02, 0x01}, {0xFE, 0xFE}, {0x02, 0x00}, {0xFE, 0xFE},
        {0x02, 0x02}, {0x06, 0x40}, {0x07, 0x6C}, {0xFE, 0xFE},
    };
    unsigned char cmd_number;
    LOG_INF("+\n");
    client->addr = DW9800S_I2C_SLAVE_ADDR >> 1;

    switch (cmd) {
    case VCM_IOC_POWER_ON:
    {
        // customized area
        /* 00:active mode , 01:Power Down mode , x1:Sleep mode */
        if (dw9800s->active)
            return 0;
        for (cmd_number = 0; cmd_number < 8; cmd_number++) {
            if (puSendCmdArray[cmd_number][0] != 0xFE) {
                ret = i2c_smbus_write_byte_data(client,
                    puSendCmdArray[cmd_number][0],
                    puSendCmdArray[cmd_number][1]);
                if (ret)
                LOG_INF("I2C failure!!!\n");
            } else {
                udelay(100);
            }
        }
        dw9800s_goto_last_pos(dw9800s);
        dw9800s->active = true;
        LOG_INF("stand by mode, power on !!!!!!!!!!!!\n");
    }
    break;
    case VCM_IOC_POWER_OFF:
    {
        // customized area
        if (!dw9800s->active)
            return 0;
        ret = i2c_smbus_write_byte_data(client, 0x02, 0x01);
        if (ret) {
            LOG_INF("I2C failure!!!\n");
        } else {
            dw9800s->active = false;
            LOG_INF("stand by mode, power off !!!!!!!!!!!!\n");
        }
    }
    break;
    default:
        ret = -ENOIOCTLCMD;
        break;
    }

    return ret;
}
static const struct v4l2_subdev_internal_ops dw9800s_int_ops = {
	.open = dw9800s_open,
	.close = dw9800s_close,
};

static struct v4l2_subdev_core_ops dw9800s_ops_core = {
    .ioctl = dw9800s_ops_core_ioctl,
};

static const struct v4l2_subdev_ops dw9800s_ops = {
    .core = &dw9800s_ops_core,
};

static void dw9800s_subdev_cleanup(struct dw9800s_device *dw9800s)
{
	v4l2_async_unregister_subdev(&dw9800s->sd);
	v4l2_ctrl_handler_free(&dw9800s->ctrls);
#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&dw9800s->sd.entity);
#endif
}

static int dw9800s_init_controls(struct dw9800s_device *dw9800s)
{
	struct v4l2_ctrl_handler *hdl = &dw9800s->ctrls;
	const struct v4l2_ctrl_ops *ops = &dw9800s_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	dw9800s->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, DW9800S_MAX_FOCUS_POS, DW9800S_FOCUS_STEPS, 0);

	if (hdl->error)
		return hdl->error;

	dw9800s->sd.ctrl_handler = hdl;

	return 0;
}

static int dw9800s_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct dw9800s_device *dw9800s;
	int ret;

	LOG_INF("+\n");

	dw9800s = devm_kzalloc(dev, sizeof(*dw9800s), GFP_KERNEL);
	if (!dw9800s)
		return -ENOMEM;

	dw9800s->vin = devm_regulator_get(dev, "vin");
	if (IS_ERR(dw9800s->vin)) {
		ret = PTR_ERR(dw9800s->vin);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vin regulator\n");
		return ret;
	}

	dw9800s->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(dw9800s->vdd)) {
		ret = PTR_ERR(dw9800s->vdd);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vdd regulator\n");
		return ret;
	}

	dw9800s->vcamaf_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(dw9800s->vcamaf_pinctrl)) {
		ret = PTR_ERR(dw9800s->vcamaf_pinctrl);
		dw9800s->vcamaf_pinctrl = NULL;
		LOG_INF("cannot get pinctrl\n");
	} else {
		dw9800s->vcamaf_on = pinctrl_lookup_state(
			dw9800s->vcamaf_pinctrl, "vcamaf_on");

		if (IS_ERR(dw9800s->vcamaf_on)) {
			ret = PTR_ERR(dw9800s->vcamaf_on);
			dw9800s->vcamaf_on = NULL;
			LOG_INF("cannot get vcamaf_on pinctrl\n");
		}

		dw9800s->vcamaf_off = pinctrl_lookup_state(
			dw9800s->vcamaf_pinctrl, "vcamaf_off");

		if (IS_ERR(dw9800s->vcamaf_off)) {
			ret = PTR_ERR(dw9800s->vcamaf_off);
			dw9800s->vcamaf_off = NULL;
			LOG_INF("cannot get vcamaf_off pinctrl\n");
		}
	}

	v4l2_i2c_subdev_init(&dw9800s->sd, client, &dw9800s_ops);
	dw9800s->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dw9800s->sd.internal_ops = &dw9800s_int_ops;

	ret = dw9800s_init_controls(dw9800s);
	if (ret)
		goto err_cleanup;

#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	ret = media_entity_pads_init(&dw9800s->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	dw9800s->sd.entity.function = MEDIA_ENT_F_LENS;
#endif

	ret = v4l2_async_register_subdev(&dw9800s->sd);
	if (ret < 0)
		goto err_cleanup;

	LOG_INF("-\n");

	return 0;

err_cleanup:
	dw9800s_subdev_cleanup(dw9800s);
	return ret;
}

static int dw9800s_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9800s_device *dw9800s = sd_to_dw9800s_vcm(sd);

	LOG_INF("+\n");

	dw9800s_subdev_cleanup(dw9800s);

	LOG_INF("-\n");
	return 0;
}

static const struct i2c_device_id dw9800s_id_table[] = {
	{ DW9800S_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, dw9800s_id_table);

static const struct of_device_id dw9800s_of_table[] = {
	{ .compatible = "mediatek,dw9800s_22023_tele" },
	{ },
};
MODULE_DEVICE_TABLE(of, dw9800s_of_table);

static struct i2c_driver dw9800s_i2c_driver = {
	.driver = {
		.name = DW9800S_NAME,
		.of_match_table = dw9800s_of_table,
	},
	.probe_new  = dw9800s_probe,
	.remove = dw9800s_remove,
	.id_table = dw9800s_id_table,
};

module_i2c_driver(dw9800s_i2c_driver);

MODULE_AUTHOR("Po-Hao Huang <Po-Hao.Huang@mediatek.com>");
MODULE_DESCRIPTION("DW9800S VCM driver");
MODULE_LICENSE("GPL v2");
