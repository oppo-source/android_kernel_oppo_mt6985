// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2015 MediaTek Inc.
 */
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>

#include "ccci_debug.h"
#include "ccci_config.h"
#include "ccci_common_config.h"
#include "ccci_modem.h"
#include "ccci_swtp.h"
#include "ccci_fsm.h"

//#ifdef OPLUS_FEATURE_SWTP
#include <linux/proc_fs.h>
static unsigned int swtp_status_value = 0;
//#endif  /*OPLUS_FEATURE_SWTP*/

/* must keep ARRAY_SIZE(swtp_of_match) = ARRAY_SIZE(irq_name) */
const struct of_device_id swtp_of_match[] = {
	{ .compatible = SWTP_COMPATIBLE_DEVICE_ID, },
	{ .compatible = SWTP1_COMPATIBLE_DEVICE_ID,},
	{ .compatible = SWTP2_COMPATIBLE_DEVICE_ID,},
	{ .compatible = SWTP3_COMPATIBLE_DEVICE_ID,},
	{ .compatible = SWTP4_COMPATIBLE_DEVICE_ID,},
	{},
};

static const char irq_name[][16] = {
	"swtp0-eint",
	"swtp1-eint",
	"swtp2-eint",
	"swtp3-eint",
	"swtp4-eint",
	"",
};

struct swtp_t swtp_data;
static const char rf_name[] = "RF_cable";
#define MAX_RETRY_CNT 30

static int swtp_send_tx_power(struct swtp_t *swtp)
{
	unsigned long flags;
	int power_mode, ret = 0;

	if (swtp == NULL) {
		CCCI_LEGACY_ERR_LOG(-1, SYS, "%s:swtp is null\n", __func__);
		return -1;
	}

	spin_lock_irqsave(&swtp->spinlock, flags);

	ret = exec_ccci_kern_func(ID_UPDATE_TX_POWER,
		(char *)&swtp->tx_power_mode, sizeof(swtp->tx_power_mode));
	power_mode = swtp->tx_power_mode;
	spin_unlock_irqrestore(&swtp->spinlock, flags);

	if (ret != 0)
		CCCI_LEGACY_ERR_LOG(0, SYS,
			"%s to MD,state=%d,ret=%d\n",
			__func__, power_mode, ret);

	return ret;
}

static int swtp_switch_state(int irq, struct swtp_t *swtp)
{
	unsigned long flags;
	int i;

	if (swtp == NULL) {
		CCCI_LEGACY_ERR_LOG(-1, SYS, "%s:data is null\n", __func__);
		return -1;
	}

	spin_lock_irqsave(&swtp->spinlock, flags);
	for (i = 0; i < MAX_PIN_NUM; i++) {
		if (swtp->irq[i] == irq)
			break;
	}
	if (i == MAX_PIN_NUM) {
		spin_unlock_irqrestore(&swtp->spinlock, flags);
		CCCI_LEGACY_ERR_LOG(-1, SYS,
			"%s:can't find match irq\n", __func__);
		return -1;
	}

	if (swtp->eint_type[i] == IRQ_TYPE_LEVEL_LOW) {
		irq_set_irq_type(swtp->irq[i], IRQ_TYPE_LEVEL_HIGH);
		swtp->eint_type[i] = IRQ_TYPE_LEVEL_HIGH;
	} else {
		irq_set_irq_type(swtp->irq[i], IRQ_TYPE_LEVEL_LOW);
		swtp->eint_type[i] = IRQ_TYPE_LEVEL_LOW;
	}

	if (swtp->gpio_state[i] == SWTP_EINT_PIN_PLUG_IN)
		swtp->gpio_state[i] = SWTP_EINT_PIN_PLUG_OUT;
	else
		swtp->gpio_state[i] = SWTP_EINT_PIN_PLUG_IN;

	swtp->tx_power_mode = SWTP_NO_TX_POWER;
	for (i = 0; i < MAX_PIN_NUM; i++) {
		if (swtp->gpio_state[i] == SWTP_EINT_PIN_PLUG_IN) {
			swtp->tx_power_mode = SWTP_DO_TX_POWER;
			break;
		}
	}

	inject_pin_status_event(swtp->curr_mode, rf_name);
	spin_unlock_irqrestore(&swtp->spinlock, flags);

	//#ifdef OPLUS_FEATURE_SWTP
	CCCI_LEGACY_ERR_LOG(0, SYS,
		"[swtp_swtich_state] tx_power_mode after change: %d\n", swtp->tx_power_mode);
	swtp_status_value = !swtp->tx_power_mode;
	//#endif  /*OPLUS_FEATURE_SWTP*/

	return swtp->tx_power_mode;
}

static void swtp_send_tx_power_state(struct swtp_t *swtp)
{
	int ret = 0;

	if (!swtp) {
		CCCI_LEGACY_ERR_LOG(-1, SYS,
			"%s:swtp is null\n", __func__);
		return;
	}

	ret = swtp_send_tx_power(swtp);
	if (ret < 0) {
		CCCI_LEGACY_ERR_LOG(0, SYS,
			"%s send tx power failed, ret=%d, schedule delayed work\n",
			__func__, ret);
		schedule_delayed_work(&swtp->delayed_work, 5 * HZ);
	}
}

static irqreturn_t swtp_irq_handler(int irq, void *data)
{
	struct swtp_t *swtp = (struct swtp_t *)data;
	int ret = 0;

	ret = swtp_switch_state(irq, swtp);
	if (ret < 0) {
		CCCI_LEGACY_ERR_LOG(0, SYS,
			"%s swtp_switch_state failed in irq, ret=%d\n",
			__func__, ret);
	} else
		swtp_send_tx_power_state(swtp);

	return IRQ_HANDLED;
}

static void swtp_tx_delayed_work(struct work_struct *work)
{
	struct swtp_t *swtp = container_of(to_delayed_work(work),
		struct swtp_t, delayed_work);
	int ret, retry_cnt = 0;

	while (retry_cnt < MAX_RETRY_CNT) {
		ret = swtp_send_tx_power(swtp);
		if (ret != 0) {
			msleep(2000);
			retry_cnt++;
		} else
			break;
	}
}

int swtp_md_tx_power_req_hdlr(int data)
{
	struct swtp_t *swtp = NULL;

	swtp = &swtp_data;
	swtp_send_tx_power_state(swtp);

	return 0;
}

//#ifdef OPLUS_FEATURE_SWTP
static int swtp_gpio_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", swtp_status_value);
	return 0;
}

static int swtp_gpio_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, swtp_gpio_show, NULL);
}

static const struct proc_ops swtp_gpio_fops = {
	.proc_open   = swtp_gpio_proc_open,
	.proc_read   = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static void swtp_gpio_create_proc(void)
{
	proc_create("swtp_status_value", 0444, NULL, &swtp_gpio_fops);
}
//#endif  /*OPLUS_FEATURE_SWTP*/

static void swtp_init_delayed_work(struct work_struct *work)
{
	int i, ret = 0;
#ifdef CONFIG_MTK_EIC
	u32 ints[2] = { 0, 0 };
#else
	u32 ints[1] = { 0 };
#endif
	u32 ints1[2] = { 0, 0 };
	struct device_node *node = NULL;

	CCCI_NORMAL_LOG(-1, SYS, "%s at the begin...\n", __func__);
	CCCI_BOOTUP_LOG(-1, SYS, "%s at the begin...\n", __func__);

	if (ARRAY_SIZE(swtp_of_match) != ARRAY_SIZE(irq_name) ||
		ARRAY_SIZE(swtp_of_match) > MAX_PIN_NUM + 1 ||
		ARRAY_SIZE(irq_name) > MAX_PIN_NUM + 1) {
		ret = -3;
		CCCI_LEGACY_ERR_LOG(-1, SYS,
			"%s: invalid array count = %d(of_match), %d(irq_name)\n",
			__func__, ARRAY_SIZE(swtp_of_match),
			ARRAY_SIZE(irq_name));
		goto SWTP_INIT_END;
	}

	for (i = 0; i < MAX_PIN_NUM; i++)
		swtp_data.gpio_state[i] = SWTP_EINT_PIN_PLUG_OUT;

	for (i = 0; i < MAX_PIN_NUM; i++) {
		node = of_find_matching_node(NULL, &swtp_of_match[i]);
		if (node) {
			ret = of_property_read_u32_array(node, "debounce",
				ints, ARRAY_SIZE(ints));
			if (ret) {
				CCCI_LEGACY_ERR_LOG(0, SYS,
					"%s:swtp%d get debounce fail\n",
					__func__, i);
				break;
			}

			ret = of_property_read_u32_array(node, "interrupts",
				ints1, ARRAY_SIZE(ints1));
			if (ret) {
				CCCI_LEGACY_ERR_LOG(0, SYS,
					"%s:swtp%d get interrupts fail\n",
					__func__, i);
				break;
			}
#ifdef CONFIG_MTK_EIC /* for chips before mt6739 */
			swtp_data.gpiopin[i] = ints[0];
			swtp_data.setdebounce[i] = ints[1];
#else /* for mt6739,and chips after mt6739 */
			swtp_data.setdebounce[i] = ints[0];
			swtp_data.gpiopin[i] =
				of_get_named_gpio(node, "deb-gpios", 0);
#endif
			gpio_set_debounce(swtp_data.gpiopin[i],
				swtp_data.setdebounce[i]);
			swtp_data.eint_type[i] = ints1[1];
			swtp_data.irq[i] = irq_of_parse_and_map(node, 0);

			//#ifdef OPLUS_FEATURE_SWTP
			CCCI_LEGACY_ERR_LOG(0, SYS,
				"swtp-eint original gpio=%d, of gpio=%d, setdebounce=%d, eint_type=%d, gpio_state=%d, txpower_mode=%d\n",
				ints1[0],
				swtp_data.gpiopin[i],
				swtp_data.setdebounce[i],
				swtp_data.eint_type[i],
				swtp_data.gpio_state[i],
				swtp_data.tx_power_mode);
			//#endif  /*OPLUS_FEATURE_SWTP*/

			ret = request_irq(swtp_data.irq[i],
				swtp_irq_handler, IRQF_TRIGGER_NONE,
				irq_name[i], &swtp_data);
			if (ret) {
				CCCI_LEGACY_ERR_LOG(0, SYS,
					"swtp%d-eint IRQ LINE NOT AVAILABLE\n",
					i);
				break;
			}
		} else {
			CCCI_LEGACY_ERR_LOG(0, SYS,
				"%s:can't find swtp%d compatible node\n",
				__func__, i);
			ret = -4;
		}
	}
	register_ccci_sys_call_back(MD_SW_MD1_TX_POWER_REQ,
		swtp_md_tx_power_req_hdlr);

SWTP_INIT_END:
	CCCI_BOOTUP_LOG(0, SYS, "%s end: ret = %d\n", __func__, ret);
	CCCI_NORMAL_LOG(0, SYS, "%s end: ret = %d\n", __func__, ret);

}

int swtp_init(void)
{
	/* init woke setting */
	INIT_DELAYED_WORK(&swtp_data.init_delayed_work,
		swtp_init_delayed_work);
	/* tx work setting */
	INIT_DELAYED_WORK(&swtp_data.delayed_work,
		swtp_tx_delayed_work);
	swtp_data.tx_power_mode = SWTP_NO_TX_POWER;

	spin_lock_init(&swtp_data.spinlock);

	/* schedule init work */
	schedule_delayed_work(&swtp_data.init_delayed_work, HZ);

	CCCI_BOOTUP_LOG(0, SYS, "%s end, init_delayed_work scheduled\n",
		__func__);
	//#ifdef OPLUS_FEATURE_SWTP
	swtp_gpio_create_proc();
	//#endif  /*OPLUS_FEATURE_SWTP*/
	return 0;
}
