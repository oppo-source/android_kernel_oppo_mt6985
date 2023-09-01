/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __LINUX_AW35616_H
#define __LINUX_AW35616_H

/* AW35616 I2C Configuration */
#define AW35616_SLAVE_ADDR 0x60
#define AW35616_I2C_RETRIES 5
#define AW35616_CHECK_RETEY  5

/* aw91xxx register read/write access */
#define REG_NONE_ACCESS                 0
#define REG_RD_ACCESS                   (1 << 0)
#define REG_WR_ACCESS                   (1 << 1)
#define AW35616_REG_MAX                  0x23

/* AW35616 Register Addresses */
#define AW35616_REG_DEV_ID   0x01
#define AW35616_REG_CTR      0x02
#define AW35616_REG_INT      0x03
#define AW35616_REG_STATUS   0x04
#define AW35616_REG_STATUS1  0x05
#define AW35616_REG_RSTN     0x06
#define AW35616_REG_USB_VID0 0x07
#define AW35616_REG_USB_VID1 0x08
#define AW35616_REG_0A  	 0x0A
#define AW35616_REG_CTR2     0x22

/* AW35616 vendor id */
#define AW35616_VENDOR_ID  0x06

/**
 * set int anable
 */
#define INT_MODE_SET       1

/*CTR2_DISABLE_RD_RP */
#define CTR2_DIS_RD_RP 0x01
#define CTR2_RD_RP_DIS_SHIFT 1

/**
 * power role mode control
 */
typedef enum {
	SNK       = 0,
	SRC       = 1,
	DRP       = 2,
	WAKE_MODE = 3
} WKMD;
#define PR_MODE_SET      WAKE_MODE

/**
 * source current mode set
 */
typedef enum {
	SRC_DEF    = 0,
	SRC_1_5A   = 1,
	SRC_3_0A   = 2,
	OR_SRC_DEF = 3
} SRC_CUR_MD;
#define SRC_CUR_SET      SRC_DEF

/**
 * try src/snk mode set
 */
typedef enum {
	NO_TRY    = 0,
	TRY_SNK   = 1,
	TRY_SRC   = 2,
	OR_NO_TRY = 3
} TRY_MD;
#define TRY_MODE_SET      TRY_SNK

/**
 * accessory mode set
 */
#define ACC_MODE_SET      1

/**
 * Disable pull-up and pull-down after a toggle cycle
 */
typedef enum {
	TOGGLE_CYCCLE_DEF   = 0,
	TOGGLE_CYCCLE_40ms  = 1,
	TOGGLE_CYCCLE_80ms  = 2,
	TOGGLE_CYCCLE_160ms = 3
} TOG_SAVE_MD;
#define TOGGLE_CYCCLE_SET  TOGGLE_CYCCLE_DEF

/**
 * intb flag
 */
typedef enum {
	NO_INTB   = 0,
	ATTACHED  = 1,
	DETACHED  = 2
} INTB_FLAG;

/**
 * plag direction
 */
typedef enum {
	STANDBY   = 0,
	CC1       = 1,
	CC2       = 2,
	CC1_CC2   = 3
} PLAG_ORI;
#define CC1_CONNECTED 0
#define CC2_CONNECTED 1

/**
 * plag status
 */
typedef enum {
	SINK      = 1,
	SOURCE    = 2,
	AUD_ACC   = 3,
	DUG_ACC   = 4
} PLAG_ST;

/**
 * charging current detection as sink
 */
typedef enum {
	SNK_DEF   = 1,
	SNK_1_5A  = 2,
	SNK_3_0A  = 3
} SINK_CUR;

/**
 * vbus detection as sink
 */
typedef enum {
	VBUS_DIS  = 0,
	VBUS_OK   = 1
} VBUS_ST;

typedef union {
	u8 byte;
	struct {
		u8 vd_id:3;
		u8 ver_id:5;
	};
} reg_dev_id;

typedef union {
	u8 byte;
	struct {
		u8 intdis:1;
		u8 wkmd:2;
		u8 src_cur_md:2;
		u8 try_md:2;
		u8 accdis:1;
	};
} reg_ctr;

typedef union {
	u8 byte;
	struct {
		u8 intb_flag:2;
		u8 wake_flag:1;
		u8 reserved:5;
	};
} reg_ints;

typedef union {
	u8 byte[2];
	struct {
		/* status */
		u8 plug_ori:2;
		u8 plug_st:3;
		u8 snk_cur_md:2;
		u8 vbusok:1;
		/* status1 */
		u8 active_cable:1;
		u8 wake_st:1;
		u8 reserved:6;
	};
} reg_status;

typedef union {
	u8 byte;
	struct {
		u8 sft_rstn:1;
		u8 typec_rstn:1;
		u8 reserved:6;
	};
} reg_rstn;

typedef union {
	u8 byte[2];
	struct {
		/* vid0 */
		u8 usb_vid_lsb:8;
		/* vid1 */
		u8 usb_vid_msb:8;
	};
} reg_vid;

typedef union {
	u8 byte;
	struct {
		u8 reserved:1;
		u8 tog_save_md:2;
		u8 reserved1:5;
	};
} reg_ctr2;

typedef struct {
	reg_dev_id       dev_id;
	reg_ctr          ctr;
	reg_ints         ints;
	reg_status       status;
	reg_rstn         rstn;
	reg_vid          vid;
	reg_ctr2         ctr2;
} aw35616_reg;

struct aw35616_chip {
	struct i2c_client *client;
	struct device *dev;
	struct semaphore io_lock;
	struct semaphore suspend_lock;
	struct tcpc_desc *tcpc_desc;
	struct tcpc_device *tcpc;
	struct kthread_worker irq_worker;
	struct kthread_work irq_work;
	struct task_struct *irq_worker_task;
	struct wakeup_source *for_irq_wake_lock;
	struct wakeup_source *for_i2c_wake_lock;

	atomic_t poll_count;
	struct delayed_work poll_work;
	struct delayed_work first_check_typec_work;

	aw35616_reg reg;

	int irq_gpio;
	uint8_t dev_id;
	uint8_t dev_sub_id;
	int irq;
	int chip_id;
};

enum aw35616_mode {
	REVERSE_CHG_DRP,
	REVERSE_CHG_SINK,
	REVERSE_CHG_SOURCE,
};

#if defined(__MEDIATEK_PLATFORM__)

enum dual_role_supported_modes {
  DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP = 0,
  DUAL_ROLE_SUPPORTED_MODES_DFP,
  DUAL_ROLE_SUPPORTED_MODES_UFP,
  /*The following should be the last element*/
  DUAL_ROLE_PROP_SUPPORTED_MODES_TOTAL,
};

enum {
  DUAL_ROLE_PROP_MODE_UFP = 0,
  DUAL_ROLE_PROP_MODE_DFP,
  DUAL_ROLE_PROP_MODE_NONE,
  /*The following should be the last element*/
  DUAL_ROLE_PROP_MODE_TOTAL,
};

enum {
   DUAL_ROLE_PROP_PR_SRC = 0,
   DUAL_ROLE_PROP_PR_SNK,
   DUAL_ROLE_PROP_PR_NONE,
   /*The following should be the last element*/
   DUAL_ROLE_PROP_PR_TOTAL,
};

enum {
   DUAL_ROLE_PROP_DR_HOST = 0,
   DUAL_ROLE_PROP_DR_DEVICE,
   DUAL_ROLE_PROP_DR_NONE,
  /*The following should be the last element*/
   DUAL_ROLE_PROP_DR_TOTAL,
};

enum {
   DUAL_ROLE_PROP_VCONN_SUPPLY_NO = 0,
   DUAL_ROLE_PROP_VCONN_SUPPLY_YES,
   /*The following should be the last element*/
   DUAL_ROLE_PROP_VCONN_SUPPLY_TOTAL,
};

enum dual_role_property {
   DUAL_ROLE_PROP_SUPPORTED_MODES = 0,
   DUAL_ROLE_PROP_MODE,
   DUAL_ROLE_PROP_PR,
   DUAL_ROLE_PROP_DR,
   DUAL_ROLE_PROP_VCONN_SUPPLY,
};
#endif

#define AWINIC_DEBUG
#ifdef AWINIC_DEBUG
#define AWINIC_LOG_NAME "AW35616"
#define AW_LOG(format, arg...) pr_info("[%s] %s %d: " format, AWINIC_LOG_NAME, \
		__func__, __LINE__, ##arg)
#else
#define AW_LOG(format, arg...)
#endif

#endif
