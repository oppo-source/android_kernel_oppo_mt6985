/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */
#ifndef __VCODEC_BW_H__
#define __VCODEC_BW_H__

#include <linux/types.h>
#include <linux/list.h>

#define VENC_BW_MONITOR 1
#define DEFAULT_VENC_CONFIG -1000
#define MTK_VCODEC_QOS_GROUP 2
#define MTK_VCODEC_QOS_TYPE 2
#define MTK_SMI_MAX_MON_REQ 4
#define MTK_SMI_MAX_MON_FRM 8

enum vcodec_port_type {
	VCODEC_PORT_BITSTREAM = 0,
	VCODEC_PORT_PICTURE_Y = 1,
	VCODEC_PORT_PICTURE_UV = 2,
	VCODEC_PORT_PICTURE_ALL = 3,
	VCODEC_PORT_RCPU = 4,
	VCODEC_PORT_WORKING = 5,
	VCODEC_PORT_LARB_SUM = 6
};

enum vcodec_larb_type {
	VCODEC_LARB_READ = 0,
	VCODEC_LARB_WRITE = 1,
	VCODEC_LARB_READ_WRITE = 2,
	VCODEC_LARB_SUM = 3
};

enum vcodec_smi_monitor_state {
	VCODEC_SMI_MONITOR_IDLE = 0,
	VCODEC_SMI_MONITOR_START = 1,
	VCODEC_SMI_MONITOR_ACQUIRE = 2,
	VCODEC_SMI_MONITOR_STOP = 3
};

enum {
	SMI_READ = 0,
	SMI_WRITE = 1,
	SMI_TYPE = 2,
};

enum {
	SMI_COMMON_ID_0 = 0,
	SMI_COMMON_ID_17 = 1,
	SMI_COMMON_NUM = 2
};

struct vcodec_port_bw {
	int port_type;
	u32 port_base_bw;
	u32 larb;
};

struct vcodec_larb_bw {
	int larb_type;
	u32 larb_base_bw;
	u32 larb_id;
};

struct vcodec_dev_qos {
	struct device *dev;
	u64 data_total[MTK_VCODEC_QOS_GROUP][MTK_VCODEC_QOS_TYPE];
	u32 monitor_ring_frame_cnt;
	bool apply_monitor_config; // 1: use monitor value; 0 use golden setting
	u32 commlarb_id[MTK_VCODEC_QOS_GROUP][MTK_SMI_MAX_MON_REQ];
	u32 flag[MTK_SMI_MAX_MON_REQ];
	u32 comm_monitor; // nums of common port to monitor
};

#endif
