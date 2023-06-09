/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __MDP_RDMA_EX_H__
#define __MDP_RDMA_EX_H__

enum MDP_RDMA_CPR_IDX {
	CPR_MDP_RDMA_SRC_OFFSET_0 = 0,
	CPR_MDP_RDMA_SRC_OFFSET_1,
	CPR_MDP_RDMA_SRC_OFFSET_2,
	CPR_MDP_RDMA_SRC_OFFSET_WP,
	CPR_MDP_RDMA_SRC_OFFSET_HP,
	CPR_MDP_RDMA_TRANSFORM_0,
	CPR_MDP_RDMA_SRC_BASE_0,
	CPR_MDP_RDMA_SRC_BASE_1,
	CPR_MDP_RDMA_SRC_BASE_2,
	CPR_MDP_RDMA_UFO_DEC_LENGTH_BASE_Y,
	CPR_MDP_RDMA_UFO_DEC_LENGTH_BASE_C,
	CPR_MDP_RDMA_SRC_BASE_0_MSB,
	CPR_MDP_RDMA_SRC_BASE_1_MSB,
	CPR_MDP_RDMA_SRC_BASE_2_MSB,
	CPR_MDP_RDMA_UFO_DEC_LENGTH_BASE_Y_MSB,
	CPR_MDP_RDMA_UFO_DEC_LENGTH_BASE_C_MSB,
	CPR_MDP_RDMA_SRC_OFFSET_0_MSB,
	CPR_MDP_RDMA_SRC_OFFSET_1_MSB,
	CPR_MDP_RDMA_SRC_OFFSET_2_MSB,
	CPR_MDP_RDMA_AFBC_PAYLOAD_OST,
	CPR_MDP_RDMA_EN,
	CPR_MDP_RDMA_CON,
	CPR_MDP_RDMA_SHADOW_CTRL,
	CPR_MDP_RDMA_GMCIF_CON,
	CPR_MDP_RDMA_SRC_CON,
	CPR_MDP_RDMA_COMP_CON,
	CPR_MDP_RDMA_MF_BKGD_SIZE_IN_BYTE,
	CPR_MDP_RDMA_MF_BKGD_SIZE_IN_PXL,
	CPR_MDP_RDMA_MF_SRC_SIZE,
	CPR_MDP_RDMA_MF_CLIP_SIZE,
	CPR_MDP_RDMA_MF_OFFSET_1,
	CPR_MDP_RDMA_SF_BKGD_SIZE_IN_BYTE,
	CPR_MDP_RDMA_MF_BKGD_H_SIZE_IN_PXL,
	CPR_MDP_RDMA_DMABUF_CON_0,
	CPR_MDP_RDMA_URGENT_TH_CON_0,
	CPR_MDP_RDMA_ULTRA_TH_CON_0,
	CPR_MDP_RDMA_PREULTRA_TH_CON_0,
	CPR_MDP_RDMA_DMABUF_CON_1,
	CPR_MDP_RDMA_URGENT_TH_CON_1,
	CPR_MDP_RDMA_ULTRA_TH_CON_1,
	CPR_MDP_RDMA_PREULTRA_TH_CON_1,
	CPR_MDP_RDMA_DMABUF_CON_2,
	CPR_MDP_RDMA_URGENT_TH_CON_2,
	CPR_MDP_RDMA_ULTRA_TH_CON_2,
	CPR_MDP_RDMA_PREULTRA_TH_CON_2,
	CPR_MDP_RDMA_DMABUF_CON_3,
	CPR_MDP_RDMA_URGENT_TH_CON_3,
	CPR_MDP_RDMA_ULTRA_TH_CON_3,
	CPR_MDP_RDMA_PREULTRA_TH_CON_3,
	CPR_MDP_RDMA_DITHER_CON,

	CPR_MDP_RDMA_PIPE_IDX,/* 50 */
};

#endif		/* __MDP_RDMA_EX_H__ */
