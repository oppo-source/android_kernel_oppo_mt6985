/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __GED_GLOBAL_H__
#define __GED_GLOBAL_H__

#include "ged_dvfs.h"

#define GED_DEFAULT_SLIDE_STRIDE_SIZE 2
#define GED_DEFAULT_SLIDE_WINDOW_SIZE 8
#define GED_DEFAULT_FRAME_TARGET_MODE 2
#define GED_DEFAULT_FRAME_TARGET_TIME 10
#define GED_DEFAULT_FALLBACK_MODE 0
#define GED_DEFAULT_FALLBACK_TIME 1
#define GED_DEFAULT_FALLBACK_WINDOW_SIZE 1

extern GED_LOG_BUF_HANDLE ghLogBuf_DVFS;
extern void (*mtk_set_bottom_gpu_freq_fp)(unsigned int idx);
extern unsigned int (*mtk_get_bottom_gpu_freq_fp)(void);
extern unsigned int (*mtk_custom_get_gpu_freq_level_count_fp)(void);
extern void (*mtk_custom_boost_gpu_freq_fp)(unsigned int ui32FreqLevel);
extern void (*mtk_custom_upbound_gpu_freq_fp)(unsigned int ui32FreqLevel);
extern unsigned int (*mtk_get_gpu_loading_fp)(void);
extern unsigned int (*mtk_get_gpu_block_fp)(void);
extern unsigned int (*mtk_get_gpu_idle_fp)(void);

extern unsigned long (*mtk_get_gpu_bottom_freq_fp)(void);

extern void ged_monitor_3D_fence_set_enable(GED_BOOL bEnable);

extern unsigned int g_ui32EventStatus;
extern unsigned int g_ui32EventDebugStatus;

extern void (*ged_dvfs_cal_gpu_utilization_ex_fp)(
		unsigned int *pui32Loading,
		unsigned int *pui32Block,
		unsigned int *pui32Idle,
		void *Util_Ex);

extern void (*ged_dvfs_gpu_freq_commit_fp)(unsigned long ui32NewFreqID,
	GED_DVFS_COMMIT_TYPE eCommitType, int *pbCommited);

extern bool mtk_get_bottom_gpu_freq(unsigned int *pui32FreqLevel);
extern GED_LOG_BUF_HANDLE ghLogBuf_GED;

extern unsigned int g_gpu_timer_based_emu;
// calculate loading reset time stamp
extern unsigned long g_ulCalResetTS_us;
// previous calculate loading reset time stamp
extern unsigned long g_ulPreCalResetTS_us;
// last frame half, t0
extern unsigned long g_ulWorkingPeriod_us;

extern unsigned int g_fastdvfs_mode;
extern unsigned int g_ged_gpueb_support;
extern unsigned int g_ged_fdvfs_support;
extern unsigned int g_ged_gpu_freq_notify_support;
extern unsigned int g_ged_segment_id;
extern unsigned int g_ged_efuse_id;
extern unsigned int g_fastdvfs_margin;
extern unsigned int g_loading_stride_size;
extern unsigned int g_loading_slide_window_size;
extern unsigned int g_loading_target_mode;
extern unsigned int g_fallback_mode;
extern unsigned int g_fallback_time;
extern unsigned int g_fallback_window_size;
extern unsigned int g_fallback_frequency_adjust;
extern unsigned int g_frame_target_mode;
extern unsigned int g_frame_target_time;

extern u64 fb_timeout;
extern u64 lb_timeout;

extern struct mutex gsPolicyLock;
extern unsigned int ged_is_fdvfs_support(void);
extern void mtk_gpueb_dvfs_commit(unsigned long ui32NewFreqID,
	GED_DVFS_COMMIT_TYPE eCommitType, int *pbCommited);
extern int mtk_gpueb_sysram_batch_read(int max_read_count,
	char *batch_string, int batch_str_size);

/* set core_mask to DDK */
extern int (*ged_dvfs_set_gpu_core_mask_fp)(u64 core_mask);
#endif
