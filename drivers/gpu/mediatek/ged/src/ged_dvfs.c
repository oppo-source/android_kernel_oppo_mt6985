// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/module.h>

#if defined(CONFIG_MTK_GPUFREQ_V2)
#include <ged_gpufreq_v2.h>
#include <gpufreq_v2.h>
#else
#include <ged_gpufreq_v1.h>
#endif /* CONFIG_MTK_GPUFREQ_V2 */

#include <mt-plat/mtk_gpu_utility.h>
#include <asm/siginfo.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/vmalloc.h>

#include "ged_dvfs.h"
#include "ged_kpi.h"
#include "ged_monitor_3D_fence.h"
#include <ged_notify_sw_vsync.h>
#include "ged_log.h"
#include "ged_base.h"
#include "ged_global.h"
#include "ged_eb.h"
#include "ged_dcs.h"

#define MTK_DEFER_DVFS_WORK_MS          10000
#define MTK_DVFS_SWITCH_INTERVAL_MS     50

/* Definition of GED_DVFS_SKIP_ROUNDS is to skip DVFS when boost raised
 *  the value stands for counting down rounds of DVFS period
 *  Current using vsync that would be 16ms as period,
 *  below boost at (32, 48] seconds per boost
 */
#define GED_DVFS_SKIP_ROUNDS 3

//sliding window
#define MAX_SLIDE_WINDOW_SIZE 64

spinlock_t gsGpuUtilLock;

static struct mutex gsDVFSLock;
static struct mutex gsVSyncOffsetLock;
struct mutex gsPolicyLock;

static unsigned int g_iSkipCount;
static int g_dvfs_skip_round;

static unsigned int gpu_power;
static unsigned int gpu_dvfs_enable;
static unsigned int gpu_debug_enable;

static struct GED_DVFS_OPP_STAT *g_aOppStat;
static int g_num;
unsigned long long g_ns_gpu_on_ts;

enum MTK_GPU_DVFS_TYPE g_CommitType;
unsigned long g_ulCommitFreq;

#ifdef ENABLE_COMMON_DVFS
static unsigned int boost_gpu_enable;
static unsigned int gpu_bottom_freq;
static unsigned int gpu_cust_boost_freq;
static unsigned int gpu_cust_upbound_freq;
#endif /* ENABLE_COMMON_DVFS */

static unsigned int g_ui32PreFreqID;
static unsigned int g_ui32CurFreqID;

static unsigned int g_bottom_freq_id;
static unsigned int g_last_def_commit_freq_id;
static unsigned int g_cust_upbound_freq_id;
static unsigned int g_cust_boost_freq_id;

#define LIMITER_FPSGO 0
#define LIMITER_APIBOOST 1

unsigned int g_gpu_timer_based_emu;
// unsigned int gpu_bw_err_debug;

static unsigned int gpu_pre_loading;
unsigned int gpu_loading;
unsigned int gpu_av_loading;
unsigned int gpu_block;
unsigned int gpu_idle;
unsigned long g_um_gpu_tar_freq;

spinlock_t g_sSpinLock;
/* calculate loading reset time stamp */
unsigned long g_ulCalResetTS_us;
/* previous calculate loading reset time stamp */
unsigned long g_ulPreCalResetTS_us;
/* last frame half, t0 */
unsigned long g_ulWorkingPeriod_us;

/* record previous DVFS applying time stamp */
unsigned long g_ulPreDVFS_TS_us;

/* calculate loading reset time stamp */
static unsigned long gL_ulCalResetTS_us;
/* previous calculate loading reset time stamp */
static unsigned long gL_ulPreCalResetTS_us;
/* last frame half, t0 */
static unsigned long gL_ulWorkingPeriod_us;

static unsigned int g_loading2_count;
static unsigned int g_loading2_sum;
static uint64_t g_LoadingTS_us;

static DEFINE_SPINLOCK(load_info_lock);

static unsigned long g_policy_tar_freq;
static int g_mode;

static unsigned int g_ui32FreqIDFromPolicy;

// struct GED_DVFS_BW_DATA gsBWprofile[MAX_BW_PROFILE];
// int g_bw_head;
// int g_bw_tail;

unsigned long g_ulvsync_period;
static GED_DVFS_TUNING_MODE g_eTuningMode;

unsigned int g_ui32EventStatus;
unsigned int g_ui32EventDebugStatus;
static int g_VsyncOffsetLevel;

static int g_probe_pid = GED_NO_UM_SERVICE;

#define DEFAULT_DVFS_STEP_MODE 0x103 /* ultra high/low step: 3/1 */
unsigned int dvfs_step_mode = DEFAULT_DVFS_STEP_MODE;
static int init;

static unsigned int g_ui32TargetPeriod_us = 16666;
static unsigned int g_ui32BoostValue = 100;

static unsigned int ged_commit_freq;
static unsigned int ged_commit_opp_freq;

/* global to store the platform min/max Freq/idx */
static unsigned int g_minfreq;
static unsigned int g_maxfreq;
static int g_minfreq_idx;
static int g_maxfreq_idx;
static int api_sync_flag;

/* need to sync to EB */
#define BATCH_MAX_READ_COUNT 32
/* formatted pattern |xxxx|yyyy 5x2 */
#define BATCH_PATTERN_LEN 10
#define BATCH_STR_SIZE (BATCH_PATTERN_LEN*BATCH_MAX_READ_COUNT)
char batch_freq[BATCH_STR_SIZE];
int avg_freq;
unsigned int pre_freq, cur_freq;

#define GED_DVFS_BUSY_CYCLE_MONITORING_WINDOW_NUM 4
#define GED_FB_DVFS_FERQ_DROP_RATIO_LIMIT 30
static int is_fb_dvfs_triggered;
static int is_fallback_mode_triggered;
static unsigned int fallback_duration;   // unit: ms

void ged_dvfs_last_and_target_cb(int t_gpu_target, int boost_accum_gpu)
{
	g_ui32TargetPeriod_us = t_gpu_target;
	g_ui32BoostValue = boost_accum_gpu;
	/* mtk_gpu_ged_hint(g_ui32TargetPeriod_us, g_ui32BoostValue); */
}

static bool ged_dvfs_policy(
	unsigned int ui32GPULoading, unsigned int *pui32NewFreqID,
	unsigned long t, long phase, unsigned long ul3DFenceDoneTime,
	bool bRefreshed);

struct ld_ud_table {
	int freq;
	int up;
	int down;
};
static struct ld_ud_table *loading_ud_table;

struct gpu_utilization_history {
	struct GpuUtilization_Ex util_data[MAX_SLIDE_WINDOW_SIZE];
	struct {
		unsigned int workload;   // unit: cycle
		unsigned long long t_gpu;   // unit: us
	} gpu_frame_property;
	unsigned int current_idx;
};
static struct gpu_utilization_history g_util_hs;

static int gx_dvfs_loading_mode = LOADING_MAX_ITERMCU;
static int gx_dvfs_workload_mode = WORKLOAD_MAX_ITERMCU;
struct GpuUtilization_Ex g_Util_Ex;
static int ged_get_dvfs_loading_mode(void);
static int ged_get_dvfs_workload_mode(void);

#define GED_DVFS_TIMER_BASED_DVFS_MARGIN 30
#define GED_DVFS_TIMER_BASED_DVFS_MARGIN_STEP 5
static int gx_tb_dvfs_margin = GED_DVFS_TIMER_BASED_DVFS_MARGIN;
static int gx_tb_dvfs_margin_cur = GED_DVFS_TIMER_BASED_DVFS_MARGIN;

#define MAX_TB_DVFS_MARGIN               99
#define MIN_TB_DVFS_MARGIN               5
#define CONFIGURE_TIMER_BASED_MODE       0x00000000
#define DYNAMIC_TB_MASK                  0x00000100
#define DYNAMIC_TB_PIPE_TIME_MASK        0x00000200   // phased out
#define DYNAMIC_TB_PERF_MODE_MASK        0x00000400
#define DYNAMIC_TB_FIX_TARGET_MASK       0x00000800   // phased out
#define TIMER_BASED_MARGIN_MASK          0x000000ff
// default loading-based margin mode + value is 30
static int g_tb_dvfs_margin_value = GED_DVFS_TIMER_BASED_DVFS_MARGIN;
static int g_tb_dvfs_margin_value_min = MIN_TB_DVFS_MARGIN;
static int g_tb_dvfs_margin_step = GED_DVFS_TIMER_BASED_DVFS_MARGIN_STEP;
static unsigned int g_tb_dvfs_margin_mode = DYNAMIC_TB_MASK | DYNAMIC_TB_PERF_MODE_MASK;

static void _init_loading_ud_table(void)
{
	int i;
	int num = ged_get_opp_num();

	if (!loading_ud_table)
		loading_ud_table = ged_alloc(sizeof(struct ld_ud_table) * num);

	for (i = 0; i < num; ++i) {
		loading_ud_table[i].freq = ged_get_freq_by_idx(i);
		loading_ud_table[i].up = (100 - gx_tb_dvfs_margin_cur);
	}

	for (i = 0; i < num - 1; ++i) {
		int a = loading_ud_table[i].freq;
		int b = loading_ud_table[i+1].freq;

		if (a != 0)
			loading_ud_table[i].down
				= ((100 - gx_tb_dvfs_margin_cur) * b) / a;
	}

	if (num >= 2)
		loading_ud_table[num-1].down = loading_ud_table[num-2].down;
}

static void gpu_util_history_init(void)
{
	memset(&g_util_hs, 0, sizeof(struct gpu_utilization_history));
}

static void gpu_util_history_update(struct GpuUtilization_Ex *util_ex)
{
	unsigned int current_idx = g_util_hs.current_idx + 1;
	unsigned long long max_workload =
		util_ex->freq * util_ex->delta_time / 1000000;   // unit: cycle
	unsigned long long max_t_gpu = util_ex->delta_time / 1000;   // unit: us
	unsigned int workload_mode = 0;

	if (current_idx >= MAX_SLIDE_WINDOW_SIZE)
		current_idx = 0;   // wrap ring buffer index position around

	/* update util history */
	memcpy((void *)&g_util_hs.util_data[current_idx], (void *)util_ex,
		sizeof(struct GpuUtilization_Ex));

	/* update frame property */
	// workload = freq * delta_time * loading
	// t_gpu = delta_time * loading
	workload_mode = ged_get_dvfs_workload_mode();
	if (workload_mode == WORKLOAD_3D) {
		g_util_hs.gpu_frame_property.workload +=
			max_workload * util_ex->util_3d / 100;
		g_util_hs.gpu_frame_property.t_gpu +=
			max_t_gpu * util_ex->util_3d / 100;
	} else if (workload_mode == WORKLOAD_ITER) {
		g_util_hs.gpu_frame_property.workload +=
			max_workload * util_ex->util_iter / 100;
		g_util_hs.gpu_frame_property.t_gpu +=
			max_t_gpu * util_ex->util_iter / 100;
	} else if (workload_mode == WORKLOAD_MAX_ITERMCU) {
		unsigned int util_temp = MAX(util_ex->util_iter, util_ex->util_mcu);

		g_util_hs.gpu_frame_property.workload +=
			max_workload * util_temp / 100;
		g_util_hs.gpu_frame_property.t_gpu +=
			max_t_gpu * util_temp / 100;
	} else {   // WORKLOAD_ACTIVE or unknown mode
		g_util_hs.gpu_frame_property.workload +=
			max_workload * util_ex->util_active / 100;
		g_util_hs.gpu_frame_property.t_gpu +=
			max_t_gpu * util_ex->util_active / 100;
	}

	g_util_hs.current_idx = current_idx;
}
static unsigned int gpu_util_history_query_loading(unsigned int window_size_us)
{
	struct GpuUtilization_Ex *util_ex;
	unsigned int loading_mode = 0;
	unsigned long long sum_loading = 0;   // unit: % * us
	unsigned long long sum_delta_time = 0;   // unit: us
	unsigned long long remaining_time, delta_time;   // unit: us
	unsigned int window_avg_loading = 0;
	unsigned int cidx = g_util_hs.current_idx;
	unsigned int his_loading = 0;
	int pre_idx = cidx - MAX_SLIDE_WINDOW_SIZE;
	int his_idx = 0;
	int i = 0;

	for (i = cidx; i > pre_idx; i--) {
		// ring buffer index wrapping
		if (i >= 0)
			his_idx = i;
		else
			his_idx = i + MAX_SLIDE_WINDOW_SIZE;
		util_ex = &g_util_hs.util_data[his_idx];

		// calculate loading based on different loading mode
		loading_mode = ged_get_dvfs_loading_mode();
		if (loading_mode == LOADING_MAX_3DTA_COM) {
			his_loading = MAX(util_ex->util_3d, util_ex->util_ta) +
				util_ex->util_compute;
		} else if (loading_mode == LOADING_MAX_3DTA) {
			his_loading = MAX(util_ex->util_3d, util_ex->util_ta);
		} else if (loading_mode == LOADING_ITER) {
			his_loading = util_ex->util_iter;
		} else if (loading_mode == LOADING_MAX_ITERMCU) {
			his_loading = MAX(util_ex->util_iter,
				util_ex->util_mcu);
		} else {   // LOADING_ACTIVE or unknown mode
			his_loading = util_ex->util_active;
		}

		// accumulate data
		remaining_time = window_size_us - sum_delta_time;
		delta_time = util_ex->delta_time / 1000;
		if (delta_time > remaining_time)
			delta_time = remaining_time;   // clip delta_time
		sum_loading += his_loading * delta_time;
		sum_delta_time += delta_time;

		// data is sufficient
		if (sum_delta_time >= window_size_us)
			break;
	}

	if (sum_delta_time != 0)
		window_avg_loading = sum_loading / sum_delta_time;

	if (window_avg_loading > 100)
		window_avg_loading = 100;

	return window_avg_loading;
}

static unsigned int gpu_util_history_query_frequency(
	unsigned int window_size_us)
{
	struct GpuUtilization_Ex *util_ex;
	unsigned long long sum_freq = 0;   // unit: KHz * us
	unsigned long long sum_delta_time = 0;   // unit: us
	unsigned long long remaining_time, delta_time;   // unit: us
	unsigned int window_avg_freq = 0;
	unsigned int cidx = g_util_hs.current_idx;
	unsigned int his_freq = 0;
	int pre_idx = cidx - MAX_SLIDE_WINDOW_SIZE;
	int his_idx = 0;
	int i = 0;

	for (i = cidx; i > pre_idx; i--) {
		// ring buffer index wrapping
		if (i >= 0)
			his_idx = i;
		else
			his_idx = i + MAX_SLIDE_WINDOW_SIZE;
		util_ex = &g_util_hs.util_data[his_idx];

		// calculate frequency
		his_freq = util_ex->freq;

		// accumulate data
		remaining_time = window_size_us - sum_delta_time;
		delta_time = util_ex->delta_time / 1000;
		if (delta_time > remaining_time)
			delta_time = remaining_time;   // clip delta_time
		sum_freq += his_freq * delta_time;
		sum_delta_time += delta_time;

		// data is sufficient
		if (sum_delta_time >= window_size_us)
			break;
	}

	if (sum_delta_time != 0)
		window_avg_freq = sum_freq / sum_delta_time;

	return window_avg_freq;
}

static void gpu_util_history_query_frame_property(unsigned int *workload,
	unsigned long long *t_gpu)
{
	// copy latest counter value
	*workload = g_util_hs.gpu_frame_property.workload;
	*t_gpu = g_util_hs.gpu_frame_property.t_gpu;

	// reset counter value
	g_util_hs.gpu_frame_property.workload = 0;
	g_util_hs.gpu_frame_property.t_gpu = 0;
}

unsigned long ged_query_info(GED_INFO eType)
{
	unsigned int gpu_loading;
	unsigned int gpu_block;
	unsigned int gpu_idle;

	gpu_loading = 0;
	gpu_idle = 0;
	gpu_block = 0;

	switch (eType) {
	case GED_LOADING:
		mtk_get_gpu_loading(&gpu_loading);
		return gpu_loading;
	case GED_IDLE:
		mtk_get_gpu_idle(&gpu_idle);
		return gpu_idle;
	case GED_BLOCKING:
		mtk_get_gpu_block(&gpu_block);
		return gpu_block;
	case GED_PRE_FREQ:
		ged_get_freq_by_idx(g_ui32PreFreqID);
	case GED_PRE_FREQ_IDX:
		return g_ui32PreFreqID;
	case GED_CUR_FREQ:
		return ged_get_cur_freq();
	case GED_CUR_FREQ_IDX:
		return ged_get_cur_oppidx();
	case GED_MAX_FREQ_IDX:
#if defined(CONFIG_MTK_GPUFREQ_V2)
		return ged_get_min_oppidx();
#else
		return ged_get_max_oppidx();
#endif
	case GED_MAX_FREQ_IDX_FREQ:
		return g_maxfreq;
	case GED_MIN_FREQ_IDX:
#if defined(CONFIG_MTK_GPUFREQ_V2)
		return ged_get_max_oppidx();
#else
		return ged_get_min_oppidx();
#endif
	case GED_MIN_FREQ_IDX_FREQ:
		return g_minfreq;
	case GED_3D_FENCE_DONE_TIME:
		return ged_monitor_3D_fence_done_time();
	case GED_VSYNC_OFFSET:
		return ged_dvfs_vsync_offset_level_get();
	case GED_EVENT_STATUS:
		return g_ui32EventStatus;
	case GED_EVENT_DEBUG_STATUS:
		return g_ui32EventDebugStatus;
	case GED_SRV_SUICIDE:
		ged_dvfs_probe_signal(GED_SRV_SUICIDE_EVENT);
		return g_probe_pid;
	case GED_PRE_HALF_PERIOD:
		return g_ulWorkingPeriod_us;
	case GED_LATEST_START:
		return g_ulPreCalResetTS_us;

	default:
		return 0;
	}
}
EXPORT_SYMBOL(ged_query_info);

void ged_opp_stat_step(void)
{
	g_aOppStat[g_ui32PreFreqID].uMem.aTrans[ged_get_cur_oppidx()]++;
}

//-----------------------------------------------------------------------------
void (*ged_dvfs_cal_gpu_utilization_ex_fp)(unsigned int *pui32Loading,
	unsigned int *pui32Block, unsigned int *pui32Idle,
	void *Util_Ex) = NULL;
EXPORT_SYMBOL(ged_dvfs_cal_gpu_utilization_ex_fp);
//-----------------------------------------------------------------------------

bool ged_dvfs_cal_gpu_utilization_ex(unsigned int *pui32Loading,
	unsigned int *pui32Block, unsigned int *pui32Idle,
	struct GpuUtilization_Ex *Util_Ex)
{
	unsigned long long TS_us;
	unsigned long long TS_base_us;
	unsigned long long TS_p_on_us;
	unsigned int oppLoading;

	unsigned long ui32IRQFlags;

	if (ged_dvfs_cal_gpu_utilization_ex_fp != NULL) {
		ged_dvfs_cal_gpu_utilization_ex_fp(pui32Loading, pui32Block,
			pui32Idle, (void *) Util_Ex);
		Util_Ex->freq = ged_get_cur_freq();

		gpu_util_history_update(Util_Ex);

		memcpy((void *)&g_Util_Ex, (void *)Util_Ex,
			sizeof(struct GpuUtilization_Ex));

		if (g_ged_gpueb_support)
			mtk_gpueb_dvfs_set_feedback_info(
				0, g_Util_Ex, 0);

		if (pui32Loading) {
			Loading__(Util_Ex->util_active, Util_Ex->util_ta, Util_Ex->util_3d,
				Util_Ex->util_compute, Util_Ex->util_iter, Util_Ex->util_mcu);

			gpu_av_loading = *pui32Loading;

			///
			oppLoading = *pui32Loading;
			TS_us = ged_get_time();
			/* to approximate us*/
			TS_us = TS_us >> 10;
			TS_p_on_us = g_ns_gpu_on_ts >> 10;
			///
			/* for rainy days */
			if (TS_p_on_us > TS_us)
				TS_base_us = g_LoadingTS_us;
			else {
				TS_base_us = (g_LoadingTS_us > TS_p_on_us)?
					g_LoadingTS_us : TS_p_on_us;
			}
			/////
			spin_lock_irqsave(&load_info_lock, ui32IRQFlags);
			g_loading2_sum += gpu_av_loading;
			g_loading2_count++;
			if (TS_base_us > TS_us || *pui32Loading > 100) {
				if (*pui32Loading > 100)
					oppLoading = 100;
			}
			g_LoadingTS_us = TS_us;

			/* the minus one should be clock
			   reference problem between threads */
			if (TS_base_us < TS_us)
				ged_dvfs_update_opp_cost(
				oppLoading,
				(TS_us - TS_base_us),
				TS_us, ged_get_cur_oppidx());
			spin_unlock_irqrestore(&load_info_lock, ui32IRQFlags);
		}
		return true;
	}
	return false;
}

/*-----------------------------------------------------------------------------
 * void (*ged_dvfs_gpu_freq_commit_fp)(unsigned long ui32NewFreqID)
 * call back function
 * This shall be registered in vendor's GPU driver,
 * since each IP has its own rule
 */
static unsigned long g_ged_dvfs_commit_idx; /* max freq opp idx */
void (*ged_dvfs_gpu_freq_commit_fp)(unsigned long ui32NewFreqID,
	GED_DVFS_COMMIT_TYPE eCommitType, int *pbCommited) = NULL;
EXPORT_SYMBOL(ged_dvfs_gpu_freq_commit_fp);

unsigned long ged_dvfs_get_last_commit_idx(void)
{
	return g_ged_dvfs_commit_idx;
}
EXPORT_SYMBOL(ged_dvfs_get_last_commit_idx);

bool ged_dvfs_gpu_freq_commit(unsigned long ui32NewFreqID,
	unsigned long ui32NewFreq, GED_DVFS_COMMIT_TYPE eCommitType)
{
	int bCommited = false;

	int ui32CurFreqID, ui32CeilingID, ui32FloorID;
	unsigned int cur_freq = 0;
	enum gpu_dvfs_policy_state policy_state;

	ui32CurFreqID = ged_get_cur_oppidx();

	if (ui32CurFreqID == -1)
		return bCommited;

	if (eCommitType == GED_DVFS_FRAME_BASE_COMMIT ||
		eCommitType == GED_DVFS_LOADING_BASE_COMMIT)
		g_last_def_commit_freq_id = ui32NewFreqID;

	if (ged_dvfs_gpu_freq_commit_fp != NULL) {

		ui32CeilingID = ged_get_cur_limit_idx_ceil();
		ui32FloorID = ged_get_cur_limit_idx_floor();

		if (ui32NewFreqID < ui32CeilingID)
			ui32NewFreqID = ui32CeilingID;

		if (ui32NewFreqID > ui32FloorID)
			ui32NewFreqID = ui32FloorID;

		g_ulCommitFreq = ged_get_freq_by_idx(ui32NewFreqID);
		ged_commit_freq = ui32NewFreq;
		ged_commit_opp_freq = ged_get_freq_by_idx(ui32NewFreqID);

		/* do change */
		if (ui32NewFreqID != ui32CurFreqID) {
			/* call to ged gpufreq wrapper module */
			g_ged_dvfs_commit_idx = ui32NewFreqID;
			ged_gpufreq_commit(ui32NewFreqID, eCommitType, &bCommited);

			/*
			 * To-Do: refine previous freq contributions,
			 * since it is possible to have multiple freq settings
			 * in previous execution period
			 * Does this fatal for precision?
			 */
			ged_log_buf_print2(ghLogBuf_DVFS, GED_LOG_ATTR_TIME,
		"[GED_K] new freq ID committed: idx=%lu type=%u, g_type=%u",
				ui32NewFreqID, eCommitType, g_CommitType);

			ged_log_buf_print(ghLogBuf_DVFS,
				"[GED_K] committed true");
			g_ui32PreFreqID = ui32CurFreqID;
			g_ui32CurFreqID = ged_get_cur_oppidx();
			if (g_aOppStat)
				ged_opp_stat_step();
		}

		if (ged_is_fdvfs_support()
			&& is_fb_dvfs_triggered && is_fdvfs_enable()) {
			memset(batch_freq, 0, sizeof(batch_freq));
			avg_freq = mtk_gpueb_sysram_batch_read(BATCH_MAX_READ_COUNT,
						batch_freq, BATCH_STR_SIZE);

			ged_log_perf_trace_batch_counter("gpu_freq",
				(long long)(avg_freq),
				5566, 0, 0, batch_freq);

			Frequency__(avg_freq*1000, gpufreq_get_cur_freq(TARGET_DEFAULT));
		} else {
			ged_log_perf_trace_counter("gpu_freq",
				(long long)(ged_get_cur_freq() / 1000), 5566, 0, 0);

			Frequency__(ged_get_cur_freq(),
				gpufreq_get_cur_freq(TARGET_DEFAULT));
		}

		ged_log_perf_trace_counter("gpu_freq_ceil",
			(long long)ged_get_freq_by_idx(ui32CeilingID) / 1000, 5566, 0, 0);
		ged_log_perf_trace_counter("gpu_freq_floor",
			(long long)ged_get_freq_by_idx(ui32FloorID) / 1000, 5566, 0, 0);
		ged_log_perf_trace_counter("limitter_ceil",
			(long long)ged_get_cur_limiter_ceil(), 5566, 0, 0);
		ged_log_perf_trace_counter("limitter_floor",
			(long long)ged_get_cur_limiter_floor(), 5566, 0, 0);
		ged_log_perf_trace_counter("commit_type",
			(long long)eCommitType, 5566, 0, 0);

		policy_state = ged_get_policy_state();
		Policy__Common(eCommitType, policy_state);

	}
	return bCommited;
}

unsigned long get_ns_period_from_fps(unsigned int ui32Fps)
{
	return 1000000/ui32Fps;
}

void ged_dvfs_set_tuning_mode(GED_DVFS_TUNING_MODE eMode)
{
	g_eTuningMode = eMode;
}

void ged_dvfs_set_tuning_mode_wrap(int eMode)
{
	ged_dvfs_set_tuning_mode((GED_DVFS_TUNING_MODE)eMode);
}

GED_DVFS_TUNING_MODE ged_dvfs_get_tuning_mode(void)
{
	return g_eTuningMode;
}

GED_ERROR ged_dvfs_vsync_offset_event_switch(
	GED_DVFS_VSYNC_OFFSET_SWITCH_CMD eEvent, bool bSwitch)
{
	unsigned int ui32BeforeSwitchInterpret;
	unsigned int ui32BeforeDebugInterpret;
	GED_ERROR ret = GED_OK;

	mutex_lock(&gsVSyncOffsetLock);

	ui32BeforeSwitchInterpret = g_ui32EventStatus;
	ui32BeforeDebugInterpret = g_ui32EventDebugStatus;

	switch (eEvent) {
	case GED_DVFS_VSYNC_OFFSET_FORCE_ON:
		g_ui32EventDebugStatus |= GED_EVENT_FORCE_ON;
		g_ui32EventDebugStatus &= (~GED_EVENT_FORCE_OFF);
		break;
	case GED_DVFS_VSYNC_OFFSET_FORCE_OFF:
		g_ui32EventDebugStatus |= GED_EVENT_FORCE_OFF;
		g_ui32EventDebugStatus &= (~GED_EVENT_FORCE_ON);
		break;
	case GED_DVFS_VSYNC_OFFSET_DEBUG_CLEAR_EVENT:
		g_ui32EventDebugStatus &= (~GED_EVENT_FORCE_ON);
		g_ui32EventDebugStatus &= (~GED_EVENT_FORCE_OFF);
		break;
	case GED_DVFS_VSYNC_OFFSET_TOUCH_EVENT:
		/* touch boost */
#ifdef ENABLE_COMMON_DVFS
		if (bSwitch == GED_TRUE)
			ged_dvfs_boost_gpu_freq();
#endif /* ENABLE_COMMON_DVFS */

		(bSwitch) ? (g_ui32EventStatus |= GED_EVENT_TOUCH) :
			(g_ui32EventStatus &= (~GED_EVENT_TOUCH));
		break;
	case GED_DVFS_VSYNC_OFFSET_THERMAL_EVENT:
		(bSwitch) ? (g_ui32EventStatus |= GED_EVENT_THERMAL) :
			(g_ui32EventStatus &= (~GED_EVENT_THERMAL));
		break;
	case GED_DVFS_VSYNC_OFFSET_WFD_EVENT:
		(bSwitch) ? (g_ui32EventStatus |= GED_EVENT_WFD) :
			(g_ui32EventStatus &= (~GED_EVENT_WFD));
		break;
	case GED_DVFS_VSYNC_OFFSET_MHL_EVENT:
		(bSwitch) ? (g_ui32EventStatus |= GED_EVENT_MHL) :
			(g_ui32EventStatus &= (~GED_EVENT_MHL));
		break;
	case GED_DVFS_VSYNC_OFFSET_VR_EVENT:
		(bSwitch) ? (g_ui32EventStatus |= GED_EVENT_VR) :
			(g_ui32EventStatus &= (~GED_EVENT_VR));
		break;
	case GED_DVFS_VSYNC_OFFSET_DHWC_EVENT:
		(bSwitch) ? (g_ui32EventStatus |= GED_EVENT_DHWC) :
			(g_ui32EventStatus &= (~GED_EVENT_DHWC));
		break;
	case GED_DVFS_VSYNC_OFFSET_GAS_EVENT:
		(bSwitch) ? (g_ui32EventStatus |= GED_EVENT_GAS) :
			(g_ui32EventStatus &= (~GED_EVENT_GAS));
		ged_monitor_3D_fence_set_enable(!bSwitch);
		ret = ged_dvfs_probe_signal(GED_GAS_SIGNAL_EVENT);
		break;
	case GED_DVFS_VSYNC_OFFSET_LOW_POWER_MODE_EVENT:
		(bSwitch) ? (g_ui32EventStatus |= GED_EVENT_LOW_POWER_MODE) :
			(g_ui32EventStatus &= (~GED_EVENT_LOW_POWER_MODE));
		ret = ged_dvfs_probe_signal(GED_LOW_POWER_MODE_SIGNAL_EVENT);
		break;
	case GED_DVFS_VSYNC_OFFSET_MHL4K_VID_EVENT:
		(bSwitch) ? (g_ui32EventStatus |= GED_EVENT_MHL4K_VID) :
			(g_ui32EventStatus &= (~GED_EVENT_MHL4K_VID));
		ret = ged_dvfs_probe_signal(GED_MHL4K_VID_SIGNAL_EVENT);
		break;
	case GED_DVFS_VSYNC_OFFSET_VILTE_VID_EVENT:
		(bSwitch) ? (g_ui32EventStatus |= GED_EVENT_VILTE_VID) :
			(g_ui32EventStatus &= (~GED_EVENT_VILTE_VID));
		ret = ged_dvfs_probe_signal(GED_VILTE_VID_SIGNAL_EVENT);
		break;
	case GED_DVFS_BOOST_HOST_EVENT:
		ret = ged_dvfs_probe_signal(GED_SIGNAL_BOOST_HOST_EVENT);
		goto CHECK_OUT;

	case GED_DVFS_VSYNC_OFFSET_LOW_LATENCY_MODE_EVENT:
		(bSwitch) ?
		(g_ui32EventStatus |= GED_EVENT_LOW_LATENCY_MODE) :
		(g_ui32EventStatus &= (~GED_EVENT_LOW_LATENCY_MODE));
		ret = ged_dvfs_probe_signal
		(GED_LOW_LATENCY_MODE_SIGNAL_EVENT);
		break;
	default:
		GED_LOGE("%s: not acceptable event:%u\n", __func__, eEvent);
		ret = GED_ERROR_INVALID_PARAMS;
		goto CHECK_OUT;
	}
	mtk_ged_event_notify(g_ui32EventStatus);

	if ((ui32BeforeSwitchInterpret != g_ui32EventStatus) ||
		(ui32BeforeDebugInterpret != g_ui32EventDebugStatus) ||
		(g_ui32EventDebugStatus & GED_EVENT_NOT_SYNC))
		ret = ged_dvfs_probe_signal(GED_DVFS_VSYNC_OFFSET_SIGNAL_EVENT);

CHECK_OUT:
	mutex_unlock(&gsVSyncOffsetLock);
	return ret;
}

void ged_dvfs_vsync_offset_level_set(int i32level)
{
	g_VsyncOffsetLevel = i32level;
}

int ged_dvfs_vsync_offset_level_get(void)
{
	return g_VsyncOffsetLevel;
}

GED_ERROR ged_dvfs_um_commit(unsigned long gpu_tar_freq, bool bFallback)
{
#ifdef ENABLE_COMMON_DVFS
	int i;
	int ui32CurFreqID;
	int minfreq_idx;
	unsigned int ui32NewFreqID = 0;
	unsigned long gpu_freq;
	unsigned int sentinalLoading = 0;

	unsigned long ui32IRQFlags;

	ui32CurFreqID = ged_get_cur_oppidx();
	minfreq_idx = ged_get_min_oppidx();

	if (g_gpu_timer_based_emu)
		return GED_ERROR_INTENTIONAL_BLOCK;

#ifdef GED_DVFS_UM_CAL
	mutex_lock(&gsDVFSLock);

	if (gL_ulCalResetTS_us  - g_ulPreDVFS_TS_us != 0) {
		sentinalLoading =
		((gpu_loading * (gL_ulCalResetTS_us - gL_ulPreCalResetTS_us)) +
		100 * gL_ulWorkingPeriod_us) /
		(gL_ulCalResetTS_us - g_ulPreDVFS_TS_us);
		if (sentinalLoading > 100) {
			ged_log_buf_print(ghLogBuf_DVFS,
			"[GED_K] g_ulCalResetTS_us: %lu g_ulPreDVFS_TS_us: %lu",
				gL_ulCalResetTS_us, g_ulPreDVFS_TS_us);
			ged_log_buf_print(ghLogBuf_DVFS,
			"[GED_K] gpu_loading: %u g_ulPreCalResetTS_us:%lu",
				gpu_loading, gL_ulPreCalResetTS_us);
			ged_log_buf_print(ghLogBuf_DVFS,
			"[GED_K] g_ulWorkingPeriod_us: %lu",
				gL_ulWorkingPeriod_us);
			ged_log_buf_print(ghLogBuf_DVFS,
				"[GED_K] gpu_av_loading: WTF");

			if (gL_ulWorkingPeriod_us == 0)
				sentinalLoading = gpu_loading;
			else
				sentinalLoading = 100;
		}
		gpu_loading = sentinalLoading;
	} else {
		ged_log_buf_print(ghLogBuf_DVFS,
			"[GED_K] gpu_av_loading: 5566/ %u", gpu_loading);
		gpu_loading = 0;
	}

	gpu_pre_loading = gpu_av_loading;
	gpu_av_loading = gpu_loading;

	spin_lock_irqsave(&load_info_lock, ui32IRQFlags);
	g_loading2_sum += gpu_loading;
	g_loading2_count += 1;
	spin_unlock_irqrestore(&load_info_lock, ui32IRQFlags);

	g_ulPreDVFS_TS_us = gL_ulCalResetTS_us;

	/* Magic to kill ged_srv */
	if (gpu_tar_freq & 0x1)
		ged_dvfs_probe_signal(GED_SRV_SUICIDE_EVENT);

	if (bFallback == true) /* fallback mode, gpu_tar_freq is freq idx */
		ged_dvfs_policy(gpu_loading, &ui32NewFreqID, 0, 0, 0, true);
	else {
		/* Search suitable frequency level */
		g_CommitType = MTK_GPU_DVFS_TYPE_VSYNCBASED;
		g_um_gpu_tar_freq = gpu_tar_freq;
		g_policy_tar_freq = g_um_gpu_tar_freq;
		g_mode = 1;

		ui32NewFreqID = minfreq_idx;
		for (i = 0; i <= minfreq_idx; i++) {
			gpu_freq = ged_get_freq_by_idx(i);

			if (gpu_tar_freq > gpu_freq) {
				if (i == 0)
					ui32NewFreqID = 0;
				else
					ui32NewFreqID = i-1;
				break;
			}
		}
	}

	ged_log_buf_print(ghLogBuf_DVFS,
		"[GED_K] rdy to commit (%u)", ui32NewFreqID);

	if (bFallback == true)
		ged_dvfs_gpu_freq_commit(ui32NewFreqID,
				ged_get_freq_by_idx(ui32NewFreqID),
				GED_DVFS_LOADING_BASE_COMMIT);
	else
		ged_dvfs_gpu_freq_commit(ui32NewFreqID, gpu_tar_freq,
				GED_DVFS_LOADING_BASE_COMMIT);

	mutex_unlock(&gsDVFSLock);
#endif /* GED_DVFS_UM_CAL */
#else
	gpu_pre_loading = 0;
#endif /* ENABLE_COMMON_DVFS */

	return GED_OK;
}

#define DEFAULT_DVFS_MARGIN 300 /* 30% margin */
#define FIXED_FPS_MARGIN 3 /* Fixed FPS margin: 3fps */

int gx_fb_dvfs_margin = DEFAULT_DVFS_MARGIN;/* 10-bias */

#define MAX_DVFS_MARGIN 990 /* 99% margin */
#define MIN_DVFS_MARGIN 50 /* 5% margin */

/* dynamic margin mode for FPSGo control fps margin */
#define DYNAMIC_MARGIN_MODE_CONFIG_FPS_MARGIN 0x10

/* dynamic margin mode for fixed fps margin */
#define DYNAMIC_MARGIN_MODE_FIXED_FPS_MARGIN 0x11

/* dynamic margin mode, margin low bound 1% */
#define DYNAMIC_MARGIN_MODE_NO_FPS_MARGIN 0x12

/* dynamic margin mode, extend-config step and low bound */
#define DYNAMIC_MARGIN_MODE_EXTEND 0x13

/* dynamic margin mode, performance */
#define DYNAMIC_MARGIN_MODE_PERF 0x14

/* configure margin mode */
#define CONFIGURE_MARGIN_MODE 0x00

/* variable margin mode OPP Iidx */
#define VARIABLE_MARGIN_MODE_OPP_INDEX 0x01

#define MIN_MARGIN_INC_STEP 10 /* 1% headroom */

#define MAX_FALLBACK_DURATION 50   // unit: ms

// default frame-based margin mode + value is 130
static int dvfs_margin_value = DEFAULT_DVFS_MARGIN;
unsigned int dvfs_margin_mode = DYNAMIC_MARGIN_MODE_PERF;

static int dvfs_min_margin_inc_step = MIN_MARGIN_INC_STEP;
static int dvfs_margin_low_bound = MIN_DVFS_MARGIN;

module_param(gx_fb_dvfs_margin, int, 0644);

static int ged_dvfs_is_fallback_mode_triggered(void)
{
	return is_fallback_mode_triggered;
}

static void ged_dvfs_trigger_fb_dvfs(void)
{
	is_fb_dvfs_triggered = 1;
}

// input argument unit: nanosecond
static void set_fb_timeout(int t_gpu_target_ori, int t_gpu_target_margin)
{
	switch (g_frame_target_mode) {
	case 1:
		fb_timeout = (u64)t_gpu_target_ori * g_frame_target_time  / 10;
		break;
	case 2:
		fb_timeout = (u64)t_gpu_target_margin * g_frame_target_time / 10;
		break;
	default:
		fb_timeout = (u64)g_frame_target_time * 1000000; //ms to ns
		break;
	}
}

/*
 *	input argument t_gpu, t_gpu_target unit: ns
 */
static int ged_dvfs_fb_gpu_dvfs(int t_gpu, int t_gpu_target,
	int target_fps_margin, unsigned int force_fallback)
{
	unsigned int ap_workload, ap_workload_pipe,
		ap_workload_real;   // unit: 100*cycle
	unsigned long long frame_t_gpu;   // for gpu_util_history api
	unsigned int frame_workload, frame_freq;   // for gpu_util_history api
	int t_gpu_target_hd;   // apply headroom target, unit: us
	int t_gpu_pipe, t_gpu_real;   // unit: us
	int gpu_freq_pre, gpu_freq_tar, gpu_freq_floor;   // unit: KHZ
	int i, ui32NewFreqID = 0;
	int minfreq_idx;
	static int num_pre_frames;
	static int cur_frame_idx;
	static int pre_frame_idx;
	static int busy_cycle[GED_DVFS_BUSY_CYCLE_MONITORING_WINDOW_NUM];
	int gpu_busy_cycle = 0;
	int busy_cycle_cur;
	unsigned long ui32IRQFlags;
	static unsigned int force_fallback_pre;
	static int margin_low_bound;
	int ultra_high_step_size = (dvfs_step_mode & 0xff);

	gpu_freq_pre = ged_get_cur_freq();

	/* DVFS is not enabled */
	if (gpu_dvfs_enable == 0) {
		ged_log_buf_print(ghLogBuf_DVFS,
			"[GED_K][FB_DVFS] skip %s due to gpu_dvfs_enable=%u",
			__func__, gpu_dvfs_enable);
		is_fb_dvfs_triggered = 0;
		return gpu_freq_pre;
	}

	/* not main producer */
	if (t_gpu == -1) {
		is_fb_dvfs_triggered = 0;
		return gpu_freq_pre;
	}

	// force_fallback 0: FB, 1: LB
	if (force_fallback_pre != force_fallback) {
		force_fallback_pre = force_fallback;

		if (force_fallback == 1) {   // first transition from FB to LB
			/* Reset frame window when FB to LB */
			num_pre_frames = 0;
			for (i = 0; i < GED_DVFS_BUSY_CYCLE_MONITORING_WINDOW_NUM; i++)
				busy_cycle[i] = 0;
		}

	}
	/* use LB policy */
	if (force_fallback == 1) {
		if (ged_get_policy_state() == POLICY_STATE_FORCE_LB)
			// reset frame property for FB control in the next iteration
			gpu_util_history_query_frame_property(&frame_workload,
				&frame_t_gpu);
		ged_set_backup_timer_timeout(lb_timeout);
		is_fb_dvfs_triggered = 0;
		return gpu_freq_pre;
	}

	spin_lock_irqsave(&gsGpuUtilLock, ui32IRQFlags);
	if (is_fallback_mode_triggered)
		is_fallback_mode_triggered = 0;
	spin_unlock_irqrestore(&gsGpuUtilLock, ui32IRQFlags);

	/* obtain GPU frame property */
	t_gpu /= 100000;   // change unit from ns to 100*us
	t_gpu_target /= 100000;   // change unit from ns to 100*us

	gpu_util_history_query_frame_property(&frame_workload, &frame_t_gpu);
	t_gpu_pipe = t_gpu;
	t_gpu_real = (int) (frame_t_gpu / 100);   // change unit from us to 100*us
	// define t_gpu = max(t_gpu_completion, t_gpu_real) for FB policy
	if (t_gpu_real > t_gpu)
		t_gpu = t_gpu_real;

	frame_freq = gpu_util_history_query_frequency(t_gpu_pipe * 100);
	ap_workload_real =
		frame_workload / 100;   // change unit from cycle to 100*cycle
	ap_workload_pipe =
		frame_freq * t_gpu_pipe / 1000;   // change unit from 0.1 to 100*cycle
	ap_workload = ap_workload_pipe;
	// define workload = max(t_gpu_completion * avg_freq, freq * t_gpu_real)
	if (ap_workload_real > ap_workload)
		ap_workload = ap_workload_real;

	/* calculate margin */
	if (dvfs_margin_mode == CONFIGURE_MARGIN_MODE) {   // fix margin mode
		gx_fb_dvfs_margin = dvfs_margin_value;
		margin_low_bound = dvfs_margin_low_bound;
	} else if ((dvfs_margin_mode & 0x10) &&
			(t_gpu_target > 0)) {   // dynamic margin mode
		/* dvfs_margin_mode == */
		/* DYNAMIC_MARGIN_MODE_CONFIG_FPS_MARGIN or */
		/* DYNAMIC_MARGIN_MODE_FIXED_FPS_MARGIN) or */
		/* DYNAMIC_MARGIN_MODE_NO_FPS_MARGIN or */
		/* DYNAMIC_MARGIN_MODE_EXTEND or */
		/* DYNAMIC_MARGIN_MODE_PERF */
		int temp;

		if (dvfs_margin_mode == DYNAMIC_MARGIN_MODE_PERF) {
			t_gpu_target_hd = t_gpu_target *
				(1000 - dvfs_margin_low_bound) / 1000;
			if (t_gpu_target_hd <= 0)
				t_gpu_target_hd = 1;
		} else
			t_gpu_target_hd = t_gpu_target;
		// from this point on, t_gpu_target & t_gpu_target_hd would be > 0

		if (t_gpu > t_gpu_target_hd) {   // previous frame overdued
			// adjust margin
			temp = (gx_fb_dvfs_margin*(t_gpu-t_gpu_target_hd))
				/ t_gpu_target_hd;
			if (temp < dvfs_min_margin_inc_step)
				temp = dvfs_min_margin_inc_step;
			gx_fb_dvfs_margin += temp;
		} else {   // previous frame in time
			// set margin low boud according to FPSGO's margin (+3/+1)
			if (dvfs_margin_mode
				== DYNAMIC_MARGIN_MODE_NO_FPS_MARGIN)
				margin_low_bound = MIN_DVFS_MARGIN;
			else {
				int target_time_low_bound;

				if (dvfs_margin_mode ==
					DYNAMIC_MARGIN_MODE_FIXED_FPS_MARGIN)
					target_fps_margin = FIXED_FPS_MARGIN;

				if (target_fps_margin <= 0)
					margin_low_bound = MIN_DVFS_MARGIN;
				else {
					target_time_low_bound =
					10000/((10000/t_gpu_target) +
					target_fps_margin);

					margin_low_bound =
					1000 *
					(t_gpu_target - target_time_low_bound)
					/ t_gpu_target;
				}

				// margin_low_bound range clipping
				if (margin_low_bound > dvfs_margin_value)
					margin_low_bound = dvfs_margin_value;

				if (margin_low_bound < dvfs_margin_low_bound)
					margin_low_bound = dvfs_margin_low_bound;
			}

			// adjust margin
			temp = (gx_fb_dvfs_margin*(t_gpu_target_hd-t_gpu))
				/ t_gpu_target_hd;
			gx_fb_dvfs_margin -= temp;
		}
	} else {   // unknown mode
		// use fix margin mode
		gx_fb_dvfs_margin = dvfs_margin_value;
		margin_low_bound = dvfs_margin_low_bound;
	}

	// check upper bound
	if (gx_fb_dvfs_margin > dvfs_margin_value)
		gx_fb_dvfs_margin = dvfs_margin_value;

	// check lower bound
	if (gx_fb_dvfs_margin < margin_low_bound)
		gx_fb_dvfs_margin = margin_low_bound;

	//  * 10 to keep unit uniform
	Policy__Frame_based__Margin(dvfs_margin_value, gx_fb_dvfs_margin,
		margin_low_bound);

	Policy__Frame_based__Margin__Detail(dvfs_margin_mode, target_fps_margin,
		dvfs_min_margin_inc_step, dvfs_margin_low_bound);

	t_gpu_target_hd = t_gpu_target * (1000 - gx_fb_dvfs_margin) / 1000;

	//  * 100 to keep unit uniform
	Policy__Frame_based__GPU_Time((t_gpu * 100),
								  (t_gpu_target * 100),
								  (t_gpu_target_hd * 100),
								  (t_gpu_real * 100),
								  (t_gpu_pipe * 100));
	ged_log_perf_trace_counter("t_gpu", (long long)(t_gpu * 100), 5566, 0, 0);
	ged_log_perf_trace_counter("t_gpu_target", (long long)(t_gpu_target * 100),
		5566, 0, 0);

	// Hint target frame time w/z headroom
	if (ged_is_fdvfs_support())
		mtk_gpueb_dvfs_set_taget_frame_time(t_gpu_target_hd, gx_fb_dvfs_margin);

	/* select workload */
	if (ged_is_fdvfs_support() && is_fb_dvfs_triggered && is_fdvfs_enable()
		&& g_eb_workload != 0xFFFF) {
		g_eb_workload /= 100;   // change unit from cycle to 100*cycle
		busy_cycle_cur = (g_eb_workload < ap_workload) ? g_eb_workload : ap_workload;
	}
	else
		busy_cycle_cur = ap_workload;

	busy_cycle[cur_frame_idx] = busy_cycle_cur;

	if (num_pre_frames != GED_DVFS_BUSY_CYCLE_MONITORING_WINDOW_NUM - 1) {
		gpu_busy_cycle = busy_cycle[cur_frame_idx];
		num_pre_frames++;
	} else {
		for (i = 0; i < GED_DVFS_BUSY_CYCLE_MONITORING_WINDOW_NUM; i++)
			gpu_busy_cycle += busy_cycle[i];
		gpu_busy_cycle /= GED_DVFS_BUSY_CYCLE_MONITORING_WINDOW_NUM;
		gpu_busy_cycle = (gpu_busy_cycle > busy_cycle_cur) ?
			gpu_busy_cycle : busy_cycle_cur;
	}

	Policy__Frame_based__Workload((busy_cycle_cur * 100),
		(gpu_busy_cycle * 100), (ap_workload_real * 100),
		(ap_workload_pipe * 100), ged_get_dvfs_loading_mode());

	/* compute target frequency */
	if (t_gpu_target_hd != 0)
		gpu_freq_tar = (gpu_busy_cycle / t_gpu_target_hd) * 1000;
	else
		gpu_freq_tar = gpu_freq_pre;

	gpu_freq_floor = gpu_freq_pre * GED_FB_DVFS_FERQ_DROP_RATIO_LIMIT / 100;
	if (gpu_freq_tar < gpu_freq_floor)
		gpu_freq_tar = gpu_freq_floor;

	Policy__Frame_based__Frequency(gpu_freq_tar, gpu_freq_floor);
	pre_frame_idx = cur_frame_idx;
	cur_frame_idx = (cur_frame_idx + 1) %
		GED_DVFS_BUSY_CYCLE_MONITORING_WINDOW_NUM;

	minfreq_idx = ged_get_min_oppidx();
	ui32NewFreqID = minfreq_idx;

	for (i = 0; i <= minfreq_idx; i++) {
		int gpu_freq;

		gpu_freq = ged_get_freq_by_idx(i);

		if (gpu_freq_tar > gpu_freq) {
			if (i == 0)
				ui32NewFreqID = 0;
			else
				ui32NewFreqID = i-1;
			break;
		}
	}

	if (dvfs_margin_mode == VARIABLE_MARGIN_MODE_OPP_INDEX)
		gx_fb_dvfs_margin = (ui32NewFreqID / 3)*10;

	ged_log_buf_print(ghLogBuf_DVFS,
	"[GED_K][FB_DVFS]t_gpu:%d,t_gpu_tar_hd:%d,gpu_freq_tar:%d,gpu_freq_pre:%d",
	t_gpu, t_gpu_target_hd, gpu_freq_tar, gpu_freq_pre);

	ged_log_buf_print(ghLogBuf_DVFS,
	"[GED_K][FB_DVFS]mode:%x,h:%d,margin:%d,l:%d,fps:+%d,l_b:%d,step:%d",
	dvfs_margin_mode, dvfs_margin_value, gx_fb_dvfs_margin,
	margin_low_bound, target_fps_margin, dvfs_margin_low_bound,
	dvfs_min_margin_inc_step);
	g_CommitType = MTK_GPU_DVFS_TYPE_VSYNCBASED;
	ged_dvfs_gpu_freq_commit((unsigned long)ui32NewFreqID,
		gpu_freq_tar, GED_DVFS_FRAME_BASE_COMMIT);

	//t_gpu_target(unit: 100us) *10^5 =nanosecond
	set_fb_timeout(t_gpu_target * 100000, t_gpu_target_hd * 100000);
	ged_set_backup_timer_timeout(fb_timeout);
	ged_cancel_backup_timer();
	is_fb_dvfs_triggered = 0;
	fallback_duration = 0;   // reset @ FB
	return gpu_freq_tar;
}

static int _loading_avg(int ui32loading)
{
	static int data[4];
	static int idx;
	static int sum;

	int cidx = ++idx % ARRAY_SIZE(data);

	sum += ui32loading - data[cidx];
	data[cidx] = ui32loading;

	return sum / ARRAY_SIZE(data);
}
int get_api_sync_flag(void)
{
	return api_sync_flag;
}
void set_api_sync_flag(int flag)
{
	api_sync_flag = flag;
}
static bool ged_dvfs_policy(
		unsigned int ui32GPULoading, unsigned int *pui32NewFreqID,
		unsigned long t, long phase, unsigned long ul3DFenceDoneTime,
		bool bRefreshed)
{
	int ui32GPUFreq = ged_get_cur_oppidx();
	unsigned int sentinalLoading = 0;
	unsigned int window_size_ms = g_loading_slide_window_size;
	int i32NewFreqID = (int)ui32GPUFreq;

	unsigned long ui32IRQFlags;

	int loading_mode;
	int minfreq_idx;
	int idx_diff = 0;

	if (ui32GPUFreq < 0 || ui32GPUFreq > ged_get_min_oppidx())
		return GED_FALSE;

	g_um_gpu_tar_freq = 0;
	if (bRefreshed == false) {
		if (gL_ulCalResetTS_us - g_ulPreDVFS_TS_us != 0) {
			sentinalLoading = ((gpu_loading *
				(gL_ulCalResetTS_us - gL_ulPreCalResetTS_us)) +
				100 * gL_ulWorkingPeriod_us) /
				(gL_ulCalResetTS_us - g_ulPreDVFS_TS_us);

			if (sentinalLoading > 100) {
				ged_log_buf_print(ghLogBuf_DVFS,
		"[GED_K1] g_ulCalResetTS_us: %lu g_ulPreDVFS_TS_us: %lu",
					gL_ulCalResetTS_us, g_ulPreDVFS_TS_us);
				ged_log_buf_print(ghLogBuf_DVFS,
		"[GED_K1] gpu_loading: %u g_ulPreCalResetTS_us:%lu",
					gpu_loading, gL_ulPreCalResetTS_us);
				ged_log_buf_print(ghLogBuf_DVFS,
		"[GED_K1] g_ulWorkingPeriod_us: %lu",
					gL_ulWorkingPeriod_us);
				ged_log_buf_print(ghLogBuf_DVFS,
						"[GED_K1] gpu_av_loading: WTF");

				if (gL_ulWorkingPeriod_us == 0)
					sentinalLoading = gpu_loading;
				else
					sentinalLoading = 100;
			}
			gpu_loading = sentinalLoading;
		} else {
			ged_log_buf_print(ghLogBuf_DVFS,
			"[GED_K1] gpu_av_loading: 5566 / %u", gpu_loading);
			gpu_loading = 0;
		}

		g_ulPreDVFS_TS_us = gL_ulCalResetTS_us;

		gpu_pre_loading = gpu_av_loading;
		ui32GPULoading = gpu_loading;
		gpu_av_loading = gpu_loading;

		spin_lock_irqsave(&load_info_lock, ui32IRQFlags);
		g_loading2_sum += gpu_loading;
		g_loading2_count += 1;
		spin_unlock_irqrestore(&load_info_lock, ui32IRQFlags);
	}

	minfreq_idx = ged_get_min_oppidx();
	if (g_gpu_timer_based_emu) {
		/* legacy timer-based policy */
		if (ui32GPULoading >= 99)
			i32NewFreqID = 0;
		else if (ui32GPULoading <= 1)
			i32NewFreqID = minfreq_idx;
		else if (ui32GPULoading >= 85)
			i32NewFreqID -= 2;
		else if (ui32GPULoading <= 30)
			i32NewFreqID += 2;
		else if (ui32GPULoading >= 70)
			i32NewFreqID -= 1;
		else if (ui32GPULoading <= 50)
			i32NewFreqID += 1;

		if (i32NewFreqID < ui32GPUFreq) {
			if (gpu_pre_loading * 17 / 10 < ui32GPULoading)
				i32NewFreqID -= 1;
		} else if (i32NewFreqID > ui32GPUFreq) {
			if (ui32GPULoading * 17 / 10 < gpu_pre_loading)
				i32NewFreqID += 1;
		}

		g_CommitType = MTK_GPU_DVFS_TYPE_TIMERBASED;
	} else {
		/* new timer-based policy */
		int t_gpu, t_gpu_complete, t_gpu_uncomplete,
			t_gpu_target, t_gpu_target_hd;
		GED_ERROR ret_risky_bq;
		struct ged_risky_bq_info info;
		int uncomplete_flag = 0;
		enum gpu_dvfs_policy_state policy_state;
		int api_sync_flag;
		int fallback_duration_flag;

		/* set t_gpu via risky BQ analysis */
		ged_kpi_update_t_gpu_latest_uncompleted();
		ret_risky_bq = ged_kpi_timer_based_pick_riskyBQ(&info);
		if (ret_risky_bq == GED_OK) {
			t_gpu_complete = (int) info.completed_bq.t_gpu;
			t_gpu_uncomplete = (int) info.uncompleted_bq.t_gpu;

			// pick the largest t_gpu/t_gpu_target & set uncomplete_flag
			t_gpu = t_gpu_uncomplete;
			t_gpu_target = info.uncompleted_bq.t_gpu_target;
			if (g_tb_dvfs_margin_mode & DYNAMIC_TB_PERF_MODE_MASK)
				t_gpu_target_hd = t_gpu_target
					* (100 - g_tb_dvfs_margin_value_min) / 100;
			else
				t_gpu_target_hd = t_gpu_target;
			if (t_gpu > t_gpu_target_hd)
				// priority #1: uncompleted frame overtime
				uncomplete_flag = 1;
			else {
				// priority #2: frame with higher t_gpu/t_gpu_target ratio
				unsigned long long risk_completed =
					info.completed_bq.risk;
				unsigned long long risk_uncompleted =
					info.uncompleted_bq.risk;

				if (risk_completed > risk_uncompleted) {
					t_gpu = t_gpu_complete;
					t_gpu_target = info.completed_bq.t_gpu_target;
				}
				uncomplete_flag = 0;
			}
		} else {
			// risky BQ cannot be obtained, set t_gpu to default
			t_gpu = -1;
			t_gpu_complete = -1;
			t_gpu_uncomplete = -1;
			t_gpu_target = -1;
		}
		t_gpu_target_hd = t_gpu_target;   // set init value

		/* update margin */
		if (g_tb_dvfs_margin_mode & DYNAMIC_TB_MASK) {
			/* dynamic margin mode */
			if (ret_risky_bq == GED_OK) {
				static unsigned int prev_gpu_completed_count;
				unsigned int gpu_completed_count =
					info.total_gpu_completed_count;

				// overwrite t_gpu_target_hd in perf mode
				if (g_tb_dvfs_margin_mode & DYNAMIC_TB_PERF_MODE_MASK)
					t_gpu_target_hd = t_gpu_target
						* (100 - g_tb_dvfs_margin_value_min) / 100;

				// margin modifying
				if (t_gpu > t_gpu_target_hd) {   // overtime
					/* increase margin flow */
					if (uncomplete_flag)
						// frame still not done, so keep updating margin
						gx_tb_dvfs_margin += g_tb_dvfs_margin_step;
					else if (gpu_completed_count != prev_gpu_completed_count)
						// frame already done, so update once per frame done
						gx_tb_dvfs_margin += g_tb_dvfs_margin_step;
				} else {   // in time
					/* decrease margin flow */
					if (gpu_completed_count != prev_gpu_completed_count)
						gx_tb_dvfs_margin -= g_tb_dvfs_margin_step;
				}

				// margin range clipping
				if (gx_tb_dvfs_margin > g_tb_dvfs_margin_value)
					gx_tb_dvfs_margin = g_tb_dvfs_margin_value;
				if (gx_tb_dvfs_margin < g_tb_dvfs_margin_value_min)
					gx_tb_dvfs_margin = g_tb_dvfs_margin_value_min;

				// update prev_gpu_completed_count for next iteration
				prev_gpu_completed_count = gpu_completed_count;
			} else
				// use fix margin if risky BQ cannot be obtained
				gx_tb_dvfs_margin = g_tb_dvfs_margin_value;
		} else
			/* fix margin mode */
			gx_tb_dvfs_margin = g_tb_dvfs_margin_value;

		// change to fallback mode if frame is overdued (only in LB)
		policy_state = ged_get_policy_state();
		api_sync_flag = get_api_sync_flag();
		if (policy_state == POLICY_STATE_LB ||
				policy_state == POLICY_STATE_LB_FALLBACK) {
			// overwrite state & timeout value set prior to ged_dvfs_run
			if (uncomplete_flag || api_sync_flag) {
				ged_set_policy_state(POLICY_STATE_LB_FALLBACK);
				ged_set_backup_timer_timeout(ged_get_fallback_time());
			} else {
				ged_set_policy_state(POLICY_STATE_LB);
				ged_set_backup_timer_timeout(lb_timeout);
			}
		} else if (policy_state == POLICY_STATE_FORCE_LB ||
				policy_state == POLICY_STATE_FORCE_LB_FALLBACK) {
			// overwrite state & timeout value set prior to ged_dvfs_run
			if (uncomplete_flag || api_sync_flag) {
				ged_set_policy_state(POLICY_STATE_FORCE_LB_FALLBACK);
				ged_set_backup_timer_timeout(ged_get_fallback_time());
			} else {
				ged_set_policy_state(POLICY_STATE_FORCE_LB);
				ged_set_backup_timer_timeout(lb_timeout);
			}
		}

		// limit fallback state duration
		policy_state = ged_get_policy_state();
		if (policy_state == POLICY_STATE_LB_FALLBACK ||
				policy_state == POLICY_STATE_FORCE_LB_FALLBACK ||
				policy_state == POLICY_STATE_FB_FALLBACK)
			fallback_duration += g_fallback_time;   // accumulate
		else
			fallback_duration = 0;   // reset @ LB & FORCE_LB
		if (fallback_duration > MAX_FALLBACK_DURATION)
			fallback_duration_flag = 1;   // fallback exceed maximum duration
		else
			fallback_duration_flag = 0;

		// use fix margin in fallback mode
		policy_state = ged_get_policy_state();
		if (policy_state == POLICY_STATE_FB_FALLBACK ||
				policy_state == POLICY_STATE_LB_FALLBACK ||
				policy_state == POLICY_STATE_FORCE_LB_FALLBACK)
			gx_tb_dvfs_margin = g_tb_dvfs_margin_value;

		// overwrite FB fallback to LB if there're no pending main head frames
		if (policy_state == POLICY_STATE_FB_FALLBACK &&
				ged_kpi_get_main_bq_uncomplete_count() <= 0)
			ged_set_policy_state(POLICY_STATE_LB);

		Policy__Loading_based__Margin(g_tb_dvfs_margin_value*10,
			gx_tb_dvfs_margin*10, g_tb_dvfs_margin_value_min*10);
		Policy__Loading_based__Margin__Detail(g_tb_dvfs_margin_mode,
			g_tb_dvfs_margin_step, g_tb_dvfs_margin_value_min*10);
		Policy__Loading_based__GPU_Time(t_gpu, t_gpu_target, t_gpu_target_hd,
			t_gpu_complete, t_gpu_uncomplete);
		ged_log_perf_trace_counter("t_gpu", (long long)t_gpu, 5566, 0, 0);
		ged_log_perf_trace_counter("t_gpu_target", (long long)t_gpu_target,
			5566, 0, 0);

		/* bound update */
		if (init == 0) {
			init = 1;
			gx_tb_dvfs_margin_cur = gx_tb_dvfs_margin;
			_init_loading_ud_table();
		}
		if (gx_tb_dvfs_margin != gx_tb_dvfs_margin_cur
				&& gx_tb_dvfs_margin < 100
				&& gx_tb_dvfs_margin >= 0) {
			gx_tb_dvfs_margin_cur = gx_tb_dvfs_margin;
			_init_loading_ud_table();
		}

		int ultra_high = 95;
		int high = loading_ud_table[ui32GPUFreq].up;
		int low = loading_ud_table[ui32GPUFreq].down;
		int ultra_low = 20;
		int ultra_high_step_size = (dvfs_step_mode & 0xff);
		int ultra_low_step_size = (dvfs_step_mode & 0xff00) >> 8;

		if (high > ultra_high)
			high = ultra_high;
		if (low < ultra_low)
			low = ultra_low;

		Policy__Loading_based__Bound(ultra_high, high, low, ultra_low);

		/* opp control */
		// use fallback window size in fallback mode
		if (policy_state == POLICY_STATE_FB_FALLBACK ||
				policy_state == POLICY_STATE_LB_FALLBACK ||
				policy_state == POLICY_STATE_FORCE_LB_FALLBACK)
			window_size_ms = g_fallback_window_size;
		ui32GPULoading = gpu_util_history_query_loading(window_size_ms * 1000);
		loading_mode = ged_get_dvfs_loading_mode();
		Policy__Loading_based__Loading(ui32GPULoading, loading_mode);

		if (ui32GPULoading >= ultra_high)
			i32NewFreqID -= ultra_high_step_size;
		else if (ui32GPULoading < ultra_low)
			i32NewFreqID += ultra_low_step_size;
		else if (ui32GPULoading >= high)
			i32NewFreqID -= 1;
		else if (ui32GPULoading <= low)
			i32NewFreqID += 1;

		// prevent decreasing opp in fallback mode
		if ((policy_state == POLICY_STATE_FB_FALLBACK ||
				policy_state == POLICY_STATE_LB_FALLBACK ||
				policy_state == POLICY_STATE_FORCE_LB_FALLBACK) &&
				i32NewFreqID > ui32GPUFreq) {
			if (g_fallback_frequency_adjust && !fallback_duration_flag)
				i32NewFreqID = ui32GPUFreq;
		}

		ged_log_buf_print(ghLogBuf_DVFS,
			"[GED_K][LB_DVFS] mode:0x%x, u_b:%d, l_b:%d, margin:%d, complete:%d, uncomplete:%d, t_gpu:%d, target:%d",
			g_tb_dvfs_margin_mode,
			g_tb_dvfs_margin_value,
			g_tb_dvfs_margin_value_min,
			gx_tb_dvfs_margin_cur,
			t_gpu_complete,
			t_gpu_uncomplete,
			t_gpu,
			t_gpu_target);
		ged_log_buf_print(ghLogBuf_DVFS,
			"[GED_K1] rdy gpu_av_loading:%u, %d(%d)-up:%d,%d, new: %d, step: 0x%x",
			ui32GPULoading,
			ui32GPUFreq,
			loading_ud_table[ui32GPUFreq].freq,
			loading_ud_table[ui32GPUFreq].up,
			loading_ud_table[ui32GPUFreq].down,
			i32NewFreqID,
			dvfs_step_mode);

		g_CommitType = MTK_GPU_DVFS_TYPE_FALLBACK;
	}

	if (i32NewFreqID > minfreq_idx)
		i32NewFreqID = minfreq_idx;
	else if (i32NewFreqID < 0)
		i32NewFreqID = 0;

	Policy__Loading_based__Opp(i32NewFreqID);
	*pui32NewFreqID = (unsigned int)i32NewFreqID;
	g_policy_tar_freq = ged_get_freq_by_idx(i32NewFreqID);

	g_mode = 2;

	return ((*pui32NewFreqID != ui32GPUFreq) || ged_log_perf_trace_enable)
		? GED_TRUE : GED_FALSE;
}

#ifdef ENABLE_COMMON_DVFS
static void ged_dvfs_freq_input_boostCB(unsigned int ui32BoostFreqID)
{
	if (g_iSkipCount > 0)
		return;

	if (boost_gpu_enable == 0)
		return;

	mutex_lock(&gsDVFSLock);

	if (ui32BoostFreqID < ged_get_cur_oppidx()) {
		if (ged_dvfs_gpu_freq_commit(ui32BoostFreqID,
				ged_get_freq_by_idx(ui32BoostFreqID),
				GED_DVFS_INPUT_BOOST_COMMIT))
			g_dvfs_skip_round = GED_DVFS_SKIP_ROUNDS;
	}
	mutex_unlock(&gsDVFSLock);
}

void ged_dvfs_boost_gpu_freq(void)
{
	if (gpu_debug_enable)
		GED_LOGD("@%s", __func__);

	ged_dvfs_freq_input_boostCB(0);
}

/* Set buttom gpufreq by from PowerHal  API Boost */
static void ged_dvfs_set_bottom_gpu_freq(unsigned int ui32FreqLevel)
{
	int minfreq_idx;
	static unsigned int s_bottom_freq_id;

	minfreq_idx = ged_get_min_oppidx();

	if (gpu_debug_enable)
		GED_LOGD("@%s: freq = %d", __func__, ui32FreqLevel);

	if (minfreq_idx < ui32FreqLevel)
		ui32FreqLevel = minfreq_idx;

	mutex_lock(&gsDVFSLock);

	/* 0 => The highest frequency */
	/* table_num - 1 => The lowest frequency */
	s_bottom_freq_id = minfreq_idx - ui32FreqLevel;

	ged_set_limit_floor(LIMITER_APIBOOST, s_bottom_freq_id);

	gpu_bottom_freq = ged_get_freq_by_idx(s_bottom_freq_id);
	if (g_bottom_freq_id < s_bottom_freq_id) {
		g_bottom_freq_id = s_bottom_freq_id;
		if (s_bottom_freq_id < g_last_def_commit_freq_id)
			ged_dvfs_gpu_freq_commit(s_bottom_freq_id,
			gpu_bottom_freq,
			GED_DVFS_SET_BOTTOM_COMMIT);
		else
			ged_dvfs_gpu_freq_commit(g_last_def_commit_freq_id,
			ged_get_freq_by_idx(g_last_def_commit_freq_id),
			GED_DVFS_SET_BOTTOM_COMMIT);
	} else {
	/* if current id is larger, ie lower freq, reflect immedately */
		g_bottom_freq_id = s_bottom_freq_id;
		if (s_bottom_freq_id < ged_get_cur_oppidx())
			ged_dvfs_gpu_freq_commit(s_bottom_freq_id,
			gpu_bottom_freq,
			GED_DVFS_SET_BOTTOM_COMMIT);
	}

	mutex_unlock(&gsDVFSLock);
}

static unsigned int ged_dvfs_get_gpu_freq_level_count(void)
{
	return ged_get_opp_num_real();
}

/* set buttom gpufreq from PowerHal by MIN_FREQ */
static void ged_dvfs_custom_boost_gpu_freq(unsigned int ui32FreqLevel)
{
	int minfreq_idx;

	minfreq_idx = ged_get_min_oppidx_real();

	if (gpu_debug_enable)
		GED_LOGD("@%s: freq = %d", __func__, ui32FreqLevel);

	if (minfreq_idx < ui32FreqLevel)
		ui32FreqLevel = minfreq_idx;

	mutex_lock(&gsDVFSLock);

	/* 0 => The highest frequency */
	/* table_num - 1 => The lowest frequency */
	g_cust_boost_freq_id = ui32FreqLevel;

	ged_set_limit_floor(LIMITER_FPSGO, g_cust_boost_freq_id);

	gpu_cust_boost_freq = ged_get_freq_by_idx(g_cust_boost_freq_id);
	if (g_cust_boost_freq_id < ged_get_cur_oppidx()) {
		ged_dvfs_gpu_freq_commit(g_cust_boost_freq_id,
			gpu_cust_boost_freq, GED_DVFS_CUSTOM_BOOST_COMMIT);
	}

	mutex_unlock(&gsDVFSLock);
}

/* set buttom gpufreq from PowerHal by MAX_FREQ */
static void ged_dvfs_custom_ceiling_gpu_freq(unsigned int ui32FreqLevel)
{
	int minfreq_idx;

	minfreq_idx = ged_get_min_oppidx_real();

	if (gpu_debug_enable)
		GED_LOGD("@%s: freq = %d", __func__, ui32FreqLevel);

	if (minfreq_idx < ui32FreqLevel)
		ui32FreqLevel = minfreq_idx;

	mutex_lock(&gsDVFSLock);

	/* 0 => The highest frequency */
	/* table_num - 1 => The lowest frequency */
	g_cust_upbound_freq_id = ui32FreqLevel;

	ged_set_limit_ceil(LIMITER_FPSGO, g_cust_upbound_freq_id);

	gpu_cust_upbound_freq = ged_get_freq_by_idx(g_cust_upbound_freq_id);
	if (g_cust_upbound_freq_id > ged_get_cur_oppidx())
		ged_dvfs_gpu_freq_commit(g_cust_upbound_freq_id,
		gpu_cust_upbound_freq, GED_DVFS_CUSTOM_CEIL_COMMIT);

	mutex_unlock(&gsDVFSLock);
}

static unsigned int ged_dvfs_get_bottom_gpu_freq(void)
{
	unsigned int ui32MaxLevel;

	ui32MaxLevel = ged_get_min_oppidx();

	return ui32MaxLevel - g_bottom_freq_id;
}

static unsigned long ged_get_gpu_bottom_freq(void)
{
	return ged_get_freq_by_idx(g_bottom_freq_id);
}
#endif /* ENABLE_COMMON_DVFS */

unsigned int ged_dvfs_get_custom_ceiling_gpu_freq(void)
{
	return g_cust_upbound_freq_id;
}

unsigned int ged_dvfs_get_custom_boost_gpu_freq(void)
{
	return g_cust_boost_freq_id;
}

static void ged_dvfs_margin_value(int i32MarginValue)
{
	/* -1:  default: configure margin mode */
	/* -2:  variable margin mode by opp index */
	/* 0~100: configure margin mode */
	/* 101~199:  dynamic margin mode - CONFIG_FPS_MARGIN */
	/* 201~299:  dynamic margin mode - FIXED_FPS_MARGIN */
	/* 301~399:  dynamic margin mode - NO_FPS_MARGIN */
	/* 401~499:  dynamic margin mode - EXTEND MODE */
	/* 501~599:  dynamic margin mode - PERF MODE */

	/****************************************************/
	/* bit0~bit9: dvfs_margin_value setting             */
	/*                                                  */
	/* EXTEND MODE (401~499):                           */
	/* PERF MODE (501~599):                             */
	/* bit10~bit15: min. inc step (0~63 step)           */
	/* bit16~bit22: (0~100%)dynamic headroom low bound  */
	/****************************************************/

	int i32MarginValue_ori;

	mutex_lock(&gsDVFSLock);

	g_fastdvfs_margin = 0;

	if (i32MarginValue == -1) {
		dvfs_margin_mode = CONFIGURE_MARGIN_MODE;
		dvfs_margin_value = DEFAULT_DVFS_MARGIN;
		mutex_unlock(&gsDVFSLock);
		return;
	} else if (i32MarginValue == -2) {
		dvfs_margin_mode = VARIABLE_MARGIN_MODE_OPP_INDEX;
		mutex_unlock(&gsDVFSLock);
		return;
	} else if (i32MarginValue == 999) {
		g_fastdvfs_margin = 1;
		mutex_unlock(&gsDVFSLock);
		return;
	}

	i32MarginValue_ori = i32MarginValue;
	i32MarginValue = i32MarginValue & 0x3ff;

	if ((i32MarginValue >= 0) && (i32MarginValue <= 100))
		dvfs_margin_mode = CONFIGURE_MARGIN_MODE;
	else if ((i32MarginValue > 100) && (i32MarginValue < 200)) {
		dvfs_margin_mode = DYNAMIC_MARGIN_MODE_CONFIG_FPS_MARGIN;
		i32MarginValue = i32MarginValue - 100;
	} else if ((i32MarginValue > 200) && (i32MarginValue < 300)) {
		dvfs_margin_mode = DYNAMIC_MARGIN_MODE_FIXED_FPS_MARGIN;
		i32MarginValue = i32MarginValue - 200;
	} else if ((i32MarginValue > 300) && (i32MarginValue < 400)) {
		dvfs_margin_mode = DYNAMIC_MARGIN_MODE_NO_FPS_MARGIN;
		i32MarginValue = i32MarginValue - 300;
	} else if ((i32MarginValue > 400) && (i32MarginValue < 500)) {
		dvfs_margin_mode = DYNAMIC_MARGIN_MODE_EXTEND;
		i32MarginValue = i32MarginValue - 400;
	} else if ((i32MarginValue > 500) && (i32MarginValue < 600)) {
		dvfs_margin_mode = DYNAMIC_MARGIN_MODE_PERF;
		i32MarginValue = i32MarginValue - 500;
	}
	// unit: % to 10*%
	dvfs_margin_value = i32MarginValue * 10;
	dvfs_min_margin_inc_step = ((i32MarginValue_ori & 0xfc00) >> 10) * 10;
	dvfs_margin_low_bound = ((i32MarginValue_ori & 0x7f0000) >> 16) * 10;

	// range clipping
	if (dvfs_margin_value > MAX_DVFS_MARGIN)
		dvfs_margin_value = MAX_DVFS_MARGIN;
	if (dvfs_margin_value < MIN_DVFS_MARGIN)
		dvfs_margin_value = MIN_DVFS_MARGIN;
	if (dvfs_margin_low_bound > MAX_DVFS_MARGIN)
		dvfs_margin_low_bound = MAX_DVFS_MARGIN;
	if (dvfs_margin_low_bound < MIN_DVFS_MARGIN)
		dvfs_margin_low_bound = MIN_DVFS_MARGIN;
	if (dvfs_min_margin_inc_step < MIN_MARGIN_INC_STEP)
		dvfs_min_margin_inc_step = MIN_MARGIN_INC_STEP;

	mutex_unlock(&gsDVFSLock);
}

static int ged_get_dvfs_margin_value(void)
{
	int ret = 0;

	if (dvfs_margin_mode == CONFIGURE_MARGIN_MODE)
		ret = dvfs_margin_value/10;
	else if (dvfs_margin_mode == DYNAMIC_MARGIN_MODE_CONFIG_FPS_MARGIN)
		ret = dvfs_margin_value/10 + 100;
	else if (dvfs_margin_mode == DYNAMIC_MARGIN_MODE_FIXED_FPS_MARGIN)
		ret = dvfs_margin_value/10 + 200;
	else if (dvfs_margin_mode == DYNAMIC_MARGIN_MODE_NO_FPS_MARGIN)
		ret = dvfs_margin_value/10 + 300;
	else if (dvfs_margin_mode == DYNAMIC_MARGIN_MODE_EXTEND)
		ret = dvfs_margin_value/10 + 400;
	else if (dvfs_margin_mode == DYNAMIC_MARGIN_MODE_PERF)
		ret = dvfs_margin_value/10 + 500;
	else if (dvfs_margin_mode == VARIABLE_MARGIN_MODE_OPP_INDEX)
		ret = -2;

	if (g_fastdvfs_margin)
		ret = 999;

	return ret;
}

int ged_get_dvfs_margin(void)
{
	return gx_fb_dvfs_margin;
}

unsigned int ged_get_dvfs_margin_mode(void)
{
	return dvfs_margin_mode;
}

static void ged_loading_base_dvfs_step(int i32StepValue)
{
	/* bit0~bit7: ultra high step */
	/* bit8~bit15: ultra low step */

	mutex_lock(&gsDVFSLock);

	dvfs_step_mode = i32StepValue;

	mutex_unlock(&gsDVFSLock);
}

static int ged_get_loading_base_dvfs_step(void)
{
	return dvfs_step_mode;
}

static void ged_timer_base_dvfs_margin(int i32MarginValue)
{
	/*
	 * value < 0: default, GED_DVFS_TIMER_BASED_DVFS_MARGIN
	 * bit   7~0: margin value
	 * bit     8: dynamic timer based dvfs margin
	 * bit     9: use gpu pipe time for dynamic timer based dvfs margin
	 * bit    10: use performance mode for dynamic timer based dvfs margin
	 * bit    11: fix target FPS to 30 for dynamic timer based dvfs margin
	 * bit 23~16: min margin value
	 * bit 27~24: margin update step
	 */
	unsigned int mode = CONFIGURE_TIMER_BASED_MODE;
	int value = i32MarginValue & TIMER_BASED_MARGIN_MASK;
	int value_min = (i32MarginValue >> 16) & TIMER_BASED_MARGIN_MASK;
	int margin_step = (i32MarginValue >> 24) & 0x0000000F;

	if (i32MarginValue < 0)
		value = GED_DVFS_TIMER_BASED_DVFS_MARGIN;
	else {
		mode = (i32MarginValue & DYNAMIC_TB_MASK) ?
				(mode | DYNAMIC_TB_MASK) : mode;
		mode = (i32MarginValue & DYNAMIC_TB_PIPE_TIME_MASK) ?
				(mode | DYNAMIC_TB_PIPE_TIME_MASK) : mode;
		mode = (i32MarginValue & DYNAMIC_TB_PERF_MODE_MASK) ?
				(mode | DYNAMIC_TB_PERF_MODE_MASK) : mode;
		mode = (i32MarginValue & DYNAMIC_TB_FIX_TARGET_MASK) ?
				(mode | DYNAMIC_TB_FIX_TARGET_MASK) : mode;
	}

	mutex_lock(&gsDVFSLock);

	g_tb_dvfs_margin_mode = mode;

	if (value > MAX_TB_DVFS_MARGIN)
		g_tb_dvfs_margin_value = MAX_TB_DVFS_MARGIN;
	else if (value < MIN_TB_DVFS_MARGIN)
		g_tb_dvfs_margin_value = MIN_TB_DVFS_MARGIN;
	else
		g_tb_dvfs_margin_value = value;

	if (value_min > MAX_TB_DVFS_MARGIN)
		g_tb_dvfs_margin_value_min = MAX_TB_DVFS_MARGIN;
	else if (value_min < MIN_TB_DVFS_MARGIN)
		g_tb_dvfs_margin_value_min = MIN_TB_DVFS_MARGIN;
	else
		g_tb_dvfs_margin_value_min = value_min;

	if (margin_step == 0)
		g_tb_dvfs_margin_step = margin_step;
	else
		g_tb_dvfs_margin_step = GED_DVFS_TIMER_BASED_DVFS_MARGIN_STEP;

	mutex_unlock(&gsDVFSLock);
}

static int ged_get_timer_base_dvfs_margin(void)
{
	return g_tb_dvfs_margin_value;
}

int ged_dvfs_get_tb_dvfs_margin_cur(void)
{
	return gx_tb_dvfs_margin_cur;
}

unsigned int ged_dvfs_get_tb_dvfs_margin_mode(void)
{
	return g_tb_dvfs_margin_mode;
}

static void ged_dvfs_loading_mode(int i32MarginValue)
{
	/* -1: default: */
	/* 0: default (Active)/(Active+Idle)*/
	/* 1: max(TA,3D)+COMPUTE/(Active+Idle) */
	/* 2: max(TA,3D)/(Active+Idle) */

	mutex_lock(&gsDVFSLock);

	if (i32MarginValue == -1)
		gx_dvfs_loading_mode = LOADING_ACTIVE;
	else if ((i32MarginValue >= 0) && (i32MarginValue < 100))
		gx_dvfs_loading_mode = i32MarginValue;

	mutex_unlock(&gsDVFSLock);
}

static int ged_get_dvfs_loading_mode(void)
{
	return gx_dvfs_loading_mode;
}

static void ged_dvfs_workload_mode(int i32WorkloadMode)
{
	/* -1: default (active) */
	/* 0: active */
	/* 1: 3d */

	mutex_lock(&gsDVFSLock);

	if (i32WorkloadMode == -1)
		gx_dvfs_workload_mode = WORKLOAD_ACTIVE;
	else if ((i32WorkloadMode >= 0) && (i32WorkloadMode < 100))
		gx_dvfs_workload_mode = i32WorkloadMode;

	mutex_unlock(&gsDVFSLock);
}

static int ged_get_dvfs_workload_mode(void)
{
	return gx_dvfs_workload_mode;
}

void ged_get_gpu_utli_ex(struct GpuUtilization_Ex *util_ex)
{
	memcpy((void *)util_ex, (void *)&g_Util_Ex,
		sizeof(struct GpuUtilization_Ex));
}

static void ged_set_fastdvfs_mode(unsigned int u32ModeValue)
{
	mutex_lock(&gsDVFSLock);
	g_fastdvfs_mode = u32ModeValue;
	mtk_gpueb_dvfs_set_mode(g_fastdvfs_mode);
	mutex_unlock(&gsDVFSLock);
}

static unsigned int ged_get_fastdvfs_mode(void)
{
	mtk_gpueb_dvfs_get_mode(&g_fastdvfs_mode);

	return g_fastdvfs_mode;
}

/* Need spinlocked */
void ged_dvfs_save_loading_page(void)
{
	gL_ulCalResetTS_us = g_ulCalResetTS_us;
	gL_ulPreCalResetTS_us = g_ulPreCalResetTS_us;
	gL_ulWorkingPeriod_us = g_ulWorkingPeriod_us;

	/* set as zero for next time */
	g_ulWorkingPeriod_us = 0;
}

void ged_dvfs_run(
	unsigned long t, long phase, unsigned long ul3DFenceDoneTime,
	GED_DVFS_COMMIT_TYPE eCommitType)
{
	unsigned long ui32IRQFlags;
	enum gpu_dvfs_policy_state policy_state;

	mutex_lock(&gsDVFSLock);

	if (gpu_dvfs_enable == 0) {
		gpu_power = 0;
		gpu_loading = 0;
		gpu_block = 0;
		gpu_idle = 0;
		ged_log_buf_print(ghLogBuf_DVFS,
			"[GED_K][LB_DVFS] skip %s due to gpu_dvfs_enable=%u",
			__func__, gpu_dvfs_enable);
		goto EXIT_ged_dvfs_run;
	}

	/* SKIP for keeping boost freq */
	if (g_dvfs_skip_round > 0)
		g_dvfs_skip_round--;

	if (g_iSkipCount > 0) {
		gpu_power = 0;
		gpu_loading = 0;
		gpu_block = 0;
		gpu_idle = 0;
		g_iSkipCount -= 1;
	} else {
		spin_lock_irqsave(&gsGpuUtilLock, ui32IRQFlags);

		if (is_fb_dvfs_triggered) {
			spin_unlock_irqrestore(&gsGpuUtilLock, ui32IRQFlags);
			goto EXIT_ged_dvfs_run;
		}

		is_fallback_mode_triggered = 1;

		ged_dvfs_cal_gpu_utilization_ex(&gpu_loading,
			&gpu_block, &gpu_idle, &g_Util_Ex);

		ged_log_buf_print(ghLogBuf_DVFS,
			"[GED_K][FB_DVFS] fallback mode");
		spin_unlock_irqrestore(&gsGpuUtilLock, ui32IRQFlags);

		spin_lock_irqsave(&g_sSpinLock, ui32IRQFlags);
		g_ulPreCalResetTS_us = g_ulCalResetTS_us;
		g_ulCalResetTS_us = t;

		ged_dvfs_save_loading_page();

		spin_unlock_irqrestore(&g_sSpinLock, ui32IRQFlags);

#ifdef GED_DVFS_UM_CAL
		if (phase == GED_DVFS_TIMER_BACKUP)
#endif /* GED_DVFS_UM_CAL */
		{
			/* timer-backup DVFS use only */
			if (ged_dvfs_policy(gpu_loading, &g_ui32FreqIDFromPolicy, t, phase,
					ul3DFenceDoneTime, false)) {
				// overwrite eCommitType in case fallback is needed in LB
				policy_state = ged_get_policy_state();
				if (policy_state == POLICY_STATE_LB ||
					policy_state == POLICY_STATE_FORCE_LB)
					eCommitType = GED_DVFS_LOADING_BASE_COMMIT;
				else
					eCommitType = GED_DVFS_FALLBACK_COMMIT;

				// commit new frequency
				ged_dvfs_gpu_freq_commit(g_ui32FreqIDFromPolicy,
						ged_get_freq_by_idx(
						g_ui32FreqIDFromPolicy),
						eCommitType);
			}
		}
	}
	if (gpu_debug_enable) {
		GED_LOGE("%s:gpu_loading=%d %d, g_iSkipCount=%d", __func__,
			gpu_loading, ged_get_cur_oppidx(),
			g_iSkipCount);
	}

EXIT_ged_dvfs_run:
	mutex_unlock(&gsDVFSLock);
}

void ged_dvfs_sw_vsync_query_data(struct GED_DVFS_UM_QUERY_PACK *psQueryData)
{
	psQueryData->ui32GPULoading = gpu_loading;

	psQueryData->ui32GPUFreqID = ged_get_cur_oppidx();
	psQueryData->gpu_cur_freq =
		ged_get_freq_by_idx(psQueryData->ui32GPUFreqID);
	psQueryData->gpu_pre_freq = ged_get_freq_by_idx(g_ui32PreFreqID);

	psQueryData->nsOffset = ged_dvfs_vsync_offset_level_get();

	psQueryData->ulWorkingPeriod_us = gL_ulWorkingPeriod_us;
	psQueryData->ulPreCalResetTS_us = gL_ulPreCalResetTS_us;

	psQueryData->ui32TargetPeriod_us = g_ui32TargetPeriod_us;
	psQueryData->ui32BoostValue = g_ui32BoostValue;
}

void ged_dvfs_track_latest_record(enum MTK_GPU_DVFS_TYPE *peType,
	unsigned long *pulFreq)
{
	*peType = g_CommitType;
	*pulFreq = g_ulCommitFreq;
}

unsigned long ged_dvfs_get_gpu_commit_freq(void)
{
	return ged_commit_freq;
}

unsigned long ged_dvfs_get_gpu_commit_opp_freq(void)
{
	return ged_commit_opp_freq;
}

unsigned int ged_dvfs_get_gpu_loading(void)
{
	return gpu_av_loading;
}

unsigned int ged_dvfs_get_gpu_loading2(int reset)
{
	int loading = 0;
	unsigned long ui32IRQFlags;

	spin_lock_irqsave(&load_info_lock, ui32IRQFlags);

	if (g_loading2_count > 0)
		loading = g_loading2_sum / g_loading2_count;

	if (reset) {
		g_loading2_sum = 0;
		g_loading2_count = 0;
	}

	spin_unlock_irqrestore(&load_info_lock, ui32IRQFlags);

	return loading;
}

unsigned int ged_dvfs_get_gpu_blocking(void)
{
	return gpu_block;
}

unsigned int ged_dvfs_get_gpu_idle(void)
{
	return 100 - gpu_av_loading;
}

void ged_dvfs_get_gpu_cur_freq(struct GED_DVFS_FREQ_DATA *psData)
{
	psData->ui32Idx = ged_get_cur_oppidx();
	psData->ulFreq = ged_get_freq_by_idx(psData->ui32Idx);
}

void ged_dvfs_get_gpu_pre_freq(struct GED_DVFS_FREQ_DATA *psData)
{
	psData->ui32Idx = g_ui32PreFreqID;
	psData->ulFreq = ged_get_freq_by_idx(g_ui32PreFreqID);
}

void ged_get_gpu_dvfs_cal_freq(unsigned long *p_policy_tar_freq, int *pmode)
{
	*p_policy_tar_freq = g_policy_tar_freq;
	*pmode = g_mode;
}

GED_ERROR ged_dvfs_probe_signal(int signo)
{
	int cache_pid = GED_NO_UM_SERVICE;
	struct task_struct *t = NULL;
	struct siginfo info;

	info.si_signo = signo;
	info.si_code = SI_QUEUE;
	info.si_int = 1234;

	if (cache_pid != g_probe_pid) {
		cache_pid = g_probe_pid;
		if (g_probe_pid == GED_NO_UM_SERVICE)
			t = NULL;
		else {
			rcu_read_lock();
			t = pid_task(find_vpid(g_probe_pid), PIDTYPE_PID);
			rcu_read_unlock();
		}
	}

	if (t != NULL) {
		/* TODO: porting*/
		/* send_sig_info(signo, &info, t); */
		return GED_OK;
	} else {
		g_probe_pid = GED_NO_UM_SERVICE;
		return GED_ERROR_INVALID_PARAMS;
	}
}

void set_target_fps(int i32FPS)
{
	g_ulvsync_period = get_ns_period_from_fps(i32FPS);
}

void ged_dvfs_reset_opp_cost(int oppsize)
{
	int i;

	if (oppsize > 0 && oppsize <= ged_get_all_available_opp_num()) {
		for (i = 0; i < oppsize; i++) {
			g_aOppStat[i].ui64Active = 0;
			g_aOppStat[i].ui64Idle = 0;
			memset(g_aOppStat[i].uMem.aTrans, 0, sizeof(uint32_t) * oppsize);
		}
	}
}

int ged_dvfs_query_opp_cost(struct GED_DVFS_OPP_STAT *psReport,
		int i32NumOpp, bool bStript)
{
	int i;

	if (g_aOppStat && psReport &&
			i32NumOpp > 0 &&
			i32NumOpp <= ged_get_opp_num()) {
		memcpy(psReport, g_aOppStat,
			i32NumOpp * sizeof(struct GED_DVFS_OPP_STAT));

		if (bStript) {
			for (i = 0; i < i32NumOpp; i++) {
				psReport[i].uMem.ui32Freq = ged_get_freq_by_idx(i);
			}
		}

		return 0;
	}

	return -1;
}

void ged_dvfs_update_opp_cost(unsigned int loading, unsigned int TSDiff_us,
		unsigned long long cur_us, unsigned int idx)
{
	unsigned int Active_us;
	(void) cur_us;

	// for the uninitialized current freq idx
	if (idx >= ged_get_all_available_opp_num())
		return;

	if (g_aOppStat) {
		Active_us = (TSDiff_us * loading / 100);

		/* update opp busy */
		g_aOppStat[idx].ui64Active += Active_us;
		g_aOppStat[idx].ui64Idle += (TSDiff_us - Active_us);
	}
}

static void ged_dvfs_deinit_opp_cost(void)
{
	int i;
	int oppsize;

	if (g_aOppStat == NULL)
		return;

	oppsize = ged_get_all_available_opp_num();
	if (g_aOppStat) {
		for (i = 0; i < oppsize; i++)
			vfree(g_aOppStat[i].uMem.aTrans);
	}

	vfree(g_aOppStat);
}

int ged_dvfs_init_opp_cost(void)
{
	int oppsize;
	int i;

	oppsize = ged_get_all_available_opp_num();

	if (oppsize <= 0)
		return -EPROBE_DEFER;

	if (g_aOppStat == NULL)
		g_aOppStat = vmalloc(sizeof(struct GED_DVFS_OPP_STAT) * oppsize);
	else
		ged_dvfs_deinit_opp_cost();

	if (g_aOppStat != NULL) {
		for (i = 0; i < oppsize; i++)
			g_aOppStat[i].uMem.aTrans = vmalloc(sizeof(uint32_t) * oppsize);

		g_num = oppsize;
		ged_dvfs_reset_opp_cost(oppsize);
	} else
		GED_LOGE("init opp cost failed!");

	return 0;
}

GED_ERROR ged_dvfs_probe(int pid)
{
	if (pid == GED_VSYNC_OFFSET_NOT_SYNC) {
		g_ui32EventDebugStatus |= GED_EVENT_NOT_SYNC;
		return GED_OK;
	}

	if (pid == GED_VSYNC_OFFSET_SYNC)	{
		g_ui32EventDebugStatus &= (~GED_EVENT_NOT_SYNC);
		return GED_OK;
	}

	g_probe_pid = pid;

	/* clear bits among start */
	if (g_probe_pid != GED_NO_UM_SERVICE) {
		g_ui32EventStatus &= (~GED_EVENT_TOUCH);
		g_ui32EventStatus &= (~GED_EVENT_WFD);
		g_ui32EventStatus &= (~GED_EVENT_GAS);

		g_ui32EventDebugStatus = 0;
	}

	return GED_OK;
}

GED_ERROR ged_dvfs_system_init(void)
{
	mutex_init(&gsDVFSLock);
	mutex_init(&gsPolicyLock);
	mutex_init(&gsVSyncOffsetLock);

	spin_lock_init(&gsGpuUtilLock);

	/* initial as locked, signal when vsync_sw_notify */
#ifdef ENABLE_COMMON_DVFS
	gpu_dvfs_enable = 1;

	gpu_util_history_init();

	g_iSkipCount = MTK_DEFER_DVFS_WORK_MS / MTK_DVFS_SWITCH_INTERVAL_MS;

	g_ulvsync_period = get_ns_period_from_fps(60);

	ged_kpi_gpu_dvfs_fp = ged_dvfs_fb_gpu_dvfs;
	ged_kpi_trigger_fb_dvfs_fp = ged_dvfs_trigger_fb_dvfs;
	ged_kpi_check_if_fallback_mode_fp = ged_dvfs_is_fallback_mode_triggered;

	g_dvfs_skip_round = 0;

	g_minfreq_idx = ged_get_min_oppidx();
	g_maxfreq_idx = 0;
	g_minfreq = ged_get_freq_by_idx(g_minfreq_idx);
	g_maxfreq = ged_get_freq_by_idx(g_maxfreq_idx);

	g_bottom_freq_id = ged_get_min_oppidx_real();
	gpu_bottom_freq = ged_get_freq_by_idx(g_bottom_freq_id);

	g_cust_boost_freq_id = ged_get_min_oppidx_real();
	gpu_cust_boost_freq = ged_get_freq_by_idx(g_cust_boost_freq_id);

	g_cust_upbound_freq_id = 0;
	gpu_cust_upbound_freq = ged_get_freq_by_idx(g_cust_upbound_freq_id);

	g_policy_tar_freq = 0;
	g_mode = 0;

	ged_commit_freq = 0;
	ged_commit_opp_freq = 0;

#ifdef ENABLE_TIMER_BACKUP
	g_gpu_timer_based_emu = 0;
#else
	g_gpu_timer_based_emu = 1;
#endif /* ENABLE_TIMER_BACKUP */

	mtk_set_bottom_gpu_freq_fp = ged_dvfs_set_bottom_gpu_freq;
	mtk_get_bottom_gpu_freq_fp = ged_dvfs_get_bottom_gpu_freq;
	mtk_custom_get_gpu_freq_level_count_fp =
		ged_dvfs_get_gpu_freq_level_count;
	mtk_custom_boost_gpu_freq_fp = ged_dvfs_custom_boost_gpu_freq;
	mtk_custom_upbound_gpu_freq_fp = ged_dvfs_custom_ceiling_gpu_freq;
	mtk_get_gpu_loading_fp = ged_dvfs_get_gpu_loading;
	mtk_get_gpu_block_fp = ged_dvfs_get_gpu_blocking;
	mtk_get_gpu_idle_fp = ged_dvfs_get_gpu_idle;

	mtk_get_gpu_bottom_freq_fp = ged_get_gpu_bottom_freq;

	ged_kpi_set_gpu_dvfs_hint_fp = ged_dvfs_last_and_target_cb;

	mtk_dvfs_margin_value_fp = ged_dvfs_margin_value;
	mtk_get_dvfs_margin_value_fp = ged_get_dvfs_margin_value;
#endif /* ENABLE_COMMON_DVFS */

	mtk_loading_base_dvfs_step_fp = ged_loading_base_dvfs_step;
	mtk_get_loading_base_dvfs_step_fp = ged_get_loading_base_dvfs_step;

	mtk_timer_base_dvfs_margin_fp =	ged_timer_base_dvfs_margin;
	mtk_get_timer_base_dvfs_margin_fp = ged_get_timer_base_dvfs_margin;

	mtk_dvfs_loading_mode_fp = ged_dvfs_loading_mode;
	mtk_get_dvfs_loading_mode_fp = ged_get_dvfs_loading_mode;

	mtk_dvfs_workload_mode_fp = ged_dvfs_workload_mode;
	mtk_get_dvfs_workload_mode_fp = ged_get_dvfs_workload_mode;

	if (ged_is_fdvfs_support()) {
		mtk_set_fastdvfs_mode_fp = ged_set_fastdvfs_mode;
		mtk_get_fastdvfs_mode_fp = ged_get_fastdvfs_mode;
	}

	spin_lock_init(&g_sSpinLock);

	return GED_OK;
}

void ged_dvfs_system_exit(void)
{
	ged_dvfs_deinit_opp_cost();
	mutex_destroy(&gsDVFSLock);
	mutex_destroy(&gsPolicyLock);
	mutex_destroy(&gsVSyncOffsetLock);
}

#ifdef ENABLE_COMMON_DVFS
module_param(gpu_loading, uint, 0644);
module_param(gpu_block, uint, 0644);
module_param(gpu_idle, uint, 0644);
module_param(gpu_dvfs_enable, uint, 0644);
module_param(boost_gpu_enable, uint, 0644);
module_param(gpu_debug_enable, uint, 0644);
module_param(gpu_bottom_freq, uint, 0644);
module_param(gpu_cust_boost_freq, uint, 0644);
module_param(gpu_cust_upbound_freq, uint, 0644);
module_param(g_gpu_timer_based_emu, uint, 0644);
// module_param(gpu_bw_err_debug, uint, 0644);
#endif /* ENABLE_COMMON_DVFS */

