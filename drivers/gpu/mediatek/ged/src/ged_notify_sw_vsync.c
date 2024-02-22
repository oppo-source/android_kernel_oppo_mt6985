// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */


#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/sched/clock.h>
#include <linux/atomic.h>

#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/mutex.h>

#include <asm/div64.h>

#include <mt-plat/mtk_gpu_utility.h>
#include "ged_notify_sw_vsync.h"
#include "ged_log.h"
#include "ged_tracepoint.h"
#include "ged_base.h"
#include "ged_monitor_3D_fence.h"
#include "ged.h"
#include "ged_dvfs.h"
#include "ged_dcs.h"
#include "ged_kpi.h"

#if defined(CONFIG_MTK_GPUFREQ_V2)
#include <ged_gpufreq_v2.h>
#include <gpufreq_v2.h>
#else
#include <ged_gpufreq_v1.h>
#endif /* CONFIG_MTK_GPUFREQ_V2 */

#define GED_DVFS_FB_TIMER_TIMEOUT 100000000
#define GED_DVFS_TIMER_TIMEOUT g_fallback_time_out

#ifndef ENABLE_TIMER_BACKUP

#undef GED_DVFS_TIMER_TIMEOUT

#define GED_DVFS_FB_TIMER_TIMEOUT 100000000
#define GED_DVFS_TIMER_TIMEOUT g_fallback_time_out

#endif /* GED_DVFS_TIMER_TIMEOUT */

static u64 g_fallback_time_out = GED_DVFS_FB_TIMER_TIMEOUT;

static struct hrtimer g_HT_hwvsync_emu;

#include "ged_dvfs.h"
#include "ged_global.h"

static struct workqueue_struct *g_psNotifyWorkQueue;

static struct mutex gsVsyncStampLock;

struct GED_NOTIFY_SW_SYNC {
	struct work_struct	sWork;
	unsigned long t;
	long phase;
	unsigned long ul3DFenceDoneTime;
	bool bUsed;
};

#define MAX_NOTIFY_CNT 125
struct GED_NOTIFY_SW_SYNC loading_base_notify[MAX_NOTIFY_CNT];
int notify_index;
static enum gpu_dvfs_policy_state g_policy_state = POLICY_STATE_INIT;
static enum gpu_dvfs_policy_state g_prev_policy_state = POLICY_STATE_INIT;

int (*ged_sw_vsync_event_fp)(bool bMode) = NULL;
EXPORT_SYMBOL(ged_sw_vsync_event_fp);
static struct mutex gsVsyncModeLock;
static int ged_sw_vsync_event(bool bMode)
{
	static bool bCurMode;
	int ret;

	ret = 0;
	mutex_lock(&gsVsyncModeLock);

	if (bCurMode != bMode) {
		bCurMode = bMode;
		if (ged_sw_vsync_event_fp) {
			ret = ged_sw_vsync_event_fp(bMode);
			ged_log_buf_print(ghLogBuf_DVFS,
			"[GED_K] ALL mode change to %d ", bCurMode);
		} else
			ged_log_buf_print(ghLogBuf_DVFS,
			"[GED_K] LOCAL mode change to %d ", bCurMode);
		if (bCurMode)
			ret = 1;
	}

	mutex_unlock(&gsVsyncModeLock);
	return ret;
}

u64 ged_get_fallback_time(void)
{
	u64 temp = 0;

	if (g_fallback_mode == 0)
		temp = (u64)g_fallback_time * 1000000;//ms to ns
	else if (g_fallback_mode == 1)
		temp = fb_timeout * g_fallback_time / 10;
	else if (g_fallback_mode == 2)
		temp = lb_timeout * g_fallback_time / 10;
	return temp;
}

enum gpu_dvfs_policy_state ged_get_policy_state(void)
{
	return g_policy_state;
}

enum gpu_dvfs_policy_state ged_get_prev_policy_state(void)
{
	return g_prev_policy_state;
}

void ged_set_policy_state(enum gpu_dvfs_policy_state state)
{
	g_policy_state = state;
}

void ged_set_prev_policy_state(enum gpu_dvfs_policy_state state)
{
	g_prev_policy_state = state;
}

static unsigned long long sw_vsync_ts;
static void ged_notify_sw_sync_work_handle(struct work_struct *psWork)
{
	struct GED_NOTIFY_SW_SYNC *psNotify =
		GED_CONTAINER_OF(psWork, struct GED_NOTIFY_SW_SYNC, sWork);
	unsigned long long temp;
	GED_DVFS_COMMIT_TYPE eCommitType;
	u64 timeout_value;
	/*only one policy at a time*/
	if (psNotify) {
		mutex_lock(&gsPolicyLock);
		timeout_value = lb_timeout;
		psNotify->bUsed = false;
		if (hrtimer_expires_remaining(&g_HT_hwvsync_emu) < 0) {
			enum gpu_dvfs_policy_state policy_state;

			policy_state = ged_get_policy_state();
			if (policy_state == POLICY_STATE_FB ||
					policy_state == POLICY_STATE_FB_FALLBACK ||
					policy_state == POLICY_STATE_LB_FALLBACK ||
					policy_state == POLICY_STATE_FORCE_LB_FALLBACK) {
				eCommitType = GED_DVFS_FALLBACK_COMMIT;
				if (policy_state == POLICY_STATE_LB_FALLBACK)
					ged_set_policy_state(POLICY_STATE_LB_FALLBACK);
				else if (policy_state == POLICY_STATE_FORCE_LB_FALLBACK)
					ged_set_policy_state(POLICY_STATE_FORCE_LB_FALLBACK);
				else
					ged_set_policy_state(POLICY_STATE_FB_FALLBACK);
				timeout_value = ged_get_fallback_time();
			} else {
				eCommitType = GED_DVFS_LOADING_BASE_COMMIT;
			}
			ged_set_backup_timer_timeout(timeout_value);   // set init value

			temp = 0;
			/* if callback is queued, send mode off to real driver */
			ged_sw_vsync_event(false);
#ifdef ENABLE_TIMER_BACKUP
			temp = ged_get_time();
			if (temp-sw_vsync_ts > GED_DVFS_TIMER_TIMEOUT) {
				do_div(temp, 1000);
				psNotify->t = temp;
				ged_dvfs_run(psNotify->t, psNotify->phase,
					psNotify->ul3DFenceDoneTime, eCommitType);
				ged_log_buf_print(ghLogBuf_DVFS,
					"[GED_K] Timer kicked	(ts=%llu) ", temp);
			} else {
				ged_log_buf_print(ghLogBuf_DVFS,
					"[GED_K] Timer kick giveup (ts=%llu)", temp);
			}
#endif
			hrtimer_start(&g_HT_hwvsync_emu,
				ns_to_ktime(GED_DVFS_TIMER_TIMEOUT), HRTIMER_MODE_REL);
		}
		mutex_unlock(&gsPolicyLock);
	}
#if defined(MTK_GPU_FW_IDLE)
	/* set initial idle time to 5ms if runtime policy stay default flavor */
	if (ged_kpi_is_fw_idle_policy_enable() == -1)
		mtk_set_gpu_idle(5);
#endif /* MTK_GPU_FW_IDLE */
}

#define GED_VSYNC_MISS_QUANTUM_NS 16666666

#ifdef ENABLE_COMMON_DVFS
static unsigned long long hw_vsync_ts;
#endif
static unsigned long long g_ns_gpu_off_ts;

static bool g_timer_on;
static unsigned long long g_timer_on_ts;
static bool g_bGPUClock;

/*
 * void timer_switch(bool bTock)
 * only set the staus, not really operating on real timer
 */
void timer_switch(bool bTock)
{
	mutex_lock(&gsVsyncStampLock);
	g_timer_on = bTock;
	if (bTock)
		g_timer_on_ts = ged_get_time();
	mutex_unlock(&gsVsyncStampLock);
}

void timer_switch_locked(bool bTock)
{
	g_timer_on = bTock;
	if (bTock)
		g_timer_on_ts = ged_get_time();
}

static void ged_timer_switch_work_handle(struct work_struct *psWork)
{
	struct GED_NOTIFY_SW_SYNC *psNotify =
		GED_CONTAINER_OF(psWork, struct GED_NOTIFY_SW_SYNC, sWork);
	if (psNotify) {
		ged_sw_vsync_event(false);

		psNotify->bUsed = false;
	}
}


void ged_set_backup_timer_timeout(u64 time_out)
{
	if (time_out != 0 && time_out < GED_DVFS_FB_TIMER_TIMEOUT)
		g_fallback_time_out = time_out;
	else
		g_fallback_time_out = GED_DVFS_FB_TIMER_TIMEOUT;
}


void ged_cancel_backup_timer(void)
{
	unsigned long long temp;

	temp = ged_get_time();
#ifdef ENABLE_TIMER_BACKUP
	if (hrtimer_try_to_cancel(&g_HT_hwvsync_emu)) {
		/* Timer is either queued or in cb
		 * cancel it to ensure it is not bother any way
		 */
		hrtimer_cancel(&g_HT_hwvsync_emu);
		hrtimer_start(&g_HT_hwvsync_emu,
			ns_to_ktime(GED_DVFS_TIMER_TIMEOUT), HRTIMER_MODE_REL);
		ged_log_buf_print(ghLogBuf_DVFS,
			"[GED_K] Timer Restart (ts=%llu)", temp);
	} else {
		/*
		 * Timer is not existed
		 */
		hrtimer_start(&g_HT_hwvsync_emu,
			ns_to_ktime(GED_DVFS_TIMER_TIMEOUT), HRTIMER_MODE_REL);
		ged_log_buf_print(ghLogBuf_DVFS,
			"[GED_K] New Timer Start (ts=%llu)", temp);
		timer_switch_locked(true);
	}
#endif /*	#ifdef ENABLE_TIMER_BACKUP	*/
}


GED_ERROR ged_notify_sw_vsync(GED_VSYNC_TYPE eType,
	struct GED_DVFS_UM_QUERY_PACK *psQueryData)
{
	ged_notification(GED_NOTIFICATION_TYPE_SW_VSYNC);

	{
#ifdef ENABLE_COMMON_DVFS

	long phase = 0;
	unsigned long t;
	bool bHWEventKick = false;
	long long llDiff = 0;

	unsigned long long temp;
	unsigned long ul3DFenceDoneTime;


	psQueryData->bFirstBorn = ged_sw_vsync_event(true);

	ul3DFenceDoneTime = ged_monitor_3D_fence_done_time();

	psQueryData->ul3DFenceDoneTime = ul3DFenceDoneTime;

	hw_vsync_ts = temp = ged_get_time();

	if (g_gpu_timer_based_emu) {
		ged_log_buf_print(ghLogBuf_DVFS,
			"[GED_K] Vsync ignored (ts=%llu)", temp);
		return GED_ERROR_INTENTIONAL_BLOCK;
	}

/* TODO: temp defined to disable vsync_dvfs when COMMON_DVFS on*/
#ifdef ENABLE_COMMON_DVFS
	return GED_ERROR_INTENTIONAL_BLOCK;
#else

	/*critical session begin*/
	mutex_lock(&gsVsyncStampLock);

	if (eType == GED_VSYNC_SW_EVENT) {
		sw_vsync_ts = temp;
#ifdef ENABLE_TIMER_BACKUP
		if (hrtimer_try_to_cancel(&g_HT_hwvsync_emu)) {
			/* Timer is either queued or in cb
			 * cancel it to ensure it is not bother any way
			 */
			hrtimer_cancel(&g_HT_hwvsync_emu);
			hrtimer_start(&g_HT_hwvsync_emu,
			ns_to_ktime(GED_DVFS_TIMER_TIMEOUT), HRTIMER_MODE_REL);
			ged_log_buf_print(ghLogBuf_DVFS,
				"[GED_K] Timer Restart (ts=%llu)", temp);
		} else {
			/*
			 * Timer is not existed
			 */
			hrtimer_start(&g_HT_hwvsync_emu,
			ns_to_ktime(GED_DVFS_TIMER_TIMEOUT), HRTIMER_MODE_REL);
			ged_log_buf_print(ghLogBuf_DVFS,
				"[GED_K] New Timer Start (ts=%llu)", temp);
			timer_switch_locked(true);
		}
#endif // #ifdef ENABLE_TIMER_BACKUP
	} else {
		hw_vsync_ts = temp;

		llDiff = (long long)(hw_vsync_ts - sw_vsync_ts);

		if (llDiff > GED_VSYNC_MISS_QUANTUM_NS)
			bHWEventKick = true;
	}
#ifdef GED_DVFS_DEBUG
	if (eType == GED_VSYNC_HW_EVENT)
		GED_LOGD("HW VSYNC: llDiff=",
		"%lld, hw_vsync_ts=%llu, sw_vsync_ts=%llu\n", llDiff,
		hw_vsync_ts, sw_vsync_ts);
	else
		GED_LOGD("SW VSYNC: llDiff=",
		"%lld, hw_vsync_ts=%llu, sw_vsync_ts=%llu\n", llDiff,
		hw_vsync_ts, sw_vsync_ts);
#endif		///	#ifdef GED_DVFS_DEBUG


	if (eType == GED_VSYNC_HW_EVENT)
		ged_log_buf_print(ghLogBuf_DVFS,
		"[GED_K] HW VSYNC (ts=%llu) ", hw_vsync_ts);
	else
		ged_log_buf_print(ghLogBuf_DVFS,
		"[GED_K] SW VSYNC (ts=%llu) ", sw_vsync_ts);

	mutex_unlock(&gsVsyncStampLock);
	/*critical session end*/

	if (eType == GED_VSYNC_SW_EVENT) {
		do_div(temp, 1000);
		t = (unsigned long)(temp);

		// for some cases just align vsync to FenceDoneTime
		if (ul3DFenceDoneTime > t) {
			if (ul3DFenceDoneTime - t < GED_DVFS_DIFF_THRESHOLD)
				t = ul3DFenceDoneTime;
		}
		psQueryData->usT = t;
		ged_dvfs_run(t, phase, ul3DFenceDoneTime,
			GED_DVFS_LOADING_BASE_COMMIT);
		ged_dvfs_sw_vsync_query_data(psQueryData);
	} else {
		if (bHWEventKick) {
#ifdef GED_DVFS_DEBUG
			GED_LOGD("HW Event: kick!\n");
#endif							/// GED_DVFS_DEBUG
			ged_log_buf_print(ghLogBuf_DVFS,
				"[GED_K] HW VSync: mending kick!");
			ged_dvfs_run(0, 0, 0, 0);
		}
	}
#endif
#else
	unsigned long long temp;

	temp = ged_get_time();
	ged_sw_vsync_event(true);
	return GED_ERROR_INTENTIONAL_BLOCK;
#endif

	return GED_OK;
	}
}


extern unsigned int gpu_loading;
enum hrtimer_restart ged_sw_vsync_check_cb(struct hrtimer *timer)
{
	unsigned long long temp;
	long long llDiff;
	unsigned int loading_mode = 0;
	unsigned int gpu_loading_temp = 0;
	struct GED_NOTIFY_SW_SYNC *psNotify;
	struct GpuUtilization_Ex util_ex;

	memset(&util_ex, 0, sizeof(util_ex));

	temp = cpu_clock(smp_processor_id());

	llDiff = (long long)(temp - sw_vsync_ts);

	if (llDiff > GED_VSYNC_MISS_QUANTUM_NS) {
		psNotify = &(loading_base_notify[((notify_index++) % MAX_NOTIFY_CNT)]);

		if (notify_index >= MAX_NOTIFY_CNT)
			notify_index = 0;

#ifndef ENABLE_TIMER_BACKUP
		ged_dvfs_cal_gpu_utilization_ex(&gpu_av_loading,
			&gpu_block, &gpu_idle, &util_ex);
		gpu_loading = gpu_av_loading;
#endif

		ged_get_gpu_utli_ex(&util_ex);
		mtk_get_dvfs_loading_mode(&loading_mode);
		if (loading_mode == LOADING_MAX_3DTA_COM) {
			gpu_loading_temp =
			MAX(util_ex.util_3d, util_ex.util_ta) +
			util_ex.util_compute;
		} else if (loading_mode == LOADING_MAX_3DTA) {
			gpu_loading_temp =
			MAX(util_ex.util_3d, util_ex.util_ta);
		} else if (loading_mode == LOADING_ITER) {
			gpu_loading_temp = util_ex.util_iter;
		} else if (loading_mode == LOADING_MAX_ITERMCU) {
			gpu_loading_temp = MAX(util_ex.util_iter, util_ex.util_mcu);
		} else {   // LOADING_ACTIVE or unknown mode
			gpu_loading_temp = util_ex.util_active;
		}

		if (false == g_bGPUClock && 0 == gpu_loading_temp
			&& (temp - g_ns_gpu_on_ts > GED_DVFS_TIMER_TIMEOUT)) {
			if (psNotify && psNotify->bUsed == false) {
				psNotify->bUsed = true;
				INIT_WORK(&psNotify->sWork,
					ged_timer_switch_work_handle);
				queue_work(g_psNotifyWorkQueue,
					&psNotify->sWork);
				timer_switch_locked(false);
			}
#ifdef GED_DVFS_DEBUG
			ged_log_buf_print(ghLogBuf_DVFS,
				"[GED_K] Timer removed	(ts=%llu) ", temp);
#endif
			return HRTIMER_NORESTART;
		}

		if (psNotify) {
			if (psNotify->bUsed == false) {
				psNotify->bUsed = true;
				INIT_WORK(&psNotify->sWork,
					ged_notify_sw_sync_work_handle);
				psNotify->phase = GED_DVFS_TIMER_BACKUP;
				psNotify->ul3DFenceDoneTime = 0;
				queue_work(g_psNotifyWorkQueue, &psNotify->sWork);
#ifdef GED_DVFS_DEBUG
				ged_log_buf_print(ghLogBuf_DVFS,
					"[GED_K] Timer queue to kick (ts=%llu)", temp);
#endif
			}
			g_timer_on_ts = temp;
		}
	}
	return HRTIMER_NORESTART;
}


void ged_dvfs_gpu_clock_switch_notify(enum ged_gpu_power_state power_state)
{

	if (power_state == GED_POWER_ON) {
		g_ns_gpu_on_ts = ged_get_time();
		g_bGPUClock = true;
		if (g_timer_on) {
			ged_log_buf_print(ghLogBuf_DVFS,
				"[GED_K] Timer Already Start");
		} else {
			mutex_lock(&gsPolicyLock);
			hrtimer_start(&g_HT_hwvsync_emu,
				ns_to_ktime(GED_DVFS_TIMER_TIMEOUT), HRTIMER_MODE_REL);
			mutex_unlock(&gsPolicyLock);
			ged_log_buf_print(ghLogBuf_DVFS,
				"[GED_K] HW Start Timer");
			timer_switch(true);
		}
	} else if (power_state == GED_POWER_OFF ||
			power_state == GED_SLEEP) {
		g_ns_gpu_off_ts = ged_get_time();
		g_bGPUClock = false;
		ged_log_buf_print(ghLogBuf_DVFS, "[GED_K] Buck-off");
	}
	// Update power on/off state
	trace_tracing_mark_write(5566, "gpu_state", power_state);
}
EXPORT_SYMBOL(ged_dvfs_gpu_clock_switch_notify);

#define GED_TIMER_BACKUP_THRESHOLD 3000

#if IS_BUILTIN(CONFIG_MTK_GPU_SUPPORT)
#endif /* CONFIG_MTK_GPU_SUPPORT */

GED_ERROR ged_notify_sw_vsync_system_init(void)
{
	g_psNotifyWorkQueue = alloc_ordered_workqueue("ged_notify_sw_vsync",
						WQ_HIGHPRI | WQ_FREEZABLE | WQ_MEM_RECLAIM);

	if (g_psNotifyWorkQueue == NULL)
		return GED_ERROR_OOM;

	mutex_init(&gsVsyncStampLock);
	mutex_init(&gsVsyncModeLock);

	hrtimer_init(&g_HT_hwvsync_emu, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_HT_hwvsync_emu.function = ged_sw_vsync_check_cb;

	return GED_OK;
}

void ged_notify_sw_vsync_system_exit(void)
{
	if (g_psNotifyWorkQueue != NULL) {
		flush_workqueue(g_psNotifyWorkQueue);
		destroy_workqueue(g_psNotifyWorkQueue);
		g_psNotifyWorkQueue = NULL;
	}

#ifdef ENABLE_COMMON_DVFS
	hrtimer_cancel(&g_HT_hwvsync_emu);
#endif
	mutex_destroy(&gsVsyncModeLock);
	mutex_destroy(&gsVsyncStampLock);
}
