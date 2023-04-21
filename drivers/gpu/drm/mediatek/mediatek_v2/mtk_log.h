/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef __MTKFB_LOG_H
#define __MTKFB_LOG_H

#include <linux/kernel.h>
#include <linux/sched/clock.h>

#if IS_ENABLED(CONFIG_MTK_AEE_FEATURE)
#include <aee.h>
#endif

extern unsigned long long mutex_time_start;
extern unsigned long long mutex_time_end;
extern long long mutex_time_period;
extern const char *mutex_locker;
extern bool g_trace_log;

#ifndef DRM_TRACE_ID
#define DRM_TRACE_ID 0xFFFF0000
#endif
extern void mtk_drm_print_trace(char *fmt, ...);

#define mtk_drm_trace_tag_begin(fmt, args...) do { \
	if (g_trace_log) { \
		mtk_drm_print_trace( \
			"B|%d|"fmt"\n", DRM_TRACE_ID, ##args); \
	} \
} while (0)

#define mtk_drm_trace_tag_end(fmt, args...) do { \
	if (g_trace_log) { \
		mtk_drm_print_trace( \
			"E|%d|"fmt"\n", DRM_TRACE_ID, ##args); \
	} \
} while (0)

enum DPREC_LOGGER_PR_TYPE {
	DPREC_LOGGER_ERROR,
	DPREC_LOGGER_FENCE,
	DPREC_LOGGER_DEBUG,
	DPREC_LOGGER_DUMP,
	DPREC_LOGGER_STATUS,
	DPREC_LOGGER_PR_NUM
};

int mtk_dprec_logger_pr(unsigned int type, char *fmt, ...);

#define DDPINFO(fmt, arg...)                                                   \
	do {                                                                   \
		mtk_dprec_logger_pr(DPREC_LOGGER_DEBUG, fmt, ##arg);           \
		if (g_mobile_log)                                              \
			pr_info("[DISP]" pr_fmt(fmt), ##arg);     \
	} while (0)

#define DDPFUNC(fmt, arg...)		\
	pr_info("[DISP][%s line:%d]"pr_fmt(fmt), __func__, __LINE__, ##arg)

#define DDPDBG(fmt, arg...)                                                    \
	do {                                                                   \
		if (!g_detail_log)                                             \
			break;                                                 \
		mtk_dprec_logger_pr(DPREC_LOGGER_DEBUG, fmt, ##arg);           \
		if (g_mobile_log)                                              \
			pr_info("[DISP]" pr_fmt(fmt), ##arg);     \
	} while (0)

#define DDP_PROFILE(fmt, arg...)                                               \
	do {                                                                   \
		if (!g_profile_log)                                            \
			break;                                                 \
		mtk_dprec_logger_pr(DPREC_LOGGER_DEBUG, fmt, ##arg);           \
		if (g_mobile_log)                                              \
			pr_info("[DISP]" pr_fmt(fmt), ##arg);     \
	} while (0)

#define DDPMSG(fmt, arg...)                                                    \
	do {                                                                   \
		mtk_dprec_logger_pr(DPREC_LOGGER_DEBUG, fmt, ##arg);           \
		pr_info("[DISP]" pr_fmt(fmt), ##arg);             \
	} while (0)

#define DDPDUMP(fmt, arg...)                                                   \
	do {                                                                   \
		mtk_dprec_logger_pr(DPREC_LOGGER_DUMP, fmt, ##arg);            \
		if (g_mobile_log)                                              \
			pr_info("[DISP]" pr_fmt(fmt), ##arg);     \
	} while (0)

#define DDPFENCE(fmt, arg...)                                                  \
	do {                                                                   \
		mtk_dprec_logger_pr(DPREC_LOGGER_FENCE, fmt, ##arg);           \
		if (g_fence_log)                                               \
			pr_info("[DISP]" pr_fmt(fmt), ##arg);     \
	} while (0)

#define DDPPR_ERR(fmt, arg...)                                                 \
	do {                                                                   \
		mtk_dprec_logger_pr(DPREC_LOGGER_ERROR, fmt, ##arg);           \
		pr_err("[DISP][E]" pr_fmt(fmt), ##arg);              \
	} while (0)

#define DDPIRQ(fmt, arg...)                                                    \
	do {                                                                   \
		mtk_dprec_logger_pr(DPREC_LOGGER_DEBUG, fmt, ##arg);   \
		if (g_irq_log)                                                 \
			pr_info("[DISP]" pr_fmt(fmt), ##arg);     \
	} while (0)

#define DDP_MUTEX_LOCK(lock, name, line)                                       \
	do {                                                                   \
		DDPINFO("M_LOCK:%s[%d] +\n", name, line);		   \
		DRM_MMP_EVENT_START(mutex_lock, (unsigned long)lock,	   \
				line);	   \
		mtk_drm_trace_tag_begin("M_LOCK_%s", name);	\
		mutex_lock(lock);		   \
		mutex_time_start = sched_clock();		   \
		mutex_locker = name;		   \
	} while (0)

#define DDP_MUTEX_UNLOCK(lock, name, line)                                     \
	do {                                                                   \
		mutex_locker = NULL;		   \
		mutex_time_end = sched_clock();		   \
		mutex_time_period = mutex_time_end - mutex_time_start;   \
		if (mutex_time_period > 1000000000) {		   \
			DDPPR_ERR("M_ULOCK:%s[%d] timeout:<%lld ns>!\n",   \
				name, line, mutex_time_period);		   \
			DRM_MMP_MARK(mutex_lock,		   \
				(unsigned long)mutex_time_period, 0);   \
			dump_stack();		   \
		}		   \
		mutex_unlock(lock);		   \
		DRM_MMP_EVENT_END(mutex_lock, (unsigned long)lock,	   \
			line);	   \
		mtk_drm_trace_tag_end("M_LOCK_%s", name);	\
		DDPINFO("M_ULOCK:%s[%d] -\n", name, line);		   \
	} while (0)

#define DDP_MUTEX_LOCK_NESTED(lock, i, name, line)                             \
	do {                                                                   \
		DDPINFO("M_LOCK_NST[%d]:%s[%d] +\n", i, name, line);   \
		mtk_drm_trace_tag_begin("M_LOCK_NST_%s", name);	\
		mutex_lock_nested(lock, i);		   \
	} while (0)

#define DDP_MUTEX_UNLOCK_NESTED(lock, i, name, line)                           \
	do {                                                                   \
		mutex_unlock(lock);		   \
		mtk_drm_trace_tag_end("M_LOCK_NST_%s", name);	\
		DDPINFO("M_ULOCK_NST[%d]:%s[%d] -\n", i, name, line);	\
	} while (0)

#if IS_ENABLED(CONFIG_MTK_AEE_FEATURE)
#define DDPAEE(string, args...)                                                \
	do {                                                                   \
		char str[200];                                                 \
		int r;	\
		r = snprintf(str, 199, "DDP:" string, ##args);                     \
		if (r < 0) {	\
			pr_err("snprintf error\n");	\
		}	\
		aee_kernel_warning_api(__FILE__, __LINE__,                     \
				       DB_OPT_DEFAULT |                        \
					       DB_OPT_MMPROFILE_BUFFER,        \
				       str, string, ##args);                   \
		DDPPR_ERR("[DDP Error]" string, ##args);                       \
	} while (0)

#define DDPAEE_FATAL(string, args...)                                          \
	do {										\
		char str[200];								\
		int r;	\
		r = snprintf(str, 199, "DDP:" string, ##args);				\
		if (r < 0) {	\
			pr_err("snprintf error\n"); \
		}	\
		aee_kernel_fatal_api(__FILE__, __LINE__,					\
					DB_OPT_DEFAULT |					\
					DB_OPT_MMPROFILE_BUFFER,		\
					str, string, ##args);			\
		DDPPR_ERR("[DDP Fatal Error]" string, ##args);			\
	} while (0)
#else /* !CONFIG_MTK_AEE_FEATURE */
#define DDPAEE(string, args...)                                                \
	do {                                                                   \
		char str[200];                                                 \
		int r;	\
		r = snprintf(str, 199, "DDP:" string, ##args);                     \
		if (r < 0) {	\
			pr_err("snprintf error\n");	\
		}	\
		pr_err("[DDP Error]" string, ##args);                          \
	} while (0)

#define DDPAEE_FATAL(string, args...)                                          \
	do {										\
		char str[200];								\
		int r;	\
		r = snprintf(str, 199, "DDP:" string, ##args);				\
		if (r < 0) {	\
			pr_err("snprintf error\n"); \
		}	\
		pr_err("[DDP Fatal Error]" string, ##args);				\
	} while (0)
#endif /* CONFIG_MTK_AEE_FEATURE */

extern bool g_mobile_log;
extern bool g_msync_debug;
extern bool g_fence_log;
extern bool g_irq_log;
extern bool g_detail_log;
extern bool g_gpuc_direct_push;
extern bool g_ovl_bwm_debug;
extern bool g_vidle_apsrc_debug;
extern bool g_profile_log;
#endif
