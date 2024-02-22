/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef _FPSGO_COMMON_H_
#define _FPSGO_COMMON_H_

#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/fs.h>

struct fps_level {
	int start;
	int end;
};

#if defined(CONFIG_MTK_FPSGO_V3)
#define FPSGO_SYSTRACE_LIST(macro) \
	macro(MANDATORY, 0), \
	macro(FBT, 1), \
	macro(FSTB, 2), \
	macro(XGF, 3), \
	macro(GBE, 4), \
	macro(FBT_CTRL, 5), \
	macro(MAX, 6), \

#define GENERATE_ENUM(name, shft) FPSGO_DEBUG_##name = 1U << shft
enum {
	FPSGO_SYSTRACE_LIST(GENERATE_ENUM)
};

#define FPSFO_DECLARE_SYSTRACE(prefix, name) \
	extern struct tracepoint __tracepoint_##name; \
	int prefix##_reg_trace_##name(void *probe, void *data) \
	{ \
		return tracepoint_probe_register( \
			&__tracepoint_##name, (void *)probe, data); \
	} \
	int prefix##_unreg_trace_##name(void *probe, void *data) \
	{ \
		return tracepoint_probe_unregister( \
			&__tracepoint_##name, (void *)probe, data); \
	}

extern uint32_t fpsgo_systrace_mask;
extern struct dentry *fpsgo_debugfs_dir;
extern int game_ppid;
extern int powerhal_tid;

void __fpsgo_systrace_c(pid_t pid, unsigned long long bufID,
	int value, const char *name, ...);
void __fpsgo_systrace_b(pid_t pid, const char *name, ...);
void __fpsgo_systrace_e(void);

#define fpsgo_systrace_c(mask, pid, bufID, val, fmt...) \
	do { \
		if (fpsgo_systrace_mask & mask) \
			__fpsgo_systrace_c(pid, bufID, val, fmt); \
	} while (0)

#define fpsgo_systrace_b(mask, tgid, fmt, ...) \
	do { \
		if (fpsgo_systrace_mask & mask) \
			__fpsgo_systrace_b(tgid, fmt); \
	} while (0)

#define fpsgo_systrace_e(mask) \
	do { \
		if (fpsgo_systrace_mask & mask) \
			__fpsgo_systrace_e(); \
	} while (0)

#define fpsgo_systrace_c_fbt(pid, bufID, val, fmt...) \
	fpsgo_systrace_c(FPSGO_DEBUG_MANDATORY, pid, bufID, val, fmt)
#define fpsgo_systrace_c_fbt_debug(pid, bufID, val, fmt...) \
	fpsgo_systrace_c(FPSGO_DEBUG_FBT, pid, bufID, val, fmt)
#define fpsgo_systrace_c_fstb_man(pid, val, fmt...) \
	fpsgo_systrace_c(FPSGO_DEBUG_MANDATORY, pid, val, fmt)
#define fpsgo_systrace_c_fstb(pid, bufID, val, fmt...) \
	fpsgo_systrace_c(FPSGO_DEBUG_FSTB, pid, 0, val, fmt)
#define fpsgo_systrace_c_xgf(pid, bufID, val, fmt...) \
	fpsgo_systrace_c(FPSGO_DEBUG_XGF, pid, 0, val, fmt)
#define fpsgo_systrace_c_log(val, fmt...) \
	do { \
		if (game_ppid > 0) \
			fpsgo_systrace_c(FPSGO_DEBUG_MANDATORY, \
					game_ppid, val, fmt); \
	} while (0)
#define gbe_trace_count(pid, bufID, val, fmt...) \
	fpsgo_systrace_c(FPSGO_DEBUG_MANDATORY, pid, bufID, val, fmt)
#define gbe_trace_count_debug(pid, bufID, val, fmt...) \
	fpsgo_systrace_c(FPSGO_DEBUG_GBE, pid, bufID, val, fmt)
#define __cpu_ctrl_systrace(val, fmt...) \
	fpsgo_systrace_c(FPSGO_DEBUG_MANDATORY, powerhal_tid, 0, val, fmt)
#define __cpu_ctrl_systrace_debug(val, fmt...) \
	fpsgo_systrace_c(FPSGO_DEBUG_FBT_CTRL, powerhal_tid, 0, val, fmt)

int fpsgo_is_fstb_enable(void);
int fpsgo_switch_fstb(int enable);
int fpsgo_fstb_sample_window(long long time_usec);
int fpsgo_fstb_fps_range(int nr_level, struct fps_level *level);

void fpsgo_switch_enable(int enable);
int fpsgo_is_enable(void);

int fbt_cpu_set_bhr(int new_bhr);
int fbt_cpu_set_bhr_opp(int new_opp);
int fbt_cpu_set_rescue_opp_c(int new_opp);
int fbt_cpu_set_rescue_opp_f(int new_opp);
int fbt_cpu_set_rescue_percent(int percent);

int fbt_cpu_get_bhr(void);
int fbt_cpu_get_bhr_opp(void);
int fbt_cpu_get_rescue_opp_c(void);

#else
static inline void fpsgo_systrace_c(uint32_t m, pid_t id,
	unsigned long long bufID, int val, const char *s, ...) { }
static inline void fpsgo_systrace_b(uint32_t m, pid_t id,
				    const char *s, ...) { }
static inline void fpsgo_systrace_e(uint32_t m) { }


static inline void fpsgo_systrace_c_fbt(pid_t id,
	unsigned long long bufID, int val, const char *s, ...) { }
static inline void fpsgo_systrace_c_fbt_debug(pid_t id,
	unsigned long long bufID, int val, const char *s, ...) { }
static inline void fpsgo_systrace_c_fstb_man(pid_t id,
	unsigned long long bufID, int val, const char *s, ...) { }
static inline void fpsgo_systrace_c_fstb(pid_t id,
	unsigned long long bufID, int val, const char *s, ...) { }
static inline void fpsgo_systrace_c_xgf(pid_t id,
	unsigned long long bufID, int val, const char *s, ...) { }
static inline void fpsgo_systrace_c_log(pid_t id, int val,
					const char *s, ...) { }
static inline void gbe_trace_count(pid_t id,
	unsigned long long bufID, int val, const char *s, ...) { }
static inline void gbe_trace_count_debug(pid_t id,
	unsigned long long bufID, int val, const char *s, ...) { }
static inline void __cpu_ctrl_systrace(pid_t id,
	unsigned long long bufID, int val, const char *s, ...) { }
static inline void __cpu_ctrl_systrace_debug(pid_t id,
	unsigned long long bufID, int val, const char *s, ...) { }

static inline int fpsgo_is_fstb_enable(void) { return 0; }
static inline int fpsgo_switch_fstb(int en) { return 0; }
static inline int fpsgo_fstb_sample_window(long long time_usec) { return 0; }
static inline int fpsgo_fstb_fps_range(int nr_level, struct fps_level *level)
{ return 0; }

static inline void fpsgo_switch_enable(int enable) { }
static inline int fpsgo_is_enable(void) { return 0; }

static inline int fbt_cpu_set_bhr(int new_bhr) { return 0; }
static inline int fbt_cpu_set_bhr_opp(int new_opp) { return 0; }
static inline int fbt_cpu_set_rescue_opp_c(int new_opp) { return 0; }
static inline int fbt_cpu_set_rescue_opp_f(int new_opp) { return 0; }
static inline int fbt_cpu_set_rescue_percent(int percent) { return 0; }
#endif

#if defined(CONFIG_MTK_FPSGO_V3)
void xgf_qudeq_notify(unsigned int cmd, unsigned long arg);
#else
static inline void xgf_qudeq_notify(unsigned int cmd, unsigned long arg) { }
#endif


#endif
