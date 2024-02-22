// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include "fpsgo_base.h"
#include <asm/page.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/irq_work.h>
#include <linux/sched/clock.h>
#include <linux/sched/task.h>
#include <linux/sched/cputime.h>
#include <linux/cpufreq.h>
#include "sugov/cpufreq.h"
#include <linux/kobject.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include "mt-plat/fpsgo_common.h"
#include "fpsgo_usedext.h"
#include "fpsgo_sysfs.h"
#include "fbt_cpu.h"
#include "fbt_cpu_platform.h"
#include "fps_composer.h"

#include <linux/preempt.h>
#include <linux/trace_events.h>
#include <linux/fs.h>
#include <linux/rbtree.h>
#include <linux/sched/task.h>
#include <linux/kernel.h>
#include <trace/trace.h>
#include "sched/sched.h"

#define TIME_1S  1000000000ULL
#define TRAVERSE_PERIOD  300000000000ULL

#define event_trace(ip, fmt, args...) \
do { \
	__trace_printk_check_format(fmt, ##args);     \
	{	\
	static const char *trace_printk_fmt     \
	__section(__trace_printk_fmt) =  \
	__builtin_constant_p(fmt) ? fmt : NULL;   \
	__trace_bprintk(ip, trace_printk_fmt, ##args);    \
	}	\
} while (0)

#define mtk_base_dprintk_always(fmt, args...) \
	pr_debug("[FPSGO_BASE]" fmt, ##args)

static int total_fps_control_pid_info_num;

static struct kobject *base_kobj;
static struct rb_root render_pid_tree;
static struct rb_root BQ_id_list;
static struct rb_root linger_tree;
static struct rb_root hwui_info_tree;
static struct rb_root sbe_info_tree;
static struct rb_root fps_control_pid_info_tree;
#if FPSGO_MW
static struct rb_root fpsgo_attr_by_pid_tree;
#endif

static DEFINE_MUTEX(fpsgo_render_lock);

long long fpsgo_task_sched_runtime(struct task_struct *p)
{
	return task_sched_runtime(p);
}

long fpsgo_sched_setaffinity(pid_t pid, const struct cpumask *in_mask)
{
	struct task_struct *p;
	int retval;

	rcu_read_lock();

	p = find_task_by_vpid(pid);
	if (!p) {
		rcu_read_unlock();
		return -ESRCH;
	}

	/* Prevent p going away */
	get_task_struct(p);
	rcu_read_unlock();

	if (p->flags & PF_NO_SETAFFINITY) {
		retval = -EINVAL;
		goto out_put_task;
	}
	retval = -EPERM;

	retval = set_cpus_allowed_ptr(p, in_mask);
out_put_task:
	put_task_struct(p);
	return retval;
}


void *fpsgo_alloc_atomic(int i32Size)
{
	void *pvBuf;

	if (i32Size <= PAGE_SIZE)
		pvBuf = kmalloc(i32Size, GFP_ATOMIC);
	else
		pvBuf = vmalloc(i32Size);

	return pvBuf;
}

void fpsgo_free(void *pvBuf, int i32Size)
{
	if (!pvBuf)
		return;

	if (i32Size <= PAGE_SIZE)
		kfree(pvBuf);
	else
		vfree(pvBuf);
}

unsigned long long fpsgo_get_time(void)
{
	unsigned long long temp;

	preempt_disable();
	temp = cpu_clock(smp_processor_id());
	preempt_enable();

	return temp;
}

int fpsgo_arch_nr_clusters(void)
{
	int cpu, num = 0;
	struct cpufreq_policy *policy;

	for_each_possible_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy) {
			num = 0;
			break;
		}

		num++;
		cpu = cpumask_last(policy->related_cpus);
		cpufreq_cpu_put(policy);
	}

	return num;

}

int fpsgo_arch_nr_get_opp_cpu(int cpu)
{
	struct cpufreq_policy curr_policy;
	struct cpufreq_frequency_table *pos, *table;
	int idx;
	int nr_opp = 0;

	cpufreq_get_policy(&curr_policy, cpu);
	table = curr_policy.freq_table;

	cpufreq_for_each_valid_entry_idx(pos, table, idx) {
		nr_opp++;
	}

	return nr_opp;
}

int fpsgo_arch_nr_get_cap_cpu(int cpu, int opp)
{
	unsigned long cap;

	if (opp < fpsgo_arch_nr_get_opp_cpu(cpu))
		cap = pd_get_opp_capacity_legacy(cpu, opp);
	else
		cap = pd_get_opp_capacity_legacy(cpu, 0);

	return (unsigned int) cap;
}

int fpsgo_arch_nr_max_opp_cpu(void)
{
	struct cpufreq_policy curr_policy;
	int num_opp = 0, max_opp = 0;
	int cpu;

	for_each_possible_cpu(cpu) {
		cpufreq_get_policy(&curr_policy, cpu);
		num_opp = fpsgo_arch_nr_get_opp_cpu(cpu);
		cpu = cpumask_last(curr_policy.related_cpus);

		if (max_opp < num_opp)
			max_opp = num_opp;
	}

	return max_opp;
}

int fpsgo_arch_nr_freq_cpu(void)
{
	int  cpu, max_opp = 0;

	for_each_possible_cpu(cpu) {
		int opp = pd_get_cpu_opp(cpu);

		if (opp > max_opp)
			max_opp = opp;
	}

	return max_opp;
}

unsigned int fpsgo_cpufreq_get_freq_by_idx(
	int cpu, unsigned int opp)
{
	struct cpufreq_policy curr_policy;
	struct cpufreq_frequency_table *pos, *table;
	int idx;
	unsigned int max_freq = 0;

	cpufreq_get_policy(&curr_policy, cpu);
	table = curr_policy.freq_table;

	cpufreq_for_each_valid_entry_idx(pos, table, idx) {
		max_freq = max(pos->frequency, max_freq);
		if (idx == opp)
			return pos->frequency;
	}

	return max_freq;
}

struct k_list {
	struct list_head queue_list;
	int fpsgo2pwr_cmd;
	int fpsgo2pwr_value1;
	int fpsgo2pwr_value2;
};
static LIST_HEAD(head);
static int condition_get_cmd;
static DEFINE_MUTEX(fpsgo2pwr_lock);
static DECLARE_WAIT_QUEUE_HEAD(pwr_queue);
void fpsgo_sentcmd(int cmd, int value1, int value2)
{
	static struct k_list *node;

	mutex_lock(&fpsgo2pwr_lock);
	node = kmalloc(sizeof(*node), GFP_KERNEL);
	if (node == NULL)
		goto out;
	node->fpsgo2pwr_cmd = cmd;
	node->fpsgo2pwr_value1 = value1;
	node->fpsgo2pwr_value2 = value2;
	list_add_tail(&node->queue_list, &head);
	condition_get_cmd = 1;
out:
	mutex_unlock(&fpsgo2pwr_lock);
	wake_up_interruptible(&pwr_queue);
}

void fpsgo_ctrl2base_get_pwr_cmd(int *cmd, int *value1, int *value2)
{
	static struct k_list *node;

	wait_event_interruptible(pwr_queue, condition_get_cmd);
	mutex_lock(&fpsgo2pwr_lock);
	if (!list_empty(&head)) {
		node = list_first_entry(&head, struct k_list, queue_list);
		*cmd = node->fpsgo2pwr_cmd;
		*value1 = node->fpsgo2pwr_value1;
		*value2 = node->fpsgo2pwr_value2;
		list_del(&node->queue_list);
		kfree(node);
	}
	if (list_empty(&head))
		condition_get_cmd = 0;
	mutex_unlock(&fpsgo2pwr_lock);
}

uint32_t fpsgo_systrace_mask;

#define GENERATE_STRING(name, unused) #name
static const char * const mask_string[] = {
	FPSGO_SYSTRACE_LIST(GENERATE_STRING)
};

static int fpsgo_update_tracemark(void)
{
	return 1;
}

static noinline int tracing_mark_write(const char *buf)
{
	trace_printk(buf);
	return 0;
}

void __fpsgo_systrace_c(pid_t pid, unsigned long long bufID,
	int val, const char *fmt, ...)
{
	char log[256];
	va_list args;
	int len;
	char buf2[256];

	if (unlikely(!fpsgo_update_tracemark()))
		return;

	memset(log, ' ', sizeof(log));
	va_start(args, fmt);
	len = vsnprintf(log, sizeof(log), fmt, args);
	va_end(args);

	if (unlikely(len < 0))
		return;
	else if (unlikely(len == 256))
		log[255] = '\0';

	if (!bufID) {
		len = snprintf(buf2, sizeof(buf2), "C|%d|%s|%d\n", pid, log, val);
	} else {
		len = snprintf(buf2, sizeof(buf2), "C|%d|%s|%d|0x%llx\n",
			pid, log, val, bufID);
	}
	if (unlikely(len < 0))
		return;
	else if (unlikely(len == 256))
		buf2[255] = '\0';

	tracing_mark_write(buf2);
}

void __fpsgo_systrace_b(pid_t tgid, const char *fmt, ...)
{
	char log[256];
	va_list args;
	int len;
	char buf2[256];

	if (unlikely(!fpsgo_update_tracemark()))
		return;

	memset(log, ' ', sizeof(log));
	va_start(args, fmt);
	len = vsnprintf(log, sizeof(log), fmt, args);
	va_end(args);

	if (unlikely(len < 0))
		return;
	else if (unlikely(len == 256))
		log[255] = '\0';

	len = snprintf(buf2, sizeof(buf2), "B|%d|%s\n", tgid, log);

	if (unlikely(len < 0))
		return;
	else if (unlikely(len == 256))
		buf2[255] = '\0';

	tracing_mark_write(buf2);
}

void __fpsgo_systrace_e(void)
{
	char buf2[256];
	int len;

	if (unlikely(!fpsgo_update_tracemark()))
		return;

	len = snprintf(buf2, sizeof(buf2), "E\n");

	if (unlikely(len < 0))
		return;
	else if (unlikely(len == 256))
		buf2[255] = '\0';

	tracing_mark_write(buf2);
}

void fpsgo_main_trace(const char *fmt, ...)
{
	char log[256];
	va_list args;
	int len;


	va_start(args, fmt);
	len = vsnprintf(log, sizeof(log), fmt, args);

	if (unlikely(len == 256))
		log[255] = '\0';
	va_end(args);
	trace_printk(log);
}
EXPORT_SYMBOL(fpsgo_main_trace);

void fpsgo_render_tree_lock(const char *tag)
{
	mutex_lock(&fpsgo_render_lock);
}

void fpsgo_render_tree_unlock(const char *tag)
{
	mutex_unlock(&fpsgo_render_lock);
}

void fpsgo_lockprove(const char *tag)
{
	WARN_ON(!mutex_is_locked(&fpsgo_render_lock));
}

void fpsgo_thread_lock(struct mutex *mlock)
{
	fpsgo_lockprove(__func__);
	mutex_lock(mlock);
}

void fpsgo_thread_unlock(struct mutex *mlock)
{
	mutex_unlock(mlock);
}

void fpsgo_thread_lockprove(const char *tag, struct mutex *mlock)
{
	WARN_ON(!mutex_is_locked(mlock));
}

int fpsgo_get_tgid(int pid)
{
	struct task_struct *tsk;
	int tgid = 0;

	rcu_read_lock();
	tsk = find_task_by_vpid(pid);
	if (tsk)
		get_task_struct(tsk);
	rcu_read_unlock();

	if (!tsk)
		return 0;

	tgid = tsk->tgid;
	put_task_struct(tsk);

	return tgid;
}

void fpsgo_add_linger(struct render_info *thr)
{
	struct rb_node **p = &linger_tree.rb_node;
	struct rb_node *parent = NULL;
	struct render_info *tmp = NULL;

	fpsgo_lockprove(__func__);

	if (!thr)
		return;

	while (*p) {
		parent = *p;
		tmp = rb_entry(parent, struct render_info, linger_node);
		if ((uintptr_t)thr < (uintptr_t)tmp)
			p = &(*p)->rb_left;
		else if ((uintptr_t)thr > (uintptr_t)tmp)
			p = &(*p)->rb_right;
		else {
			FPSGO_LOGE("linger exist %d(%p)\n", thr->pid, thr);
			return;
		}
	}

	rb_link_node(&thr->linger_node, parent, p);
	rb_insert_color(&thr->linger_node, &linger_tree);
	thr->linger_ts = fpsgo_get_time();
	FPSGO_LOGI("add to linger %d(%p)(%llu)\n",
			thr->pid, thr, thr->linger_ts);
}

void fpsgo_del_linger(struct render_info *thr)
{
	fpsgo_lockprove(__func__);

	if (!thr)
		return;

	rb_erase(&thr->linger_node, &linger_tree);
	FPSGO_LOGI("del from linger %d(%p)\n", thr->pid, thr);
}

void fpsgo_traverse_linger(unsigned long long cur_ts)
{
	struct rb_node *n;
	struct render_info *pos;
	unsigned long long expire_ts;

	fpsgo_lockprove(__func__);

	if (cur_ts < TRAVERSE_PERIOD)
		return;

	expire_ts = cur_ts - TRAVERSE_PERIOD;

	n = rb_first(&linger_tree);
	while (n) {
		int tofree = 0;

		pos = rb_entry(n, struct render_info, linger_node);
		FPSGO_LOGI("-%d(%p)(%llu),", pos->pid, pos, pos->linger_ts);

		fpsgo_thread_lock(&pos->thr_mlock);

		if (pos->linger_ts && pos->linger_ts < expire_ts) {
			FPSGO_LOGI("timeout %d(%p)(%llu),",
				pos->pid, pos, pos->linger_ts);
			fpsgo_base2fbt_cancel_jerk(pos);
			fpsgo_del_linger(pos);
			tofree = 1;
			n = rb_first(&linger_tree);
		} else
			n = rb_next(n);

		fpsgo_thread_unlock(&pos->thr_mlock);

		if (tofree)
			kfree(pos);
	}
}

int fpsgo_base_is_finished(struct render_info *thr)
{
	fpsgo_lockprove(__func__);
	fpsgo_thread_lockprove(__func__, &(thr->thr_mlock));

	if (!fpsgo_base2fbt_is_finished(thr))
		return 0;

	return 1;
}

void fpsgo_reset_pid_attr(struct fpsgo_boost_attr *boost_attr)
{
	if (boost_attr) {
		boost_attr->llf_task_policy_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->light_loading_policy_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->loading_th_by_pid = BY_PID_DEFAULT_VAL;

		boost_attr->rescue_second_enable_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->rescue_second_time_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->rescue_second_group_by_pid = BY_PID_DEFAULT_VAL;

		boost_attr->filter_frame_enable_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->filter_frame_window_size_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->filter_frame_kmin_by_pid = BY_PID_DEFAULT_VAL;

		boost_attr->boost_affinity_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->boost_lr_by_pid = BY_PID_DEFAULT_VAL;

		boost_attr->separate_aa_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->separate_release_sec_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->limit_uclamp_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->limit_ruclamp_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->limit_uclamp_m_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->limit_ruclamp_m_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->separate_pct_b_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->separate_pct_m_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->qr_enable_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->qr_t2wnt_x_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->qr_t2wnt_y_p_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->qr_t2wnt_y_n_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->gcc_enable_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->gcc_fps_margin_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->gcc_up_sec_pct_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->gcc_up_step_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->gcc_down_sec_pct_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->gcc_down_step_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->gcc_reserved_up_quota_pct_by_pid =
			BY_PID_DEFAULT_VAL;
		boost_attr->gcc_reserved_down_quota_pct_by_pid =
			BY_PID_DEFAULT_VAL;
		boost_attr->gcc_enq_bound_thrs_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->gcc_deq_bound_thrs_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->gcc_enq_bound_quota_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->gcc_deq_bound_quota_by_pid = BY_PID_DEFAULT_VAL;
		boost_attr->blc_boost_by_pid = BY_PID_DEFAULT_VAL;

		boost_attr->reset_taskmask = BY_PID_DEFAULT_VAL;

		boost_attr->limit_cfreq2cap = BY_PID_DEFAULT_VAL;
		boost_attr->limit_rfreq2cap = BY_PID_DEFAULT_VAL;
		boost_attr->limit_cfreq2cap_m = BY_PID_DEFAULT_VAL;
		boost_attr->limit_rfreq2cap_m = BY_PID_DEFAULT_VAL;
	}
}

static int render_key_compare(struct fbt_render_key target,
		struct fbt_render_key node_key)
{
	if (target.key1 > node_key.key1)
		return 1;
	else if (target.key1 < node_key.key1)
		return -1;

	/* key 1 is equal, compare key 2 */
	if (target.key2 > node_key.key2)
		return 1;
	else if (target.key2 < node_key.key2)
		return -1;
	else
		return 0;
}

void fpsgo_reset_render_pid_attr(int tgid)
{
	struct rb_node *n;
	struct render_info *iter;

	if (!tgid)
		return;

	fpsgo_lockprove(__func__);

	for (n = rb_first(&render_pid_tree); n != NULL; n = rb_next(n)) {
		iter = rb_entry(n, struct render_info, render_key_node);

		if (iter->tgid == tgid) {
			fpsgo_thread_lock(&(iter->thr_mlock));
			fpsgo_reset_pid_attr(&(iter->attr));
			fpsgo_thread_unlock(&(iter->thr_mlock));
		}
	}
}

struct render_info *eara2fpsgo_search_render_info(int pid,
		unsigned long long buffer_id)
{
	struct render_info *iter = NULL;
	struct rb_node *n;

	fpsgo_lockprove(__func__);

	for (n = rb_first(&render_pid_tree); n != NULL; n = rb_next(n)) {
		iter = rb_entry(n, struct render_info, render_key_node);
		if (iter->pid == pid && iter->buffer_id == buffer_id)
			return iter;
	}
	return NULL;
}

struct render_info *fpsgo_search_and_add_render_info(int pid,
	unsigned long long identifier, int force)
{
	struct rb_node **p = &render_pid_tree.rb_node;
	struct rb_node *parent = NULL;
	struct render_info *iter_thr = NULL;
	int tgid;
	struct fbt_render_key render_key;

	render_key.key1 = pid;
	render_key.key2 = identifier;

	fpsgo_lockprove(__func__);

	tgid = fpsgo_get_tgid(pid);

	while (*p) {
		parent = *p;
		iter_thr = rb_entry(parent, struct render_info, render_key_node);

		if (render_key_compare(render_key, iter_thr->render_key) < 0)
			p = &(*p)->rb_left;
		else if (render_key_compare(render_key, iter_thr->render_key) > 0)
			p = &(*p)->rb_right;
		else
			return iter_thr;
	}

	if (!force)
		return NULL;

	iter_thr = kzalloc(sizeof(*iter_thr), GFP_KERNEL);
	if (!iter_thr)
		return NULL;

	mutex_init(&iter_thr->thr_mlock);
	INIT_LIST_HEAD(&(iter_thr->bufferid_list));
	iter_thr->pid = pid;
	iter_thr->render_key.key1 = pid;
	iter_thr->render_key.key2 = identifier;
	iter_thr->identifier = identifier;
	iter_thr->tgid = tgid;
	iter_thr->frame_type = BY_PASS_TYPE;

	fbt_set_render_boost_attr(iter_thr);

	rb_link_node(&iter_thr->render_key_node, parent, p);
	rb_insert_color(&iter_thr->render_key_node, &render_pid_tree);

	return iter_thr;
}

void fpsgo_delete_render_info(int pid,
	unsigned long long buffer_id, unsigned long long identifier)
{
	struct render_info *data;
	int delete = 0;
	int check_max_blc = 0;
	int max_pid = 0;
	unsigned long long max_buffer_id = 0;
	int max_ret;

	fpsgo_lockprove(__func__);

	data = fpsgo_search_and_add_render_info(pid, identifier, 0);

	if (!data)
		return;
	fpsgo_thread_lock(&data->thr_mlock);
	max_ret = fpsgo_base2fbt_get_max_blc_pid(&max_pid, &max_buffer_id);
	if (max_ret && pid == max_pid && buffer_id == max_buffer_id)
		check_max_blc = 1;

	rb_erase(&data->render_key_node, &render_pid_tree);
	list_del(&(data->bufferid_list));
	fpsgo_base2fbt_item_del(data->p_blc, data->dep_arr, data);
	data->p_blc = NULL;
	data->dep_arr = NULL;

	if (fpsgo_base_is_finished(data))
		delete = 1;
	else {
		delete = 0;
		data->linger = 1;
		fpsgo_add_linger(data);
	}
	fpsgo_thread_unlock(&data->thr_mlock);

	fpsgo_delete_hwui_info(data->pid);

	if (check_max_blc)
		fpsgo_base2fbt_check_max_blc();

	if (delete == 1)
		kfree(data);
}

struct hwui_info *fpsgo_search_and_add_hwui_info(int pid, int force)
{
	struct rb_node **p = &hwui_info_tree.rb_node;
	struct rb_node *parent = NULL;
	struct hwui_info *tmp = NULL;

	fpsgo_lockprove(__func__);

	while (*p) {
		parent = *p;
		tmp = rb_entry(parent, struct hwui_info, entry);

		if (pid < tmp->pid)
			p = &(*p)->rb_left;
		else if (pid > tmp->pid)
			p = &(*p)->rb_right;
		else
			return tmp;
	}

	if (!force)
		return NULL;

	tmp = kzalloc(sizeof(*tmp), GFP_KERNEL);
	if (!tmp)
		return NULL;

	tmp->pid = pid;

	rb_link_node(&tmp->entry, parent, p);
	rb_insert_color(&tmp->entry, &hwui_info_tree);

	return tmp;
}

void fpsgo_delete_hwui_info(int pid)
{
	struct hwui_info *data;

	fpsgo_lockprove(__func__);

	data = fpsgo_search_and_add_hwui_info(pid, 0);

	if (!data)
		return;

	rb_erase(&data->entry, &hwui_info_tree);
	kfree(data);
}

#if FPSGO_MW
int is_to_delete_fpsgo_attr(struct fpsgo_attr_by_pid *fpsgo_attr)
{
	struct fpsgo_boost_attr boost_attr;

	if (!fpsgo_attr)
		return 0;

	boost_attr = fpsgo_attr->attr;
	if	(boost_attr.rescue_second_enable_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.rescue_second_time_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.rescue_second_group_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.llf_task_policy_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.light_loading_policy_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.loading_th_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.filter_frame_enable_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.filter_frame_window_size_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.filter_frame_kmin_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.boost_affinity_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.boost_lr_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.separate_aa_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.separate_release_sec_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.limit_uclamp_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.limit_ruclamp_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.limit_uclamp_m_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.limit_ruclamp_m_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.separate_pct_b_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.separate_pct_m_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.qr_enable_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.qr_t2wnt_x_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.qr_t2wnt_y_n_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.qr_t2wnt_y_p_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.blc_boost_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.gcc_deq_bound_quota_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.gcc_deq_bound_thrs_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.gcc_down_sec_pct_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.gcc_down_step_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.gcc_enable_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.gcc_enq_bound_quota_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.gcc_enq_bound_thrs_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.gcc_fps_margin_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.gcc_reserved_down_quota_pct_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.gcc_reserved_up_quota_pct_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.gcc_up_sec_pct_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.gcc_up_step_by_pid == BY_PID_DEFAULT_VAL &&
			boost_attr.reset_taskmask == BY_PID_DEFAULT_VAL &&
			boost_attr.limit_cfreq2cap == BY_PID_DEFAULT_VAL &&
			boost_attr.limit_rfreq2cap == BY_PID_DEFAULT_VAL &&
			boost_attr.limit_cfreq2cap_m == BY_PID_DEFAULT_VAL &&
			boost_attr.limit_rfreq2cap_m == BY_PID_DEFAULT_VAL) {
		return 1;
	}
	if (boost_attr.rescue_second_enable_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.rescue_second_time_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.rescue_second_group_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.llf_task_policy_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.light_loading_policy_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.loading_th_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.filter_frame_enable_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.filter_frame_window_size_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.filter_frame_kmin_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.boost_affinity_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.boost_lr_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.separate_aa_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.separate_release_sec_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.limit_uclamp_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.limit_ruclamp_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.limit_uclamp_m_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.limit_ruclamp_m_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.separate_pct_b_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.separate_pct_m_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.qr_enable_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.qr_t2wnt_x_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.qr_t2wnt_y_n_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.qr_t2wnt_y_p_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.blc_boost_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.gcc_deq_bound_quota_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.gcc_deq_bound_thrs_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.gcc_down_sec_pct_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.gcc_down_step_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.gcc_enable_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.gcc_enq_bound_quota_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.gcc_enq_bound_thrs_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.gcc_fps_margin_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.gcc_reserved_down_quota_pct_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.gcc_reserved_up_quota_pct_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.gcc_up_sec_pct_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.gcc_up_step_by_pid == BY_PID_DELETE_VAL ||
			boost_attr.reset_taskmask == BY_PID_DELETE_VAL ||
			boost_attr.limit_cfreq2cap == BY_PID_DELETE_VAL ||
			boost_attr.limit_rfreq2cap == BY_PID_DELETE_VAL ||
			boost_attr.limit_cfreq2cap_m == BY_PID_DELETE_VAL ||
			boost_attr.limit_rfreq2cap_m == BY_PID_DELETE_VAL) {
		return 1;
	}
	return 0;
}

static int delete_oldest_attr(void)
{
	struct rb_node *n;
	struct fpsgo_attr_by_pid *iter;
	unsigned long long oldest_ts = (unsigned long long)-1; // max val
	int tgid = 0, count = 0;

	fpsgo_lockprove(__func__);

	for (n = rb_first(&fpsgo_attr_by_pid_tree), count = 0; n != NULL;
		n = rb_next(n), count++) {
		iter = rb_entry(n,  struct fpsgo_attr_by_pid, entry);
		if (iter->ts < oldest_ts) {
			tgid = iter->tgid;
			oldest_ts = iter->ts;
		}
	}

	if (count >= FPSGO_MAX_TREE_SIZE)
		return tgid;
	else
		return 0;
}

struct fpsgo_attr_by_pid *fpsgo_find_attr_by_pid(int pid, int add_new)
{
	struct rb_node **p = &fpsgo_attr_by_pid_tree.rb_node;
	struct rb_node *parent = NULL;
	struct fpsgo_attr_by_pid *iter_attr = NULL;
	int delete_tgid = 0;

	fpsgo_lockprove(__func__);

	while (*p) {
		parent = *p;
		iter_attr = rb_entry(parent, struct fpsgo_attr_by_pid, entry);

		if (pid < iter_attr->tgid)
			p = &(*p)->rb_left;
		else if (pid > iter_attr->tgid)
			p = &(*p)->rb_right;
		else
			return iter_attr;
	}

	if (!add_new)
		return NULL;

	iter_attr = kzalloc(sizeof(*iter_attr), GFP_KERNEL);
	if (!iter_attr)
		return NULL;

	iter_attr->ts = fpsgo_get_time();
	iter_attr->tgid = pid;
	fpsgo_reset_pid_attr(&(iter_attr->attr));
	rb_link_node(&iter_attr->entry, parent, p);
	rb_insert_color(&iter_attr->entry, &fpsgo_attr_by_pid_tree);

	/* If the tree size exceeds, then delete the oldest node. */
	delete_tgid = delete_oldest_attr();
	if (delete_tgid)
		delete_attr_by_pid(delete_tgid);

	return iter_attr;
}

void delete_attr_by_pid(int tgid)
{
	struct fpsgo_attr_by_pid *data;

	fpsgo_lockprove(__func__);
	data = fpsgo_find_attr_by_pid(tgid, 0);
	if (!data)
		return;
	rb_erase(&data->entry, &fpsgo_attr_by_pid_tree);
	kfree(data);
}

void delete_all_attr_items_in_tree(void)
{
	struct rb_node *n;
	struct fpsgo_attr_by_pid *iter;

	fpsgo_lockprove(__func__);

	n = rb_first(&fpsgo_attr_by_pid_tree);

	while (n) {
		iter = rb_entry(n,  struct fpsgo_attr_by_pid, entry);

		rb_erase(&iter->entry, &fpsgo_attr_by_pid_tree);
		n = rb_first(&fpsgo_attr_by_pid_tree);

		kfree(iter);
	}
}

#endif  // FPSGO_MW

struct sbe_info *fpsgo_search_and_add_sbe_info(int pid, int force)
{
	struct rb_node **p = &sbe_info_tree.rb_node;
	struct rb_node *parent = NULL;
	struct sbe_info *tmp = NULL;

	fpsgo_lockprove(__func__);

	while (*p) {
		parent = *p;
		tmp = rb_entry(parent, struct sbe_info, entry);

		if (pid < tmp->pid)
			p = &(*p)->rb_left;
		else if (pid > tmp->pid)
			p = &(*p)->rb_right;
		else
			return tmp;
	}

	if (!force)
		return NULL;

	tmp = kzalloc(sizeof(*tmp), GFP_KERNEL);
	if (!tmp)
		return NULL;

	tmp->pid = pid;

	rb_link_node(&tmp->entry, parent, p);
	rb_insert_color(&tmp->entry, &sbe_info_tree);

	return tmp;
}

void fpsgo_delete_sbe_info(int pid)
{
	struct sbe_info *data;

	fpsgo_lockprove(__func__);

	data = fpsgo_search_and_add_sbe_info(pid, 0);

	if (!data)
		return;

	rb_erase(&data->entry, &sbe_info_tree);
	kfree(data);
}

void fpsgo_delete_oldest_fps_control_pid_info(void)
{
	unsigned long long min_ts = ULLONG_MAX;
	struct fps_control_pid_info *min_iter = NULL, *tmp_iter = NULL;
	struct rb_root *rbr = NULL;
	struct rb_node *rbn = NULL;

	if (RB_EMPTY_ROOT(&fps_control_pid_info_tree))
		return;

	rbr = &fps_control_pid_info_tree;
	for (rbn = rb_first(rbr); rbn; rbn = rb_next(rbn)) {
		tmp_iter = rb_entry(rbn, struct fps_control_pid_info, entry);
		if (tmp_iter->ts < min_ts) {
			min_ts = tmp_iter->ts;
			min_iter = tmp_iter;
		}
	}

	if (!min_iter)
		return;

	rb_erase(&min_iter->entry, &fps_control_pid_info_tree);
	kfree(min_iter);
	total_fps_control_pid_info_num--;
}

struct fps_control_pid_info *fpsgo_search_and_add_fps_control_pid(int pid, int force)
{
	struct rb_node **p = &fps_control_pid_info_tree.rb_node;
	struct rb_node *parent = NULL;
	struct fps_control_pid_info *tmp = NULL;

	fpsgo_lockprove(__func__);

	while (*p) {
		parent = *p;
		tmp = rb_entry(parent, struct fps_control_pid_info, entry);

		if (pid < tmp->pid)
			p = &(*p)->rb_left;
		else if (pid > tmp->pid)
			p = &(*p)->rb_right;
		else
			return tmp;
	}

	if (!force)
		return NULL;

	tmp = kzalloc(sizeof(*tmp), GFP_KERNEL);
	if (!tmp)
		return NULL;

	tmp->pid = pid;
	tmp->ts = fpsgo_get_time();

	rb_link_node(&tmp->entry, parent, p);
	rb_insert_color(&tmp->entry, &fps_control_pid_info_tree);
	total_fps_control_pid_info_num++;

	if (total_fps_control_pid_info_num > FPSGO_MAX_TREE_SIZE)
		fpsgo_delete_oldest_fps_control_pid_info();

	return tmp;
}

void fpsgo_delete_fpsgo_control_pid(int pid)
{
	struct fps_control_pid_info *data;

	fpsgo_lockprove(__func__);

	data = fpsgo_search_and_add_fps_control_pid(pid, 0);

	if (!data)
		return;

	rb_erase(&data->entry, &fps_control_pid_info_tree);
	kfree(data);
	total_fps_control_pid_info_num--;
}

int fpsgo_get_all_fps_control_pid_info(struct fps_control_pid_info *arr)
{
	int index = 0;
	struct fps_control_pid_info *iter = NULL;
	struct rb_root *rbr = NULL;
	struct rb_node *rbn = NULL;

	rbr = &fps_control_pid_info_tree;
	for (rbn = rb_first(rbr); rbn; rbn = rb_next(rbn)) {
		iter = rb_entry(rbn, struct fps_control_pid_info, entry);
		arr[index].pid = iter->pid;
		arr[index].ts = iter->ts;
		index++;
		if (index >= FPSGO_MAX_TREE_SIZE)
			break;
	}

	return index;
}

static void fpsgo_check_BQid_status(void)
{
	struct rb_node *n;
	struct BQ_id *pos;
	int tgid = 0;

	fpsgo_lockprove(__func__);

	n = rb_first(&BQ_id_list);
	while (n) {
		pos = rb_entry(n, struct BQ_id, entry);
		tgid = fpsgo_get_tgid(pos->pid);
		if (!tgid) {
			rb_erase(&pos->entry, &BQ_id_list);
			n = rb_first(&BQ_id_list);
			kfree(pos);
		} else {
			n = rb_next(n);
		}
	}
}

void fpsgo_clear_llf_cpu_policy(void)
{
	struct rb_node *n;
	struct render_info *iter;

	fpsgo_render_tree_lock(__func__);

	for (n = rb_first(&render_pid_tree); n; n = rb_next(n)) {
		iter = rb_entry(n, struct render_info, render_key_node);

		fpsgo_thread_lock(&iter->thr_mlock);
		fpsgo_base2fbt_clear_llf_policy(iter);
		fpsgo_thread_unlock(&iter->thr_mlock);
	}

	fpsgo_render_tree_unlock(__func__);
}

#if FPSGO_MW
void fpsgo_clear_llf_cpu_policy_by_pid(int tgid)
{
	struct rb_node *n;
	struct render_info *iter;

	fpsgo_lockprove(__func__);
	for (n = rb_first(&render_pid_tree); n != NULL; n = rb_next(n)) {
		iter = rb_entry(n, struct render_info, render_key_node);

		if (iter->tgid == tgid) {
			fpsgo_thread_lock(&iter->thr_mlock);
			fpsgo_base2fbt_clear_llf_policy(iter);
			fpsgo_thread_unlock(&iter->thr_mlock);
		}
	}

}
#endif  // FPSGO_MW

static void fpsgo_clear_uclamp_boost_locked(void)
{
	struct rb_node *n;
	struct render_info *iter;

	fpsgo_lockprove(__func__);

	for (n = rb_first(&render_pid_tree); n; n = rb_next(n)) {
		iter = rb_entry(n, struct render_info, render_key_node);

		fpsgo_thread_lock(&iter->thr_mlock);
		fpsgo_base2fbt_set_min_cap(iter, 0, 0, 0);
		fpsgo_thread_unlock(&iter->thr_mlock);
	}
}

void fpsgo_clear_uclamp_boost(void)
{
	fpsgo_render_tree_lock(__func__);

	fpsgo_clear_uclamp_boost_locked();

	fpsgo_render_tree_unlock(__func__);
}

void fpsgo_check_thread_status(void)
{
	unsigned long long ts = fpsgo_get_time();
	unsigned long long expire_ts;
	int delete = 0;
	int check_max_blc = 0;
	struct rb_node *n;
	struct render_info *iter;
	int temp_max_pid = 0;
	unsigned long long temp_max_bufid = 0;
	int rb_tree_empty = 0;

	if (ts < TIME_1S)
		return;

	expire_ts = ts - TIME_1S;

	fpsgo_render_tree_lock(__func__);
	fpsgo_base2fbt_get_max_blc_pid(&temp_max_pid, &temp_max_bufid);

	n = rb_first(&render_pid_tree);
	while (n) {
		iter = rb_entry(n, struct render_info, render_key_node);

		fpsgo_thread_lock(&iter->thr_mlock);

		if (iter->t_enqueue_start < expire_ts) {
			if (iter->pid == temp_max_pid &&
				iter->buffer_id == temp_max_bufid)
				check_max_blc = 1;

			rb_erase(&iter->render_key_node, &render_pid_tree);
			list_del(&(iter->bufferid_list));
			fpsgo_base2fbt_item_del(iter->p_blc, iter->dep_arr, iter);
			iter->p_blc = NULL;
			iter->dep_arr = NULL;
			n = rb_first(&render_pid_tree);

			if (fpsgo_base_is_finished(iter))
				delete = 1;
			else {
				delete = 0;
				iter->linger = 1;
				fpsgo_add_linger(iter);
			}

			fpsgo_thread_unlock(&iter->thr_mlock);

			fpsgo_delete_hwui_info(iter->pid);

			if (delete == 1)
				kfree(iter);

		} else {

			n = rb_next(n);

			fpsgo_thread_unlock(&iter->thr_mlock);
		}
	}

	fpsgo_check_BQid_status();
	fpsgo_traverse_linger(ts);

	fpsgo_render_tree_unlock(__func__);

	fpsgo_fstb2comp_check_connect_api();

	if (check_max_blc)
		fpsgo_base2fbt_check_max_blc();

	fpsgo_render_tree_lock(__func__);
	rb_tree_empty = RB_EMPTY_ROOT(&render_pid_tree);
	fpsgo_render_tree_unlock(__func__);

	if (rb_tree_empty)
		fpsgo_base2fbt_no_one_render();
}

void fpsgo_clear(void)
{
	int delete = 0;
	struct rb_node *n;
	struct render_info *iter;

	fpsgo_render_tree_lock(__func__);

	n = rb_first(&render_pid_tree);
	while (n) {
		iter = rb_entry(n, struct render_info, render_key_node);
		fpsgo_thread_lock(&iter->thr_mlock);

		rb_erase(&iter->render_key_node, &render_pid_tree);
		list_del(&(iter->bufferid_list));
		fpsgo_base2fbt_item_del(iter->p_blc, iter->dep_arr, iter);
		iter->p_blc = NULL;
		iter->dep_arr = NULL;
		n = rb_first(&render_pid_tree);

		if (fpsgo_base_is_finished(iter))
			delete = 1;
		else {
			delete = 0;
			iter->linger = 1;
			fpsgo_add_linger(iter);
		}

		fpsgo_thread_unlock(&iter->thr_mlock);

		fpsgo_delete_hwui_info(iter->pid);

		if (delete == 1)
			kfree(iter);
	}

#if FPSGO_MW
	delete_all_attr_items_in_tree();
#endif  // FPSGO_MW

	fpsgo_render_tree_unlock(__func__);
}

int fpsgo_update_swap_buffer(int pid)
{
	fpsgo_render_tree_lock(__func__);
	fpsgo_search_and_add_hwui_info(pid, 1);
	fpsgo_render_tree_unlock(__func__);
	return 0;
}

int fpsgo_sbe_rescue_traverse(int pid, int start, int enhance, unsigned long long frame_id)
{
	struct rb_node *n;
	struct render_info *iter;

	fpsgo_render_tree_lock(__func__);
	for (n = rb_first(&render_pid_tree); n != NULL; n = rb_next(n)) {
		iter = rb_entry(n, struct render_info, render_key_node);
		fpsgo_thread_lock(&iter->thr_mlock);
		if (iter->pid == pid)
			fpsgo_sbe2fbt_rescue(iter, start, enhance, frame_id);
		fpsgo_thread_unlock(&iter->thr_mlock);
	}
	fpsgo_render_tree_unlock(__func__);
	return 0;
}

void fpsgo_stop_boost_by_render(struct render_info *thr)
{
	fpsgo_lockprove(__func__);
	fpsgo_thread_lockprove(__func__, &(thr->thr_mlock));
	fpsgo_base2fbt_stop_boost(thr);
}

void fpsgo_stop_boost_by_pid(int pid)
{
	struct rb_node *n;
	struct render_info *iter;

	if (pid <= 1)
		return;

	fpsgo_lockprove(__func__);

	for (n = rb_first(&render_pid_tree); n != NULL; n = rb_next(n)) {
		iter = rb_entry(n, struct render_info, render_key_node);
		fpsgo_thread_lock(&iter->thr_mlock);
		if (iter->pid == pid)
			fpsgo_base2fbt_stop_boost(iter);
		fpsgo_thread_unlock(&iter->thr_mlock);
	}
}

static struct BQ_id *fpsgo_get_BQid_by_key(struct fbt_render_key key,
		int add, int pid, long long identifier)
{
	struct rb_node **p = &BQ_id_list.rb_node;
	struct rb_node *parent = NULL;
	struct BQ_id *pos;

	fpsgo_lockprove(__func__);

	while (*p) {
		parent = *p;
		pos = rb_entry(parent, struct BQ_id, entry);

		if (render_key_compare(key, pos->key) < 0)
			p = &(*p)->rb_left;
		else if (render_key_compare(key, pos->key) > 0)
			p = &(*p)->rb_right;
		else
			return pos;
	}

	if (!add)
		return NULL;

	pos = kzalloc(sizeof(*pos), GFP_KERNEL);
	if (!pos)
		return NULL;

	pos->key.key1 = key.key1;
	pos->key.key2 = key.key2;
	pos->pid = pid;
	pos->identifier = identifier;
	rb_link_node(&pos->entry, parent, p);
	rb_insert_color(&pos->entry, &BQ_id_list);

	FPSGO_LOGI("add BQid key1 %d, key2 %llu, pid %d, id 0x%llx\n",
		   key.key1, key.key2, pid, identifier);
	return pos;
}

struct BQ_id *fpsgo_find_BQ_id(int pid, int tgid,
		long long identifier, int action)
{
	struct rb_node *n;
	struct rb_node *next;
	struct BQ_id *pos;
	struct fbt_render_key key;
	int tgid_key = tgid;
	unsigned long long identifier_key = identifier;
	int done = 0;

	if (!tgid_key) {
		tgid_key = fpsgo_get_tgid(pid);
		if (!tgid_key)
			return NULL;
	}

	key.key1 = tgid_key;
	key.key2 = identifier_key;

	fpsgo_lockprove(__func__);

	switch (action) {
	case ACTION_FIND:
	case ACTION_FIND_ADD:
		FPSGO_LOGI("find %s pid %d, id %llu, key1 %d, key2 %llu\n",
			(action == ACTION_FIND_ADD)?"add":"",
			pid, identifier, key.key1, key.key2);

		return fpsgo_get_BQid_by_key(key, action == ACTION_FIND_ADD,
					     pid, identifier);

	case ACTION_FIND_DEL:
		for (n = rb_first(&BQ_id_list); n; n = next) {
			next = rb_next(n);

			pos = rb_entry(n, struct BQ_id, entry);
			if (render_key_compare(pos->key, key) == 0) {
				FPSGO_LOGI(
					"find del pid %d, id %llu, key1 %d, key2 %llu\n",
					pid, identifier, key.key1, key.key2);
				rb_erase(n, &BQ_id_list);
				kfree(pos);
				done = 1;
				break;
			}
		}
		if (!done)
			FPSGO_LOGE("del fail key1 %d, key2 %llu\n", key.key1, key.key2);
		return NULL;
	default:
		FPSGO_LOGE("[ERROR] unknown action %d\n", action);
		return NULL;
	}
}

int fpsgo_get_BQid_pair(int pid, int tgid, long long identifier,
		unsigned long long *buffer_id, int *queue_SF, int enqueue)
{
	struct BQ_id *pair;

	fpsgo_lockprove(__func__);

	pair = fpsgo_find_BQ_id(pid, tgid, identifier, ACTION_FIND);

	if (pair) {
		*buffer_id = pair->buffer_id;
		*queue_SF = pair->queue_SF;
		if (enqueue)
			pair->queue_pid = pid;
		return 1;
	}

	return 0;
}

static ssize_t systrace_mask_show(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	int i;
	char *temp = NULL;
	int pos = 0;
	int length = 0;

	temp = kcalloc(FPSGO_SYSFS_MAX_BUFF_SIZE, sizeof(char), GFP_KERNEL);
	if (!temp)
		goto out;

	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
			" Current enabled systrace:\n");
	pos += length;

	for (i = 0; (1U << i) < FPSGO_DEBUG_MAX; i++) {
		length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
			"  %-*s ... %s\n", 12, mask_string[i],
		   fpsgo_systrace_mask & (1U << i) ?
		   "On" : "Off");
		pos += length;

	}

	length = scnprintf(buf, PAGE_SIZE, "%s", temp);

out:
	kfree(temp);
	return length;
}

static ssize_t systrace_mask_store(struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	uint32_t val = -1;
	char *acBuffer = NULL;
	uint32_t arg;

	acBuffer = kcalloc(FPSGO_SYSFS_MAX_BUFF_SIZE, sizeof(char), GFP_KERNEL);
	if (!acBuffer)
		goto out;

	if ((count > 0) && (count < FPSGO_SYSFS_MAX_BUFF_SIZE)) {
		if (scnprintf(acBuffer, FPSGO_SYSFS_MAX_BUFF_SIZE, "%s", buf)) {
			if (kstrtou32(acBuffer, 0, &arg) == 0)
				val = arg;
			else
				goto out;
		}
	}

	fpsgo_systrace_mask = val & (FPSGO_DEBUG_MAX - 1U);

out:
	kfree(acBuffer);
	return count;
}

static KOBJ_ATTR_RW(systrace_mask);

static ssize_t fpsgo_enable_show(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", fpsgo_is_enable());
}

static ssize_t fpsgo_enable_store(struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int val = -1;
	char *acBuffer = NULL;
	int arg;

	acBuffer = kcalloc(FPSGO_SYSFS_MAX_BUFF_SIZE, sizeof(char), GFP_KERNEL);
	if (!acBuffer)
		goto out;

	if ((count > 0) && (count < FPSGO_SYSFS_MAX_BUFF_SIZE)) {
		if (scnprintf(acBuffer, FPSGO_SYSFS_MAX_BUFF_SIZE, "%s", buf)) {
			if (kstrtoint(acBuffer, 0, &arg) == 0)
				val = arg;
			else
				goto out;
		}
	}

	if (val > 1 || val < 0)
		goto out;

	fpsgo_switch_enable(val);

out:
	kfree(acBuffer);
	return count;
}

static KOBJ_ATTR_RWO(fpsgo_enable);

static ssize_t render_info_show(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	struct rb_node *n;
	struct render_info *iter;
	struct hwui_info *h_info;
	struct task_struct *tsk;
	char *temp = NULL;
	int pos = 0;
	int length = 0;

	temp = kcalloc(FPSGO_SYSFS_MAX_BUFF_SIZE, sizeof(char), GFP_KERNEL);
	if (!temp)
		goto out;

	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
			"\n  PID  NAME  TGID  TYPE  API  BufferID");
	pos += length;
	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
			"    FRAME_L    ENQ_L    ENQ_S    ENQ_E");
	pos += length;
	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
			"    DEQ_L     DEQ_S    DEQ_E    HWUI\n");
	pos += length;

	fpsgo_render_tree_lock(__func__);
	rcu_read_lock();

	for (n = rb_first(&render_pid_tree); n != NULL; n = rb_next(n)) {
		iter = rb_entry(n, struct render_info, render_key_node);
		tsk = find_task_by_vpid(iter->tgid);
		if (tsk) {
			get_task_struct(tsk);

			length = scnprintf(temp + pos,
					FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
					"%5d %4s %4d %4d %4d 0x%llx",
				iter->pid, tsk->comm,
				iter->tgid, iter->frame_type,
				iter->api, iter->buffer_id);
			pos += length;
			length = scnprintf(temp + pos,
					FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
					"  %4llu %4llu %4llu %4llu %4llu %4llu %d\n",
				iter->enqueue_length,
				iter->t_enqueue_start, iter->t_enqueue_end,
				iter->dequeue_length, iter->t_dequeue_start,
				iter->t_dequeue_end, iter->hwui);
			pos += length;
			put_task_struct(tsk);
		}
	}

	rcu_read_unlock();

	for (n = rb_first(&linger_tree); n != NULL; n = rb_next(n)) {
		iter = rb_entry(n, struct render_info, linger_node);
		length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
			"(%5d %4llu) linger %d timer %d\n",
			iter->pid, iter->buffer_id, iter->linger,
			fpsgo_base2fbt_is_finished(iter));
		pos += length;
	}

	/* hwui tree */
	for (n = rb_first(&hwui_info_tree); n != NULL; n = rb_next(n)) {
		h_info = rb_entry(n, struct hwui_info, entry);
		length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
			"HWUI: %d\n", h_info->pid);
		pos += length;
	}

	fpsgo_render_tree_unlock(__func__);

	length = scnprintf(buf, PAGE_SIZE, "%s", temp);

out:
	kfree(temp);
	return length;
}

static KOBJ_ATTR_RO(render_info);

#if FPSGO_MW
static ssize_t render_info_params_show(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	struct rb_node *n;
	struct render_info *iter;
	struct task_struct *tsk;
	struct fpsgo_boost_attr attr_item;
	char *temp = NULL;
	int pos = 0;
	int length = 0;

	temp = kcalloc(FPSGO_SYSFS_MAX_BUFF_SIZE, sizeof(char), GFP_KERNEL);
	if (!temp)
		goto out;

	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
		"\nNEW PID: PID, NAME, TGID\n");
	pos += length;

	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
		" llf_task_policy, loading_th, light_loading_policy,\n");
	pos += length;

	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
		" rescue_second_enable, rescue_second_time, rescue_second_group\n");
	pos += length;

	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
		" ff_enable, ff_window_size, ff_k_min\n");
	pos += length;

	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
		" boost_affinity, boost_LR, reset_taskmask\n");
	pos += length;

	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
		" separate_aa, separate_release_sec_by_pid, pct_b, pct_m, blc_boost\n");
	pos += length;

	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
		" uclamp, ruclamp, uclamp_m, ruclamp_m\n");
	pos += length;

	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
				" qr_enable, qr_t2wnt_x, qr_t2wnt_y_p, qr_t2wnt_y_n\n");
	pos += length;
	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
				" gcc_enable, gcc_fps_margin, gcc_up_sec_pct, gcc_up_step, gcc_reserved_up_quota_pct\n");
	pos += length;
	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
				" gcc_down_sec_pct, gcc_down_step, gcc_reserved_down_quota_pct\n");
	pos += length;
	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
				" gcc_enq_bound_thrs, gcc_enq_bound_quota, gcc_deq_bound_thrs, gcc_deq_bound_quota\n");
	pos += length;
	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
			" reset_taskmask, limit_cfreq2cap, limit_rfreq2cap, limit_cfreq2cap_m, limit_rfreq2cap_m\n");
	pos += length;

	fpsgo_render_tree_lock(__func__);
	rcu_read_lock();

	for (n = rb_first(&render_pid_tree); n != NULL; n = rb_next(n)) {
		iter = rb_entry(n, struct render_info, render_key_node);
		attr_item = iter->attr;
		tsk = find_task_by_vpid(iter->tgid);
		if (tsk) {
			get_task_struct(tsk);

			length = scnprintf(temp + pos,
				FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
				"\nNEW PID: %5d, %4s, %4d\n",
				iter->pid, tsk->comm,
				iter->tgid);
			pos += length;

			length = scnprintf(temp + pos,
				FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
				" %4d, %4d, %4d\n",
				attr_item.llf_task_policy_by_pid, attr_item.loading_th_by_pid,
				attr_item.light_loading_policy_by_pid);
			pos += length;

			length = scnprintf(temp + pos,
				FPSGO_SYSFS_MAX_BUFF_SIZE - pos, " %4d, %4d, %4d\n",
				attr_item.rescue_second_enable_by_pid,
				attr_item.rescue_second_time_by_pid,
				attr_item.rescue_second_group_by_pid);
			pos += length;

			length = scnprintf(temp + pos,
				FPSGO_SYSFS_MAX_BUFF_SIZE - pos, " %4d, %4d, %4d\n",
				attr_item.filter_frame_enable_by_pid,
				attr_item.filter_frame_window_size_by_pid,
				attr_item.filter_frame_kmin_by_pid);
			pos += length;

			length = scnprintf(temp + pos,
				FPSGO_SYSFS_MAX_BUFF_SIZE - pos, " %4d, %4d, %4d\n",
				attr_item.boost_affinity_by_pid,
				attr_item.boost_lr_by_pid, attr_item.reset_taskmask);
			pos += length;

			length = scnprintf(temp + pos,
				FPSGO_SYSFS_MAX_BUFF_SIZE - pos, " %4d, %4d, %4d, %4d, %4d\n",
				attr_item.separate_aa_by_pid,
				attr_item.separate_release_sec_by_pid,
				attr_item.separate_pct_b_by_pid,
				attr_item.separate_pct_m_by_pid,
				attr_item.blc_boost_by_pid);
			pos += length;

			length = scnprintf(temp + pos,
				FPSGO_SYSFS_MAX_BUFF_SIZE - pos, " %4d, %4d, %4d, %4d\n",
				attr_item.limit_uclamp_by_pid,
				attr_item.limit_ruclamp_by_pid,
				attr_item.limit_uclamp_m_by_pid,
				attr_item.limit_ruclamp_m_by_pid);
			pos += length;

			length = scnprintf(temp + pos,
				FPSGO_SYSFS_MAX_BUFF_SIZE - pos, " %4d, %4d, %4d, %4d\n",
				attr_item.qr_enable_by_pid,
				attr_item.qr_t2wnt_x_by_pid,
				attr_item.qr_t2wnt_y_p_by_pid,
				attr_item.qr_t2wnt_y_n_by_pid);
			pos += length;

			length = scnprintf(temp + pos,
				FPSGO_SYSFS_MAX_BUFF_SIZE - pos, " %4d, %4d, %4d, %4d, %4d\n",
				attr_item.gcc_enable_by_pid,
				attr_item.gcc_fps_margin_by_pid,
				attr_item.gcc_up_sec_pct_by_pid,
				attr_item.gcc_up_step_by_pid,
				attr_item.gcc_reserved_up_quota_pct_by_pid);
			pos += length;

			length = scnprintf(temp + pos,
				FPSGO_SYSFS_MAX_BUFF_SIZE - pos, " %4d, %4d, %4d\n",
				attr_item.gcc_down_sec_pct_by_pid,
				attr_item.gcc_down_step_by_pid,
				attr_item.gcc_reserved_down_quota_pct_by_pid);
			pos += length;

			length = scnprintf(temp + pos,
				FPSGO_SYSFS_MAX_BUFF_SIZE - pos, " %4d, %4d, %4d, %4d\n",
				attr_item.gcc_enq_bound_thrs_by_pid,
				attr_item.gcc_enq_bound_quota_by_pid,
				attr_item.gcc_deq_bound_thrs_by_pid,
				attr_item.gcc_deq_bound_quota_by_pid);
			pos += length;

			length = scnprintf(temp + pos,
				FPSGO_SYSFS_MAX_BUFF_SIZE - pos, " %4d, %4d, %4d, %4d, %4d\n",
				attr_item.reset_taskmask,
				attr_item.limit_cfreq2cap,
				attr_item.limit_rfreq2cap,
				attr_item.limit_cfreq2cap_m,
				attr_item.limit_rfreq2cap_m);
			pos += length;

			put_task_struct(tsk);
		}
	}

	rcu_read_unlock();
	fpsgo_render_tree_unlock(__func__);

	length = scnprintf(buf, PAGE_SIZE, "%s", temp);

out:
	kfree(temp);
	return length;
}
static KOBJ_ATTR_RO(render_info_params);

static ssize_t render_attr_params_show(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	struct rb_node *n;
	struct fpsgo_attr_by_pid *iter;
	struct fpsgo_boost_attr attr_item;
	char *temp = NULL;
	int pos = 0;
	int length = 0;

	temp = kcalloc(FPSGO_SYSFS_MAX_BUFF_SIZE, sizeof(char), GFP_KERNEL);
	if (!temp)
		goto out;

	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
		"\n NEW tgid: TGID, llf_task_policy, loading_th, light_loading_policy,\n");
	pos += length;

	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
		" rescue_second_enable, rescue_second_time, rescue_second_group,\n");
	pos += length;

	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
		" ff_enable, ff_window_size, ff_k_min,\n");
	pos += length;

	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
		" boost_affinity, boost_LR, reset_taskmask\n");
	pos += length;

	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
		" separate_aa, separate_release_sec_by_pid, pct_b, pct_m, blc_boost\n");
	pos += length;

	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
		" uclamp, ruclamp, uclamp_m, ruclamp_m\n");
	pos += length;

	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
				" qr_enable, qr_t2wnt_x, qr_t2wnt_y_p, qr_t2wnt_y_n\n");
	pos += length;
	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
				" gcc_enable, gcc_fps_margin, gcc_up_sec_pct, gcc_up_step gcc_reserved_up_quota_pct\n");
	pos += length;
	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
				" gcc_down_sec_pct, gcc_down_step, gcc_reserved_down_quota_pct\n");
	pos += length;
	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
				" gcc_enq_bound_thrs, gcc_enq_bound_quota, gcc_deq_bound_thrs, gcc_deq_bound_quota\n");
	pos += length;
	length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
			" reset_taskmask, limit_cfreq2cap, limit_rfreq2cap, limit_cfreq2cap_m, limit_rfreq2cap_m\n");
	pos += length;

	fpsgo_render_tree_lock(__func__);

	for (n = rb_first(&fpsgo_attr_by_pid_tree); n != NULL; n = rb_next(n)) {
		iter = rb_entry(n,  struct fpsgo_attr_by_pid, entry);
		attr_item = iter->attr;

		length = scnprintf(temp + pos,
				FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
				"\n NEW tgid: %4d, %4d, %4d, %4d,\n",
			iter->tgid, attr_item.llf_task_policy_by_pid,
			attr_item.loading_th_by_pid, attr_item.light_loading_policy_by_pid);
		pos += length;

		length = scnprintf(temp + pos,
			FPSGO_SYSFS_MAX_BUFF_SIZE - pos, " %4d, %4d, %4d,\n",
			attr_item.rescue_second_enable_by_pid,
			attr_item.rescue_second_time_by_pid,
			attr_item.rescue_second_group_by_pid);
		pos += length;

		length = scnprintf(temp + pos,
			FPSGO_SYSFS_MAX_BUFF_SIZE - pos, " %4d, %4d, %4d,\n",
			attr_item.filter_frame_enable_by_pid,
			attr_item.filter_frame_window_size_by_pid,
			attr_item.filter_frame_kmin_by_pid);
		pos += length;

		length = scnprintf(temp + pos,
			FPSGO_SYSFS_MAX_BUFF_SIZE - pos, " %4d, %4d, %4d\n",
			attr_item.boost_affinity_by_pid,
			attr_item.boost_lr_by_pid, attr_item.reset_taskmask);
		pos += length;

		length = scnprintf(temp + pos,
			FPSGO_SYSFS_MAX_BUFF_SIZE - pos, " %4d, %4d, %4d, %4d, %4d\n",
			attr_item.separate_aa_by_pid,
			attr_item.separate_release_sec_by_pid,
			attr_item.separate_pct_b_by_pid,
			attr_item.separate_pct_m_by_pid,
			attr_item.blc_boost_by_pid);
		pos += length;

		length = scnprintf(temp + pos,
			FPSGO_SYSFS_MAX_BUFF_SIZE - pos, " %4d, %4d, %4d, %4d\n",
			attr_item.limit_uclamp_by_pid,
			attr_item.limit_ruclamp_by_pid,
			attr_item.limit_uclamp_m_by_pid,
			attr_item.limit_ruclamp_m_by_pid);
		pos += length;

		length = scnprintf(temp + pos,
			FPSGO_SYSFS_MAX_BUFF_SIZE - pos, " %4d, %4d, %4d, %4d\n",
			attr_item.qr_enable_by_pid,
			attr_item.qr_t2wnt_x_by_pid,
			attr_item.qr_t2wnt_y_p_by_pid,
			attr_item.qr_t2wnt_y_n_by_pid);
		pos += length;

		length = scnprintf(temp + pos,
			FPSGO_SYSFS_MAX_BUFF_SIZE - pos, " %4d, %4d, %4d, %4d, %4d\n",
			attr_item.gcc_enable_by_pid,
			attr_item.gcc_fps_margin_by_pid,
			attr_item.gcc_up_sec_pct_by_pid,
			attr_item.gcc_up_step_by_pid,
			attr_item.gcc_reserved_up_quota_pct_by_pid);
		pos += length;

		length = scnprintf(temp + pos,
			FPSGO_SYSFS_MAX_BUFF_SIZE - pos, " %4d, %4d, %4d\n",
			attr_item.gcc_down_sec_pct_by_pid,
			attr_item.gcc_down_step_by_pid,
			attr_item.gcc_reserved_down_quota_pct_by_pid);
		pos += length;

		length = scnprintf(temp + pos,
			FPSGO_SYSFS_MAX_BUFF_SIZE - pos, " %4d, %4d, %4d, %4d\n",
			attr_item.gcc_enq_bound_thrs_by_pid,
			attr_item.gcc_enq_bound_quota_by_pid,
			attr_item.gcc_deq_bound_thrs_by_pid,
			attr_item.gcc_deq_bound_quota_by_pid);
		pos += length;

		length = scnprintf(temp + pos,
			FPSGO_SYSFS_MAX_BUFF_SIZE - pos, " %4d, %4d, %4d, %4d, %4d\n",
			attr_item.reset_taskmask,
			attr_item.limit_cfreq2cap,
			attr_item.limit_rfreq2cap,
			attr_item.limit_cfreq2cap_m,
			attr_item.limit_rfreq2cap_m);
		pos += length;
	}

	fpsgo_render_tree_unlock(__func__);

	length = scnprintf(buf, PAGE_SIZE, "%s", temp);

out:
	kfree(temp);
	return length;
}
static KOBJ_ATTR_RO(render_attr_params);

#endif  // FPSGO_MW

static ssize_t force_onoff_show(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", fpsgo_is_force_enable());
}

static ssize_t force_onoff_store(struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int val = -1;
	char *acBuffer = NULL;
	int arg;

	acBuffer = kcalloc(FPSGO_SYSFS_MAX_BUFF_SIZE, sizeof(char), GFP_KERNEL);
	if (!acBuffer)
		goto out;

	if ((count > 0) && (count < FPSGO_SYSFS_MAX_BUFF_SIZE)) {
		if (scnprintf(acBuffer, FPSGO_SYSFS_MAX_BUFF_SIZE, "%s", buf)) {
			if (kstrtoint(acBuffer, 0, &arg) == 0)
				val = arg;
			else
				goto out;
		}
	}

	if (val > 2 || val < 0)
		goto out;

	fpsgo_force_switch_enable(val);

out:
	kfree(acBuffer);
	return count;
}

static KOBJ_ATTR_RW(force_onoff);


static ssize_t BQid_show(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	struct rb_node *n;
	struct BQ_id *pos;
	char *temp = NULL;
	int posi = 0;
	int length = 0;

	temp = kcalloc(FPSGO_SYSFS_MAX_BUFF_SIZE, sizeof(char), GFP_KERNEL);
	if (!temp)
		goto out;

	fpsgo_render_tree_lock(__func__);

	for (n = rb_first(&BQ_id_list); n; n = rb_next(n)) {
		pos = rb_entry(n, struct BQ_id, entry);
		length = scnprintf(temp + posi,
				FPSGO_SYSFS_MAX_BUFF_SIZE - posi,
				"pid %d, tgid %d, key1 %d, key2 %llu, buffer_id %llu, queue_SF %d\n",
				pos->pid, fpsgo_get_tgid(pos->pid),
				pos->key.key1, pos->key.key2, pos->buffer_id, pos->queue_SF);
		posi += length;

	}

	fpsgo_render_tree_unlock(__func__);

	length = scnprintf(buf, PAGE_SIZE, "%s", temp);

out:
	kfree(temp);
	return length;
}

static KOBJ_ATTR_RO(BQid);

static ssize_t perfserv_ta_show(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n",
		fpsgo_perfserv_ta_value());
}

static ssize_t perfserv_ta_store(struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int val = -1;
	char *acBuffer = NULL;
	int arg;

	acBuffer = kcalloc(FPSGO_SYSFS_MAX_BUFF_SIZE, sizeof(char), GFP_KERNEL);
	if (!acBuffer)
		goto out;

	if ((count > 0) && (count < FPSGO_SYSFS_MAX_BUFF_SIZE)) {
		if (scnprintf(acBuffer, FPSGO_SYSFS_MAX_BUFF_SIZE, "%s", buf)) {
			if (kstrtoint(acBuffer, 0, &arg) == 0)
				val = arg;
			else
				goto out;
		}
	}

	if (val > 101 || val < -1)
		goto out;

	fpsgo_set_perfserv_ta(val);

out:
	kfree(acBuffer);
	return count;
}

static KOBJ_ATTR_RW(perfserv_ta);

static ssize_t render_loading_show(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	struct rb_node *n;
	struct render_info *iter;
	char *temp = NULL;
	int pos = 0, i;
	int length = 0;

	temp = kcalloc(FPSGO_SYSFS_MAX_BUFF_SIZE, sizeof(char), GFP_KERNEL);
	if (!temp)
		goto out;

	fpsgo_render_tree_lock(__func__);

	for (n = rb_first(&render_pid_tree); n != NULL; n = rb_next(n)) {
		iter = rb_entry(n, struct render_info, render_key_node);

		length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
				"PID  TGID  BufferID\n");
		pos += length;

		length = scnprintf(temp + pos,
				FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
				"%d %4d 0x%llx\n", iter->pid, iter->tgid, iter->buffer_id);
		pos += length;

		length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
				"AVG FREQ\n");
		pos += length;

		length = scnprintf(temp + pos,
				FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
				"%d\n", iter->avg_freq);
		pos += length;

		length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
				"DEP LOADING\n");
		pos += length;

		for (i = 0; i < iter->dep_valid_size; i++) {
			length = scnprintf(temp + pos,
				FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
				"%d(%d) ", iter->dep_arr[i].pid, iter->dep_arr[i].loading);
			pos += length;
		}

		length = scnprintf(temp + pos, FPSGO_SYSFS_MAX_BUFF_SIZE - pos,
				"\n");
		pos += length;
	}

	fpsgo_render_tree_unlock(__func__);

	length = scnprintf(buf, PAGE_SIZE, "%s", temp);

out:
	kfree(temp);
	return length;
}

static KOBJ_ATTR_RO(render_loading);

int init_fpsgo_common(void)
{
	render_pid_tree = RB_ROOT;
	BQ_id_list = RB_ROOT;
	linger_tree = RB_ROOT;
	hwui_info_tree = RB_ROOT;
	sbe_info_tree = RB_ROOT;
	fps_control_pid_info_tree = RB_ROOT;
	fpsgo_attr_by_pid_tree = RB_ROOT;

	if (!fpsgo_sysfs_create_dir(NULL, "common", &base_kobj)) {
		fpsgo_sysfs_create_file(base_kobj, &kobj_attr_systrace_mask);
		fpsgo_sysfs_create_file(base_kobj, &kobj_attr_fpsgo_enable);
		fpsgo_sysfs_create_file(base_kobj, &kobj_attr_force_onoff);
		fpsgo_sysfs_create_file(base_kobj, &kobj_attr_render_info);
		fpsgo_sysfs_create_file(base_kobj, &kobj_attr_BQid);
		fpsgo_sysfs_create_file(base_kobj, &kobj_attr_perfserv_ta);
		fpsgo_sysfs_create_file(base_kobj, &kobj_attr_render_loading);
		#if FPSGO_MW
		fpsgo_sysfs_create_file(base_kobj, &kobj_attr_render_info_params);
		fpsgo_sysfs_create_file(base_kobj, &kobj_attr_render_attr_params);
		#endif
	}

	fpsgo_update_tracemark();
	fpsgo_systrace_mask = FPSGO_DEBUG_MANDATORY;

	return 0;
}

