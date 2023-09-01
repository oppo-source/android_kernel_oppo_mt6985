// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
#include <linux/module.h>
#include <sched/sched.h>
#include <trace/hooks/sched.h>
#include <sugov/cpufreq.h>
#include "common.h"
#if IS_ENABLED(CONFIG_MTK_GEARLESS_SUPPORT)
#include "mtk_energy_model/v2/energy_model.h"
#else
#include "mtk_energy_model/v1/energy_model.h"
#endif
#include "eas_plus.h"
#include "eas_trace.h"
#include <linux/sort.h>
#if IS_ENABLED(CONFIG_MTK_THERMAL_INTERFACE)
#include <thermal_interface.h>
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GKI_CPUFREQ_BOUNCING)
#include <linux/cpufreq_bouncing.h>
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
#include <../kernel/oplus_cpu/sched/frame_boost/frame_group.h>
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_ASSIST)
#include <../kernel/oplus_cpu/sched/sched_assist/sa_common.h>
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_ABNORMAL_FLAG)
#include <../kernel/oplus_cpu/oplus_overload/task_overload.h>
#endif

MODULE_LICENSE("GPL");

#define CORE_PAUSE_OUT		0
#define IB_ASYM_MISFIT		(0x02)
#define IB_SAME_CLUSTER		(0x01)
#define IB_OVERUTILIZATION	(0x04)

DEFINE_PER_CPU(struct update_util_data __rcu *, cpufreq_update_util_data);
DEFINE_PER_CPU(__u32, active_softirqs);

struct cpumask __cpu_pause_mask;
EXPORT_SYMBOL(__cpu_pause_mask);

#ifdef CONFIG_RT_SOFTINT_OPTIMIZATION
/*
 * Return whether the task on the given cpu is currently non-preemptible
 * while handling a potentially long softint, or if the task is likely
 * to block preemptions soon because it is a ksoftirq thread that is
 * handling slow softints.
 */
bool task_may_not_preempt(struct task_struct *task, int cpu)
{
	__u32 softirqs = per_cpu(active_softirqs, cpu) |
			local_softirq_pending();

	struct task_struct *cpu_ksoftirqd = per_cpu(ksoftirqd, cpu);

	return ((softirqs & LONG_SOFTIRQ_MASK) &&
		(task == cpu_ksoftirqd ||
		 task_thread_info(task)->preempt_count & SOFTIRQ_MASK));
}
#else
bool task_may_not_preempt(struct task_struct *task, int cpu)
{
	return false;
}
#endif /* CONFIG_RT_SOFTINT_OPTIMIZATION */

static struct perf_domain *find_pd(struct perf_domain *pd, int cpu)
{
	while (pd) {
		if (cpumask_test_cpu(cpu, perf_domain_span(pd)))
			return pd;

		pd = pd->next;
	}

	return NULL;
}

static inline bool check_faster_idle_balance(struct sched_group *busiest, struct rq *dst_rq)
{

	int src_cpu = group_first_cpu(busiest);
	int dst_cpu = cpu_of(dst_rq);
	int cpu;

	if (capacity_orig_of(dst_cpu) <= capacity_orig_of(src_cpu))
		return false;

	for_each_cpu(cpu, sched_group_span(busiest)) {
		if (cpu_rq(cpu)->misfit_task_load)
			return true;
	}

	return false;
}

static inline bool check_has_overutilize_cpu(struct cpumask *grp)
{

	int cpu;

	for_each_cpu(cpu, grp) {
		if (cpu_rq(cpu)->nr_running >= 2 &&
			!fits_capacity(cpu_util(cpu), capacity_of(cpu)))
			return true;
	}
	return false;
}

void mtk_find_busiest_group(void *data, struct sched_group *busiest,
		struct rq *dst_rq, int *out_balance)
{
	int src_cpu = -1;
	int dst_cpu = dst_rq->cpu;

	if (cpu_paused(dst_cpu)) {
		*out_balance = 1;
		trace_sched_find_busiest_group(src_cpu, dst_cpu, *out_balance, CORE_PAUSE_OUT);
		return;
	}

	if (busiest) {
		struct perf_domain *pd = NULL;
		int dst_cpu = dst_rq->cpu;
		int fbg_reason = 0;

		pd = rcu_dereference(dst_rq->rd->pd);
		pd = find_pd(pd, dst_cpu);
		if (!pd)
			return;

		src_cpu = group_first_cpu(busiest);

		/*
		 *  1.same cluster
		 *  2.not same cluster but dst_cpu has a higher capacity and
		 *    busiest group has misfit task. The purpose of this condition
		 *    is trying to let misfit task goto hiehger cpu.
		 */
		if (cpumask_test_cpu(src_cpu, perf_domain_span(pd))) {
			*out_balance = 0;
			fbg_reason |= IB_SAME_CLUSTER;
		} else if (check_faster_idle_balance(busiest, dst_rq)) {
			*out_balance = 0;
			fbg_reason |= IB_ASYM_MISFIT;
		} else if (check_has_overutilize_cpu(sched_group_span(busiest))) {
			*out_balance = 0;
			fbg_reason |= IB_OVERUTILIZATION;
		}

		trace_sched_find_busiest_group(src_cpu, dst_cpu, *out_balance, fbg_reason);
	}
}

void mtk_cpu_overutilized(void *data, int cpu, int *overutilized)
{
	struct perf_domain *pd = NULL;
	struct rq *rq = cpu_rq(cpu);
	unsigned long sum_util = 0, sum_cap = 0;
	int i = 0;

	rcu_read_lock();
	pd = rcu_dereference(rq->rd->pd);
	pd = find_pd(pd, cpu);
	if (!pd) {
		rcu_read_unlock();
		return;
	}

	if (cpumask_weight(perf_domain_span(pd)) == 1 &&
		capacity_orig_of(cpu) == SCHED_CAPACITY_SCALE) {
		*overutilized = 0;
		rcu_read_unlock();
		return;
	}

	for_each_cpu(i, perf_domain_span(pd)) {
		sum_util += cpu_util(i);
		sum_cap += capacity_of(i);
	}


	*overutilized = !fits_capacity(sum_util, sum_cap);
	trace_sched_cpu_overutilized(cpu, perf_domain_span(pd), sum_util, sum_cap, *overutilized);

	rcu_read_unlock();
}

#if IS_ENABLED(CONFIG_MTK_THERMAL_AWARE_SCHEDULING)
int __read_mostly thermal_headroom[NR_CPUS]  ____cacheline_aligned;
unsigned long next_update_thermal;
static DEFINE_SPINLOCK(thermal_headroom_lock);
static void update_thermal_headroom(int this_cpu)
{
	int cpu;

	if (spin_trylock(&thermal_headroom_lock)) {
		if (time_before(jiffies, next_update_thermal)) {
			spin_unlock(&thermal_headroom_lock);
			return;
		}

		next_update_thermal = jiffies + thermal_headroom_interval_tick;
		for_each_cpu(cpu, cpu_possible_mask) {
			thermal_headroom[cpu] = get_thermal_headroom(cpu);
		}
		spin_unlock(&thermal_headroom_lock);
	}

	trace_sched_next_update_thermal_headroom(jiffies, next_update_thermal);
}

int sort_thermal_headroom(struct cpumask *cpus, int *cpu_order)
{
	int i, j, cpu, cnt = 0;
	int headroom_order[NR_CPUS] ____cacheline_aligned;

	if (cpumask_weight(cpus) == 1) {
		cpu = cpumask_first(cpus);
		*cpu_order = cpu;

		return 1;
	}

	spin_lock(&thermal_headroom_lock);
	for_each_cpu_and(cpu, cpus, cpu_active_mask) {
		int headroom;

		headroom = thermal_headroom[cpu];

		for (i = 0; i < cnt; i++) {
			if (headroom > headroom_order[i])
				break;
		}

		for (j = cnt; j >= i; j--) {
			headroom_order[j+1] = headroom_order[j];
			cpu_order[j+1] = cpu_order[j];
		}

		headroom_order[i] = headroom;
		cpu_order[i] = cpu;
		cnt++;
	}
	spin_unlock(&thermal_headroom_lock);

	return cnt;
}

#endif

/**
 * em_cpu_energy() - Estimates the energy consumed by the CPUs of a
		performance domain
 * @pd		: performance domain for which energy has to be estimated
 * @max_util	: highest utilization among CPUs of the domain
 * @sum_util	: sum of the utilization of all CPUs in the domain
 *
 * This function must be used only for CPU devices. There is no validation,
 * i.e. if the EM is a CPU type and has cpumask allocated. It is called from
 * the scheduler code quite frequently and that is why there is not checks.
 *
 * Return: the sum of the energy consumed by the CPUs of the domain assuming
 * a capacity state satisfying the max utilization of the domain.
 */
unsigned long mtk_em_cpu_energy(struct em_perf_domain *pd,
		unsigned long max_util, unsigned long sum_util,
		unsigned long allowed_cpu_cap, unsigned int *cpu_temp)
{
	unsigned long scale_cpu, freq = 0;
	struct em_perf_state *ps;
	int cpu, opp = -1;
#if IS_ENABLED(CONFIG_MTK_OPP_CAP_INFO)
	unsigned long pwr_eff, cap, freq_legacy, sum_cap = 0;
	struct mtk_em_perf_state *mtk_ps;
#else
	int i;
#endif
	unsigned long dyn_pwr = 0, static_pwr = 0;
	unsigned long energy;
	struct cpufreq_policy curr_policy;

	if (!sum_util)
		return 0;

	/*
	 * In order to predict the performance state, map the utilization of
	 * the most utilized CPU of the performance domain to a requested
	 * frequency, like schedutil.
	 */
	cpu = cpumask_first(to_cpumask(pd->cpus));
	scale_cpu = arch_scale_cpu_capacity(cpu);
	ps = &pd->table[pd->nr_perf_states - 1];
	if (!cpufreq_get_policy(&curr_policy, cpu))
		trace_android_vh_map_util_freq_new(max_util, ps->frequency, scale_cpu, &freq, &curr_policy, NULL);
	if (!freq) {
		max_util = map_util_perf(max_util);
		max_util = min(max_util, allowed_cpu_cap);
		freq = map_util_freq(max_util, ps->frequency, scale_cpu);
	}
	freq = max(freq, per_cpu(min_freq, cpu));

#if IS_ENABLED(CONFIG_MTK_OPP_CAP_INFO)
	mtk_ps = pd_get_freq_ps(cpu, freq, &opp);
	pwr_eff = mtk_ps->pwr_eff;
	cap = mtk_ps->capacity;
	freq_legacy = pd_get_opp_freq_legacy(cpu, pd_get_freq_opp_legacy(cpu, freq));
#else
	/*
	 * Find the lowest performance state of the Energy Model above the
	 * requested frequency.
	 */
	for (i = 0; i < pd->nr_perf_states; i++) {
		ps = &pd->table[i];
		if (ps->frequency >= freq)
			break;
	}

	i = min(i, pd->nr_perf_states - 1);
	opp = pd->nr_perf_states - i - 1;
#endif

#if IS_ENABLED(CONFIG_MTK_LEAKAGE_AWARE_TEMP)
	for_each_cpu_and(cpu, to_cpumask(pd->cpus), cpu_online_mask) {
		unsigned int cpu_static_pwr;

		cpu_static_pwr = pd_get_opp_leakage(cpu, opp, cpu_temp[cpu]);
		static_pwr += cpu_static_pwr;
		sum_cap += cap;

		if (trace_sched_leakage_enabled())
			trace_sched_leakage(cpu, opp, cpu_temp[cpu], cpu_static_pwr,
					static_pwr, sum_cap);
	}
	static_pwr = (likely(sum_cap) ? (static_pwr * sum_util) / sum_cap : 0);
#endif

	/*
	 * The capacity of a CPU in the domain at the performance state (ps)
	 * can be computed as:
	 *
	 *             ps->freq * scale_cpu
	 *   ps->cap = --------------------                          (1)
	 *                 cpu_max_freq
	 *
	 * So, ignoring the costs of idle states (which are not available in
	 * the EM), the energy consumed by this CPU at that performance state
	 * is estimated as:
	 *
	 *             ps->power * cpu_util
	 *   cpu_nrg = --------------------                          (2)
	 *                   ps->cap
	 *
	 * since 'cpu_util / ps->cap' represents its percentage of busy time.
	 *
	 *   NOTE: Although the result of this computation actually is in
	 *         units of power, it can be manipulated as an energy value
	 *         over a scheduling period, since it is assumed to be
	 *         constant during that interval.
	 *
	 * By injecting (1) in (2), 'cpu_nrg' can be re-expressed as a product
	 * of two terms:
	 *
	 *             ps->power * cpu_max_freq   cpu_util
	 *   cpu_nrg = ------------------------ * ---------          (3)
	 *                    ps->freq            scale_cpu
	 *
	 * The first term is static, and is stored in the em_perf_state struct
	 * as 'ps->cost'.
	 *
	 * Since all CPUs of the domain have the same micro-architecture, they
	 * share the same 'ps->cost', and the same CPU capacity. Hence, the
	 * total energy of the domain (which is the simple sum of the energy of
	 * all of its CPUs) can be factorized as:
	 *
	 *            ps->cost * \Sum cpu_util
	 *   pd_nrg = ------------------------                       (4)
	 *                  scale_cpu
	 */

#if IS_ENABLED(CONFIG_MTK_OPP_CAP_INFO)
	dyn_pwr = pwr_eff * sum_util;

	/* for pd_opp_capacity is scaled based on maximum scale 1024, so cost = pwr_eff * 1024 */
	if (trace_sched_em_cpu_energy_enabled())
		trace_sched_em_cpu_energy(opp, freq_legacy, "pwr_eff", pwr_eff,
			scale_cpu, dyn_pwr, static_pwr);
#else
	dyn_pwr = (ps->cost * sum_util / scale_cpu);
	if (trace_sched_em_cpu_energy_enabled())
		trace_sched_em_cpu_energy(opp, freq, "ps->cost", ps->cost,
			scale_cpu, dyn_pwr, static_pwr);
#endif

	energy = dyn_pwr + static_pwr;

	return energy;
}

#define CSRAM_BASE 0x0011BC00
#define OFFS_THERMAL_LIMIT_S 0x1208
#define THERMAL_INFO_SIZE 200

static void __iomem *sram_base_addr;
int init_sram_info(void)
{
	sram_base_addr =
		ioremap(CSRAM_BASE + OFFS_THERMAL_LIMIT_S, THERMAL_INFO_SIZE);

	if (!sram_base_addr) {
		pr_info("Remap thermal info failed\n");

		return -EIO;
	}

	return 0;
}

void mtk_tick_entry(void *data, struct rq *rq)
{
	void __iomem *base = sram_base_addr;
	struct em_perf_domain *pd;
	int this_cpu, gear_id, opp_idx, offset;
	unsigned int freq_thermal;
	unsigned long max_capacity, capacity;
	u32 opp_ceiling;
	u64 idle_time, wall_time, cpu_utilize;
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	u64 ts[4] = {0};

	ts[0] = sched_clock();
#endif

	if (rq->curr->android_vendor_data1[T_SBB_FLG] || is_busy_tick_boost_all() ||
		rq->curr->sched_task_group->android_vendor_data1[TG_SBB_FLG]) {

		if (rq->android_vendor_data1[RQ_SBB_TICK_START]) {
			idle_time = get_cpu_idle_time(rq->cpu, &wall_time, 1);

			cpu_utilize = 100 - (100 * (idle_time -
				rq->android_vendor_data1[RQ_SBB_IDLE_TIME])) /
				(wall_time - rq->android_vendor_data1[RQ_SBB_WALL_TIME]);

			rq->android_vendor_data1[RQ_SBB_IDLE_TIME] = idle_time;
			rq->android_vendor_data1[RQ_SBB_WALL_TIME] = wall_time;

			if (cpu_utilize >= get_sbb_active_ratio()) {
				rq->android_vendor_data1[RQ_SBB_ACTIVE] = 1;

				rq->android_vendor_data1[RQ_SBB_BOOST_FACTOR] =
				min_t(u32, rq->android_vendor_data1[RQ_SBB_BOOST_FACTOR] * 2, 4);

				rq->android_vendor_data1[RQ_SBB_CPU_UTILIZE] = cpu_utilize;
			} else {
				rq->android_vendor_data1[RQ_SBB_ACTIVE] = 0;
				rq->android_vendor_data1[RQ_SBB_BOOST_FACTOR] = 1;
			}
		} else {
			rq->android_vendor_data1[RQ_SBB_ACTIVE] = 0;
			rq->android_vendor_data1[RQ_SBB_TICK_START] = 1;
			rq->android_vendor_data1[RQ_SBB_BOOST_FACTOR] = 1;
		}
	} else {
		rq->android_vendor_data1[RQ_SBB_ACTIVE] = 0;
		rq->android_vendor_data1[RQ_SBB_TICK_START] = 0;
		rq->android_vendor_data1[RQ_SBB_BOOST_FACTOR] = 1;
	}

	this_cpu = cpu_of(rq);
#if IS_ENABLED(CONFIG_MTK_THERMAL_AWARE_SCHEDULING)
	update_thermal_headroom(this_cpu);
#endif
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	ts[1] = sched_clock();
#endif
	pd = em_cpu_get(this_cpu);
	if (!pd)
		return;

	if (this_cpu != cpumask_first(to_cpumask(pd->cpus)))
		return;

	gear_id = topology_physical_package_id(this_cpu);
	offset = gear_id << 2;
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	ts[2] = sched_clock();
#endif
	opp_ceiling = ioread32(base + offset);
	if ((opp_ceiling < 0) || (opp_ceiling > pd->nr_perf_states - 1)) {
		pr_info("ERROR: invalid value from thermal, cpu = %d, opp_ceiling = %d, nr_perf_states = %d\n",
			this_cpu, opp_ceiling, pd->nr_perf_states);
		WARN_ON(1);
	}
	opp_idx = pd->nr_perf_states - opp_ceiling - 1;
	freq_thermal = pd->table[opp_idx].frequency;

	max_capacity = arch_scale_cpu_capacity(this_cpu);
	capacity = freq_thermal * max_capacity;
	capacity /= pd->table[pd->nr_perf_states-1].frequency;
	arch_set_thermal_pressure(to_cpumask(pd->cpus), max_capacity - capacity);

	trace_sched_frequency_limits(this_cpu, freq_thermal);
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	ts[3] = sched_clock();

	if ((ts[3] - ts[0] > 1000000ULL) && in_hardirq()) {
		int i;

		printk_deferred("%s duration %llu, ts[0]=%llu\n", __func__, ts[3] - ts[0], ts[0]);
		for (i = 0; i < 3; i++) {
			printk_deferred("%s ts[%d]=%llu, duration=%llu\n",
					__func__, i+1, ts[i + 1], ts[i + 1] - ts[i]);
		}
	}
#endif
}

/*
 * Enable/Disable honoring sync flag in energy-aware wakeups
 */
unsigned int sched_sync_hint_enable = 1;
void set_wake_sync(unsigned int sync)
{
	sched_sync_hint_enable = sync;
}
EXPORT_SYMBOL_GPL(set_wake_sync);

unsigned int get_wake_sync(void)
{
	return sched_sync_hint_enable;
}
EXPORT_SYMBOL_GPL(get_wake_sync);

void mtk_set_wake_flags(void *data, int *wake_flags, unsigned int *mode)
{
	if (!sched_sync_hint_enable)
		*wake_flags &= ~WF_SYNC;
}

unsigned int new_idle_balance_interval_ns  =  1000000;
unsigned int thermal_headroom_interval_tick =  1;

void set_newly_idle_balance_interval_us(unsigned int interval_us)
{
	new_idle_balance_interval_ns = interval_us * 1000;

	trace_sched_newly_idle_balance_interval(interval_us);
}
EXPORT_SYMBOL_GPL(set_newly_idle_balance_interval_us);

unsigned int get_newly_idle_balance_interval_us(void)
{
	return new_idle_balance_interval_ns / 1000;
}
EXPORT_SYMBOL_GPL(get_newly_idle_balance_interval_us);

void set_get_thermal_headroom_interval_tick(unsigned int tick)
{
	thermal_headroom_interval_tick = tick;

	trace_sched_headroom_interval_tick(tick);
}
EXPORT_SYMBOL_GPL(set_get_thermal_headroom_interval_tick);

unsigned int get_thermal_headroom_interval_tick(void)
{
	return thermal_headroom_interval_tick;
}
EXPORT_SYMBOL_GPL(get_thermal_headroom_interval_tick);

static DEFINE_RAW_SPINLOCK(migration_lock);

int select_idle_cpu_from_domains(struct task_struct *p,
					struct perf_domain **prefer_pds, int len)
{
	int i = 0;
	struct perf_domain *pd;
	int cpu, best_cpu = -1;

	for (; i < len; i++) {
		pd = prefer_pds[i];
		for_each_cpu_and(cpu, perf_domain_span(pd),
						cpu_active_mask) {
			if (!cpumask_test_cpu(cpu, p->cpus_ptr))
				continue;
			if (idle_cpu(cpu)) {
				best_cpu = cpu;
				break;
			}
		}
		if (best_cpu != -1)
			break;
	}

	return best_cpu;
}

int select_bigger_idle_cpu(struct task_struct *p)
{
	struct root_domain *rd = cpu_rq(smp_processor_id())->rd;
	struct perf_domain *pd, *prefer_pds[NR_CPUS];
	int cpu = task_cpu(p), bigger_idle_cpu = -1;
	int i = 0;
	long max_capacity = capacity_orig_of(cpu);
	long capacity;

	rcu_read_lock();
	pd = rcu_dereference(rd->pd);

	for (; pd; pd = pd->next) {
		capacity = capacity_orig_of(cpumask_first(perf_domain_span(pd)));
		if (capacity > max_capacity &&
			cpumask_intersects(p->cpus_ptr, perf_domain_span(pd))) {
			prefer_pds[i++] = pd;
		}
	}

	if (i != 0)
		bigger_idle_cpu = select_idle_cpu_from_domains(p, prefer_pds, i);

	rcu_read_unlock();
	return bigger_idle_cpu;
}

void check_for_migration(struct task_struct *p)
{
	int new_cpu = -1, better_idle_cpu = -1;
	int cpu = task_cpu(p);
	struct rq *rq = cpu_rq(cpu);
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	bool need_up_migrate = false;
#endif

#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	u64 ts[16] = {0};

	ts[0] = sched_clock();
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	if (fbg_need_up_migration(p, rq))
		need_up_migrate = true;
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	if (rq->misfit_task_load || need_up_migrate) {
#else
	if (rq->misfit_task_load) {
#endif
		struct em_perf_domain *pd;
		struct cpufreq_policy *policy;
		int opp_curr = 0, thre = 0, thre_idx = 0;

		if (rq->curr->__state != TASK_RUNNING ||
			rq->curr->nr_cpus_allowed == 1)
			return;

		pd = em_cpu_get(cpu);
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
		ts[1] = sched_clock();
#endif
		if (!pd) {
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
			if ((ts[1] - ts[0] > 1000000ULL) && in_hardirq()) {
				printk_deferred("%s duration %llu, ts[0]=%llu, ts[1]=%llu\n",
						__func__, ts[1] - ts[0], ts[0], ts[1]);
			}
#endif
			return;
		}

		thre_idx = (pd->nr_perf_states >> 3) - 1;
		if (thre_idx >= 0)
			thre = pd->table[thre_idx].frequency;
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
		ts[2] = sched_clock();
#endif
		policy = cpufreq_cpu_get(cpu);
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
		ts[3] = sched_clock();
#endif
		if (policy) {
			opp_curr = policy->cur;
			cpufreq_cpu_put(policy);
		}
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
		ts[4] = sched_clock();
#endif
		if (opp_curr <= thre) {
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
			if ((ts[4] - ts[0] > 1000000ULL) && in_hardirq()) {
				int i;

				printk_deferred("%s duration %llu, ts[0]=%llu\n",
						__func__, ts[4] - ts[0], ts[0]);
				for (i = 0; i < 4; i++) {
					printk_deferred("%s ts[%d]=%llu, duration=%llu\n",
						__func__, i+1, ts[i + 1], ts[i + 1] - ts[i]);
				}
			}
#endif
			return;
		}
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
		ts[5] = sched_clock();
#endif

		raw_spin_lock(&migration_lock);
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
		ts[6] = sched_clock();
#endif
		raw_spin_lock(&p->pi_lock);
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
		ts[7] = sched_clock();
#endif
		new_cpu = p->sched_class->select_task_rq(p, cpu, WF_TTWU);
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
		ts[8] = sched_clock();
#endif
		raw_spin_unlock(&p->pi_lock);
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
		ts[9] = sched_clock();
#endif
		if ((new_cpu < 0) ||
			(capacity_orig_of(new_cpu) <= capacity_orig_of(cpu)))
			better_idle_cpu = select_bigger_idle_cpu(p);
		if (better_idle_cpu >= 0)
			new_cpu = better_idle_cpu;
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
		ts[10] = sched_clock();
#endif
		if (new_cpu < 0) {
			raw_spin_unlock(&migration_lock);
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
			if ((ts[10] - ts[0] > 5000000ULL) && in_hardirq()) {
				int i;

				printk_deferred("%s duration %llu, ts[0]=%llu\n",
						__func__, ts[10] - ts[0], ts[0]);
				for (i = 0; i < 10; i++) {
					printk_deferred("%s ts[%d]=%llu, duration=%llu\n",
						__func__, i+1, ts[i + 1], ts[i + 1] - ts[i]);
				}
			}
#endif
			return;
		}
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
		ts[11] = sched_clock();
#endif
		if ((better_idle_cpu >= 0) ||
			(capacity_orig_of(new_cpu) > capacity_orig_of(cpu))) {
			raw_spin_unlock(&migration_lock);
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
			ts[12] = sched_clock();
#endif
			migrate_running_task(new_cpu, p, rq, MIGR_TICK_PULL_MISFIT_RUNNING);
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
			ts[13] = sched_clock();
#endif
		} else {
#if IS_ENABLED(CONFIG_MTK_SCHED_BIG_TASK_ROTATE)
			int thre_rot = 0, thre_rot_idx = 0;

			thre_rot_idx = (pd->nr_perf_states >> 1) - 1;
			if (thre_rot_idx >= 0)
				thre_rot = pd->table[thre_rot_idx].frequency;

			if (opp_curr > thre_rot) {
				task_check_for_rotation(rq);
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
				ts[14] = sched_clock();
#endif
			}

#endif
			raw_spin_unlock(&migration_lock);
		}
	}
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	ts[15] = sched_clock();

	if ((ts[15] - ts[0] > 5000000ULL) && in_hardirq()) {
		int i;

		printk_deferred("%s duration %llu, ts[0]=%llu\n",
				__func__, ts[15] - ts[0], ts[0]);
		for (i = 0; i < 15; i++) {
			printk_deferred("%s ts[%d]=%llu, duration=%llu\n",
					__func__, i+1, ts[i + 1], ts[i + 1] - ts[i]);
		}
	}
#endif
}

void hook_scheduler_tick(void *data, struct rq *rq)
{
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_ABNORMAL_FLAG)
	int ret;
#endif /* #OPLUS_FEATURE_ABNORMAL_FLAG */
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GKI_CPUFREQ_BOUNCING)
	int this_cpu = cpu_of(rq);
	struct cpufreq_policy *pol = cpufreq_cpu_get_raw(this_cpu);

	if (pol)
		cb_update(pol, ktime_get_ns());
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_ABNORMAL_FLAG)
	ret = get_ux_state_type(rq->curr);
	if (ret != UX_STATE_INHERIT && ret != UX_STATE_SCHED_ASSIST)
		test_task_overload(rq->curr);
#endif /* #OPLUS_FEATURE_ABNORMAL_FLAG */
	if (rq->curr->policy == SCHED_NORMAL)
		check_for_migration(rq->curr);
}

void mtk_hook_after_enqueue_task(void *data, struct rq *rq,
				struct task_struct *p, int flags)
{
	struct update_util_data *fdata;
	bool should_update = false;
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	u64 ts[5] = {0};

	ts[0] = sched_clock();
#endif
#if IS_ENABLED(CONFIG_MTK_SCHED_BIG_TASK_ROTATE)
	rotat_after_enqueue_task(data, rq, p);
#endif
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	ts[1] = sched_clock();
#endif
#if IS_ENABLED(CONFIG_MTK_CPUFREQ_SUGOV_EXT)
	if (rq->nr_running != 1) {
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
		if ((ts[1] - ts[0] > 1000000ULL) && in_hardirq()) {
			printk_deferred("%s duration %llu, ts[0]=%llu, ts[1]=%llu\n",
					__func__, ts[1] - ts[0], ts[0], ts[1]);
		}
#endif
		return;
	}
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	ts[2] = sched_clock();
#endif
	fdata = rcu_dereference_sched(*per_cpu_ptr(&cpufreq_update_util_data,
							  cpu_of(rq)));
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	ts[3] = sched_clock();
#endif
	if (fdata) {
		int this_cpu = cpu_of(rq);
		struct cpufreq_policy *policy = cpufreq_cpu_get_raw(this_cpu);

		if (policy && !strcmp(policy->governor->name, "uag"))
			should_update = !check_freq_update_for_time_uag(fdata, rq_clock(rq));
		else
			should_update = !check_freq_update_for_time(fdata, rq_clock(rq));
		if (should_update)
			fdata->func(fdata, rq_clock(rq), 0);
	}
#endif
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	ts[4] = sched_clock();

	if ((ts[4] - ts[0] > 1000000ULL) && in_hardirq()) {
		int i;

		printk_deferred("%s duration %llu, ts[0]=%llu\n", __func__, ts[4] - ts[0], ts[0]);
		for (i = 0; i < 4; i++) {
			printk_deferred("%s ts[%d]=%llu, duration=%llu\n",
					__func__, i+1, ts[i + 1], ts[i + 1] - ts[i]);
		}
	}
#endif
}

#if IS_ENABLED(CONFIG_UCLAMP_TASK)
/*
 * Verify the fitness of task @p to run on @cpu taking into account the uclamp
 * settings.
 *
 * This check is only important for heterogeneous systems where uclamp_min value
 * is higher than the capacity of a @cpu. For non-heterogeneous system this
 * function will always return true.
 *
 * The function will return true if the capacity of the @cpu is >= the
 * uclamp_min and false otherwise.
 *
 * Note that uclamp_min will be clamped to uclamp_max if uclamp_min
 * > uclamp_max.
 */
static inline bool mtk_rt_task_fits_capacity(struct task_struct *p, int cpu,
					unsigned long min_cap, unsigned long max_cap)
{
	unsigned int cpu_cap;
	unsigned long util;
	struct rq *rq;

	/* Only heterogeneous systems can benefit from this check */
	if (!likely(mtk_sched_asym_cpucapacity))
		return true;

	rq = cpu_rq(cpu);
	/* refer code from sched.h: effective_cpu_util -> cpu_util_rt */
	util = cpu_util_rt(rq);
	util = mtk_uclamp_rq_util_with(rq, util, p, min_cap, max_cap);
	cpu_cap = capacity_orig_of(cpu);

	return cpu_cap >= clamp_val(util, min_cap, max_cap);
}
#else
static inline bool mtk_rt_task_fits_capacity(struct task_struct *p, int cpu,
					unsigned long min_cap, unsigned long max_cap)
{
	return true;
}
#endif

static inline unsigned int mtk_task_cap(struct task_struct *p, int cpu,
					unsigned long min_cap, unsigned long max_cap)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long cpu_cap = arch_scale_cpu_capacity(cpu);
	struct util_rq util_rq;

	util_rq.util_cfs = cpu_util_cfs(rq);
	util_rq.base = 1;

	return  mtk_cpu_util(cpu, &util_rq, cpu_cap, FREQUENCY_UTIL, p, min_cap, max_cap);
}

#if IS_ENABLED(CONFIG_MTK_OPP_CAP_INFO)
static inline
unsigned long aligned_freq_to_legacy_freq(int cpu, unsigned long freq)
{
	return pd_get_opp_freq_legacy(cpu, pd_get_freq_opp_legacy(cpu, freq));
}

__always_inline
unsigned long calc_pwr(int cpu, unsigned long task_util)
{
	int opp;
	struct mtk_em_perf_state *ps;
	unsigned long dyn_pwr, static_pwr, pwr;

	ps = pd_get_util_ps(cpu, map_util_perf(task_util), &opp);

	dyn_pwr = ps->pwr_eff * task_util;
	static_pwr = (mtk_get_leakage(cpu, opp, get_cpu_temp(cpu)/1000) * task_util) / ps->capacity;
	pwr = dyn_pwr + static_pwr;

	if (trace_sched_em_cpu_energy_enabled())
		trace_sched_em_cpu_energy(opp,
				aligned_freq_to_legacy_freq(cpu, ps->freq), "pwr_eff",
				ps->pwr_eff, ps->capacity, dyn_pwr, static_pwr);

	return pwr;
}

__always_inline
unsigned long calc_pwr_eff(int cpu, unsigned long cpu_util)
{
	int opp;
	struct mtk_em_perf_state *ps;
	unsigned long static_pwr_eff, pwr_eff;

	ps = pd_get_util_ps(cpu, map_util_perf(cpu_util), &opp);

	static_pwr_eff = mtk_get_leakage(cpu, opp, get_cpu_temp(cpu)/1000) / ps->capacity;
	pwr_eff = ps->pwr_eff + static_pwr_eff;

	if (trace_sched_calc_pwr_eff_enabled())
		trace_sched_calc_pwr_eff(cpu, cpu_util, opp, ps->capacity,
				ps->pwr_eff, static_pwr_eff, pwr_eff);

	return pwr_eff;
}
#else
__always_inline
unsigned long calc_pwr(int cpu, unsigned long task_util)
{
	return 0;
}
#endif

#if IS_ENABLED(CONFIG_SMP)
static inline bool should_honor_rt_sync(struct rq *rq, struct task_struct *p,
					bool sync)
{
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	fbg_skip_rt_sync(rq, p, &sync);
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_ASSIST)
	sa_skip_rt_sync(rq, p, &sync);
#endif
	/*
	 * If the waker is CFS, then an RT sync wakeup would preempt the waker
	 * and force it to run for a likely small time after the RT wakee is
	 * done. So, only honor RT sync wakeups from RT wakers.
	 */
	return sync && task_has_rt_policy(rq->curr) &&
		p->prio <= rq->rt.highest_prio.next &&
		rq->rt.rt_nr_running <= 2;
}
#else
static inline bool should_honor_rt_sync(struct rq *rq, struct task_struct *p,
					bool sync)
{
	return 0;
}
#endif

void mtk_select_task_rq_rt(void *data, struct task_struct *p, int source_cpu,
				int sd_flag, int flags, int *target_cpu)
{
	struct task_struct *curr;
	struct rq *rq;
	int lowest_cpu = -1, rt_lowest_cpu = -1;
	int lowest_prio = 0, rt_lowest_prio = p->prio;
	int cpu;
	int select_reason = -1;
	bool sync = !!(flags & WF_SYNC);
	struct rq *this_cpu_rq;
	int this_cpu;
	bool test, may_not_preempt;
	struct cpuidle_state *idle;
	unsigned int min_exit_lat;
	struct root_domain *rd = cpu_rq(smp_processor_id())->rd;
	struct perf_domain *pd;
	unsigned int cpu_util;
	unsigned long occupied_cap, occupied_cap_per_gear;
	int best_idle_cpu_per_gear;
	int best_idle_cpu = -1;
	unsigned long pwr_eff = ULONG_MAX;
	unsigned long this_pwr_eff = ULONG_MAX;
	unsigned long min_cap = uclamp_eff_value(p, UCLAMP_MIN);
	unsigned long max_cap = uclamp_eff_value(p, UCLAMP_MAX);
	unsigned int cfs_cpus = 0;
	unsigned int idle_cpus = 0;
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	u64 ts[2];

	ts[0] = sched_clock();
#endif

	*target_cpu = -1;
	/* For anything but wake ups, just return the task_cpu */
	if (!(flags & (WF_TTWU | WF_FORK))) {
		if (!cpu_paused(source_cpu)) {
			select_reason = LB_RT_FAIL;
			*target_cpu = source_cpu;
			goto out;
		}
	}

	rcu_read_lock();
	this_cpu = smp_processor_id();
	this_cpu_rq = cpu_rq(this_cpu);

	/*
	 * Respect the sync flag as long as the task can run on this CPU.
	 */
	if (should_honor_rt_sync(this_cpu_rq, p, sync) &&
	    cpumask_test_cpu(this_cpu, p->cpus_ptr)) {
		*target_cpu = this_cpu;
		select_reason = LB_RT_SYNC;
		goto unlock;
	}

	/*
	 * Select one CPU from each cluster and
	 * compare its power / capacity.
	 */
	pd = rcu_dereference(rd->pd);
	if (!pd) {
		select_reason = LB_RT_FAIL_PD;
		goto source;
	}

	for (; pd; pd = pd->next) {
		min_exit_lat = UINT_MAX;
		occupied_cap_per_gear = ULONG_MAX;
		best_idle_cpu_per_gear = -1;
		for_each_cpu_and(cpu, perf_domain_span(pd), cpu_active_mask) {
			if (!cpumask_test_cpu(cpu, p->cpus_ptr))
				continue;

			if (cpu_paused(cpu))
				continue;

			if (!mtk_rt_task_fits_capacity(p, cpu, min_cap, max_cap))
				continue;

			if (idle_cpu(cpu)
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
				&& fbg_rt_task_fits_capacity(p, cpu)
#endif
				) {
				/* WFI > non-WFI */
				idle_cpus = (idle_cpus | (1 << cpu));
				idle = idle_get_state(cpu_rq(cpu));
				occupied_cap = mtk_task_cap(p, cpu, min_cap, max_cap);
				if (idle) {
					/* non WFI, find shortest exit_latency */
					if (idle->exit_latency < min_exit_lat) {
						min_exit_lat = idle->exit_latency;
						best_idle_cpu_per_gear = cpu;
						occupied_cap_per_gear = occupied_cap;
					} else if ((idle->exit_latency == min_exit_lat)
						&& (occupied_cap_per_gear > occupied_cap)) {
						best_idle_cpu_per_gear = cpu;
						occupied_cap_per_gear = occupied_cap;
					}
				} else {
					/* WFI, find max_spare_cap (least occupied_cap) */
					if (min_exit_lat > 0) {
						min_exit_lat = 0;
						best_idle_cpu_per_gear = cpu;
						occupied_cap_per_gear = occupied_cap;
					} else if (occupied_cap_per_gear > occupied_cap) {
						best_idle_cpu_per_gear = cpu;
						occupied_cap_per_gear = occupied_cap;
					}
				}
				continue;
			}
			rq = cpu_rq(cpu);
			curr = rq->curr;
			if (curr && (curr->policy == SCHED_NORMAL)
					&& (curr->prio > lowest_prio)
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
				&& (fbg_rt_task_fits_capacity(p, cpu))
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_ASSIST)
                                && !sa_rt_skip_ux_cpu(cpu)
#endif
					&& (!task_may_not_preempt(curr, cpu))) {
				lowest_prio = curr->prio;
				lowest_cpu = cpu;
				cfs_cpus = (cfs_cpus | (1 << cpu));
			}
		}
		if (best_idle_cpu_per_gear != -1) {
			cpu_util = occupied_cap_per_gear;
			this_pwr_eff = calc_pwr_eff(best_idle_cpu_per_gear, cpu_util);

			if (trace_sched_aware_energy_rt_enabled()) {
				trace_sched_aware_energy_rt(best_idle_cpu_per_gear, this_pwr_eff,
						pwr_eff, cpu_util);
			}

			if (this_pwr_eff < pwr_eff) {
				pwr_eff = this_pwr_eff;
				best_idle_cpu = best_idle_cpu_per_gear;
			}
		}
	}

	if (best_idle_cpu != -1) {
		*target_cpu = best_idle_cpu;
		select_reason = LB_RT_IDLE;
		goto unlock;
	}

	if (lowest_cpu != -1) {
		*target_cpu =  lowest_cpu;
		select_reason = LB_RT_LOWEST_PRIO_NORMAL;
		goto unlock;
	}

source:
	rq = cpu_rq(source_cpu);
	/* unlocked access */
	curr = READ_ONCE(rq->curr);
	/* check source_cpu status */
	may_not_preempt = task_may_not_preempt(curr, source_cpu);
	test = (curr && (may_not_preempt ||
			 (unlikely(rt_task(curr)) &&
			  (curr->nr_cpus_allowed < 2 || curr->prio <= p->prio))));

	if (!test && mtk_rt_task_fits_capacity(p, source_cpu, min_cap, max_cap)) {
		select_reason = ((select_reason == -1) ? LB_RT_SOURCE_CPU : select_reason);
		*target_cpu = source_cpu;
	}

unlock:
	rcu_read_unlock();

	/* if no cpu fufill condition above,
	 * then select cpu with lowest prioity
	 */
	if (-1 == *target_cpu) {
		for_each_cpu(cpu, p->cpus_ptr) {

			if (cpu_paused(cpu))
				continue;

			if (!mtk_rt_task_fits_capacity(p, cpu, min_cap, max_cap))
				continue;

			rq = cpu_rq(cpu);
			curr = rq->curr;
			if (curr && rt_task(curr)
					&& (curr->prio > rt_lowest_prio)) {
				rt_lowest_prio = curr->prio;
				rt_lowest_cpu = cpu;
			}
		}
		*target_cpu =  rt_lowest_cpu;
		select_reason = LB_RT_LOWEST_PRIO_RT;
	}

out:
	if (trace_sched_select_task_rq_rt_enabled())
		trace_sched_select_task_rq_rt(p, select_reason, *target_cpu, idle_cpus, cfs_cpus,
					sd_flag, sync);

#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	ts[1] = sched_clock();

	if ((ts[1] - ts[0] > 500000ULL) && in_hardirq()) {
		printk_deferred("%s duration %llu, ts[0]=%llu, ts[1]=%llu\n",
				__func__, ts[1] - ts[0], ts[0], ts[1]);

	}
#endif
}

#if IS_ENABLED(CONFIG_MTK_EAS)
void mtk_pelt_rt_tp(void *data, struct rq *rq)
{
	cpufreq_update_util(rq, 0);
}

void mtk_sched_switch(void *data, struct task_struct *prev,
		struct task_struct *next, struct rq *rq)
{
	if (next->pid == 0)
		rq->android_vendor_data1[RQ_SBB_ACTIVE] = 0;
}
#endif

void mtk_find_lowest_rq(void *data, struct task_struct *p, struct cpumask *lowest_mask,
			int ret, int *lowest_cpu)
{
	int cpu = -1;
	int this_cpu = smp_processor_id();
	cpumask_t avail_lowest_mask;
	int lowest_prio_cpu = -1, lowest_prio = 0;
	int select_reason = -1;
	unsigned int gear_id, nr_gear;
	struct cpumask *gear_cpus;
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	u64 ts[10] = {0};

	ts[0] = sched_clock();
#endif

	cpumask_andnot(&avail_lowest_mask, lowest_mask, cpu_pause_mask);
	if (!ret) {
		select_reason = LB_RT_NO_LOWEST_RQ;
		goto out; /* No targets found */
	}

#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	ts[1] = sched_clock();
#endif

	cpu = task_cpu(p);

	/*
	 * At this point we have built a mask of CPUs representing the
	 * lowest priority tasks in the system.  Now we want to elect
	 * the best one based on our affinity and topology.
	 *
	 * We prioritize the last CPU that the task executed on since
	 * it is most likely cache-hot in that location.
	 */
	if (cpumask_test_cpu(cpu, &avail_lowest_mask)) {
		*lowest_cpu = cpu;
		select_reason = LB_RT_SOURCE_CPU;
		goto out;
	}
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	ts[2] = sched_clock();
#endif

	/*
	 * Otherwise, we consult the sched_domains span maps to figure
	 * out which CPU is logically closest to our hot cache data.
	 */
	if (!cpumask_test_cpu(this_cpu, &avail_lowest_mask))
		this_cpu = -1; /* Skip this_cpu opt if not among lowest */

#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	ts[3] = sched_clock();
#endif

	nr_gear = get_nr_gears();

#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	ts[4] = sched_clock();
#endif

	/* Find best_cpu on same cluster with task_cpu(p) */
	for (gear_id = 0; gear_id < nr_gear; gear_id++) {
		gear_cpus = get_gear_cpumask(gear_id);

		/*
		 * "this_cpu" is cheaper to preempt than a
		 * remote processor.
		 */
		if (this_cpu != -1 && cpumask_test_cpu(this_cpu, gear_cpus)) {
			*lowest_cpu = this_cpu;
			select_reason = LB_RT_SYNC;
			goto out;
		}

		for_each_cpu_and(cpu, &avail_lowest_mask, gear_cpus) {
			struct task_struct *curr;

			if (idle_cpu(cpu)) {
				*lowest_cpu = cpu;
				select_reason = LB_RT_IDLE;
				goto out;
			}
			curr = cpu_curr(cpu);
			/* &fair_sched_class undefined in scheduler.ko */
			if (fair_policy(curr->policy) && (curr->prio > lowest_prio)) {
				lowest_prio = curr->prio;
				lowest_prio_cpu = cpu;
			}
		}
	}

#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	ts[5] = sched_clock();
#endif

	if (lowest_prio_cpu != -1) {
		*lowest_cpu = lowest_prio_cpu;
		select_reason = LB_RT_LOWEST_PRIO;
		goto out;
	}

	/*
	 * And finally, if there were no matches within the domains
	 * just give the caller *something* to work with from the compatible
	 * locations.
	 */
	if (this_cpu != -1) {
		*lowest_cpu = this_cpu;
		select_reason = LB_RT_FAIL_SYNC;
		goto out;
	}
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	ts[6] = sched_clock();
#endif

	cpu = cpumask_any_and_distribute(&avail_lowest_mask, cpu_possible_mask);
	if (cpu < nr_cpu_ids) {
		*lowest_cpu = cpu;
		select_reason = LB_RT_FAIL_RANDOM;
		goto out;
	}

#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	ts[7] = sched_clock();
#endif

	/* Let find_lowest_rq not to choose dst_cpu */
	*lowest_cpu = -1;
	select_reason = LB_RT_FAIL;
	cpumask_clear(lowest_mask);

out:
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	ts[8] = sched_clock();
#endif

	if (trace_sched_find_lowest_rq_enabled())
		trace_sched_find_lowest_rq(p, select_reason, *lowest_cpu,
				&avail_lowest_mask, lowest_mask);

#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	ts[9] = sched_clock();

	if ((ts[9] - ts[0] > 200000ULL) && in_hardirq()) {
		int i, i_prev;
		u64 prev, curr;

		printk_deferred("%s duration %llu, ts[0]=%llu\n", __func__, ts[9] - ts[0], ts[0]);
		i_prev = 0;
		for (i = 0; i < 9; i++) {
			if (ts[i+1]) {
				prev = ts[i_prev];
				curr = ts[i+1];
				printk_deferred("%s ts[%d]=%llu, ts[%d]=%llu, duration=%llu\n",
						__func__, i_prev, prev, i+1, curr, curr - prev);
				i_prev = i+1;
			}
		}
	}
#endif
	return;
}

int set_util_est_ctrl(bool enable)
{
	sysctl_util_est = enable;
	return 0;
}

int set_task_idle_prefer(int pid, bool prefer)
{
	struct task_struct *task = NULL;

	rcu_read_lock();
	task = find_task_by_vpid(pid);
	if (!task) {
		rcu_read_unlock();
		return -ESRCH;
	}
	get_task_struct(task);
	task->android_vendor_data1[T_TASK_IDLE_PREFER_FLAG] = prefer;
	rcu_read_unlock();
	put_task_struct(task);

	return 0;
}

bool get_task_idle_prefer_by_pid(int pid)
{
	struct task_struct *task = NULL;
	bool rslt = false;

	rcu_read_lock();
	task = find_task_by_vpid(pid);
	if (!task) {
		rcu_read_unlock();
		return false;
	}
	get_task_struct(task);

	rslt = task->android_vendor_data1[T_TASK_IDLE_PREFER_FLAG];
	rcu_read_unlock();
	put_task_struct(task);
	return rslt;
}

inline bool get_task_idle_prefer_by_task(struct task_struct *task)
{
	return task->android_vendor_data1[T_TASK_IDLE_PREFER_FLAG];
}

