/*
 * drivers/cpufreq/cpufreq_governor.c
 *
 * CPUFREQ governors common code
 *
 * Copyright	(C) 2001 Russell King
 *		(C) 2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *		(C) 2003 Jun Nakajima <jun.nakajima@intel.com>
 *		(C) 2009 Alexander Clouter <alex@digriz.org.uk>
 *		(c) 2012 Viresh Kumar <viresh.kumar@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <asm/cputime.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/export.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/tick.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "cpufreq_governor.h"

static inline u64 get_cpu_idle_time_jiffy(unsigned int cpu, u64 *wall)
{
	struct cpu_dbs_info *cdbs = container_of(data, struct cpu_dbs_info, update_util);
	struct policy_dbs_info *policy_dbs = cdbs->policy_dbs;
	u64 delta_ns, lst;

	/*
	 * The work may not be allowed to be queued up right now.
	 * Possible reasons:
	 * - Work has already been queued up or is in progress.
	 * - It is too early (too little time from the previous sample).
	 */
	if (policy_dbs->work_in_progress)
		return;

	/*
	 * If the reads below are reordered before the check above, the value
	 * of sample_delay_ns used in the computation may be stale.
	 */
	smp_rmb();
	lst = READ_ONCE(policy_dbs->last_sample_time);
	delta_ns = time - lst;
	if ((s64)delta_ns < policy_dbs->sample_delay_ns)
		return;

	idle_time = cur_wall_time - busy_time;
	if (wall)
		*wall = jiffies_to_usecs(cur_wall_time);

	return jiffies_to_usecs(idle_time);
}

static void gov_set_update_util(struct policy_dbs_info *policy_dbs,
				unsigned int delay_us)
{
	struct cpufreq_policy *policy = policy_dbs->policy;
	int cpu;

	gov_update_sample_delay(policy_dbs, delay_us);
	policy_dbs->last_sample_time = 0;

	for_each_cpu(cpu, policy->cpus) {
		struct cpu_dbs_info *cdbs = &per_cpu(cpu_dbs, cpu);

		cpufreq_add_update_util_hook(cpu, &cdbs->update_util,
					     dbs_update_util_handler);
	}
}

static inline void gov_clear_update_util(struct cpufreq_policy *policy)
{
	int i;

	for_each_cpu(i, policy->cpus)
		cpufreq_remove_update_util_hook(i);

	synchronize_sched();
}

static struct policy_dbs_info *alloc_policy_dbs_info(struct cpufreq_policy *policy,
						     struct dbs_governor *gov)
{
	if (dbs_data->cdata->governor == GOV_CONSERVATIVE) {
		struct cs_dbs_tuners *cs_tuners = dbs_data->tuners;
		cs_tuners->sampling_rate = max(cs_tuners->sampling_rate,
			sampling_rate);
	} else if (dbs_data->cdata->governor == GOV_ALUCARD) {
		struct ac_dbs_tuners *ac_tuners = dbs_data->tuners;
		ac_tuners->sampling_rate = max(ac_tuners->sampling_rate, 
			sampling_rate);
	} else if (dbs_data->cdata->governor == GOV_DARKNESS) {
		struct dk_dbs_tuners *dk_tuners = dbs_data->tuners;
		dk_tuners->sampling_rate = max(dk_tuners->sampling_rate, 
			sampling_rate);
	} else if (dbs_data->cdata->governor == GOV_NIGHTMARE) {
		struct nm_dbs_tuners *nm_tuners = dbs_data->tuners;
		nm_tuners->sampling_rate = max(nm_tuners->sampling_rate, 
			sampling_rate);
	} else {
		struct od_dbs_tuners *od_tuners = dbs_data->tuners;
		od_tuners->sampling_rate = max(od_tuners->sampling_rate, 
			sampling_rate);
	}
	return policy_dbs;
}

static void free_policy_dbs_info(struct policy_dbs_info *policy_dbs,
				 struct dbs_governor *gov)
{
	struct dbs_data *dbs_data;
	struct od_cpu_dbs_info_s *od_dbs_info = NULL;
	struct cs_cpu_dbs_info_s *cs_dbs_info = NULL;
	struct ac_cpu_dbs_info_s *ac_dbs_info = NULL;
	struct dk_cpu_dbs_info_s *dk_dbs_info = NULL;
	struct nm_cpu_dbs_info_s *nm_dbs_info = NULL;
	struct od_ops *od_ops = NULL;
	struct ac_ops *ac_ops = NULL;
	struct dk_ops *dk_ops = NULL;
	struct nm_ops *nm_ops = NULL;
	struct od_dbs_tuners *od_tuners = NULL;
	struct cs_dbs_tuners *cs_tuners = NULL;
	struct ac_dbs_tuners *ac_tuners = NULL;
	struct dk_dbs_tuners *dk_tuners = NULL;
	struct nm_dbs_tuners *nm_tuners = NULL;
	struct cpu_dbs_common_info *cpu_cdbs;
	unsigned int sampling_rate, latency, ignore_nice, j, cpu = policy->cpu;
	int io_busy = 0;
	int rc;

	if (have_governor_per_policy())
		dbs_data = policy->governor_data;
	else
		dbs_data = cdata->gdbs_data;

	WARN_ON(!dbs_data && (event != CPUFREQ_GOV_POLICY_INIT));

	switch (event) {
	case CPUFREQ_GOV_POLICY_INIT:
		if (have_governor_per_policy()) {
			WARN_ON(dbs_data);
		} else if (dbs_data) {
			dbs_data->usage_count++;
			policy->governor_data = dbs_data;
			return 0;
		}

	mutex_destroy(&policy_dbs->timer_mutex);

	for_each_cpu(j, policy_dbs->policy->related_cpus) {
		struct cpu_dbs_info *j_cdbs = &per_cpu(cpu_dbs, j);

		j_cdbs->policy_dbs = NULL;
		j_cdbs->update_util.func = NULL;
	}
	gov->free(policy_dbs);
}

int cpufreq_dbs_governor_init(struct cpufreq_policy *policy)
{
	struct dbs_governor *gov = dbs_governor_of(policy);
	struct dbs_data *dbs_data;
	struct policy_dbs_info *policy_dbs;
	unsigned int latency;
	int ret = 0;

	/* State should be equivalent to EXIT */
	if (policy->governor_data)
		return -EBUSY;

	policy_dbs = alloc_policy_dbs_info(policy, gov);
	if (!policy_dbs)
		return -ENOMEM;

	/* Protect gov->gdbs_data against concurrent updates. */
	mutex_lock(&gov_dbs_data_mutex);

	dbs_data = gov->gdbs_data;
	if (dbs_data) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto free_policy_dbs_info;
		}
		policy_dbs->dbs_data = dbs_data;
		policy->governor_data = policy_dbs;

		gov_attr_set_get(&dbs_data->attr_set, &policy_dbs->list);
		goto out;
	}

	dbs_data = kzalloc(sizeof(*dbs_data), GFP_KERNEL);
	if (!dbs_data) {
		ret = -ENOMEM;
		goto free_policy_dbs_info;
	}

	gov_attr_set_init(&dbs_data->attr_set, &policy_dbs->list);

	ret = gov->init(dbs_data);
	if (ret)
		goto free_policy_dbs_info;

	/* policy latency is in ns. Convert it to us first */
	latency = policy->cpuinfo.transition_latency / 1000;
	if (latency == 0)
		latency = 1;

	/* Bring kernel and HW constraints together */
	dbs_data->min_sampling_rate = max(dbs_data->min_sampling_rate,
					  MIN_LATENCY_MULTIPLIER * latency);
	dbs_data->sampling_rate = max(dbs_data->min_sampling_rate,
				      LATENCY_MULTIPLIER * latency);

	if (!have_governor_per_policy())
		gov->gdbs_data = dbs_data;

	policy_dbs->dbs_data = dbs_data;
	policy->governor_data = policy_dbs;

	gov->kobj_type.sysfs_ops = &governor_sysfs_ops;
	ret = kobject_init_and_add(&dbs_data->attr_set.kobj, &gov->kobj_type,
				   get_governor_parent_kobj(policy),
				   "%s", gov->gov.name);
	if (!ret)
		goto out;

	/* Failure, so roll back. */
	pr_err("initialization failed (dbs_data kobject init error %d)\n", ret);

	kobject_put(&dbs_data->attr_set.kobj);

	policy->governor_data = NULL;

	cpu_cdbs = dbs_data->cdata->get_cpu_cdbs(cpu);

	if (dbs_data->cdata->governor == GOV_CONSERVATIVE) {
		cs_tuners = dbs_data->tuners;
		cs_dbs_info = dbs_data->cdata->get_cpu_dbs_info_s(cpu);
		sampling_rate = cs_tuners->sampling_rate;
		ignore_nice = cs_tuners->ignore_nice_load;
	} else if (dbs_data->cdata->governor == GOV_ALUCARD) {
		ac_tuners = dbs_data->tuners;
		ac_dbs_info = dbs_data->cdata->get_cpu_dbs_info_s(cpu);
		sampling_rate = ac_tuners->sampling_rate;
		ignore_nice = ac_tuners->ignore_nice_load;
		ac_ops = dbs_data->cdata->gov_ops;
	} else if (dbs_data->cdata->governor == GOV_DARKNESS) {
		dk_tuners = dbs_data->tuners;
		dk_dbs_info = dbs_data->cdata->get_cpu_dbs_info_s(cpu);
		sampling_rate = dk_tuners->sampling_rate;
		ignore_nice = dk_tuners->ignore_nice_load;
		dk_ops = dbs_data->cdata->gov_ops;
	} else if (dbs_data->cdata->governor == GOV_NIGHTMARE) {
		nm_tuners = dbs_data->tuners;
		nm_dbs_info = dbs_data->cdata->get_cpu_dbs_info_s(cpu);
		sampling_rate = nm_tuners->sampling_rate;
		ignore_nice = nm_tuners->ignore_nice_load;
		nm_ops = dbs_data->cdata->gov_ops;
	} else {
		od_tuners = dbs_data->tuners;
		od_dbs_info = dbs_data->cdata->get_cpu_dbs_info_s(cpu);
		sampling_rate = od_tuners->sampling_rate;
		ignore_nice = od_tuners->ignore_nice_load;
		od_ops = dbs_data->cdata->gov_ops;
		io_busy = od_tuners->io_is_busy;
	}

free_policy_dbs_info:
	free_policy_dbs_info(policy_dbs, gov);

out:
	mutex_unlock(&gov_dbs_data_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(cpufreq_dbs_governor_init);

void cpufreq_dbs_governor_exit(struct cpufreq_policy *policy)
{
	struct dbs_governor *gov = dbs_governor_of(policy);
	struct policy_dbs_info *policy_dbs = policy->governor_data;
	struct dbs_data *dbs_data = policy_dbs->dbs_data;
	unsigned int count;

	/* Protect gov->gdbs_data against concurrent updates. */
	mutex_lock(&gov_dbs_data_mutex);

	count = gov_attr_set_put(&dbs_data->attr_set, &policy_dbs->list);

	policy->governor_data = NULL;

	if (!count) {
		if (!have_governor_per_policy())
			gov->gdbs_data = NULL;

		if (dbs_data->cdata->governor == GOV_CONSERVATIVE) {
			cs_dbs_info->down_skip = 0;
			cs_dbs_info->enable = 1;
			cs_dbs_info->requested_freq = policy->cur;
		} else if (dbs_data->cdata->governor == GOV_ALUCARD) {
			ac_ops->get_cpu_frequency_table(cpu);
			ac_ops->get_cpu_frequency_table_minmax(policy, cpu);
			ac_dbs_info->up_rate = 1;
			ac_dbs_info->down_rate = 1;
		} else if (dbs_data->cdata->governor == GOV_DARKNESS) {
			dk_ops->get_cpu_frequency_table(cpu);
		} else if (dbs_data->cdata->governor == GOV_NIGHTMARE) {
			nm_ops->get_cpu_frequency_table(cpu);
		} else {
			od_dbs_info->rate_mult = 1;
			od_dbs_info->sample_type = OD_NORMAL_SAMPLE;
			od_ops->powersave_bias_init_cpu(cpu);
		}

	free_policy_dbs_info(policy_dbs, gov);

	mutex_unlock(&gov_dbs_data_mutex);
}
EXPORT_SYMBOL_GPL(cpufreq_dbs_governor_exit);

int cpufreq_dbs_governor_start(struct cpufreq_policy *policy)
{
	struct dbs_governor *gov = dbs_governor_of(policy);
	struct policy_dbs_info *policy_dbs = policy->governor_data;
	struct dbs_data *dbs_data = policy_dbs->dbs_data;
	unsigned int sampling_rate, ignore_nice, j;
	unsigned int io_busy;

	if (!policy->cur)
		return -EINVAL;

	policy_dbs->is_shared = policy_is_shared(policy);
	policy_dbs->rate_mult = 1;

	sampling_rate = dbs_data->sampling_rate;
	ignore_nice = dbs_data->ignore_nice_load;
	io_busy = dbs_data->io_is_busy;

	for_each_cpu(j, policy->cpus) {
		struct cpu_dbs_info *j_cdbs = &per_cpu(cpu_dbs, j);

		j_cdbs->prev_cpu_idle = get_cpu_idle_time(j, &j_cdbs->prev_update_time, io_busy);
		/*
		 * Make the first invocation of dbs_update() compute the load.
		 */
		j_cdbs->prev_load = 0;

		if (ignore_nice)
			j_cdbs->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];
	}

	gov->start(policy);

	gov_set_update_util(policy_dbs, sampling_rate);
	return 0;
}
EXPORT_SYMBOL_GPL(cpufreq_dbs_governor_start);

void cpufreq_dbs_governor_stop(struct cpufreq_policy *policy)
{
	struct policy_dbs_info *policy_dbs = policy->governor_data;

	gov_clear_update_util(policy_dbs->policy);
	irq_work_sync(&policy_dbs->irq_work);
	cancel_work_sync(&policy_dbs->work);
	atomic_set(&policy_dbs->work_count, 0);
	policy_dbs->work_in_progress = false;
}
EXPORT_SYMBOL_GPL(cpufreq_dbs_governor_stop);

void cpufreq_dbs_governor_limits(struct cpufreq_policy *policy)
{
	struct policy_dbs_info *policy_dbs = policy->governor_data;

	mutex_lock(&policy_dbs->timer_mutex);
	cpufreq_policy_apply_limits(policy);
	gov_update_sample_delay(policy_dbs, 0);

	mutex_unlock(&policy_dbs->timer_mutex);
}
EXPORT_SYMBOL_GPL(get_cpu_idle_time);

void dbs_check_cpu(struct dbs_data *dbs_data, int cpu)
{
	struct cpu_dbs_common_info *cdbs = dbs_data->get_cpu_cdbs(cpu);
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	struct cs_dbs_tuners *cs_tuners = dbs_data->tuners;
	struct cpufreq_policy *policy;
	unsigned int max_load = 0;
	unsigned int ignore_nice;
	unsigned int j;

	if (dbs_data->governor == GOV_ONDEMAND)
		ignore_nice = od_tuners->ignore_nice;
	else
		ignore_nice = cs_tuners->ignore_nice;

	policy = cdbs->cur_policy;

	/* Get Absolute Load (in terms of freq for ondemand gov) */
	for_each_cpu(j, policy->cpus) {
		struct cpu_dbs_common_info *j_cdbs;
		cputime64_t cur_wall_time, cur_idle_time, cur_iowait_time;
		unsigned int idle_time, wall_time, iowait_time;
		unsigned int load;

		j_cdbs = dbs_data->get_cpu_cdbs(j);

		cur_idle_time = get_cpu_idle_time(j, &cur_wall_time);

		wall_time = (unsigned int)
			(cur_wall_time - j_cdbs->prev_cpu_wall);
		j_cdbs->prev_cpu_wall = cur_wall_time;

		idle_time = (unsigned int)
			(cur_idle_time - j_cdbs->prev_cpu_idle);
		j_cdbs->prev_cpu_idle = cur_idle_time;

		if (ignore_nice) {
			u64 cur_nice;
			unsigned long cur_nice_jiffies;

			cur_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE] -
					 cdbs->prev_cpu_nice;
			/*
			 * Assumption: nice time between sampling periods will
			 * be less than 2^32 jiffies for 32 bit sys
			 */
			cur_nice_jiffies = (unsigned long)
					cputime64_to_jiffies64(cur_nice);

			cdbs->prev_cpu_nice =
				kcpustat_cpu(j).cpustat[CPUTIME_NICE];
			idle_time += jiffies_to_usecs(cur_nice_jiffies);
		}

		if (dbs_data->governor == GOV_ONDEMAND) {
			struct od_cpu_dbs_info_s *od_j_dbs_info =
				dbs_data->get_cpu_dbs_info_s(cpu);

			cur_iowait_time = get_cpu_iowait_time_us(j,
					&cur_wall_time);
			if (cur_iowait_time == -1ULL)
				cur_iowait_time = 0;

			iowait_time = (unsigned int) (cur_iowait_time -
					od_j_dbs_info->prev_cpu_iowait);
			od_j_dbs_info->prev_cpu_iowait = cur_iowait_time;

			/*
			 * For the purpose of ondemand, waiting for disk IO is
			 * an indication that you're performance critical, and
			 * not that the system is actually idle. So subtract the
			 * iowait time from the cpu idle time.
			 */
			if (od_tuners->io_is_busy && idle_time >= iowait_time)
				idle_time -= iowait_time;
		}

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		load = 100 * (wall_time - idle_time) / wall_time;

		if (dbs_data->governor == GOV_ONDEMAND) {
			int freq_avg = __cpufreq_driver_getavg(policy, j);
			if (freq_avg <= 0)
				freq_avg = policy->cur;

			load *= freq_avg;
		}

		if (load > max_load)
			max_load = load;
	}

	dbs_data->gov_check_cpu(cpu, max_load);
}
EXPORT_SYMBOL_GPL(dbs_check_cpu);

static inline void dbs_timer_init(struct dbs_data *dbs_data,
		struct cpu_dbs_common_info *cdbs, unsigned int sampling_rate)
{
	int delay = delay_for_sampling_rate(sampling_rate);

	INIT_DEFERRABLE_WORK(&cdbs->work, dbs_data->gov_dbs_timer);
	schedule_delayed_work_on(cdbs->cpu, &cdbs->work, delay);
}

static inline void dbs_timer_exit(struct cpu_dbs_common_info *cdbs)
{
	cancel_delayed_work_sync(&cdbs->work);
}

int cpufreq_governor_dbs(struct dbs_data *dbs_data,
		struct cpufreq_policy *policy, unsigned int event)
{
	struct od_cpu_dbs_info_s *od_dbs_info = NULL;
	struct cs_cpu_dbs_info_s *cs_dbs_info = NULL;
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	struct cs_dbs_tuners *cs_tuners = dbs_data->tuners;
	struct cpu_dbs_common_info *cpu_cdbs;
	unsigned int *sampling_rate, latency, ignore_nice, j, cpu = policy->cpu;
	int rc;

	cpu_cdbs = dbs_data->get_cpu_cdbs(cpu);

	if (dbs_data->governor == GOV_CONSERVATIVE) {
		cs_dbs_info = dbs_data->get_cpu_dbs_info_s(cpu);
		sampling_rate = &cs_tuners->sampling_rate;
		ignore_nice = cs_tuners->ignore_nice;
	} else {
		od_dbs_info = dbs_data->get_cpu_dbs_info_s(cpu);
		sampling_rate = &od_tuners->sampling_rate;
		ignore_nice = od_tuners->ignore_nice;
	}

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		mutex_lock(&dbs_data->mutex);

		dbs_data->enable++;
		cpu_cdbs->cpu = cpu;
		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_common_info *j_cdbs;
			j_cdbs = dbs_data->get_cpu_cdbs(j);

			j_cdbs->cur_policy = policy;
			j_cdbs->prev_cpu_idle = get_cpu_idle_time(j,
					&j_cdbs->prev_cpu_wall);
			if (ignore_nice)
				j_cdbs->prev_cpu_nice =
					kcpustat_cpu(j).cpustat[CPUTIME_NICE];
		}

		/*
		 * Start the timerschedule work, when this governor is used for
		 * first time
		 */
		if (dbs_data->enable != 1)
			goto second_time;

		rc = sysfs_create_group(cpufreq_global_kobject,
				dbs_data->attr_group);
		if (rc) {
			mutex_unlock(&dbs_data->mutex);
			return rc;
		}

		/* policy latency is in nS. Convert it to uS first */
		latency = policy->cpuinfo.transition_latency / 1000;
		if (latency == 0)
			latency = 1;

		/*
		 * conservative does not implement micro like ondemand
		 * governor, thus we are bound to jiffes/HZ
		 */
		if (dbs_data->governor == GOV_CONSERVATIVE) {
			struct cs_ops *ops = dbs_data->gov_ops;

			cpufreq_register_notifier(ops->notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);

			dbs_data->min_sampling_rate = MIN_SAMPLING_RATE_RATIO *
				jiffies_to_usecs(10);
		} else {
			struct od_ops *ops = dbs_data->gov_ops;

			od_tuners->io_is_busy = ops->io_busy();
		}

		/* Bring kernel and HW constraints together */
		dbs_data->min_sampling_rate = max(dbs_data->min_sampling_rate,
				MIN_LATENCY_MULTIPLIER * latency);
		*sampling_rate = max(dbs_data->min_sampling_rate, latency *
				LATENCY_MULTIPLIER);

second_time:
		if (dbs_data->governor == GOV_CONSERVATIVE) {
			cs_dbs_info->down_skip = 0;
			cs_dbs_info->enable = 1;
			cs_dbs_info->requested_freq = policy->cur;
		} else {
			struct od_ops *ops = dbs_data->gov_ops;
			od_dbs_info->rate_mult = 1;
			od_dbs_info->sample_type = OD_NORMAL_SAMPLE;
			ops->powersave_bias_init_cpu(cpu);
		}
		mutex_unlock(&dbs_data->mutex);

		mutex_init(&cpu_cdbs->timer_mutex);
		dbs_timer_init(dbs_data, cpu_cdbs, *sampling_rate);
		break;

	case CPUFREQ_GOV_STOP:
		if (dbs_data->governor == GOV_CONSERVATIVE)
			cs_dbs_info->enable = 0;

		dbs_timer_exit(cpu_cdbs);

		mutex_lock(&dbs_data->mutex);
		mutex_destroy(&cpu_cdbs->timer_mutex);
		dbs_data->enable--;
		if (!dbs_data->enable) {
			struct cs_ops *ops = dbs_data->gov_ops;

			sysfs_remove_group(cpufreq_global_kobject,
					dbs_data->attr_group);
			if (dbs_data->governor == GOV_CONSERVATIVE)
				cpufreq_unregister_notifier(ops->notifier_block,
						CPUFREQ_TRANSITION_NOTIFIER);
		}
		mutex_unlock(&dbs_data->mutex);

		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&cpu_cdbs->timer_mutex);
		if (policy->max < cpu_cdbs->cur_policy->cur)
			__cpufreq_driver_target(cpu_cdbs->cur_policy,
					policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > cpu_cdbs->cur_policy->cur)
			__cpufreq_driver_target(cpu_cdbs->cur_policy,
					policy->min, CPUFREQ_RELATION_L);
		dbs_check_cpu(dbs_data, cpu);
		mutex_unlock(&cpu_cdbs->timer_mutex);
		break;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(cpufreq_governor_dbs);
