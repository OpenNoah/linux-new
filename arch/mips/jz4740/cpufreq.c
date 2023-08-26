/*
 * linux/arch/mips/jz4740/cpufreq.c
 *
 * cpufreq driver for JZ4740
 *
 * Copyright (c) 2010       Ulrich Hecht <ulrich.hecht@gmail.com>
 * Copyright (c) 2010       Maarten ter Huurne <maarten@treewalker.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>

#include <linux/cpufreq.h>

#include <linux/clk.h>
#include <asm/mach-jz4740/base.h>

#include "clock.h"

#define DEBUG_CPUFREQ

#ifdef DEBUG_CPUFREQ
#define dprintk(X...) printk(KERN_INFO X)
#else
#define dprintk(X...) do { } while(0)
#endif

#define HCLK_MIN 30000
/* TODO: The maximum MCLK most likely depends on the SDRAM chips used,
         so it is board-specific. */
#define MCLK_MAX 140000

/* Same as jz_clk_main_divs, but with 24 and 32 removed because the hardware
   spec states those dividers must not be used for CCLK or HCLK. */
static const unsigned int jz4740_freq_cpu_divs[] = {1, 2, 3, 4, 6, 8, 12, 16};

struct jz4740_freq_percpu_info {
	unsigned int pll_rate;
	struct cpufreq_frequency_table table[
		ARRAY_SIZE(jz4740_freq_cpu_divs) + 1];
};

static struct clk *pll;
static struct clk *cclk;

static struct jz4740_freq_percpu_info jz4740_freq_info;

static struct cpufreq_driver cpufreq_jz4740_driver;

static void jz4740_freq_fill_table(struct cpufreq_policy *policy,
				   unsigned int pll_rate)
{
	struct cpufreq_frequency_table *table = &jz4740_freq_info.table[0];
	int i;

#ifdef CONFIG_CPU_FREQ_STAT_DETAILS
	/* for showing /sys/devices/system/cpu/cpuX/cpufreq/stats/ */
	static bool init = false;
	if (init)
		cpufreq_frequency_table_put_attr(policy->cpu);
	else
		init = true;
#endif

	jz4740_freq_info.pll_rate = pll_rate;

	for (i = 0; i < ARRAY_SIZE(jz4740_freq_cpu_divs); i++) {
		unsigned int freq = pll_rate / jz4740_freq_cpu_divs[i];
		if (freq < HCLK_MIN) break;
		table[i].index = i;
		table[i].frequency = freq;
	}
	table[i].index = i;
	table[i].frequency = CPUFREQ_TABLE_END;

	policy->min = table[i - 1].frequency;
	policy->max = table[0].frequency;

#ifdef CONFIG_CPU_FREQ_STAT_DETAILS
	cpufreq_frequency_table_get_attr(table, policy->cpu);
#endif
}

static unsigned int jz4740_freq_get(unsigned int cpu)
{
	return clk_get_rate(cclk) / 1000;
}

static int jz4740_freq_verify(struct cpufreq_policy *policy)
{
	unsigned int new_pll;

	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
				     policy->cpuinfo.max_freq);

	new_pll = clk_round_rate(pll, policy->max * 1000) / 1000;
	if (jz4740_freq_info.pll_rate != new_pll)
		jz4740_freq_fill_table(policy, new_pll);

	return 0;
}

static int jz4740_freq_target(struct cpufreq_policy *policy,
			  unsigned int target_freq,
			  unsigned int relation)
{
	struct cpufreq_frequency_table *table = &jz4740_freq_info.table[0];
	struct cpufreq_freqs freqs;
	unsigned int new_index = 0;
	unsigned int old_pll = clk_get_rate(pll) / 1000;
	unsigned int new_pll = jz4740_freq_info.pll_rate;
	int ret = 0;

	if (cpufreq_frequency_table_target(policy, table,
					   target_freq, relation, &new_index))
		return -EINVAL;
	freqs = (struct cpufreq_freqs) {
		.old = jz4740_freq_get(policy->cpu),
		.new = table[new_index].frequency,
		.cpu = policy->cpu,
		.flags = cpufreq_jz4740_driver.flags,
	};
	if (freqs.new != freqs.old || new_pll != old_pll) {
		unsigned int cdiv, hdiv, mdiv, pdiv;
		cdiv = jz4740_freq_cpu_divs[new_index];
		hdiv = (cdiv == 3 || cdiv == 6) ? cdiv * 2 : cdiv * 3;
		while (new_pll < HCLK_MIN * hdiv)
			hdiv -= cdiv;
		mdiv = hdiv;
		if (new_pll > MCLK_MAX * mdiv) {
			/* 4,4 performs better than 3,6 */
			if (new_pll > MCLK_MAX * 4)
				mdiv *= 2;
			else
				hdiv = mdiv = cdiv * 4;
		}
		pdiv = mdiv;
		dprintk(KERN_INFO "%s: cclk %p, setting from %d to %d, "
			"dividers %d, %d, %d, %d\n",
			__FUNCTION__, cclk, freqs.old, freqs.new,
			cdiv, hdiv, mdiv, pdiv);
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
		ret = clk_main_set_dividers(new_pll == old_pll,
					    cdiv, hdiv, mdiv, pdiv);
		if (ret) {
			dprintk(KERN_INFO "failed to set dividers\n");
		} else if (new_pll != old_pll) {
			dprintk(KERN_INFO "%s: pll %p, setting from %d to %d\n",
				__FUNCTION__, pll, old_pll, new_pll);
			ret = clk_set_rate(pll, new_pll * 1000);
		}
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}

	return ret;
}

static int jz4740_cpufreq_driver_init(struct cpufreq_policy *policy)
{
	int ret;

	dprintk(KERN_INFO "Jz4740 cpufreq driver\n");

	if (policy->cpu != 0)
		return -EINVAL;

	pll = clk_get(NULL, "pll");
	if (IS_ERR(pll)) {
		ret = PTR_ERR(pll);
		goto err_exit;
	}

	cclk = clk_get(NULL, "cclk");
	if (IS_ERR(cclk)) {
		ret = PTR_ERR(cclk);
		goto err_clk_put_pll;
	}

	policy->cpuinfo.min_freq = HCLK_MIN;
	policy->cpuinfo.max_freq = 500000;
	policy->cpuinfo.transition_latency = 100000; /* in nanoseconds */
	policy->cur = jz4740_freq_get(policy->cpu);
	policy->governor = CPUFREQ_DEFAULT_GOVERNOR;
	/* min and max are set by jz4740_freq_fill_table() */

	jz4740_freq_fill_table(policy, clk_get_rate(pll) / 1000 /* in kHz */);

	return 0;

err_clk_put_pll:
	clk_put(pll);
err_exit:
	return ret;
}

static struct cpufreq_driver cpufreq_jz4740_driver = {
	.init	= jz4740_cpufreq_driver_init,
	.verify	= jz4740_freq_verify,
	.target	= jz4740_freq_target,
	.get	= jz4740_freq_get,
	.name	= "jz4740",
};

static int __init jz4740_cpufreq_init(void)
{
	return cpufreq_register_driver(&cpufreq_jz4740_driver);
}

static void __exit jz4740_cpufreq_exit(void)
{
	cpufreq_unregister_driver(&cpufreq_jz4740_driver);
}

module_init(jz4740_cpufreq_init);
module_exit(jz4740_cpufreq_exit);

MODULE_AUTHOR("Ulrich Hecht <ulrich.hecht@gmail.com>, "
	      "Maarten ter Huurne <maarten@treewalker.org>");
MODULE_DESCRIPTION("cpufreq driver for Jz4740");
MODULE_LICENSE("GPL");
