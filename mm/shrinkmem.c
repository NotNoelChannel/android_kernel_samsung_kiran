
/*
 * shrinkmem.c (Aggressive Memory Shrinker)
 * It is a driver to recover memory system-wide to its maximum.
 * It is based on the hibernation memory claim techniques.
 * This requires CONFIG_HIBERNATION to be enabled in your system.
 * Also shrink_all_memory() to be exported.
 *
 * Copyright (C) 2014 Samsung Electronics Co., Ltd.
 * Author: Pintu Kumar <pintu.k@samsung.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include<linux/module.h>
#include<linux/errno.h>
#include<linux/kernel.h>
#include<linux/string.h>
#include<linux/swap.h>
#include <linux/shrinkmem.h>


static DEFINE_MUTEX(shk_lock);
int sysctl_shrink_memory;

/* This is the entry point for shrinking all memory via /proc/sys/vm */
int sysctl_shrinkmem_handler(struct ctl_table *table, int write,
			void __user *buffer, size_t *length, loff_t *ppos)
{
	proc_dointvec(table, write, buffer, length, ppos);
	if (write) {
		unsigned long npages = 0;
		mutex_lock(&shk_lock);
		/* Call shrink all memory with atleast half of total
		available RAM */
		npages = shrink_all_memory(totalram_pages/2);
		pr_info("[shrinkmem]: number of pages reclaimed: %lu\n",
							npages);
		mutex_unlock(&shk_lock);
	}
	return 0;
}


MODULE_LICENSE("GPL");
MODULE_AUTHOR("PINTU KUMAR");
MODULE_DESCRIPTION("shrinkmem handler!");
