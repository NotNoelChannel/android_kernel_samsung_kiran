/*
 *  Dynamic Binary Instrumentation Module based on KProbes
 *  energy/energy_mod.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Copyright (C) Samsung Electronics, 2013
 *
 * 2013         Vyacheslav Cherkashin <v.cherkashin@samsung.com>
 *
 */


#include <linux/module.h>
#include "energy.h"
#include "debugfs_energy.h"


static int __init swap_energy_init(void)
{
	int ret;

	ret = init_debugfs_energy();
	if (ret) {
		printk("Cannot init debugfs for energy\n");
		return ret;
	}

	ret = energy_init();
	if (ret) {
		printk("Cannot init energy\n");
		exit_debugfs_energy();
	}

	return ret;
}

static void __exit swap_energy_exit(void)
{
	energy_uninit();
	exit_debugfs_energy();
}

module_init(swap_energy_init);
module_exit(swap_energy_exit);

MODULE_LICENSE("GPL");
