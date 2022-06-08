/* include/linux/partialsuspend_slp.h
 *
 * Copyright (C) 2012 SAMSUNG, Inc.
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

#ifndef _LINUX_PARTIALSUSPEND_H
#define _LINUX_PARTIALSUSPEND_H

#include <linux/list.h>

/* The partial_suspend structure defines suspend and resume hooks to be called
 * when the user visible sleep state of the system changes, and a level to
 * control the order. They can be used to turn off devices that are not used for wakeup.
 * Suspend handlers are called in low to high level order, resume handlers are
 * called in the opposite order. If, when calling register_pre_suspend,
 * the suspend handlers have already been called without a matching call to the
 * resume handlers, the suspend handler will be called directly from
 * register_pre_suspend. This direct call can violate the normal level order.
 */
enum {
	PRE_SUSPEND_LEVEL_BLANK_SCREEN = 50,
	PRE_SUSPEND_LEVEL_STOP_DRAWING = 100,
	PRE_SUSPEND_LEVEL_DISABLE_FB = 150,
};
struct pre_suspend {
	struct list_head link;
	int level;
	void (*suspend)(struct pre_suspend *h);
	void (*resume)(struct pre_suspend *h);
};

struct suspend_status_struct {
	char req;
	char ing;
	char done;
};

void register_pre_suspend(struct pre_suspend *handler);
void unregister_pre_suspend(struct pre_suspend *handler);
#endif
