/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _SPRD_DRM_IRQ_H_
#define _SPRD_DRM_IRQ_H_
#include "drmP.h"
#include "sprd_drm.h"
#include <mach/irqs.h>
#include <linux/types.h>

irqreturn_t sprd_drm_irq_handler(DRM_IRQ_ARGS);
int sprd_drm_irq_init(struct drm_device *dev, unsigned long flags);
int sprd_drm_irq_uninit(struct drm_device *dev);
u32 sprd_drm_get_vblank_counter(struct drm_device *dev, int crtc);
int sprd_prepare_vblank(struct drm_device *dev, int crtc, struct drm_file *file_priv);
int sprd_enable_vblank(struct drm_device *dev, int crtc);
void sprd_disable_vblank(struct drm_device *dev, int crtc);
int sprd_drm_notifier_ctrl(struct notifier_block *this,
			unsigned long cmd, void *_data);
int sprd_drm_cpuidle_notify(struct notifier_block *nb, unsigned long event, void *dummy);

#endif/* _SPRD_DRM_IRQ_H_ */

