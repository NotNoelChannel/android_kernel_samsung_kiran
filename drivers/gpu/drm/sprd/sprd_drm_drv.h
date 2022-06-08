/* sprd_drm_drv.h
 * Copyright (c) 2014 Spreadtrum Communications, Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _SPRD_DRM_DRV_H_
#define _SPRD_DRM_DRV_H_

#include <linux/module.h>
#include <linux/ion.h>
#include "drm.h"

#define MAX_CRTC	1

struct drm_device;
extern unsigned int drm_vblank_offdelay;

struct sprd_drm_ipp_private {
	struct device	*dev;
	struct list_head	event_list;
};

struct drm_sprd_file_private {
	struct sprd_drm_ipp_private	*ipp_priv;
	pid_t tgid;
};

/*
 * Spreadtrum drm private structure.
 */
struct sprd_drm_private {
	struct drm_device	*drm_dev;
	unsigned int		irq;
	unsigned long		vblank_interval_us;
	struct drm_fb_helper	*fb_helper;
	struct ion_client	*sprd_drm_ion_client;
	/* list head for new event to be added. */
	struct list_head	pageflip_event_list;
	void __iomem *regs;
	size_t reg_size;

	/*
	 * created crtc object would be contained at this array and
	 * this array is used to be aware of which crtc did it request vblank.
	 */
	struct drm_crtc		*crtc[MAX_CRTC];
	u32 dpms[MAX_CRTC];
	struct notifier_block	nb_ctrl;
	atomic_t vbl_trg_cnt[MAX_CRTC];
	atomic_t cpuidle_counter;
	int	dbg_cnt;
};

/*
 * Spreadtrum drm sub driver structure.
 *
 * @list: sub driver has its own list object to register to sprd drm driver.
 * @dev: pointer to device object for subdrv device driver.
 * @drm_dev: pointer to drm_device and this pointer would be set
 *	when sub driver calls sprd_drm_subdrv_register().
 * @manager: subdrv has its own manager to control a hardware appropriately
 *	and we can access a hardware drawing on this manager.
 * @probe: this callback would be called by sprd drm driver after
 *	subdrv is registered to it.
 * @remove: this callback is used to release resources created
 *	by probe callback.
 * @open: this would be called with drm device file open.
 * @close: this would be called with drm device file close.
 * @encoder: encoder object owned by this sub driver.
 * @connector: connector object owned by this sub driver.
 */
struct sprd_drm_subdrv {
	struct list_head list;
	struct device *dev;
	struct drm_device *drm_dev;

	int (*probe)(struct drm_device *drm_dev, struct device *dev);
	void (*remove)(struct drm_device *drm_dev, struct device *dev);
	int (*open)(struct drm_device *drm_dev, struct device *dev,
			struct drm_file *file);
	void (*close)(struct drm_device *drm_dev, struct device *dev,
			struct drm_file *file);
};

/*
 * this function calls a probe callback registered to sub driver list and
 * create its own encoder and connector and then set drm_device object
 * to global one.
 */
int sprd_drm_device_register(struct drm_device *dev);
/*
 * this function calls a remove callback registered to sub driver list and
 * destroy its own encoder and connetor.
 */
int sprd_drm_device_unregister(struct drm_device *dev);

/*
 * this function would be called by sub drivers such as display controller
 * or hdmi driver to register this sub driver object to sprd drm driver
 * and when a sub driver is registered to sprd drm driver a probe callback
 * of the sub driver is called and creates its own encoder and connector.
 */
int sprd_drm_subdrv_register(struct sprd_drm_subdrv *drm_subdrv);

/* this function removes subdrv list from sprd drm driver */
int sprd_drm_subdrv_unregister(struct sprd_drm_subdrv *drm_subdrv);

int sprd_drm_subdrv_open(struct drm_device *dev, struct drm_file *file);
void sprd_drm_subdrv_close(struct drm_device *dev, struct drm_file *file);

/*
 * this function registers sprd drm ipp platform device.
 */
int sprd_platform_device_ipp_register(void);

/*
 * this function unregisters sprd drm ipp platform device if it exists.
 */
void sprd_platform_device_ipp_unregister(void);

extern struct platform_driver gsp_driver;
extern struct platform_driver ipp_driver;
#endif
