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


#include "sprd_drm_irq.h"
#include "sprd_drm_drv.h"
#include <mach/hardware.h>
#include "sprdfb.h"
#include <mach/cpuidle.h>

extern struct drm_device *sprd_drm_dev;

#define SPRD_DISPC_BASE				SPRD_LCDC_BASE
#define IRQ_DISPC_INT				IRQ_DISPC0_INT
#define DISPC_INT_EN				(0x0070)
#define DISPC_INT_CLR				(0x0074)
#define DISPC_INT_STATUS			(0x0078)
#define DISPC_INT_DONE_MASK          BIT(0)
#define DISPC_INT_TE_MASK            BIT(1)
#define DISPC_INT_ERR_MASK           BIT(2)
#define DISPC_INT_EDPI_TE_MASK       BIT(3)
#define DISPC_INT_UPDATE_DONE_MASK   BIT(4)
#define DISPC_INT_DPI_VSYNC_MASK     BIT(5)
#define LIMIT_VBL_REQ	60

#ifdef CONFIG_FB_SCX30G
#define DISPC_INT_HWVSYNC DISPC_INT_DPI_VSYNC_MASK
#else
#define DISPC_INT_HWVSYNC DISPC_INT_DONE_MASK
#endif

int sci_write(u32 reg, u32 val, u32 msk)
{
	__raw_writel((__raw_readl(reg) & ~msk) | val, reg);
	return 0;
}

static inline uint32_t dispc_read(uint32_t reg)
{
	return __raw_readl(SPRD_DISPC_BASE + reg) & 0xffffffff;
}

#define dispc_write(value, reg)	\
	do {	\
		sci_write((SPRD_DISPC_BASE + reg), value, 0xffffffff);	\
		DRM_DEBUG("dispc_write:reg[0x%x]val[0x%08x][%d %s]\n", \
				(int)reg, (int)value, __LINE__,  __func__);	\
	} while(0)

#define dispc_set_bits(bits, reg)	\
	do {	\
		uint32_t value = dispc_read(reg) | bits;	\
		sci_write((SPRD_DISPC_BASE + reg), value, 0xffffffff);	\
		DRM_DEBUG("dispc_set_bits:reg[0x%x]val[0x%08x][%d %s]\n", \
				(int)reg, (int)value, __LINE__,  __func__);	\
	} while(0)

#define dispc_clear_bits(bits, reg)	\
	do {	\
		uint32_t value = dispc_read(reg) & ~bits;	\
		sci_write((SPRD_DISPC_BASE + reg), value, 0xffffffff);	\
		DRM_DEBUG("dispc_clear_bits:reg[0x%x]val[0x%08x][%d %s]\n", \
				(int)reg, (int)value, __LINE__,  __func__);	\
	} while(0)

int sprd_drm_notifier_ctrl(struct notifier_block *this,
			unsigned long cmd, void *_data)
{
	struct drm_device *dev = sprd_drm_dev;
	struct sprd_drm_private *dev_priv;

	struct sprdfb_nb_event *event =
		(struct sprdfb_nb_event *)_data;
	int crtc = 0, ret = NOTIFY_DONE;
	unsigned long flags;

	if (!dev) {
		DRM_ERROR("failed to get drm_dev:cmd[%d]crtc[%d]\n",
			(int)cmd, crtc);
		ret = NOTIFY_BAD;
		goto out;
	}

	dev_priv = dev->dev_private;
	dev_priv->dbg_cnt = 2;

	if (atomic_read(&dev->vblank_refcount[crtc]) ||
		dev->vblank_enabled[crtc])
		DRM_INFO("[notifier_%d]cmd[%d]en[%d]r[%d]t[%d]\n",
			crtc, (int)cmd, dev->vblank_enabled[crtc],
			atomic_read(&dev->vblank_refcount[crtc]),
			atomic_read(&dev_priv->vbl_trg_cnt[crtc]));

	switch (cmd) {
	case SPRDFB_SET_DPMS:
		spin_lock_irqsave(&dev->vbl_lock, flags);

		if (dev->vblank_enabled[crtc]) {
			if (atomic_read(&dev->vblank_refcount[crtc]) > 0) {
				drm_handle_vblank(dev, crtc);
				DRM_INFO("[wake_vbl_%d]r[%d]t[%d]\n", crtc,
					atomic_read(&dev->vblank_refcount[crtc]),
					atomic_read(&dev_priv->vbl_trg_cnt[crtc]));
			}
		}

		spin_unlock_irqrestore(&dev->vbl_lock, flags);

		switch ((int)event->data) {
		case SPRDFB_DPMS_ON:
			dev_priv->dpms[crtc] = SPRDFB_DPMS_ON;
			break;
		case SPRDFB_DPMS_OFF:
			dev_priv->dpms[crtc] = SPRDFB_DPMS_OFF;

			if (dev->vblank_enabled[crtc])
				drm_vblank_off(dev, crtc );
			break;
		default:
			DRM_ERROR("invalid dpms[%d]\n", (int)event->data);
			ret = NOTIFY_BAD;
			break;
		}
		break;
	default:
		DRM_ERROR("invalid command[%d]\n", (int)cmd);
		ret = NOTIFY_BAD;
		break;
	}

	DRM_INFO("[notifier_%d]cmd[%d]en[%d]r[%d]t[%d][dpms_%s]ret[%d]\n",
		crtc, (int)cmd, dev->vblank_enabled[crtc],
		atomic_read(&dev->vblank_refcount[crtc]),
		atomic_read(&dev_priv->vbl_trg_cnt[crtc]),
		dev_priv->dpms[crtc] == SPRDFB_DPMS_ON ? "on" : "off", ret);
out:
	return ret;
}

/* FIXME:!! when we not clear the register in dispc, phone doesn't bootup */
u32 dispc_int_status;

irqreturn_t sprd_drm_irq_handler(DRM_IRQ_ARGS)
{
	struct drm_device *dev = (struct drm_device *) arg;
	struct sprd_drm_private *dev_priv = dev->dev_private;
	u32 isr, crtc = 0;
	unsigned long flags;

/* FIXME:!! when we not clear the register in dispc, phone doesn't bootup */
#if 0
	isr = dispc_read(DISPC_INT_STATUS);
#endif
	isr = dispc_int_status;

	if (isr == 0)
		goto irq_done;

/* FIXME:!! when we not clear the register in dispc, phone doesn't bootup */
#if 0
	mask = dispc_read(DISPC_INT_EN);
	dispc_write(mask, DISPC_INT_CLR);
#endif

	spin_lock_irqsave(&dev->vbl_lock, flags);

	if (isr & DISPC_INT_HWVSYNC || isr & DISPC_INT_TE_MASK) {
		dispc_int_status = 0;

		if (atomic_read(&dev->vblank_refcount[crtc]) > 0) {
			drm_handle_vblank(dev, crtc);

			if (dev_priv->dbg_cnt) {
				DRM_INFO("[wake_vbl_%d]r[%d]t[%d]\n", crtc,
					atomic_read(&dev->vblank_refcount[crtc]),
					atomic_read(&dev_priv->vbl_trg_cnt[crtc]));
				dev_priv->dbg_cnt--;
			}
		}
	}

	if (isr & DISPC_INT_UPDATE_DONE_MASK) {
		if (dev_priv->dbg_cnt)
			DRM_INFO("[done_%d]r[%d]t[%d]\n", crtc,
				atomic_read(&dev->vblank_refcount[crtc]),
				atomic_read(&dev_priv->vbl_trg_cnt[crtc]));

		atomic_set(&dev_priv->vbl_trg_cnt[crtc], 0);
	}

	spin_unlock_irqrestore(&dev->vbl_lock, flags);

irq_done:
	return IRQ_HANDLED;
}

int sprd_prepare_vblank(struct drm_device *dev, int crtc, struct drm_file *file_priv)
{
	struct sprd_drm_private *dev_priv = dev->dev_private;
	int limit = LIMIT_VBL_REQ;

	if (crtc > MAX_CRTC) {
		DRM_ERROR("crtc[%d]\n", crtc);
		return -EINVAL;
	}

	if (!file_priv->is_master && !dev->vblank_enabled[crtc]) {
		DRM_DEBUG("[pre_vbl_%d]r[%d]t[%d]VBL_OFF\n", crtc,
			atomic_read(&dev->vblank_refcount[crtc]),
			atomic_read(&dev_priv->vbl_trg_cnt[crtc]));
		return -EACCES;
	}

	if (file_priv->is_master)
		atomic_set(&dev_priv->vbl_trg_cnt[crtc], 0);
	else
		atomic_inc(&dev_priv->vbl_trg_cnt[crtc]);

	if (dev_priv->dbg_cnt)
		DRM_DEBUG("[pre_vbl_%d]r[%d]t[%d]\n", crtc,
			atomic_read(&dev->vblank_refcount[crtc]),
			atomic_read(&dev_priv->vbl_trg_cnt[crtc]));

	if (atomic_read(&dev_priv->vbl_trg_cnt[crtc]) >= limit) {
		DRM_INFO("[limit_vbl_%d]r[%d]t[%d]\n",
			crtc, atomic_read(&dev->vblank_refcount[crtc]),
			atomic_read(&dev_priv->vbl_trg_cnt[crtc]));
		return -EACCES;
	}

	return 0;
}

int sprd_enable_vblank(struct drm_device *dev, int crtc)
{
	struct sprd_drm_private *dev_priv = dev->dev_private;

	if (crtc > MAX_CRTC) {
		DRM_ERROR("crtc[%d]\n", crtc);
		return -EINVAL;
	}

	if (dev_priv->dpms[crtc] != SPRDFB_DPMS_ON) {
		DRM_DEBUG("[on_vbl_%d]r[%d]t[%d]DPMS_OFF\n",
			crtc, atomic_read(&dev->vblank_refcount[crtc]),
			atomic_read(&dev_priv->vbl_trg_cnt[crtc]));
		return -EPERM;
	}

	DRM_INFO("[on_vbl_%d]r[%d]t[%d]\n", crtc,
		atomic_read(&dev->vblank_refcount[crtc]),
		atomic_read(&dev_priv->vbl_trg_cnt[crtc]));

	dev_priv->dbg_cnt = 2;

	return 0;
}

void sprd_disable_vblank(struct drm_device *dev, int crtc)
{
	struct sprd_drm_private *dev_priv = dev->dev_private;

	if (crtc > MAX_CRTC) {
		DRM_ERROR("crtc[%d]\n", crtc);
		return;
	}

	DRM_INFO("[off_vbl_%d]r[%d]t[%d]\n", crtc,
		atomic_read(&dev->vblank_refcount[crtc]),
		atomic_read(&dev_priv->vbl_trg_cnt[crtc]));
}

u32 sprd_drm_get_vblank_counter(struct drm_device *dev, int crtc)
{
	DRM_DEBUG("%s:crtc[%d]\n", __func__, crtc);

	if (crtc > MAX_CRTC) {
		DRM_ERROR("crtc[%d]\n", crtc);
		return -EINVAL;
	}

	return drm_vblank_count(dev,crtc);
}

int sprd_drm_cpuidle_notify(struct notifier_block *nb, unsigned long event, void *dummy)
{
	struct drm_device *dev = sprd_drm_dev;
	struct sprd_drm_private *dev_priv = dev->dev_private;
	int crtc = 0, ret = 0, vbl_ref, pended = 10;

	if (!dev) {
		DRM_ERROR("failed to get drm_dev\n");
		ret = NOTIFY_BAD;
		goto out;
	}

	if (event != SC_CPUIDLE_PREPARE){
		DRM_ERROR("invalid cpuidle notify type\n");
		goto out;
	}

	vbl_ref = atomic_read(&dev->vblank_refcount[crtc]);

	if (vbl_ref > 0 && atomic_read(&dev_priv->cpuidle_counter) < pended) {
		atomic_inc(&dev_priv->cpuidle_counter);
		ret = NOTIFY_BAD;
	} else {
		atomic_set(&dev_priv->cpuidle_counter, 0);
		if (vbl_ref > 0)
			DRM_INFO("%s:forced entering\n", "cpuidle");
		ret = NOTIFY_OK;
	}

out:
	DRM_DEBUG("%s:vbl_ref[%d]pending[%d]ret[%d]\n", __func__,
		vbl_ref, atomic_read(&dev_priv->cpuidle_counter), ret);

	return ret;
}
