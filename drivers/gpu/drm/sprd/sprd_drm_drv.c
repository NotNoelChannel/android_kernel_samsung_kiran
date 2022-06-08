/*
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

#include "drmP.h"
#include "drm.h"

#include <drm/sprd_drm.h>
#ifdef CONFIG_OF
#include <linux/of_irq.h>
#endif

#include "sprd_drm_drv.h"
#include "sprd_drm_gem.h"
#ifdef CONFIG_DRM_SPRD_DMABUF
#include "sprd_drm_dmabuf.h"
#endif
#include "sprd_drm_irq.h"
#include "sprd_drm_ipp.h"

#include <video/ion_sprd.h>
#include "sprdfb.h"
#include <mach/cpuidle.h>

#define DRIVER_NAME	"sprd"
#define DRIVER_DESC	"SPRD SoC DRM"
#define DRIVER_DATE	"2014"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

#define VBLANK_OFF_DELAY	500

struct sprd_drm_gem_info_data {
	struct drm_file *filp;
	struct seq_file *m;
};

static int sprd_drm_gem_one_info(int id, void *ptr, void *data)
{
	struct drm_gem_object *obj = (struct drm_gem_object *)ptr;
	struct sprd_drm_gem_info_data *gem_info_data = data;
	struct pid *pid = gem_info_data->filp->pid;
	struct drm_sprd_file_private *file_priv =
			gem_info_data->filp->driver_priv;
	struct sprd_drm_gem_obj *sprd_gem = to_sprd_gem_obj(obj);
	struct sprd_drm_gem_buf *buf = sprd_gem->buffer;

	seq_printf(gem_info_data->m,
			"%5d\t%5d\t%4d\t%4d\t\t%4d\t0x%08lx\t0x%x\t%4d\t%4d\t\t"
			"%4d\t0x%p\n",
				pid_nr(pid),
				file_priv->tgid,
				id,
				atomic_read(&obj->refcount.refcount),
				atomic_read(&obj->handle_count),
				sprd_gem->size,
				sprd_gem->flags,
				buf->pfnmap,
				obj->export_dma_buf ? 1 : 0,
				obj->import_attach ? 1 : 0,
				obj);
	return 0;
}

static int sprd_drm_gem_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *)m->private;
	struct drm_device *drm_dev = node->minor->dev;
	struct sprd_drm_gem_info_data gem_info_data;

	gem_info_data.m = m;
	seq_puts(gem_info_data.m,
			"pid\ttgid\thandle\trefcount\thcount\tsize\t\tflags\t"
			"pfnmap\texport_to_fd\timport_from_fd\tobj_addr\n");
	mutex_lock(&drm_dev->struct_mutex);
	list_for_each_entry(gem_info_data.filp, &drm_dev->filelist, lhead) {
		spin_lock(&gem_info_data.filp->table_lock);
		idr_for_each(&gem_info_data.filp->object_idr,
			sprd_drm_gem_one_info, &gem_info_data);
		spin_unlock(&gem_info_data.filp->table_lock);
	}
	mutex_unlock(&drm_dev->struct_mutex);
	return 0;
}

static struct drm_info_list sprd_drm_debugfs_list[] = {
	{"gem_info", sprd_drm_gem_info, DRIVER_GEM},
};
#define SPRD_DRM_DEBUGFS_ENTRIES ARRAY_SIZE(sprd_drm_debugfs_list)

static struct notifier_block sprd_drm_cpu_idle_notifier = {
	.notifier_call = sprd_drm_cpuidle_notify,
};

static int sprd_drm_load(struct drm_device *dev, unsigned long flags)
{
	struct sprd_drm_private *private;
	struct platform_device *pdev;
#ifdef CONFIG_OF
	struct resource *res;
#endif
	struct drm_minor *minor;
	int ret;

	minor = dev->primary;

	DRM_DEBUG_DRIVER("%s\n", __FILE__);

	pdev = dev->driver->kdriver.platform_device;
	if (!pdev) {
		DRM_ERROR("failed to get platform device.\n");
		return -EFAULT;
	}

	private = devm_kzalloc(&pdev->dev, sizeof(*private), GFP_KERNEL);
	if (!private) {
		DRM_ERROR("failed to alloc dev private data.\n");
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&private->pageflip_event_list);
	dev->dev_private = (void *)private;

	private->sprd_drm_ion_client = sprd_ion_client_create("drm");
	if (!private->sprd_drm_ion_client) {
		DRM_ERROR("Unable to create ION client\n");
		return -ENOMEM;
	}

#ifdef CONFIG_OF
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sprdfb");
	if (!res) {
		DRM_ERROR("failed to get DISPC base address\n");
		return -ENOMEM;
	}

	private->reg_size = resource_size(res);
	private->regs = devm_ioremap(&pdev->dev, res->start,
					private->reg_size);
	if (unlikely(!private->regs)) {
		DRM_ERROR("failed to map DISPC base\n");
		return -ENOMEM;
	}

	private->irq = platform_get_irq_byname(pdev, "sprd_drm_irq");
#else
	private->irq = platform_get_irq(pdev, 1);
#endif
	if (private->irq <= 0)
		DRM_ERROR("interrupt not registered\n");

	private->drm_dev = dev;

	private->nb_ctrl.notifier_call = sprd_drm_notifier_ctrl;
	if (sprdfb_nb_register(&private->nb_ctrl))
		DRM_ERROR("could not register sprd_fb notify callback\n");

	drm_mode_config_init(dev);
	ret = drm_vblank_init(dev, MAX_CRTC);
	if (ret)
		goto err;

	/*
	 * probe sub drivers such as display controller and hdmi driver,
	 * that were registered at probe() of platform driver
	 * to the sub driver and create encoder and connector for them.
	 */
	ret = sprd_drm_device_register(dev);
	if (ret)
		goto err_vblank;

	dev->vblank_disable_allowed = 1;
	drm_vblank_offdelay = VBLANK_OFF_DELAY;
	minor = dev->primary;
	ret = drm_debugfs_create_files(sprd_drm_debugfs_list,
			SPRD_DRM_DEBUGFS_ENTRIES,
			minor->debugfs_root, minor);
	if (ret)
		DRM_DEBUG_DRIVER("failed to create sprd-drm debugfs.\n");

	if (register_sc_cpuidle_notifier(&sprd_drm_cpu_idle_notifier))
		DRM_ERROR("failed to setup light sleep notifier\n");

	return 0;

err_vblank:
	drm_vblank_cleanup(dev);
err:
	drm_mode_config_cleanup(dev);
	kfree(private);

	return ret;
}

static int sprd_drm_unload(struct drm_device *dev)
{
	struct sprd_drm_private *private =
		(struct sprd_drm_private *)dev->dev_private;

	DRM_DEBUG_DRIVER("%s\n", __FILE__);

	if (unregister_sc_cpuidle_notifier(&sprd_drm_cpu_idle_notifier))
		DRM_ERROR("failed to unset light sleep notifier\n");

	sprdfb_nb_unregister(&private->nb_ctrl);

	sprd_drm_device_unregister(dev);
	drm_vblank_cleanup(dev);
	drm_mode_config_cleanup(dev);
	kfree(private);

	private = NULL;

	drm_debugfs_remove_files(sprd_drm_debugfs_list,
		SPRD_DRM_DEBUGFS_ENTRIES, dev->primary);

	return 0;
}

static int sprd_drm_open(struct drm_device *dev, struct drm_file *file)
{
	struct drm_sprd_file_private *file_priv;

	DRM_DEBUG_DRIVER("%s\n", __FILE__);

	file_priv = kzalloc(sizeof(*file_priv), GFP_KERNEL);
	if (!file_priv)
		return -ENOMEM;

	file_priv->tgid = task_tgid_nr(current);
	file->driver_priv = file_priv;

	return sprd_drm_subdrv_open(dev, file);
}

static void sprd_drm_preclose(struct drm_device *dev,
					struct drm_file *file)
{
	struct sprd_drm_private *private = dev->dev_private;
	struct drm_pending_vblank_event *e, *t;
	unsigned long flags;

	DRM_DEBUG_DRIVER("%s\n", __FILE__);

	/* release events of current file */
	spin_lock_irqsave(&dev->event_lock, flags);
	list_for_each_entry_safe(e, t, &private->pageflip_event_list,
			base.link) {
		if (e->base.file_priv == file) {
			list_del(&e->base.link);
			e->base.destroy(&e->base);
		}
	}
	spin_unlock_irqrestore(&dev->event_lock, flags);

	sprd_drm_subdrv_close(dev, file);
}

static void sprd_drm_postclose(struct drm_device *dev, struct drm_file *file)
{
	DRM_DEBUG_DRIVER("%s\n", __FILE__);

	if (!file->driver_priv)
		return;

	kfree(file->driver_priv);
	file->driver_priv = NULL;
}

static void sprd_drm_lastclose(struct drm_device *dev)
{
	DRM_DEBUG_DRIVER("%s\n", __FILE__);
}

static void
sprd_drm_irq_preinstall(struct drm_device *dev)
{
	DRM_DEBUG("%s %d\n", __func__, __LINE__);
	dev->irq_enabled = 0;
	return;
}

static int
sprd_drm_irq_postinstall(struct drm_device *dev)
{
	struct sprd_drm_private *private = dev->dev_private;
	int i;

	DRM_DEBUG("%s %d\n", __func__, __LINE__);

	for (i = 0; i < MAX_CRTC; i++)
		atomic_set(&private->vbl_trg_cnt[i], 0);
	atomic_set(&private->cpuidle_counter, 0);

	dev->irq_enabled = 1;
	return 0;
}

static void
sprd_drm_irq_uninstall(struct drm_device *dev)
{
	struct sprd_drm_private *private = dev->dev_private;
	int i;

	DRM_DEBUG("%s %d\n", __func__, __LINE__);

	for (i = 0; i < MAX_CRTC; i++)
		atomic_set(&private->vbl_trg_cnt[i], 0);
	atomic_set(&private->cpuidle_counter, 0);

	dev->irq_enabled = 0;
	return;
}

static const struct vm_operations_struct sprd_drm_gem_vm_ops = {
	.fault = sprd_drm_gem_fault,
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static struct drm_ioctl_desc sprd_ioctls[] = {
	DRM_IOCTL_DEF_DRV(SPRD_GEM_CREATE, sprd_drm_gem_create_ioctl,
			DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(SPRD_GEM_MAP_OFFSET,
			sprd_drm_gem_map_offset_ioctl, DRM_UNLOCKED |
			DRM_AUTH),
	DRM_IOCTL_DEF_DRV(SPRD_GEM_MMAP,
			sprd_drm_gem_mmap_ioctl, DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(SPRD_GEM_MMAP_IOMMU,
			sprd_drm_gem_mmap_iommu_ioctl, DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(SPRD_GEM_UNMAP_IOMMU, sprd_drm_gem_unmap_iommu_ioctl,
			DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(SPRD_GEM_GET,
			sprd_drm_gem_get_ioctl, DRM_UNLOCKED),
	DRM_IOCTL_DEF_DRV(SPRD_GEM_LOCK_HANDLE,
			sprd_gem_lock_handle_ioctl, DRM_UNLOCKED),
	DRM_IOCTL_DEF_DRV(SPRD_GEM_UNLOCK_HANDLE,
			sprd_gem_unlock_handle_ioctl, DRM_UNLOCKED),
	DRM_IOCTL_DEF_DRV(SPRD_GEM_CACHE_OP,
			sprd_gem_cache_op_ioctl, DRM_UNLOCKED),
	DRM_IOCTL_DEF_DRV(SPRD_GEM_INDEX_CREATE, sprd_drm_gem_create_index_ioctl,
			DRM_UNLOCKED | DRM_AUTH),
#ifdef CONFIG_DRM_SPRD_IPP
	DRM_IOCTL_DEF_DRV(SPRD_IPP_GET_PROPERTY,
			sprd_drm_ipp_get_property, DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(SPRD_IPP_SET_PROPERTY,
			sprd_drm_ipp_set_property, DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(SPRD_IPP_QUEUE_BUF,
			sprd_drm_ipp_queue_buf, DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(SPRD_IPP_CMD_CTRL,
			sprd_drm_ipp_cmd_ctrl, DRM_UNLOCKED | DRM_AUTH),
#endif
};

static const struct file_operations sprd_drm_driver_fops = {
	.owner		= THIS_MODULE,
	.open		= drm_open,
	.mmap		= sprd_drm_gem_mmap,
	.poll		= drm_poll,
	.read		= drm_read,
	.unlocked_ioctl	= drm_ioctl,
	.release	= drm_release,
};

static struct drm_driver sprd_drm_driver = {
	.driver_features	= DRIVER_HAVE_IRQ | DRIVER_IRQ_SHARED |
					DRIVER_GEM | DRIVER_PRIME,
	.load			= sprd_drm_load,
	.unload			= sprd_drm_unload,
	.open			= sprd_drm_open,
	.preclose		= sprd_drm_preclose,
	.lastclose		= sprd_drm_lastclose,
	.postclose		= sprd_drm_postclose,
	.get_vblank_counter	= sprd_drm_get_vblank_counter,
	.prepare_vblank		= sprd_prepare_vblank,
	.enable_vblank		= sprd_enable_vblank,
	.disable_vblank		= sprd_disable_vblank,
	.gem_init_object	= sprd_drm_gem_init_object,
	.gem_free_object	= sprd_drm_gem_free_object,
	.gem_vm_ops		= &sprd_drm_gem_vm_ops,
	.dumb_create		= sprd_drm_gem_dumb_create,
	.dumb_map_offset	= sprd_drm_gem_dumb_map_offset,
	.dumb_destroy		= sprd_drm_gem_dumb_destroy,
	.prime_handle_to_fd	= sprd_drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= sprd_drm_gem_prime_fd_to_handle,
#ifdef CONFIG_DRM_SPRD_DMABUF
	.gem_prime_export	= sprd_dmabuf_prime_export,
	.gem_prime_import	= sprd_dmabuf_prime_import,
#endif
	.ioctls			= sprd_ioctls,
	.fops			= &sprd_drm_driver_fops,
	.irq_handler		= sprd_drm_irq_handler,
	.irq_preinstall		= sprd_drm_irq_preinstall,
	.irq_postinstall	= sprd_drm_irq_postinstall,
	.irq_uninstall		= sprd_drm_irq_uninstall,
	.name	= DRIVER_NAME,
	.desc	= DRIVER_DESC,
	.date	= DRIVER_DATE,
	.major	= DRIVER_MAJOR,
	.minor	= DRIVER_MINOR,
};

static int sprd_drm_platform_probe(struct platform_device *pdev)
{
	DRM_DEBUG_DRIVER("%s\n", __FILE__);

	sprd_drm_driver.num_ioctls = DRM_ARRAY_SIZE(sprd_ioctls);
	sprd_drm_driver.kdriver.platform_device = pdev;

	return drm_platform_init(&sprd_drm_driver, pdev);
}

static int sprd_drm_platform_remove(struct platform_device *pdev)
{
	DRM_DEBUG_DRIVER("%s\n", __FILE__);

	drm_platform_exit(&sprd_drm_driver, pdev);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sprd_drm_dt_match[] = {
	{ .compatible = "sprd,sprd_drm",},
	{}
};
MODULE_DEVICE_TABLE(of, sprd_drm_dt_match);
#endif

static struct platform_driver sprd_drm_platform_driver = {
	.probe		= sprd_drm_platform_probe,
	.remove		= sprd_drm_platform_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "sprd-drm",
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(sprd_drm_dt_match),
#endif
	},
};

static int __init sprd_drm_init(void)
{
	int ret;

	DRM_DEBUG_DRIVER("%s\n", __FILE__);

#ifdef CONFIG_DRM_SPRD_GSP
	ret = platform_driver_register(&gsp_driver);
	if (ret < 0)
		goto out_gsp;
#endif

#ifdef CONFIG_DRM_SPRD_IPP
	ret = platform_driver_register(&ipp_driver);
	if (ret < 0)
		goto out_ipp;
#endif

	ret = platform_driver_register(&sprd_drm_platform_driver);
	if (ret < 0)
		goto out_drm;

	return 0;

out_drm:
#ifdef CONFIG_DRM_SPRD_IPP
	platform_driver_unregister(&ipp_driver);
out_ipp:
#endif

#ifdef CONFIG_DRM_SPRD_GSP
	platform_driver_unregister(&gsp_driver);
out_gsp:
#endif

	return ret;
}

static void __exit sprd_drm_exit(void)
{
	DRM_DEBUG_DRIVER("%s\n", __FILE__);

	platform_driver_unregister(&sprd_drm_platform_driver);

#ifdef CONFIG_DRM_SPRD_IPP
	platform_driver_unregister(&ipp_driver);
#endif

#ifdef CONFIG_DRM_SPRD_GSP
	platform_driver_unregister(&gsp_driver);
#endif
}

//module_init(sprd_drm_init);
late_initcall(sprd_drm_init);
module_exit(sprd_drm_exit);

MODULE_AUTHOR("frank.dong@spreadtrum.com");
MODULE_DESCRIPTION("Spreadtrum SoC DRM Driver");
MODULE_LICENSE("GPL");
