/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#ifdef CONFIG_LCD_CLASS_DEVICE
#include <linux/platform_device.h>
#include <linux/lcd.h>
#endif
#include "sprdfb.h"
#include "sprdfb_panel.h"
#include "sprdfb_dispc_reg.h"
#include "sprdfb_lcdc_reg.h"

#ifdef CONFIG_MDNIE_LITE_TUNING
#include "mdnie/mdnie_lite_tuning.h"
static bool mdnie_init;
#endif

#ifdef CONFIG_FB_ESD_DETECT_BY_INT
#define SPRDFB_LCD_ESD_DETECT_GPIO 236
/*#define SPRDFB_LCD_ERR_FG_GPIO 105*/
#define SPRDFB_ESD_IRQ_EN_DELAY 120

enum esd_status {
	ESD_NONE, ESD_DETECTED,
};

typedef struct {
	struct work_struct uevent_notifier;
	struct delayed_work irq_enable;
#ifdef SPRDFB_LCD_ESD_DETECT_GPIO
	int esd_det_gpio;
	int esd_det_irq;
#endif
#ifdef SPRDFB_LCD_ERR_FG_GPIO
	int err_fg_gpio;
	int err_fg_irq;
#endif
	int status;
	int irq_enable_count;
	int notify_in_progress;
	struct sprdfb_device *fbdev;
	struct class *class;
	struct device *device;
} esd_context_t;

static esd_context_t esd_context;

static int sprdfb_panel_esd_detect_init(struct sprdfb_device *fbdev);
#ifdef SPRDFB_LCD_ESD_DETECT_GPIO
static irqreturn_t sprdfb_panel_esd_det_irq_handler(int irq, void *handle);
#endif
#ifdef SPRDFB_LCD_ERR_FG_GPIO
static irqreturn_t sprdfb_panel_err_fg_irq_handler(int irq, void *handle);
#endif
#endif

static LIST_HEAD(panel_list_main);/* for main_lcd*/
static LIST_HEAD(panel_list_sub);/* for sub_lcd */
static DEFINE_MUTEX(panel_mutex);

static uint32_t lcd_id_from_uboot = 0;
static uint32_t lcd_base_from_uboot = 0;

extern struct panel_if_ctrl sprdfb_mcu_ctrl;
extern struct panel_if_ctrl sprdfb_rgb_ctrl;
#ifndef CONFIG_FB_SCX15
extern struct panel_if_ctrl sprdfb_mipi_ctrl;
#endif
extern void sprdfb_panel_remove(struct sprdfb_device *dev);

#ifdef CONFIG_FB_SC8825
typedef struct {
	uint32_t reg;
	uint32_t val;
} panel_pinmap_t;

panel_pinmap_t panel_rstpin_map[] = {
	{REG_PIN_LCD_RSTN, BITS_PIN_DS(1)|BITS_PIN_AF(0)|BIT_PIN_NUL|BIT_PIN_SLP_WPU|BIT_PIN_SLP_OE},
	{REG_PIN_LCD_RSTN, BITS_PIN_DS(3)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_NUL|BIT_PIN_SLP_OE},
};

static void sprd_panel_set_rstn_prop(unsigned int if_slp)
{
	int i = 0;

	if (if_slp){
		panel_rstpin_map[0].val = sci_glb_read(CTL_PIN_BASE+REG_PIN_LCD_RSTN, 0xffffffff);
		i = 1;
	}else{
		i = 0;
	}

	sci_glb_write(CTL_PIN_BASE + panel_rstpin_map[i].reg,  panel_rstpin_map[i].val, 0xffffffff);
}
#endif

int lcd_panel_cabc_pwm_backlight = 0;

int is_lcd_attached(void)
{
	return (lcd_id_from_uboot > 0);
}
EXPORT_SYMBOL(is_lcd_attached);

static int __init lcd_id_get(char *str)
{
	if ((str != NULL) && (str[0] == 'I') && (str[1] == 'D')) {
		sscanf(&str[2], "%x", &lcd_id_from_uboot);
	}

	if (lcd_id_from_uboot == 0x55C090 || lcd_id_from_uboot == 0x554CC0) {
		lcd_panel_cabc_pwm_backlight = 1;
	}

	pr_info("lcd_panel_cabc_pwm_backlight = %d\n", lcd_panel_cabc_pwm_backlight);
	printk(KERN_INFO "sprdfb: [%s]LCD Panel ID from uboot: 0x%x\n", __FUNCTION__, lcd_id_from_uboot);
	return 1;
}
__setup("lcd_id=", lcd_id_get);
static int __init lcd_base_get(char *str)
{
	if (str != NULL) {
		sscanf(&str[0], "%x", &lcd_base_from_uboot);
	}
	printk(KERN_INFO "sprdfb: [%s]LCD Panel Base from uboot: 0x%x\n", __FUNCTION__, lcd_base_from_uboot);
	return 1;
}
__setup("lcd_base=", lcd_base_get);

static int32_t panel_reset_dispc(struct panel_spec *self)
{
        uint16_t timing1, timing2, timing3;

        if((NULL != self) && (0 != self->reset_timing.time1) &&
            (0 != self->reset_timing.time2) && (0 != self->reset_timing.time3)) {
            timing1 = self->reset_timing.time1;
            timing2 = self->reset_timing.time2;
            timing3 = self->reset_timing.time3;
        }else {
            timing1 = 20;
            timing2 = 20;
            timing3 = 130;
        }

	dispc_write(1, DISPC_RSTN);
	usleep_range(timing1*1000, timing1*1000+500);
	dispc_write(0, DISPC_RSTN);
	usleep_range(timing2*1000, timing2*1000+500);
	dispc_write(1, DISPC_RSTN);

	/* wait 130ms util the lcd is stable */
	usleep_range(timing3*1000, timing3*1000+500);

	return 0;
}

static int32_t panel_reset_lcdc(struct panel_spec *self)
{
	lcdc_write(0, LCM_RSTN);
	msleep(20);
	lcdc_write(1, LCM_RSTN);

	/* wait 10ms util the lcd is stable */
	msleep(20);
	return 0;
}

static int32_t panel_set_resetpin_dispc( uint32_t status)
{
	if(0 == status){
		dispc_write(0, DISPC_RSTN);
	}else{
		dispc_write(1, DISPC_RSTN);
	}
	return 0;
}

#ifdef CONFIG_FB_SC8825
static int32_t panel_set_resetpin_lcdc(uint32_t status)
{
	if(0 == status){
		lcdc_write(0, LCM_RSTN);
	}else{
		lcdc_write(1, LCM_RSTN);
	}
	return 0;
}
#endif

static int panel_reset(struct sprdfb_device *dev)
{
	if((NULL == dev) || (NULL == dev->panel)){
		printk(KERN_ERR "sprdfb: [%s]: Invalid param\n", __FUNCTION__);
		return -1;
	}

	pr_debug("sprdfb: [%s], enter\n",__FUNCTION__);

	//clk/data lane enter LP
	if(NULL != dev->panel->if_ctrl->panel_if_before_panel_reset){
		dev->panel->if_ctrl->panel_if_before_panel_reset(dev);
	}
	usleep_range(5000, 5500);

	//reset panel
	dev->panel->ops->panel_reset(dev->panel);

	return 0;
}

static int panel_sleep(struct sprdfb_device *dev)
{
	if((NULL == dev) || (NULL == dev->panel)){
		printk(KERN_ERR "sprdfb: [%s]: Invalid param\n", __FUNCTION__);
		return -1;
	}

	pr_debug("sprdfb: [%s], enter\n",__FUNCTION__);

	//send sleep cmd to lcd
	if (dev->panel->ops->panel_enter_sleep != NULL) {
		dev->panel->ops->panel_enter_sleep(dev->panel,1);
	}
	msleep(100);
	//clk/data lane enter LP
	if((NULL != dev->panel->if_ctrl->panel_if_before_panel_reset)
		&&(SPRDFB_PANEL_TYPE_MIPI == dev->panel->type))
	{
		dev->panel->if_ctrl->panel_if_before_panel_reset(dev);
	}
	return 0;
}

static void panel_set_resetpin(uint16_t dev_id,  uint32_t status, struct panel_spec *panel )
{
	pr_debug("sprdfb: [%s].\n",__FUNCTION__);

	/*panel set reset pin status*/
	if(SPRDFB_MAINLCD_ID == dev_id){
		panel_set_resetpin_dispc(status);
	}else{
	#ifdef CONFIG_FB_SC8825
		panel_set_resetpin_lcdc(status);
	#endif
	}
}


static int32_t panel_before_resume(struct sprdfb_device *dev)
{
#ifdef CONFIG_FB_SC8825
	/*restore the reset pin status*/
	sprd_panel_set_rstn_prop(0);
#endif
	/*restore  the reset pin to high*/
	panel_set_resetpin(dev->dev_id, 1, dev->panel);
	return 0;
}

static int32_t panel_after_suspend(struct sprdfb_device *dev)
{
	/*set the reset pin to low*/
	panel_set_resetpin(dev->dev_id, 0, dev->panel);
#ifdef CONFIG_FB_SC8825
	/*set the reset pin status and set */
	sprd_panel_set_rstn_prop(1);
#endif
	return 0;
}

static bool panel_check(struct panel_cfg *cfg)
{
	bool rval = true;

	if(NULL == cfg || NULL == cfg->panel){
		printk(KERN_ERR "sprdfb: [%s] :Invalid Param!\n", __FUNCTION__);
		return false;
	}

	pr_debug("sprdfb: [%s], dev_id = %d, lcd_id = 0x%x, type = %d\n",__FUNCTION__, cfg->dev_id, cfg->lcd_id, cfg->panel->type);

	switch(cfg->panel->type){
	case SPRDFB_PANEL_TYPE_MCU:
		cfg->panel->if_ctrl = &sprdfb_mcu_ctrl;
		break;
	case SPRDFB_PANEL_TYPE_RGB:
		cfg->panel->if_ctrl = &sprdfb_rgb_ctrl;
		break;
#ifndef CONFIG_FB_SCX15
	case SPRDFB_PANEL_TYPE_MIPI:
		cfg->panel->if_ctrl = &sprdfb_mipi_ctrl;
		break;
#endif
	default:
		printk("sprdfb: [%s]: erro panel type.(%d,%d, %d)",__FUNCTION__, cfg->dev_id, cfg->lcd_id, cfg->panel->type);
		cfg->panel->if_ctrl = NULL;
		break;
	};

	if(cfg->panel->if_ctrl->panel_if_check){
		rval = cfg->panel->if_ctrl->panel_if_check(cfg->panel);
	}
	return rval;
}

static int panel_mount(struct sprdfb_device *dev, struct panel_spec *panel)
{
	printk("sprdfb: [%s], dev_id = %d\n",__FUNCTION__, dev->dev_id);

	/* TODO: check whether the mode/res are supported */
	dev->panel = panel;

	if(NULL == dev->panel->ops->panel_reset){
		if(SPRDFB_MAINLCD_ID == dev->dev_id){
			dev->panel->ops->panel_reset = panel_reset_dispc;
		}else{
			dev->panel->ops->panel_reset = panel_reset_lcdc;
		}
	}

	panel->if_ctrl->panel_if_mount(dev);

#ifdef CONFIG_FB_ESD_DETECT_BY_INT
	sprdfb_panel_esd_detect_init(dev);
#endif

	return 0;
}


int panel_init(struct sprdfb_device *dev)
{
	if((NULL == dev) || (NULL == dev->panel)){
		printk(KERN_ERR "sprdfb: [%s]: Invalid param\n", __FUNCTION__);
		return -1;
	}

	pr_debug("sprdfb: [%s], dev_id= %d, type = %d\n",__FUNCTION__, dev->dev_id, dev->panel->type);

	if(!dev->panel->if_ctrl->panel_if_init(dev)){
		printk(KERN_ERR "sprdfb: [%s]: panel_if_init fail!\n", __FUNCTION__);
		return -1;
	}
#if defined(CONFIG_MDNIE_LITE_TUNING)
	/* Enable mdnie only for S6D7AA1 panel (MTP_ID = 0x55B810) */
	if (mdnie_init == false && lcd_id_from_uboot == 0x55b810) {
		pr_info("[%s] Initialize mdnie tuning!\n", __func__);
		init_mdnie_class();
		mdnie_lite_tuning_init(dev);
		mdnie_init = true;
	}
#endif

	return 0;
}

int panel_ready(struct sprdfb_device *dev)
{
	if((NULL == dev) || (NULL == dev->panel)){
		printk(KERN_ERR "sprdfb: [%s]: Invalid param\n", __FUNCTION__);
		return -1;
	}

	pr_debug("sprdfb: [%s], dev_id= %d, type = %d\n",__FUNCTION__, dev->dev_id, dev->panel->type);

	if(NULL != dev->panel->if_ctrl->panel_if_ready){
		dev->panel->if_ctrl->panel_if_ready(dev);
	}

	return 0;
}


static struct panel_spec *adapt_panel_from_uboot(uint16_t dev_id)
{
	struct panel_cfg *cfg;
	struct list_head *panel_list;

	pr_debug("sprdfb: [%s], dev_id = %d\n",__FUNCTION__, dev_id);

	if (lcd_id_from_uboot == 0) {
		printk("sprdfb: [%s]: Not got lcd id from uboot\n", __FUNCTION__);
		return NULL;
	}

	if(SPRDFB_MAINLCD_ID == dev_id){
		panel_list = &panel_list_main;
	}else{
		panel_list = &panel_list_sub;
	}

	list_for_each_entry(cfg, panel_list, list) {
		if(lcd_id_from_uboot == cfg->lcd_id) {
			printk(KERN_INFO "sprdfb: [%s]: LCD Panel 0x%x is attached!\n", __FUNCTION__,cfg->lcd_id);
			return cfg->panel;
		}
	}
	printk(KERN_ERR "sprdfb: [%s]: Failed to match LCD Panel from uboot!\n", __FUNCTION__);

	return NULL;
}

static struct panel_spec *adapt_panel_from_readid(struct sprdfb_device *dev)
{
	struct panel_cfg *cfg;
	struct panel_cfg *dummy_cfg = NULL;
	struct list_head *panel_list;
	int id;

	printk("sprdfb: [%s], dev_id = %d\n",__FUNCTION__, dev->dev_id);

	if(SPRDFB_MAINLCD_ID == dev->dev_id){
		panel_list = &panel_list_main;
	}else{
		panel_list = &panel_list_sub;
	}

	list_for_each_entry(cfg, panel_list, list) {
		if(0xFFFFFFFF == cfg->lcd_id){
			dummy_cfg = cfg;
			continue;
		}
		printk("sprdfb: [%s]: try panel 0x%x\n", __FUNCTION__, cfg->lcd_id);
		panel_mount(dev, cfg->panel);
#ifndef CONFIG_MACH_SPX15FPGA
		dev->ctrl->update_clk(dev);
#endif
		panel_init(dev);
		panel_reset(dev);
		id = dev->panel->ops->panel_readid(dev->panel);
		if(id == cfg->lcd_id) {
			pr_debug(KERN_INFO "sprdfb: [%s]: LCD Panel 0x%x is attached!\n", __FUNCTION__, cfg->lcd_id);
			dev->panel->ops->panel_init(dev->panel);
			panel_ready(dev);
			return cfg->panel;
		}
		sprdfb_panel_remove(dev);
	}
	if(dummy_cfg != NULL){
		printk("sprdfb: [%s]: Can't find read panel, Use dummy panel!\n", __FUNCTION__);
		panel_mount(dev, dummy_cfg->panel);
#ifndef CONFIG_MACH_SPX15FPGA
		dev->ctrl->update_clk(dev);
#endif
		panel_init(dev);
		panel_reset(dev);
		id = dev->panel->ops->panel_readid(dev->panel);
		if(id == dummy_cfg->lcd_id) {
			pr_debug(KERN_INFO "sprdfb: [%s]: LCD Panel 0x%x is attached!\n", __FUNCTION__, dummy_cfg->lcd_id);
			dev->panel->ops->panel_init(dev->panel);
			panel_ready(dev);
			return dummy_cfg->panel;
		}
		sprdfb_panel_remove(dev);
	}
	printk(KERN_ERR "sprdfb:  [%s]: failed to attach LCD Panel!\n", __FUNCTION__);
	return NULL;
}


bool sprdfb_panel_get(struct sprdfb_device *dev)
{
	struct panel_spec *panel = NULL;

	if(NULL == dev){
		printk("sprdfb: [%s]: Invalid param\n", __FUNCTION__);
		return false;
	}

	printk("sprdfb: [%s], dev_id = %d\n",__FUNCTION__, dev->dev_id);

	panel = adapt_panel_from_uboot(dev->dev_id);
	if (panel) {
		dev->panel_ready = true;

		if (lcd_base_from_uboot == 0) {
			pr_err("sprdfb: [%s]: lcd_base_from_uboot[0x%x]\n", __func__,
					lcd_base_from_uboot);

			goto error_exit;
		}

		dev->logo_buffer_addr_v = __va(lcd_base_from_uboot);
		if (dev->logo_buffer_addr_v == 0) {
			pr_err("sprdfb: [%s]: logo_buffer_addr_v[0x%0x8]\n", __func__,
					dev->logo_buffer_addr_v);

			goto error_exit;
		}

		pr_info("sprdfb: [%s]: logo_buffer_addr_v[0x%0x8]\n", __func__,
				dev->logo_buffer_addr_v);

		panel_mount(dev, panel);
		panel_init(dev);

		/* RGB 565 boot logo size = 2 * Width * Height
		 * RGB 888 boot logo size = 4 * Width * Height
		 */
		dev->logo_buffer_size = 4 * dev->panel->width * dev->panel->height;

		printk("sprdfb: [%s] got panel\n", __FUNCTION__);
		return true;
	}

error_exit:
	printk("sprdfb: [%s] can not got panel\n", __FUNCTION__);
	dev->panel_ready = false;

	return false;
}

#if defined(CONFIG_LCD_CLASS_DEVICE)
static ssize_t show_lcd_info(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	unsigned int panel_id = lcd_id_from_uboot;

	int DBh_value = (panel_id >> 8) & 0xFF;

	if(DBh_value & 0x04)
		return sprintf(buf, "INH_%X%X%X",(panel_id >> 16) & 0xFF,
				(panel_id >> 8) & 0xFF, panel_id  & 0xFF);
	else {
		switch(DBh_value >> 3) {
		case 0x08 : /* CPT */
			return sprintf(buf, "CPT_%X%X%X",
					(panel_id >> 16) & 0xFF,
					(panel_id >> 8) & 0xFF,
					panel_id  & 0xFF);
			break;
		case 0x09 : /* AUO */
			return sprintf(buf, "AUO_%X%X%X",
					(panel_id >> 16) & 0xFF,
					(panel_id >> 8) & 0xFF,
					panel_id  & 0xFF);
			break;
		case 0x14 : /* TIANMA */
			return sprintf(buf, "TIA_%X%X%X",
					(panel_id >> 16) & 0xFF,
					(panel_id >> 8) & 0xFF,
					panel_id  & 0xFF);
			break;
		case 0x17 : /* BOE */
			return sprintf(buf, "BOE_%X%X%X",
					(panel_id >> 16) & 0xFF,
					(panel_id >> 8) & 0xFF,
					panel_id  & 0xFF);
			break;
		case 0x18 : /* DTC */
			return sprintf(buf, "DTC_%X%X%X",
					(panel_id >> 16) & 0xFF,
					(panel_id >> 8) & 0xFF,
					panel_id  & 0xFF);
			break;
		default :
			return sprintf(buf, "Unknown panel %X", panel_id);
			break;
		}
	}
}

static int sprdfb_set_power(struct lcd_device *ld, int power)
{
	int ret = 0;
	struct sprdfb_device *lcd = lcd_get_data(ld);

	if (power != FB_BLANK_UNBLANK && power != FB_BLANK_POWERDOWN &&
		power != FB_BLANK_NORMAL)
		return -EINVAL;

	if (lcd->power == power) {
		printk("power is same as previous mode\n");
		return -EINVAL;
	}

	lcd->power = power;

	printk("sprdfb: [%s] power[%d]\n", __FUNCTION__, lcd->power);

	return ret;
}

static int sprdfb_get_power(struct lcd_device *ld)
{
	struct sprdfb_device *lcd = lcd_get_data(ld);

	pr_debug("sprdfb: [%s] power[%d]\n", __FUNCTION__, lcd->power);

	return lcd->power;
}

static struct lcd_ops ld_ops = {
	.get_power = sprdfb_get_power,
	.set_power = sprdfb_set_power,
};

static ssize_t show_lcd_info(struct device *dev,
			struct device_attribute *attr, char *buf);

#ifdef CONFIG_FB_LCD_DEBUG
static ssize_t sprdfb_esd_check_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int rc;

	rc = snprintf((char *)buf, 20, "esd count:%d\n", esd_context.irq_enable_count);

	return rc;
}
static ssize_t sprdfb_esd_check_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
#ifdef SPRDFB_LCD_ESD_DETECT_GPIO
	sprdfb_panel_esd_det_irq_handler(0, NULL);
#else
	sprdfb_panel_err_fg_irq_handler(0, NULL);
#endif

	return 1;
}
#endif

#ifdef CONFIG_FB_DEBUG_LCD_TUNING
static ssize_t lcd_init_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	printk("++ [%s] current lcd init seq\n", __func__);

	return 0;
}

static ssize_t lcd_init_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int ret = 0;
	printk("++ [%s] current lcd init seq\n", __func__);
	ret = lcd_store_panel_init_cmd();

	return ret;
}
#endif

static struct device_attribute lcd_device_attributes[] = {
	__ATTR(lcd_type, S_IRUGO, show_lcd_info, NULL),
#ifdef CONFIG_FB_ESD_DETECT_BY_INT
#ifdef CONFIG_FB_LCD_DEBUG
	__ATTR(esd_check, 0644 , sprdfb_esd_check_show, sprdfb_esd_check_store),
#endif
#endif
#ifdef CONFIG_FB_DEBUG_LCD_TUNING
	__ATTR(lcd_tune, 0664, lcd_init_show, lcd_init_store),
#endif
};

bool sprdfb_lcd_class_register(struct platform_device *pdev,
						struct sprdfb_device *dev)
{
	int i, ret;

	dev->ld = lcd_device_register("panel", &pdev->dev, dev, &ld_ops);
	if (IS_ERR(dev->ld)) {
		printk("failed to register ld ops\n");
		return false;
	}

	dev->power = FB_BLANK_UNBLANK;

	for (i = 0; i < ARRAY_SIZE(lcd_device_attributes); i++) {
		ret = device_create_file(&dev->ld->dev,
				&lcd_device_attributes[i]);
		if (ret < 0) {
			printk("failed to add ld dev sysfs entries\n");
			goto err_lcd;
		}
	}

	return true;
err_lcd:
	lcd_device_unregister(dev->ld);
	return false;

}
#endif

bool sprdfb_panel_probe(struct platform_device *pdev, struct sprdfb_device *dev)
{
	struct panel_spec *panel;

	if(NULL == dev){
		printk("sprdfb: [%s]: Invalid param\n", __FUNCTION__);
		return false;
	}

	printk("sprdfb: [%s], dev_id = %d\n",__FUNCTION__, dev->dev_id);

	/* can not be here in normal; we should get correct device id from uboot */
	panel = adapt_panel_from_readid(dev);

	if (panel) {
		printk("sprdfb: [%s] got panel\n", __FUNCTION__);
		return true;
	}

	printk("sprdfb: [%s] can not got panel\n", __FUNCTION__);

	return false;
}

void sprdfb_panel_invalidate_rect(struct panel_spec *self,
				uint16_t left, uint16_t top,
				uint16_t right, uint16_t bottom)
{
	/*Jessica TODO: */
	if(NULL != self->ops->panel_invalidate_rect){
		self->ops->panel_invalidate_rect(self, left, top, right, bottom);
	}
	/*Jessica TODO: Need set timing to GRAM timing*/
}

void sprdfb_panel_invalidate(struct panel_spec *self)
{
	/*Jessica TODO:*/
	if(NULL != self->ops->panel_invalidate){
		self->ops->panel_invalidate(self);
	}
	/*Jessica TODO: Need set timing to GRAM timing*/
}

void sprdfb_panel_before_refresh(struct sprdfb_device *dev)
{
	if(NULL != dev->panel->if_ctrl->panel_if_before_refresh){
		dev->panel->if_ctrl->panel_if_before_refresh(dev);
	}
}

void sprdfb_panel_after_refresh(struct sprdfb_device *dev)
{
	if(NULL != dev->panel->if_ctrl->panel_if_after_refresh){
		dev->panel->if_ctrl->panel_if_after_refresh(dev);
	}
}

#ifdef CONFIG_FB_DYNAMIC_FREQ_SCALING
void sprdfb_panel_change_fps(struct sprdfb_device *dev, int fps_level)
{
	if (dev->panel->ops->panel_change_fps!= NULL) {
		printk("sprdfb: [%s] fps_level= %d\n", __FUNCTION__,fps_level);
		dev->panel->ops->panel_change_fps(dev->panel,fps_level);
	}
}
#endif

#ifdef CONFIG_FB_ESD_SUPPORT
/*return value:  0--panel OK.1-panel has been reset*/
uint32_t sprdfb_panel_ESD_check(struct sprdfb_device *dev)
{
	int32_t result = 0;
	uint32_t if_status = 0;

//	printk("sprdfb: [%s] (%d, %d, %d)\n",__FUNCTION__, dev->check_esd_time, dev->panel_reset_time, dev->reset_dsi_time);

	dev->check_esd_time++;

	if(SPRDFB_PANEL_IF_EDPI == dev->panel_if_type){
		if (dev->panel->ops->panel_esd_check != NULL) {
			result = dev->panel->ops->panel_esd_check(dev->panel);
			pr_debug("sprdfb: [%s] panel check return %d\n", __FUNCTION__, result);
		}
	}else if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
#ifdef FB_CHECK_ESD_BY_TE_SUPPORT
		dev->esd_te_waiter++;
		dev->esd_te_done = 0;
		dispc_set_bits(BIT(1), DISPC_INT_EN);
		result  = wait_event_interruptible_timeout(dev->esd_te_queue,
			          dev->esd_te_done, msecs_to_jiffies(600));
		pr_debug("sprdfb: after wait (%d)\n", result);
		dispc_clear_bits(BIT(1), DISPC_INT_EN);
		if(!result){ /*time out*/
			printk("sprdfb: [%s] esd check  not got te signal!!!!\n", __FUNCTION__);
			dev->esd_te_waiter = 0;
			result = 0;
		}else{
			pr_debug("sprdfb: [%s] esd check  got te signal!\n", __FUNCTION__);
			result = 1;
		}
#else
		if (dev->panel->ops->panel_esd_check != NULL) {
			result = dev->panel->ops->panel_esd_check(dev->panel);
//			pr_debug("sprdfb: [%s] panel check return %d\n", __FUNCTION__, result);
		}

#endif
	}


	if(0 == dev->enable){
		printk("sprdfb: [%s] leave (Invalid device status)!\n", __FUNCTION__);
		return 0;
	}

	if(result == 0){
		dev->panel_reset_time++;

		if(SPRDFB_PANEL_IF_EDPI == dev->panel_if_type){
			if(NULL != dev->panel->if_ctrl->panel_if_get_status){
				if_status = dev->panel->if_ctrl->panel_if_get_status(dev);
			}
		}else if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
			if_status = 2; /*need reset dsi as default for dpi mode*/
		}

		if(0 == if_status){
			printk("sprdfb: [%s] fail! Need reset panel.(%d,%d,%d)\n",	__FUNCTION__,
                            dev->check_esd_time, dev->panel_reset_time, dev->reset_dsi_time);
			panel_reset(dev);

			if(0 == dev->enable){
				printk("sprdfb: [%s] leave (Invalid device status)!\n", __FUNCTION__);
				return 0;
			}

			dev->panel->ops->panel_init(dev->panel);
			panel_ready(dev);
		}else{
			printk("sprdfb: [%s] fail! Need reset panel and panel if!!!!(%d,%d,%d)\n",__FUNCTION__,
                            dev->check_esd_time, dev->panel_reset_time, dev->reset_dsi_time);
			dev->reset_dsi_time++;
			if(NULL != dev->panel->if_ctrl->panel_if_suspend){
				dev->panel->if_ctrl->panel_if_suspend(dev);
			}

			mdelay(10);

			if(0 == dev->enable){
				printk("sprdfb: [%s] leave (Invalid device status)!\n", __FUNCTION__);
				return 0;
			}

			panel_init(dev);
			panel_reset(dev);

			if(0 == dev->enable){
				printk("sprdfb: [%s] leave (Invalid device status)!\n", __FUNCTION__);
				return 0;
			}

			dev->panel->ops->panel_init(dev->panel);
			panel_ready(dev);
		}
		pr_debug("sprdfb: [%s]return 1\n",__FUNCTION__);
		return 1;
	}
//	pr_debug("sprdfb: [%s]return 0\n",__FUNCTION__);
	return 0;
}
#endif

void sprdfb_panel_suspend(struct sprdfb_device *dev)
{
	if(NULL == dev->panel){
		return;
	}

	printk("sprdfb: [%s], dev_id = %d\n",__FUNCTION__, dev->dev_id);

#ifdef CONFIG_FB_ESD_DETECT_BY_INT
	if (!esd_context.notify_in_progress) {
#ifdef SPRDFB_LCD_ESD_DETECT_GPIO
		disable_irq_nosync(esd_context.esd_det_irq);
#endif
#ifdef SPRDFB_LCD_ERR_FG_GPIO
		disable_irq_nosync(esd_context.err_fg_irq);
#endif
		cancel_delayed_work_sync(&(esd_context.irq_enable));
		cancel_work_sync(&(esd_context.uevent_notifier));
	} else {
		cancel_delayed_work_sync(&(esd_context.irq_enable));
		cancel_work_sync(&(esd_context.uevent_notifier));
	}
#endif

#if 0
	//step1-1 clk/data lane enter LP
	if(NULL != dev->panel->if_ctrl->panel_if_before_panel_reset){
		dev->panel->if_ctrl->panel_if_before_panel_reset(dev);
	}

	//step1-2 enter sleep  (another way : reset panel)
	/*Jessica TODO: Need do some I2c, SPI, mipi sleep here*/
	/* let lcdc sleep in */

	if (dev->panel->ops->panel_enter_sleep != NULL) {
		dev->panel->ops->panel_enter_sleep(dev->panel,1);
	}
	msleep(100);
#else
	//step1 send lcd sleep cmd or reset panel directly
	if(dev->panel->suspend_mode == SEND_SLEEP_CMD){
		panel_sleep(dev);
	}else{
		panel_reset(dev);
	}
#endif

	//step2 clk/data lane enter ulps
	if(NULL != dev->panel->if_ctrl->panel_if_enter_ulps){
		dev->panel->if_ctrl->panel_if_enter_ulps(dev);
	}

	//step3 turn off mipi
	if(NULL != dev->panel->if_ctrl->panel_if_suspend){
		dev->panel->if_ctrl->panel_if_suspend(dev);
	}

	//step4 reset pin to low
	if (dev->panel->ops->panel_after_suspend != NULL) {
		//himax mipi lcd may define empty function
		dev->panel->ops->panel_after_suspend(dev->panel);
	}
	else{
		panel_after_suspend(dev);
	}
}

void sprdfb_panel_resume(struct sprdfb_device *dev, bool from_deep_sleep)
{
	if (NULL == dev->panel)
		return;

	if (!dev->panel_ready) {
		pr_info("sprdfb: [%s] lcd is not attached\n", __func__);
		return;
	}

	pr_info("sprdfb: [%s], dev->enable= %d, from_deep_sleep = %d\n",
				__func__, dev->enable, from_deep_sleep);
#if 0
	/*Jessica TODO: resume i2c, spi, mipi*/
	if(NULL != dev->panel->if_ctrl->panel_if_resume){
		dev->panel->if_ctrl->panel_if_resume(dev);
	}
	panel_ready(dev);
#endif
	//step1 reset pin to high
	if (dev->panel->ops->panel_before_resume != NULL) {
		//himax mipi lcd may define empty function
		dev->panel->ops->panel_before_resume(dev->panel);
	}
	else{
		panel_before_resume(dev);
	}

	if(from_deep_sleep){
		//step2 turn on mipi
		panel_init(dev);

		//step3 reset panel
		panel_reset(dev);

		//step4 panel init
		dev->panel->ops->panel_init(dev->panel);

		//step5 clk/data lane enter HS
		panel_ready(dev);
	}else{
		//step2 turn on mipi
		/*Jessica TODO: resume i2c, spi, mipi*/
		if(NULL != dev->panel->if_ctrl->panel_if_resume){
			dev->panel->if_ctrl->panel_if_resume(dev);
		}

		//step3 sleep out
		if(NULL != dev->panel->ops->panel_enter_sleep){
			dev->panel->ops->panel_enter_sleep(dev->panel,0);
		}

		//step4 clk/data lane enter HS
		panel_ready(dev);
	}

#ifdef CONFIG_FB_ESD_DETECT_BY_INT
	if (
#if defined(SPRDFB_LCD_ESD_DETECT_GPIO) && defined(SPRDFB_LCD_ERR_FG_GPIO)
			gpio_is_valid(esd_context.esd_det_gpio)
			|| gpio_is_valid(esd_context.err_fg_gpio)
#elif defined(SPRDFB_LCD_ESD_DETECT_GPIO)
			gpio_is_valid(esd_context.esd_det_gpio)
#elif defined(SPRDFB_LCD_ERR_FG_GPIO)
			gpio_is_valid(esd_context.err_fg_gpio)
#else
			0
#endif
	) {
		esd_context.status = ESD_NONE;

		schedule_delayed_work(&(esd_context.irq_enable),
				msecs_to_jiffies(SPRDFB_ESD_IRQ_EN_DELAY));
	}
#endif
}

void sprdfb_panel_remove(struct sprdfb_device *dev)
{
	if(NULL == dev->panel){
		return;
	}

#ifdef CONFIG_FB_ESD_DETECT_BY_INT
	if (!esd_context.notify_in_progress) {
#ifdef SPRDFB_LCD_ESD_DETECT_GPIO
		disable_irq_nosync(esd_context.esd_det_irq);
#endif
#ifdef SPRDFB_LCD_ERR_FG_GPIO
		disable_irq_nosync(esd_context.err_fg_irq);
#endif
		cancel_delayed_work_sync(&(esd_context.irq_enable));
		cancel_work_sync(&(esd_context.uevent_notifier));
	} else {
		cancel_delayed_work_sync(&(esd_context.irq_enable));
		cancel_work_sync(&(esd_context.uevent_notifier));
	}
#ifdef SPRDFB_LCD_ESD_DETECT_GPIO
	gpio_free(esd_context.esd_det_gpio);
#endif
#ifdef SPRDFB_LCD_ERR_FG_GPIO
	gpio_free(esd_context.err_fg_gpio);
#endif
#endif

	/*Jessica TODO:close panel, i2c, spi, mipi*/
	if(NULL != dev->panel->if_ctrl->panel_if_uninit){
		dev->panel->if_ctrl->panel_if_uninit(dev);
	}
	dev->panel = NULL;
}


int sprdfb_panel_register(struct panel_cfg *cfg)
{
	pr_debug("sprdfb: [%s], panel id = %d\n",__FUNCTION__, cfg->dev_id);

	if(!panel_check(cfg)){
		printk("sprdfb: [%s]: panel check fail!id = %d\n",__FUNCTION__,  cfg->dev_id);
		return -1;
	}

	mutex_lock(&panel_mutex);

	if (cfg->dev_id == SPRDFB_MAINLCD_ID) {
		list_add_tail(&cfg->list, &panel_list_main);
	} else if (cfg->dev_id == SPRDFB_SUBLCD_ID) {
		list_add_tail(&cfg->list, &panel_list_sub);
	} else {
		list_add_tail(&cfg->list, &panel_list_main);
		list_add_tail(&cfg->list, &panel_list_sub);
	}

	mutex_unlock(&panel_mutex);

	return 0;
}

#ifdef CONFIG_FB_ESD_DETECT_BY_INT
#ifdef SPRDFB_LCD_ESD_DETECT_GPIO
static irqreturn_t sprdfb_panel_esd_det_irq_handler(int irq, void *handle)
{
	pr_debug("%s : esd det irq handler.\n", __func__);
	if (!work_busy(&(esd_context.uevent_notifier))) {
		if (gpio_get_value(esd_context.esd_det_gpio)) {
			schedule_work(&(esd_context.uevent_notifier));
			esd_context.notify_in_progress = 1;
			disable_irq_nosync(esd_context.esd_det_irq);
			pr_info("%s : add esd schedule_work for esd\n", __func__);
		}
	}

	return IRQ_HANDLED;
}
#endif

#ifdef SPRDFB_LCD_ERR_FG_GPIO
static irqreturn_t sprdfb_panel_err_fg_irq_handler(int irq, void *handle)
{
	pr_debug("%s : err fg irq handler.\n", __func__);
	if (!work_busy(&(esd_context.uevent_notifier))) {
		if (gpio_get_value(esd_context.err_fg_gpio)) {
			schedule_work(&(esd_context.uevent_notifier));
			esd_context.notify_in_progress = 1;
			disable_irq_nosync(esd_context.err_fg_irq);
			pr_info("%s : add esd schedule_work for esd\n", __func__);
		}
	}

	return IRQ_HANDLED;
}
#endif

static void sprdfb_panel_esd_uevent_notifier(struct work_struct *work)
{
	struct sprdfb_device *fbdev = esd_context.fbdev;
	char *event_string = "LCD_ESD=ON";
	char *envp[] = { event_string, NULL };

	if (fbdev->enable == 0) {
		pr_err("%s: Display off Skip ESD recovery\n", __func__);
		return;
	}

	esd_context.status = ESD_DETECTED;

#if defined(CONFIG_LCD_CLASS_DEVICE)
	kobject_uevent_env(&(esd_context.device->kobj), KOBJ_CHANGE, envp);
#endif

	pr_err("ESD DETECTED\n");

	return;
}

static void sprdfb_panel_esd_irq_enable(struct work_struct *work)
{
	pr_debug("%s: enable esd irq.\n", __func__);
#ifdef SPRDFB_LCD_ESD_DETECT_GPIO
	enable_irq(esd_context.esd_det_irq);
#endif
#ifdef SPRDFB_LCD_ERR_FG_GPIO
	enable_irq(esd_context.err_fg_irq);
#endif
	esd_context.notify_in_progress = 0;
	esd_context.irq_enable_count++;
}

static int sprdfb_panel_esd_detect_init(struct sprdfb_device *fbdev)
{
	int result = 0;

	esd_context.status = ESD_NONE;
	esd_context.fbdev = fbdev;

	INIT_WORK(&(esd_context.uevent_notifier), sprdfb_panel_esd_uevent_notifier);
	INIT_DELAYED_WORK(&(esd_context.irq_enable), sprdfb_panel_esd_irq_enable);

#ifdef SPRDFB_LCD_ESD_DETECT_GPIO
	esd_context.esd_det_gpio = -1;

	result = gpio_request(SPRDFB_LCD_ESD_DETECT_GPIO, "esd_det");
	if (result) {
		pr_err(
				"%s: request gpio SPRDFB_LCD_ESD_DETECT_GPIO failed, Error code[%d]\n",
				__func__, result);
		goto err_exit;
	}

	esd_context.esd_det_gpio = SPRDFB_LCD_ESD_DETECT_GPIO;

	result = gpio_direction_input(esd_context.esd_det_gpio);
	if (unlikely(result < 0)) {
		pr_err("%s: failed to set gpio %d as input, Error code[%d]\n", __func__,
				esd_context.esd_det_gpio, result);

		goto err_exit;
	}

	esd_context.esd_det_irq = gpio_to_irq(esd_context.esd_det_gpio);

	result = request_threaded_irq(esd_context.esd_det_irq, NULL,
			sprdfb_panel_esd_det_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, "esd_detect", NULL);
	if (result) {
		pr_err("%s: Failed to request_irq, Error code[%d]", __func__, result);

		goto err_exit;
	}
#endif
#ifdef SPRDFB_LCD_ERR_FG_GPIO
	esd_context.err_fg_gpio = -1;

	result = gpio_request(SPRDFB_LCD_ERR_FG_GPIO, "err_fg");
	if (result) {
		pr_err(
				"%s: request gpio SPRDFB_LCD_ERR_FG_GPIO failed, Error code[%d]\n",
				__func__, result);
		goto err_exit;
	}

	esd_context.err_fg_gpio = SPRDFB_LCD_ERR_FG_GPIO;

	result = gpio_direction_input(esd_context.err_fg_gpio);
	if (unlikely(result < 0)) {
		pr_err("%s: failed to set gpio %d as input, Error code[%d]\n", __func__,
				esd_context.err_fg_gpio, result);

		goto err_exit;
	}

	esd_context.err_fg_irq = gpio_to_irq(esd_context.err_fg_gpio);

	result = request_threaded_irq(esd_context.err_fg_irq, NULL,
			sprdfb_panel_err_fg_irq_handler, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"err_fg", NULL);
	if (result) {
		pr_err("%s: Failed to request_irq, Error code[%d]", __func__, result);

		goto err_exit;
	}
#endif

	esd_context.class = class_create(THIS_MODULE, "lcd_event");
	if (IS_ERR(esd_context.class)) {
		pr_err("Failed to create class(lcd_event)!\n");
		result = PTR_ERR(esd_context.class);
		goto err_exit;
	}
	esd_context.device = device_create(esd_context.class, NULL, 0, NULL, "esd");

	return 0;

err_exit:
#ifdef SPRDFB_LCD_ERR_FG_GPIO
	if (gpio_is_valid(esd_context.err_fg_gpio))
		gpio_free(esd_context.err_fg_gpio);
#endif
#ifdef SPRDFB_LCD_ESD_DETECT_GPIO
	if (gpio_is_valid(esd_context.esd_det_gpio))
		gpio_free(esd_context.esd_det_gpio);
#endif
	return result;
}
#endif
