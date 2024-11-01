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
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/sysrq.h>
#include <linux/sched.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>

#include <mach/globalregs.h>
#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/adi.h>
#include <mach/kpd.h>
#include <mach/sci.h>
#include <mach/sci_glb_regs.h>
#include <linux/wakelock.h>
//#include <linux/input-hook.h>
#if CONFIG_SEC_DEBUG
#include <mach/sec_debug.h>
#endif

#define WAKELOCK_TIME	HZ/10
#define DEBUG_KEYPAD	0
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
#define PRINT_KEY_LOG 1
#else
#define PRINT_KEY_LOG 0
#endif

#ifndef CONFIG_OF
#define KPD_REG_BASE                (SPRD_KPD_BASE)
#else
static unsigned int KPD_REG_BASE;
#endif

#define KPD_CTRL                	(KPD_REG_BASE + 0x00)
#define KPD_EN						(0x01 << 0)
#define KPD_SLEEP_EN				(0x01 << 1)
#define KPD_LONG_KEY_EN				(0x01 << 2)

//v0
#define KPDCTL_ROW_MSK_V0                  (0x3f << 18)	/* enable rows 2 - 7 */
#define KPDCTL_COL_MSK_V0                  (0x3f << 10)	/* enable cols 2 - 7 */

//v1
#define KPDCTL_ROW_MSK_V1                  (0xff << 16)	/* enable rows 0 - 7 */
#define KPDCTL_COL_MSK_V1                  (0xff << 8)	/* enable cols 0 - 7 */

#define KPD_INT_EN              	(KPD_REG_BASE + 0x04)
#define KPD_INT_RAW_STATUS          (KPD_REG_BASE + 0x08)
#define KPD_INT_MASK_STATUS     	(KPD_REG_BASE + 0x0C)
#define KPD_INT_ALL                 (0xfff)
#define KPD_INT_DOWNUP              (0x0ff)
#define KPD_INT_LONG				(0xf00)
#define KPD_PRESS_INT0              (1 << 0)
#define KPD_PRESS_INT1              (1 << 1)
#define KPD_PRESS_INT2              (1 << 2)
#define KPD_PRESS_INT3              (1 << 3)
#define KPD_RELEASE_INT0            (1 << 4)
#define KPD_RELEASE_INT1            (1 << 5)
#define KPD_RELEASE_INT2            (1 << 6)
#define KPD_RELEASE_INT3            (1 << 7)
#define KPD_LONG_KEY_INT0           (1 << 8)
#define KPD_LONG_KEY_INT1           (1 << 9)
#define KPD_LONG_KEY_INT2           (1 << 10)
#define KPD_LONG_KEY_INT3           (1 << 11)

#define KPD_INT_CLR             	(KPD_REG_BASE + 0x10)
#define KPD_POLARITY            	(KPD_REG_BASE + 0x18)
#define KPD_CFG_ROW_POLARITY		(0xff)
#define KPD_CFG_COL_POLARITY		(0xFF00)
#define CFG_ROW_POLARITY            (KPD_CFG_ROW_POLARITY & 0x00FF)
#define CFG_COL_POLARITY            (KPD_CFG_COL_POLARITY & 0xFF00)

#define KPD_DEBOUNCE_CNT        	(KPD_REG_BASE + 0x1C)
#define KPD_LONG_KEY_CNT        	(KPD_REG_BASE + 0x20)

#define KPD_SLEEP_CNT           	(KPD_REG_BASE + 0x24)
#define KPD_SLEEP_CNT_VALUE(_X_MS_)	(_X_MS_ * 32.768 -1)
#define KPD_CLK_DIV_CNT         	(KPD_REG_BASE + 0x28)

#define KPD_KEY_STATUS          	(KPD_REG_BASE + 0x2C)
#define KPD_INT0_COL(_X_)	(((_X_)>> 0) & 0x7)
#define KPD_INT0_ROW(_X_)	(((_X_)>> 4) & 0x7)
#define KPD_INT0_DOWN(_X_)	(((_X_)>> 7) & 0x1)
#define KPD_INT1_COL(_X_)	(((_X_)>> 8) & 0x7)
#define KPD_INT1_ROW(_X_)	(((_X_)>> 12) & 0x7)
#define KPD_INT1_DOWN(_X_)	(((_X_)>> 15) & 0x1)
#define KPD_INT2_COL(_X_)	(((_X_)>> 16) & 0x7)
#define KPD_INT2_ROW(_X_)	(((_X_)>> 20) & 0x7)
#define KPD_INT2_DOWN(_X_)	(((_X_)>> 23) & 0x1)
#define KPD_INT3_COL(_X_)	(((_X_)>> 24) & 0x7)
#define KPD_INT3_ROW(_X_)	(((_X_)>> 28) & 0x7)
#define KPD_INT3_DOWN(_X_)	(((_X_)>> 31) & 0x1)

#define KPD_SLEEP_STATUS        	(KPD_REG_BASE + 0x0030)
#define KPD_DEBUG_STATUS1        	(KPD_REG_BASE + 0x0034)
#define KPD_DEBUG_STATUS2        	(KPD_REG_BASE + 0x0038)

#ifndef CONFIG_OF
#define PB_INT                  EIC_KEY_POWER
#else
static int PB_INT;
#endif

#if defined(CONFIG_ARCH_SC8825)
static __devinit void __keypad_enable(void)
{
	sci_glb_set(REG_GLB_SOFT_RST, BIT_KPD_RST);
	mdelay(2);
	sci_glb_clr(REG_GLB_SOFT_RST, BIT_KPD_RST);
	sci_glb_set(REG_GLB_GEN0, BIT_KPD_EB | BIT_RTC_KPD_EB);
}
static __devexit void __keypad_disable(void)
{
	sci_glb_clr(REG_GLB_GEN0, BIT_KPD_EB | BIT_RTC_KPD_EB);
}

static int __keypad_controller_ver(void)
{
	return 0;
}

#elif defined(CONFIG_ARCH_SCX35)
static void __keypad_enable(void)
{
	sci_glb_set(REG_AON_APB_APB_RST0, BIT_KPD_SOFT_RST);
	mdelay(2);
	sci_glb_clr(REG_AON_APB_APB_RST0, BIT_KPD_SOFT_RST);
	sci_glb_set(REG_AON_APB_APB_EB0, BIT_KPD_EB);
	sci_glb_set(REG_AON_APB_APB_RTC_EB, BIT_KPD_RTC_EB);
}

static void __keypad_disable(void)
{
	sci_glb_clr(REG_AON_APB_APB_EB0, BIT_KPD_EB);
	sci_glb_clr(REG_AON_APB_APB_RTC_EB, BIT_KPD_RTC_EB);
}
static int __keypad_controller_ver(void)
{
	return 1;
}

#else
#error "Pls fill the low level enable function"
#endif

struct sci_keypad_t {
	struct input_dev *input_dev;
	int irq;
	int rows;
	int cols;
	unsigned int keyup_test_jiffies;
	struct wake_lock	 key_wake_lock;
};

/* sys fs  */
extern struct class *sec_class;
struct device *key_dev,*powerkey_dev;
EXPORT_SYMBOL(key_dev);
EXPORT_SYMBOL(powerkey_dev);

static ssize_t key_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t powerkey_show(struct device *dev, struct device_attribute *attr, char *buf);

static DEVICE_ATTR(sec_key_pressed, S_IRUGO, key_show, NULL);
static DEVICE_ATTR(sec_powerkey_pressed, S_IRUGO, powerkey_show, NULL);
static ssize_t key_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t keys_pressed;
	uint8_t default_press = 0x80;
	int is_key_checked = 0;

	keys_pressed = __raw_readl(KPD_KEY_STATUS);
	if(keys_pressed & default_press)
		is_key_checked = sprintf(buf,"%s\n","PRESS");
	else
		is_key_checked = sprintf(buf,"%s\n","RELEASE");

	printk("[KEY] Keyshort Press Check ==> keys_pressed : %x\n", keys_pressed);

	return is_key_checked;
}
static ssize_t powerkey_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int is_key_checked = 0;
	unsigned long value = !(gpio_get_value(PB_INT));

	if(!value)
		is_key_checked = sprintf(buf,"%s\n","PRESS");
	else
		is_key_checked = sprintf(buf,"%s\n","RELEASE");

	printk("[KEY] Keyshort Press Check ==> powerKeys_pressed : %ld \n",value);

	return is_key_checked;
}
/* sys fs */

#if	DEBUG_KEYPAD
static void dump_keypad_register(void)
{
#define INT_MASK_STS                (SPRD_INTC1_BASE + 0x0000)
#define INT_RAW_STS                 (SPRD_INTC1_BASE + 0x0004)
#define INT_EN                      (SPRD_INTC1_BASE + 0x0008)
#define INT_DIS                     (SPRD_INTC1_BASE + 0x000C)

	printk("\nREG_INT_MASK_STS = 0x%08x\n", __raw_readl(INT_MASK_STS));
	printk("REG_INT_RAW_STS = 0x%08x\n", __raw_readl(INT_RAW_STS));
	printk("REG_INT_EN = 0x%08x\n", __raw_readl(INT_EN));
	printk("REG_INT_DIS = 0x%08x\n", __raw_readl(INT_DIS));
	printk("REG_KPD_CTRL = 0x%08x\n", __raw_readl(KPD_CTRL));
	printk("REG_KPD_INT_EN = 0x%08x\n", __raw_readl(KPD_INT_EN));
	printk("REG_KPD_INT_RAW_STATUS = 0x%08x\n",
	       __raw_readl(KPD_INT_RAW_STATUS));
	printk("REG_KPD_INT_MASK_STATUS = 0x%08x\n",
	       __raw_readl(KPD_INT_MASK_STATUS));
	printk("REG_KPD_INT_CLR = 0x%08x\n", __raw_readl(KPD_INT_CLR));
	printk("REG_KPD_POLARITY = 0x%08x\n", __raw_readl(KPD_POLARITY));
	printk("REG_KPD_DEBOUNCE_CNT = 0x%08x\n",
	       __raw_readl(KPD_DEBOUNCE_CNT));
	printk("REG_KPD_LONG_KEY_CNT = 0x%08x\n",
	       __raw_readl(KPD_LONG_KEY_CNT));
	printk("REG_KPD_SLEEP_CNT = 0x%08x\n", __raw_readl(KPD_SLEEP_CNT));
	printk("REG_KPD_CLK_DIV_CNT = 0x%08x\n", __raw_readl(KPD_CLK_DIV_CNT));
	printk("REG_KPD_KEY_STATUS = 0x%08x\n", __raw_readl(KPD_KEY_STATUS));
	printk("REG_KPD_SLEEP_STATUS = 0x%08x\n",
	       __raw_readl(KPD_SLEEP_STATUS));
}
#else
static void dump_keypad_register(void)
{
}
#endif

#if PRINT_KEY_LOG
static void sci_print_keydbg(unsigned short key, unsigned short state)
{
	const char* key_strings[] = {"KEY_VOLUMEUP", "KEY_VOLUMEDOWN",\
				"KEY_HOME", "KEY_POWER", "UNKNOWN"};

	if (key == KEY_VOLUMEUP)
		key = 0;
	else if (key == KEY_VOLUMEDOWN)
		key = 1;
	else if (key == KEY_MENU)
		key = 2;
	else if (key == KEY_POWER)
		key = 3;
	else
		key = 4;

	printk("%s: [%s] %s\n", __func__, state ? "P":"R", key_strings[key]);
}
#endif

static int sci_print_check_validity(unsigned short key)
{
	if ((key == KEY_VOLUMEUP) || (key == KEY_VOLUMEDOWN) ||\
		(key == KEY_MENU) || (key == KEY_POWER))
		return 0;
	else
		return -1;
}

static void sci_print_report_key(struct sci_keypad_t *sci_kpd, int col, int row, int state)
{
	unsigned short *keycodes = sci_kpd->input_dev->keycode;
	unsigned int row_shift = get_count_order(sci_kpd->cols);
	unsigned short key;

	key = keycodes[MATRIX_SCAN_CODE(row, col, row_shift)];

	if (sci_print_check_validity(key))
		return;

	if (key == KEY_MENU)
		wake_lock_timeout
			(&sci_kpd->key_wake_lock, WAKELOCK_TIME);

	input_report_key(sci_kpd->input_dev, key, state);
	input_sync(sci_kpd->input_dev);
	sec_debug_check_crash_key(key, state);
	sci_print_keydbg(key, state);

	return;
}

static irqreturn_t sci_keypad_isr(int irq, void *dev_id)
{
	unsigned long value;
	unsigned long int_status = __raw_readl(KPD_INT_MASK_STATUS);
	unsigned long key_status = __raw_readl(KPD_KEY_STATUS);
	int col, row;

	value = __raw_readl(KPD_INT_CLR);
	value |= KPD_INT_ALL;
	__raw_writel(value, KPD_INT_CLR);

	if ((int_status & KPD_PRESS_INT0)) {
		col = KPD_INT0_COL(key_status);
		row = KPD_INT0_ROW(key_status);
		sci_print_report_key(dev_id, col, row, true);
	}
	if (int_status & KPD_RELEASE_INT0) {
		col = KPD_INT0_COL(key_status);
		row = KPD_INT0_ROW(key_status);
		sci_print_report_key(dev_id, col, row, false);
	}
	if (int_status & KPD_PRESS_INT1) {
		col = KPD_INT1_COL(key_status);
		row = KPD_INT1_ROW(key_status);
		sci_print_report_key(dev_id, col, row, true);
	}
	if (int_status & KPD_RELEASE_INT1) {
		col = KPD_INT1_COL(key_status);
		row = KPD_INT1_ROW(key_status);
		sci_print_report_key(dev_id, col, row, false);
	}
	if ((int_status & KPD_PRESS_INT2)) {
		col = KPD_INT2_COL(key_status);
		row = KPD_INT2_ROW(key_status);
		sci_print_report_key(dev_id, col, row, true);
	}
	if (int_status & KPD_RELEASE_INT2) {
		col = KPD_INT2_COL(key_status);
		row = KPD_INT2_ROW(key_status);
		sci_print_report_key(dev_id, col, row, false);

	}
	if (int_status & KPD_PRESS_INT3) {
		col = KPD_INT3_COL(key_status);
		row = KPD_INT3_ROW(key_status);
		sci_print_report_key(dev_id, col, row, true);
	}
	if (int_status & KPD_RELEASE_INT3) {
		col = KPD_INT3_COL(key_status);
		row = KPD_INT3_ROW(key_status);
		sci_print_report_key(dev_id, col, row, false);
	} else
		return IRQ_HANDLED;

	return IRQ_HANDLED;
}

static irqreturn_t sci_powerkey_isr(int irq, void *dev_id)
{				//TODO: if usign gpio(eic), need add row , cols to platform data.
	static unsigned long last_value = 1;
	unsigned short key = KEY_POWER;
	unsigned long value = !(gpio_get_value(PB_INT));
	struct sci_keypad_t *sci_kpd = dev_id;

	wake_lock_timeout(&sci_kpd->key_wake_lock, WAKELOCK_TIME);

	if (last_value == value) {
		/* seems an event is missing, just report it */
		input_report_key(sci_kpd->input_dev, key, last_value);
		input_sync(sci_kpd->input_dev);
#if PRINT_KEY_LOG
		sci_print_keydbg(key, !value);
#endif
	}

	if (value) {
		/* Release : low level */
#ifdef HOOK_POWER_KEY
		input_report_key_hook(sci_kpd->input_dev, key, 0);
#endif
		input_report_key(sci_kpd->input_dev, key, 0);
		input_sync(sci_kpd->input_dev);
		irq_set_irq_type(irq, IRQF_TRIGGER_HIGH);
	} else {
		/* Press : high level */
#ifdef HOOK_POWER_KEY
		input_report_key_hook(sci_kpd->input_dev, key, 1);
#endif
		input_report_key(sci_kpd->input_dev, key, 1);
		input_sync(sci_kpd->input_dev);
		irq_set_irq_type(irq, IRQF_TRIGGER_LOW);
	}

	sec_debug_check_crash_key(key, value);
	last_value = value;

#if PRINT_KEY_LOG
	sci_print_keydbg(key, !value);
#endif

	return IRQ_HANDLED;
}
#ifdef CONFIG_OF
static struct sci_keypad_platdata *sci_keypad_parse_dt(
                struct device *dev)
{
	struct sci_keypad_platform_data *pdata;
	struct device_node *np = dev->of_node, *key_np;
	uint32_t num_rows, num_cols;
	uint32_t rows_choose_hw, cols_choose_hw;
	uint32_t debounce_time;
	struct matrix_keymap_data *keymap_data;
	uint32_t *keymap, key_count;
	struct resource res;
	int ret;

	ret = of_address_to_resource(np, 0, &res);
	if(ret < 0){
		dev_err(dev, "no reg of property specified\n");
		return NULL;
	}
	KPD_REG_BASE = res.start;
	PB_INT = of_get_gpio(np, 0);
	if(PB_INT < 0){
		dev_err(dev, "no gpios of property specified\n");
		return NULL;
	}
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for platform data\n");
		return NULL;
	}
	ret = of_property_read_u32(np, "sprd,keypad-num-rows", &num_rows);
	if(ret){
		dev_err(dev, "no sprd,keypad-num-rows of property specified\n");
		goto fail;
	}
	pdata->rows_number = num_rows;

	ret = of_property_read_u32(np, "sprd,keypad-num-columns", &num_cols);
	if(ret){
		dev_err(dev, "no sprd,keypad-num-columns of property specified\n");
		goto fail;
	}
	pdata->cols_number = num_cols;

	ret = of_property_read_u32(np, "sprd,debounce_time", &debounce_time);
	if(ret){
		debounce_time = 0;
	}
	pdata->debounce_time = debounce_time;

	if (of_get_property(np, "linux,input-no-autorepeat", NULL))
		pdata->repeat = false;
	if(of_get_property(np, "sprd,support_long_key", NULL))
		pdata->support_long_key = true;
	if (of_get_property(np, "linux,input-wakeup", NULL))
		pdata->wakeup = true;


	ret = of_property_read_u32(np, "sprd,keypad-rows-choose-hw", &rows_choose_hw);
	if(ret){
		dev_err(dev, "no sprd,keypad-rows-choose-hw of property specified\n");
		goto fail;
	}
	pdata->rows_choose_hw = rows_choose_hw;

	ret = of_property_read_u32(np, "sprd,keypad-cols-choose-hw", &cols_choose_hw);
	if(ret){
		dev_err(dev, "no sprd,keypad-cols-choose-hw of property specified\n");
		goto fail;
	}
	pdata->cols_choose_hw = cols_choose_hw;

	keymap_data = kzalloc(sizeof(*keymap_data), GFP_KERNEL);
	if (!keymap_data) {
		dev_err(dev, "could not allocate memory for keymap_data\n");
		goto fail;
	}
	pdata->keymap_data = keymap_data;

	key_count = of_get_child_count(np);
	keymap_data->keymap_size = key_count;
	keymap = kzalloc(sizeof(uint32_t) * key_count, GFP_KERNEL);
	if (!keymap) {
		dev_err(dev, "could not allocate memory for keymap\n");
		goto fail_keymap;
	}
	keymap_data->keymap = keymap;

	for_each_child_of_node(np, key_np) {
		u32 row, col, key_code;
		ret = of_property_read_u32(key_np, "keypad,row", &row);
		if(ret)
			goto fail_parse_keymap;
		ret = of_property_read_u32(key_np, "keypad,column", &col);
		if(ret)
			goto fail_parse_keymap;
		ret = of_property_read_u32(key_np, "linux,code", &key_code);
		if(ret)
			goto fail_parse_keymap;
		*keymap++ = KEY(row, col, key_code);
		pr_info("sci_keypad_parse_dt: %d, %d, %d\n",row, col, key_code);
	}


	return pdata;

fail_parse_keymap:
	dev_err(dev, "failed parsing keymap\n");
	kfree(keymap);
	keymap_data->keymap = NULL;
fail_keymap:
	kfree(keymap_data);
	pdata->keymap_data = NULL;
fail:
	kfree(pdata);
	return NULL;
}
#else
static struct sci_keypad_platdata *sci_keypad_parse_dt(
                struct device *dev)
{
	return NULL;
}
#endif

static int sci_keypad_probe(struct platform_device *pdev)
{
	struct sci_keypad_t *sci_kpd;
	struct input_dev *input_dev;
	struct sci_keypad_platform_data *pdata = pdev->dev.platform_data;
	int error;
	unsigned long value;
	unsigned int row_shift, keycodemax;

	if (pdev->dev.of_node && !pdata){
		pdata = sci_keypad_parse_dt(&pdev->dev);
		if(pdata)
			pdev->dev.platform_data = pdata;
	}

	if (!pdata) {
		printk(KERN_WARNING "sci_keypad_probe get platform_data NULL\n");
		error = -EINVAL;
		goto out0;
	}
	row_shift = get_count_order(pdata->cols_number);
	keycodemax = pdata->rows_number << row_shift;

	sci_kpd = kzalloc(sizeof(struct sci_keypad_t) +
			  keycodemax * sizeof(unsigned short), GFP_KERNEL);
	input_dev = input_allocate_device();

	if (!sci_kpd || !input_dev) {
		error = -ENOMEM;
		goto out1;
	}
	platform_set_drvdata(pdev, sci_kpd);

	wake_lock_init(&sci_kpd->key_wake_lock,
			WAKE_LOCK_SUSPEND, "power-keys_wake_lock");

	sci_kpd->input_dev = input_dev;
	sci_kpd->rows = pdata->rows_number;
	sci_kpd->cols = pdata->cols_number;

	__keypad_enable();

	__raw_writel(KPD_INT_ALL, KPD_INT_CLR);
	__raw_writel(CFG_ROW_POLARITY | CFG_COL_POLARITY, KPD_POLARITY);
	__raw_writel(1, KPD_CLK_DIV_CNT);
	__raw_writel(0xc, KPD_LONG_KEY_CNT);
	__raw_writel(0x5, KPD_DEBOUNCE_CNT);

	sci_kpd->irq = platform_get_irq(pdev, 0);
	if (sci_kpd->irq < 0) {
		error = -ENODEV;
		dev_err(&pdev->dev, "Get irq number error,Keypad Module\n");
		goto out2;
	}
	error =
	    request_irq(sci_kpd->irq, sci_keypad_isr, IRQF_NO_SUSPEND, "sci-keypad", sci_kpd);
	if (error) {
		dev_err(&pdev->dev, "unable to claim irq %d\n", sci_kpd->irq);
		goto out2;
	}
#ifndef CONFIG_OF
	input_dev->name = pdev->name;
#else
	input_dev->name = "sci-keypad";
#endif
	input_dev->phys = "sci-key/input0";
	input_dev->dev.parent = &pdev->dev;
	input_set_drvdata(input_dev, sci_kpd);

	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;

	input_dev->keycode = &sci_kpd[1];
	input_dev->keycodesize = sizeof(unsigned short);
	input_dev->keycodemax = keycodemax;

	matrix_keypad_build_keymap(pdata->keymap_data, NULL, pdata->rows_number, pdata->cols_number,
				   input_dev->keycode, input_dev);

	/* there are keys from hw other than keypad controller */
	__set_bit(KEY_POWER, input_dev->keybit);
	__set_bit(EV_KEY, input_dev->evbit);
	if (pdata->repeat)
		__set_bit(EV_REP, input_dev->evbit);

	error = input_register_device(input_dev);
	if (error) {
		dev_err(&pdev->dev, "unable to register input device\n");
		goto out3;
	}
	device_init_wakeup(&pdev->dev, 1);

	value = KPD_INT_DOWNUP;
	if (pdata->support_long_key)
		value |= KPD_INT_LONG;
	__raw_writel(value, KPD_INT_EN);
	value = KPD_SLEEP_CNT_VALUE(1000);
	__raw_writel(value, KPD_SLEEP_CNT);

	if (__keypad_controller_ver() == 0) {
		if ((pdata->rows_choose_hw & ~KPDCTL_ROW_MSK_V0)
		    || (pdata->cols_choose_hw & ~KPDCTL_COL_MSK_V0)) {
			pr_warn("Error rows_choose_hw Or cols_choose_hw\n");
		} else {
			pdata->rows_choose_hw &= KPDCTL_ROW_MSK_V0;
			pdata->cols_choose_hw &= KPDCTL_COL_MSK_V0;
		}
	} else if (__keypad_controller_ver() == 1) {
		if ((pdata->rows_choose_hw & ~KPDCTL_ROW_MSK_V1)
		    || (pdata->cols_choose_hw & ~KPDCTL_COL_MSK_V1)) {
			pr_warn("Error rows_choose_hw\n");
		} else {
			pdata->rows_choose_hw &= KPDCTL_ROW_MSK_V1;
			pdata->cols_choose_hw &= KPDCTL_COL_MSK_V1;
		}
	} else {
		pr_warn
		    ("This driver don't support this keypad controller version Now\n");
	}
	value =
	    KPD_EN | KPD_SLEEP_EN | pdata->
	    rows_choose_hw | pdata->cols_choose_hw;
	if (pdata->support_long_key)
		value |= KPD_LONG_KEY_EN;
	__raw_writel(value, KPD_CTRL);

	gpio_request(PB_INT, "powerkey");
	gpio_direction_input(PB_INT);
	error = request_irq(gpio_to_irq(PB_INT), sci_powerkey_isr,
			    IRQF_TRIGGER_HIGH | IRQF_NO_SUSPEND,
			    "powerkey", sci_kpd);
	if (error) {
		dev_err(&pdev->dev, "unable to claim irq %d\n",
			gpio_to_irq(PB_INT));
		goto out3;
	}

	/* sys fs */
	key_dev = device_create(sec_class, NULL, 0, "%s", "sec_key");
	if(device_create_file(key_dev, &dev_attr_sec_key_pressed) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_sec_key_pressed.attr.name);

	powerkey_dev = device_create(sec_class, NULL, 0, NULL, "sec_powerkey");
	if (device_create_file(powerkey_dev, &dev_attr_sec_powerkey_pressed) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_sec_powerkey_pressed.attr.name);
	/* sys fs */
	dump_keypad_register();
	return 0;


out3:
	free_irq(gpio_to_irq(PB_INT), pdev);
	free_irq(sci_kpd->irq, pdev);
out2:
	platform_set_drvdata(pdev, NULL);
	input_unregister_device(input_dev);
out1:
	kfree(sci_kpd);
	input_free_device(input_dev);
out0:
	return error;
}

static int sci_keypad_remove(struct
				       platform_device
				       *pdev)
{
	unsigned long value;
	struct sci_keypad_t *sci_kpd = platform_get_drvdata(pdev);
	struct sci_keypad_platform_data *pdata = pdev->dev.platform_data;
	/* disable sci keypad controller */
	__raw_writel(KPD_INT_ALL, KPD_INT_CLR);
	value = __raw_readl(KPD_CTRL);
	value &= ~(1 << 0);
	__raw_writel(value, KPD_CTRL);

	__keypad_disable();
	wake_lock_destroy(&sci_kpd->key_wake_lock);
	free_irq(sci_kpd->irq, pdev);
	input_unregister_device(sci_kpd->input_dev);
	kfree(sci_kpd);
	platform_set_drvdata(pdev, NULL);
	if(pdev->dev.of_node){
		kfree(pdata->keymap_data->keymap);
		kfree(pdata->keymap_data);
		kfree(pdata);
	}
	return 0;
}

#ifdef CONFIG_PM
static int sci_keypad_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int sci_keypad_resume(struct platform_device *dev)
{
       struct sci_keypad_platform_data *pdata = dev->dev.platform_data;
	unsigned long value;

       __keypad_enable();
	__raw_writel(KPD_INT_ALL, KPD_INT_CLR);
	__raw_writel(CFG_ROW_POLARITY | CFG_COL_POLARITY, KPD_POLARITY);
	__raw_writel(1, KPD_CLK_DIV_CNT);
	__raw_writel(0xc, KPD_LONG_KEY_CNT);
	__raw_writel(0x5, KPD_DEBOUNCE_CNT);

	value = KPD_INT_DOWNUP;
	if (pdata->support_long_key)
		value |= KPD_INT_LONG;
	__raw_writel(value, KPD_INT_EN);
	value = KPD_SLEEP_CNT_VALUE(1000);
	__raw_writel(value, KPD_SLEEP_CNT);

	value =
	    KPD_EN | KPD_SLEEP_EN | pdata->
	    rows_choose_hw | pdata->cols_choose_hw;
	if (pdata->support_long_key)
		value |= KPD_LONG_KEY_EN;
	__raw_writel(value, KPD_CTRL);

	return 0;
}
#else
#define sci_keypad_suspend	NULL
#define sci_keypad_resume	NULL
#endif

static struct of_device_id keypad_match_table[] = {
	{ .compatible = "sprd,sci-keypad", },
	{ },
};
struct platform_driver sci_keypad_driver = {
	.probe = sci_keypad_probe,
	.remove = sci_keypad_remove,
	.suspend = sci_keypad_suspend,
	.resume = sci_keypad_resume,
	.driver = {
		   .name = "sci-keypad",.owner = THIS_MODULE,
		   .of_match_table = keypad_match_table,
		   },
};

static int __init sci_keypad_init(void)
{
#ifdef HOOK_POWER_KEY
	input_hook_init();
#endif
	return platform_driver_register(&sci_keypad_driver);
}

static void __exit sci_keypad_exit(void)
{
#ifdef HOOK_POWER_KEY
	input_hook_exit();
#endif
	platform_driver_unregister(&sci_keypad_driver);
}

module_init(sci_keypad_init);
module_exit(sci_keypad_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("spreadtrum.com");
MODULE_DESCRIPTION("Keypad driver for spreadtrum:questions contact steve zhan");
MODULE_ALIAS("platform:sci-keypad");
