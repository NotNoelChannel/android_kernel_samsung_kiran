/************************************************************************/
/*                                                                      */
/*  Copyright 2012  Broadcom Corporation                                */
/*                                                                      */
/* Unless you and Broadcom execute a separate written software license  */
/* agreement governing use of this software, this software is licensed  */
/* to you under the terms of the GNU General Public License version 2   */
/* (the GPL), available at						*/
/*                                                                      */
/*          http://www.broadcom.com/licenses/GPLv2.php                  */
/*                                                                      */
/*  with the following added to such license:                           */
/*                                                                      */
/*  As a special exception, the copyright holders of this software give */
/*  you permission to link this software with independent modules, and  */
/*  to copy and distribute the resulting executable under terms of your */
/*  choice, provided that you also meet, for each linked independent    */
/*  module, the terms and conditions of the license of that module. An  */
/*  independent module is a module which is not derived from this       */
/*  software.  The special   exception does not apply to any            */
/*  modifications of the software.					*/
/*									*/
/*  Notwithstanding the above, under no circumstances may you combine	*/
/*  this software in any way with any other Broadcom software provided	*/
/*  under a license other than the GPL, without Broadcom's express	*/
/*  prior written consent.						*/
/*									*/
/************************************************************************/
#include <linux/version.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/clk.h>
#include <linux/bootmem.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/i2c.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/mach/map.h>
#include <mach/board.h>
#include "devices.h"
#include <linux/broadcom/bcm-bt-rfkill.h>
#include <linux/broadcom/bcm-bt-lpm.h>

#ifdef CONFIG_BCM_BT_LPM

static struct bcm_bt_rfkill_platform_data bcm_bt_rfkill_cfg = {
	.bcm_bt_rfkill_vreg_gpio = GPIO_BT_POWER,
#ifdef CONFIG_BT_BCM4330
	.bcm_bt_rfkill_n_reset_gpio = 0,
#else
	.bcm_bt_rfkill_n_reset_gpio = GPIO_BT_RESET,
#endif
};

static struct platform_device bcm_bt_rfkill_device = {
	.name = "bcm-bt-rfkill",
	.id = -1,
	.dev =	{
		.platform_data = &bcm_bt_rfkill_cfg,
	},
};
static struct bcm_bt_lpm_platform_data brcm_bt_lpm_data = {
	.bt_wake_gpio = GPIO_BT2AP_WAKE,
	.host_wake_gpio = GPIO_AP2BT_WAKE,
};

static struct platform_device board_bcm_bt_lpm_device = {
	.name = "bcm-bt-lpm",
	.id = -1,
	.dev = {
		.platform_data = &brcm_bt_lpm_data,
	},
};

static struct platform_device *bt_devices[] __initdata = {
	&bcm_bt_rfkill_device,
	&board_bcm_bt_lpm_device,
};
void __init dev_bluetooth_init(void)
{
	platform_add_devices(bt_devices, ARRAY_SIZE(bt_devices));
	return;
}

#endif
