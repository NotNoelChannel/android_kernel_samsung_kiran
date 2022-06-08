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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/adi.h>
#include <linux/leds.h>
#include <linux/mfd/sm5701_core.h>

int sprd_flash_on(void)
{
	printk("sprd_flash_on core3\n");
        sm5701_led_ready(MOVIE_MODE);
        SM5701_operation_mode_function_control();
        sm5701_set_fleden(SM5701_FLEDEN_ON_MOVIE);
}

int sprd_flash_high_light(void)
{
		printk("sprd_flash_high_light core3\n");
        sm5701_led_ready(FLASH_MODE);
        SM5701_operation_mode_function_control();
        sm5701_set_fleden(SM5701_FLEDEN_ON_FLASH);
}

int sprd_flash_close(void)
{
		printk("sprd_flash_close core3\n");
        sm5701_led_ready(LED_DISABLE);
        sm5701_set_fleden(SM5701_FLEDEN_DISABLED);
        SM5701_operation_mode_function_control();
}
