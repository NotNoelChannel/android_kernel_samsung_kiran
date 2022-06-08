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
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <mach/hardware.h>
#include <mach/board.h>
#include "../sensor_drv_sprd.h"

static int sensor_hi544_poweron(struct sensor_power *main_cfg, struct sensor_power *sub_cfg)
{
	int ret = 0;

	/*set default status for main and sub sensor*/
	sensor_k_sensor_sel(SENSOR_SUB);//select sub sensor(sensor hi255);
	sensor_k_set_pd_level(0);//power down valid for hi255
	sensor_k_set_rst_level(0);//reset valid for hi255

	sensor_k_sensor_sel(SENSOR_MAIN);//select main sensor(sensor hi255);
	sensor_k_set_pd_level(0);//power down valid for hi544
	sensor_k_set_rst_level(0);//reset valid for hi544
	sensor_k_set_mclk(SENSOR_DISABLE_MCLK);	
	sensor_k_set_voltage_cammot(SENSOR_VDD_CLOSED);//AF monitor
	sensor_k_set_voltage_iovdd(SENSOR_VDD_CLOSED);//IO vdd
	sensor_k_set_voltage_avdd(SENSOR_VDD_CLOSED);//anolog vdd	
	mdelay(1);

	/*power on sequence*/
	sensor_k_set_voltage_cammot(SENSOR_VDD_2800MV);//AF monitor
	mdelay(4);//delay >= 2 ms
	sensor_k_set_voltage_iovdd(SENSOR_VDD_1800MV);//IO vdd
	mdelay(1);//delay < 10ms
	sensor_k_set_voltage_avdd(SENSOR_VDD_2800MV);//anolog vdd
	mdelay(1);
	sensor_k_set_pd_level(1);//power down invalid for hi544
	mdelay(1);
	sensor_k_set_mclk(24);
	mdelay(20);//delay >= 10ms
	sensor_k_set_rst_level(1);//reset invalid for hi544	
	mdelay(2);//delay >= 16 MCLK
	printk("hi544_poweron OK \n");
	return ret;
}

static int sensor_hi544_poweroff(struct sensor_power *main_cfg, struct sensor_power *sub_cfg)
{
	int ret = 0;

	sensor_k_sensor_sel(SENSOR_MAIN);//select main sensor(sensor hi255);
	mdelay(2);//delay >= 16 MCLK
	sensor_k_set_rst_level(0);//reset valid for hi544	
	mdelay(20);//delay >= 10ms
	sensor_k_set_mclk(0);// disable mclk
	mdelay(1);
	sensor_k_set_pd_level(0);//power down valid for hi544
	mdelay(1);
	sensor_k_set_voltage_avdd(SENSOR_VDD_CLOSED);//close anolog vdd
	mdelay(1);//delay < 10ms
	sensor_k_set_voltage_cammot(SENSOR_VDD_CLOSED);//AF monitor
	mdelay(1);//delay > 9 us
	sensor_k_set_voltage_iovdd(SENSOR_VDD_CLOSED);
	mdelay(1);
	printk("hi544_poweroff OK \n");
	return ret;

}

static int sensor_hi255_poweron(struct sensor_power *main_cfg, struct sensor_power *sub_cfg)
{
	int ret = 0;

	sensor_k_sensor_sel(SENSOR_MAIN);//select main sensor(sensor hi544);
	sensor_k_set_pd_level(0);//power down valid for hi544
	sensor_k_sensor_sel(SENSOR_SUB);//select main sensor(sensor hi544);
	sensor_k_set_pd_level(0);//power down valid for hi255
	sensor_k_set_rst_level(0);//reset valid for hi255
	sensor_k_set_voltage_iovdd(SENSOR_VDD_CLOSED);//IO vdd
	sensor_k_set_voltage_avdd(SENSOR_VDD_CLOSED); //anolog vdd	
	mdelay(1);
	
	sensor_k_set_voltage_iovdd(SENSOR_VDD_1800MV);//IO vdd
	mdelay(1);//delay < 10ms
	sensor_k_set_voltage_avdd(SENSOR_VDD_2800MV);
	mdelay(2);//delay > 1ms
	sensor_k_set_pd_level(1);
	mdelay(2);//delay > 1ms
	sensor_k_set_mclk(24);
	mdelay(50);// >= 30ms
	sensor_k_set_rst_level(1);
	mdelay(2);//delay > 16 MCLK
	printk("hi255_poweron OK \n");

	return ret;
}

static int sensor_hi255_poweroff(struct sensor_power *main_cfg, struct sensor_power *sub_cfg)
{
	int ret = 0;

	sensor_k_sensor_sel(SENSOR_SUB);//select main sensor(sensor hi255);
	mdelay(2);//delay > 16 MCLK
	sensor_k_set_rst_level(0);//reset valid for hi544	
	mdelay(2);//delay > 16 MCLK
	sensor_k_set_mclk(0);// disable mclk
	mdelay(2);
	sensor_k_set_pd_level(0);//power down valid for hi255
	mdelay(2);//delay > 1 ms
	sensor_k_set_voltage_avdd(SENSOR_VDD_CLOSED);
	mdelay(1);//delay <10 ms
	sensor_k_set_voltage_iovdd(SENSOR_VDD_CLOSED);
	mdelay(1);
	printk("hi255_poweroff OK \n");

	return ret;
}

int sensor_power_on(uint8_t sensor_id, struct sensor_power *main_cfg, struct sensor_power *sub_cfg)
{
	int ret = 0;

	if (!main_cfg || !sub_cfg) {
		printk("sensor_power_on, para err 0x%x 0x%x \n", main_cfg, sub_cfg);
	}
	if (SENSOR_MAIN == sensor_id) {
		ret = sensor_hi544_poweron(main_cfg, sub_cfg);
	} else {
		ret = sensor_hi255_poweron(main_cfg, sub_cfg);
	}
	return ret;
}

int sensor_power_off(uint8_t sensor_id, struct sensor_power *main_cfg, struct sensor_power *sub_cfg)
{
	int ret = 0;

	if (!main_cfg || !sub_cfg) {
		printk("sensor_power_off, para err 0x%x 0x%x \n", main_cfg, sub_cfg);
	}

	if (SENSOR_MAIN == sensor_id) {
		ret = sensor_hi544_poweroff(main_cfg, sub_cfg);
	} else {
		ret = sensor_hi255_poweroff(main_cfg, sub_cfg);
	}

	return ret;
}
