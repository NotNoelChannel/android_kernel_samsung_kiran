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

static int sensor_sr352_poweron(struct sensor_power *main_cfg, struct sensor_power *sub_cfg)
{
	int ret = 0;

	sensor_k_sensor_sel(SENSOR_SUB);
	sensor_k_set_pd_level(0);
	sensor_k_set_rst_level(0);
	sensor_k_sensor_sel(SENSOR_MAIN);//select main sensor(sensor sr352);
	sensor_k_set_pd_level(0);//3M Stby Off
	sensor_k_set_rst_level(0);//3M Reset Off
	sensor_k_set_mclk(SENSOR_DISABLE_MCLK);

	sensor_k_set_voltage_dvdd(SENSOR_VDD_CLOSED, SENSOR_SUB);
	sensor_k_set_voltage_dvdd(SENSOR_VDD_CLOSED, SENSOR_MAIN);
	sensor_k_set_voltage_iovdd(SENSOR_VDD_CLOSED);//IO vdd
	sensor_k_set_voltage_avdd(SENSOR_VDD_CLOSED);//anolog vdd
	udelay(1);

	// Sensor I/O : 1.8V On
	sensor_k_set_voltage_iovdd(SENSOR_VDD_1800MV);
	udelay(1);

	// Sensor AVDD : 2.8V On
	sensor_k_set_voltage_avdd(SENSOR_VDD_2800MV);
	udelay(1);

	// VT Core : 1.8V On
	sensor_k_set_voltage_dvdd(SENSOR_VDD_1800MV, SENSOR_SUB);
	udelay(1);

	// 3M Core : 1.2V On
	sensor_k_set_voltage_dvdd(SENSOR_VDD_1300MV, SENSOR_MAIN);
	mdelay(3);

	// Mclk Enable
	sensor_k_set_mclk(24);
	mdelay(3);

	// 3M Stby Enable
	sensor_k_set_pd_level(1);
	mdelay(10);

	// 3M Reset Enable
	sensor_k_set_rst_level(1);
	mdelay(1);

	printk("sensor_sr352_poweron OK \n");

	return ret;
}

static int sensor_sr352_poweroff(struct sensor_power *main_cfg, struct sensor_power *sub_cfg)
{
	int ret = 0;

	sensor_k_sensor_sel(SENSOR_MAIN);

	// 3M Reset Disable
	sensor_k_set_rst_level(0);
	mdelay(1);

	// Mclk Disable
	sensor_k_set_mclk(SENSOR_DISABLE_MCLK);
	mdelay(1);

	// 3M Stby Disable
	sensor_k_set_pd_level(0);
	mdelay(1);

	// 3M Core : 1.2V Off
	sensor_k_set_voltage_dvdd(SENSOR_VDD_CLOSED, SENSOR_MAIN);
	udelay(1);

	// VT Core : 1.8V Off
	sensor_k_set_voltage_dvdd(SENSOR_VDD_CLOSED, SENSOR_SUB);
	udelay(1);

	// Sensor AVDD : 2.8V Off
	sensor_k_set_voltage_avdd(SENSOR_VDD_CLOSED);
	udelay(1);

	// Sensor IO : 1.8V Off
	sensor_k_set_voltage_iovdd(SENSOR_VDD_CLOSED);
	udelay(1);

	printk("sensor_sr352_poweroff OK \n");

	return ret;

}

static int sensor_sr030pc50_poweron(struct sensor_power *main_cfg, struct sensor_power *sub_cfg)
{
	int ret = 0;

	sensor_k_sensor_sel(SENSOR_MAIN);
	sensor_k_set_pd_level(0);
	sensor_k_set_rst_level(0);
	sensor_k_sensor_sel(SENSOR_SUB);
	sensor_k_set_pd_level(0);
	sensor_k_set_rst_level(0);
	sensor_k_set_voltage_iovdd(SENSOR_VDD_CLOSED);//IO vdd
	sensor_k_set_voltage_avdd(SENSOR_VDD_CLOSED); //anolog vdd
	sensor_k_set_voltage_dvdd(SENSOR_VDD_CLOSED, SENSOR_SUB);
	sensor_k_set_voltage_dvdd(SENSOR_VDD_CLOSED, SENSOR_MAIN);
	udelay(1);

	// Sensor I/O : 1.8V On
	sensor_k_set_voltage_iovdd(SENSOR_VDD_1800MV);
	udelay(1);

	// Sensor AVDD : 2.8VOn
	sensor_k_set_voltage_avdd(SENSOR_VDD_2800MV);
	udelay(1);

	// VT Core : 1.8V On
	sensor_k_set_voltage_dvdd(SENSOR_VDD_1800MV, SENSOR_SUB);
	udelay(1);

	// 3M Core : 1.2V On
	sensor_k_set_voltage_dvdd(SENSOR_VDD_1300MV, SENSOR_MAIN);
	mdelay(3);

	// MCLK Enable
	sensor_k_set_mclk(24);

	// VT STBY Enable
	sensor_k_set_pd_level(1);
	mdelay(30);

	// VT Reset Enable
	sensor_k_set_rst_level(1);
	udelay(1);

	printk("sensor_sr030pc50_poweron OK \n");

	return ret;
}

static int sensor_sr030pc50_poweroff(struct sensor_power *main_cfg, struct sensor_power *sub_cfg)
{
	int ret = 0;

	sensor_k_sensor_sel(SENSOR_SUB);
	// VT Reset Disable
	sensor_k_set_rst_level(0);
	mdelay(5);

	// MCLK Disable
	sensor_k_set_mclk(0);
	mdelay(1);

	// VT STBY Disable
	sensor_k_set_pd_level(0);
	mdelay(1);

	// 3M Core : 1.2V Off
	sensor_k_set_voltage_dvdd(SENSOR_VDD_CLOSED, SENSOR_MAIN);
	udelay(1);

	// VT Core : 1.8V Off
	sensor_k_set_voltage_dvdd(SENSOR_VDD_CLOSED, SENSOR_SUB);
	udelay(1);

	// Sensor AVDD : 2.8V Off
	sensor_k_set_voltage_avdd(SENSOR_VDD_CLOSED);
	udelay(1);

	// Sensor I/O : 1.8V Off
	sensor_k_set_voltage_iovdd(SENSOR_VDD_CLOSED);
	udelay(1);
	printk("sensor_sr030pc50_poweroff OK \n");

	return ret;
}

int sensor_power_on(uint8_t sensor_id, struct sensor_power *main_cfg, struct sensor_power *sub_cfg)
{
	int ret = 0;

	if (!main_cfg || !sub_cfg) {
		printk("sensor_power_on, para err 0x%x 0x%x \n", main_cfg, sub_cfg);
	}
	if (SENSOR_MAIN == sensor_id) {
		ret = sensor_sr352_poweron(main_cfg, sub_cfg);
	} else {
		ret = sensor_sr030pc50_poweron(main_cfg, sub_cfg);
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
		ret = sensor_sr352_poweroff(main_cfg, sub_cfg);
	} else {
		ret = sensor_sr030pc50_poweroff(main_cfg, sub_cfg);
	}

	return ret;
}
