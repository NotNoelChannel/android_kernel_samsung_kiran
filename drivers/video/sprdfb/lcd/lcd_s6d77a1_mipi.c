/* drivers/video/sprdfb/lcd_s6d77a1_mipi.c
 *
 * Support for s6d77a1 mipi LCD device
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd
 *
 * Eunchul Kim <chulspro.kim@samsung.com>
 * Vijayakumar Kuppusamy <vijay.bvb@samsung.com>
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
#include <linux/bug.h>
#include <linux/delay.h>
#include "sprdfb_panel.h"
#include <mach/gpio.h>

#define  LCD_DEBUG
#ifdef LCD_DEBUG
#define LCD_PRINT printk
#else
#define LCD_PRINT(...)
#endif

#define MAX_DATA   150

#ifdef CONFIG_FB_ESD_SUPPORT
#define PANEL_POWER_MODE_REG 0x0A
#define PANEL_POWER_MODE_OFFSET 1
#define PANEL_POWER_MODE_LEN 2
#define PANEL_NOT_DEAD_STS 0x1C
#define PANEL_READ_CNT 4
#define DATA_TYPE_SET_RX_PKT_SIZE 0x37
#endif

struct lcm_init_code {
	unsigned int tag;
	unsigned char data[MAX_DATA];
};

struct lcm_force_cmd_code {
	unsigned int datatype;
	struct lcm_init_code real_cmd_code;
};

#define LCM_TAG_SHIFT 24
#define LCM_TAG_MASK  ((1 << 24) -1)
#define LCM_SEND(len) ((1 << LCM_TAG_SHIFT)| len)
#define LCM_SLEEP(ms) ((2 << LCM_TAG_SHIFT)| ms)

#define LCM_TAG_SEND  (1<< 0)
#define LCM_TAG_SLEEP (1 << 1)

struct lcm_init_code *g_init_data = NULL;
static struct lcm_init_code init_data[] = {
	/* Password */
	{LCM_SEND(5),{3,0,0xF0,0x5A,0x5A}},
	{LCM_SEND(5),{3,0,0xFC,0xA5,0xA5}},

	/* Resolution Control */
	{LCM_SEND(2),{0xB3, 0x01}},
	{LCM_SEND(2),{0xB5, 0x10}},
	{LCM_SEND(6),{4,0,0xBC,0x00,0x29,0x51}}, /* 141006 Battery detect function disabled */

	/* MIPI SETTING */
	{LCM_SEND(5),{3,0,0xE3,0x24,0x2C}},

	/* Display Inversion Off */
	{LCM_SEND(1),{0x21}},

	/* Display Control */
	{LCM_SEND(11), {9,0,0xF2,0x11,0x04,0x08,0x14,0x08,0x69,0x73,0x0A}},

	/* power control 1 */
	{LCM_SEND(8),{6,0,0xF3,0x91,0x00,0x00,0x00,0x10}},

	/* power control 2 */
	{LCM_SEND(46),{44,0,0xF4,0x01,0x02,0x23,0x24,0x25,0x25,0x26,0x26,0x29,0x29,0x2C,0x2B,0x2C,0x07,0x08,0x05,0x04,0x04,0x01,0x01,0x22,0x0A,0x17,0x0F,0x11,0x23,0x0D,0x0A,0x0B,0x02,0x19,0x13,0x1E,0x1F,0x23,0x20,0x05,0x16,0x1A,0x16,0x1E,0x24,0x1E}},

	/* power control 3 */
	{LCM_SEND(31),{29,0,0xF5,0x14,0x14,0xB9,0x21,0x35,0x55,0x36,0x0A,0x00,0x39,0x00,0x00,0x0B,0x4B,0xC4,0xCB,0x4B,0x12,0x12,0x1A,0x00,0x10,0xE0,0xE2,0x0E,0x34,0x34,0x03}},

	/* 9.4 */
	{LCM_SEND(20),{18,0,0xFE,0x00,0x02,0x01,0x39,0x60,0x40,0x21,0x00,0x4B,0x00,0x80,0x00,0xF0,0x00,0x00,0x00,0x04}},

	/* ASG EQ Control */
	{LCM_SEND(13),{11,0,0xEE,0x40,0xA0,0x40,0xA0,0x00,0x00,0x00,0x00,0x00,0x00}},

	{LCM_SEND(23),{21,0,0xEF,0x23,0x01,0x00,0x00,0x2A,0x49,0x08,0x27,0x21,0x40,0x10,0x24,0x02,0x21,0x21,0x03,0x03,0x40,0x00,0x08}},

	/* Panel Gate Control */
	{LCM_SEND(33),{31,0,0xF7,0x02,0x1B,0x1A,0x00,0x01,0x0E,0x0E,0x0A,0x0A,0x0F,0x0F,0x0B,0x0B,0x05,0x07,0x02,0x1B,0x1A,0x00,0x01,0x0C,0x0C,0x08,0x08,0x0D,0x0D,0x09,0x09,0x04,0x06}},

	/* Source Control */
	{LCM_SEND(9),{7,0,0xF6,0x63,0x25,0x15,0x00,0x00,0x20}},

	/* Memory Data Access Control */
	{LCM_SEND(2),{0x36,0x10}},

	/* Positive Gamma Control */
	{LCM_SEND(20),{18,0,0xFA,0x0C,0x36,0x0C,0x12,0x08,0x0D,0x11,0x11,0x15,0x1E,0x21,0x21,0x22,0x24,0x26,0x29,0x33}},

	/* Negative Gamma Control */
	{LCM_SEND(20),{18,0,0xFB,0x0C,0x36,0x0C,0x12,0x08,0x0D,0x11,0x0F,0x0F,0x16,0x1D,0x20,0x22,0x22,0x24,0x29,0x33}},

	{LCM_SEND(5),{3,0,0xFC,0x5A,0x5A}},

	/* display on */
	{LCM_SEND(1),{0x11}},
	{LCM_SLEEP(120)},

	{LCM_SEND(1),{0x29}},
	{LCM_SLEEP(120)},
};

static struct lcm_init_code sleep_in[] =  {
	{LCM_SEND(1), {0x28}},
	{LCM_SLEEP(150)}, 	/* > 10ms */
	{LCM_SEND(1), {0x10}},
	{LCM_SLEEP(150)},	/* > 120ms */
};

static struct lcm_init_code sleep_out[] =  {
	{LCM_SEND(1), {0x11}},
	{LCM_SLEEP(120)},/* > 120ms */
	{LCM_SEND(1), {0x29}},
	{LCM_SLEEP(20)}, /* 10ms ~ 50ms */
};

static int32_t s6d77a1_mipi_init(struct panel_spec *self) {
	int32_t i = 0;
	struct lcm_init_code *init;
	unsigned int tag;
	struct ops_mipi *mipi_ops = self->info.mipi->ops;
	mipi_set_cmd_mode_t mipi_set_cmd_mode = mipi_ops->mipi_set_cmd_mode;
	mipi_dcs_write_t mipi_dcs_write = mipi_ops->mipi_dcs_write;
	mipi_eotp_set_t mipi_eotp_set = mipi_ops->mipi_eotp_set;

	LCD_PRINT("kernel s6d77a1_mipi_init\n");

	if (g_init_data == NULL)
		g_init_data = init_data;
	init = g_init_data;
	mipi_set_cmd_mode();
	mipi_eotp_set(0, 0);

	for (i = 0; i < ARRAY_SIZE(init_data); i++) {
		tag = (init->tag >> 24);
		if (tag & LCM_TAG_SEND)
			mipi_dcs_write(init->data, (init->tag & LCM_TAG_MASK));
		else if (tag & LCM_TAG_SLEEP)
			msleep(init->tag & LCM_TAG_MASK);

		init++;
	}
	mipi_eotp_set(0, 0);

	return 0;
}

static uint32_t s6d77a1_readid(struct panel_spec *self)
{
	int32_t j = 0;
	uint8_t read_data[4] = { 0 };
	int32_t read_rtn = 0;
	uint8_t param[2] = { 0 };
	struct ops_mipi *mipi_ops = self->info.mipi->ops;
	mipi_set_cmd_mode_t mipi_set_cmd_mode = mipi_ops->mipi_set_cmd_mode;
	mipi_force_write_t mipi_force_write = mipi_ops->mipi_force_write;
	mipi_force_read_t mipi_force_read = mipi_ops->mipi_force_read;
	mipi_eotp_set_t mipi_eotp_set = mipi_ops->mipi_eotp_set;

	LCD_PRINT("kernel lcd_s6d77a1_mipi read id!\n");

	mipi_set_cmd_mode();
	mipi_eotp_set(0, 0);
	for (j = 0; j < 4; j++) {
		param[0] = 0x03;
		param[1] = 0x00;
		mipi_force_write(0x37, param, 2);
		read_rtn = mipi_force_read(0x04, 3, read_data);
		LCD_PRINT(
				"lcd_s6d77a1_mipi read id 0xda, 0xdb,0xdc is 0x%x,0x%x,0x%x!\n",
				read_data[0], read_data[1], read_data[2]);
		if ((0x55 == read_data[0]) && (0xB8 == read_data[1])
				&& (0x10 == read_data[2])) {
			LCD_PRINT("lcd_s6d77a1_mipi read id success!\n");
			return 0x55B810;
		}
	}
	mipi_eotp_set(0, 0);

	return 0;
}

static int32_t s6d77a1_enter_sleep(struct panel_spec *self, uint8_t is_sleep)
{
	int32_t i = 0;
	struct lcm_init_code *sleep_in_out = NULL;
	unsigned int tag;
	int32_t size = 0;
	struct ops_mipi *mipi_ops = self->info.mipi->ops;
	mipi_set_cmd_mode_t mipi_set_cmd_mode = mipi_ops->mipi_set_cmd_mode;
	mipi_dcs_write_t mipi_dcs_write = mipi_ops->mipi_dcs_write;
	mipi_eotp_set_t mipi_eotp_set = mipi_ops->mipi_eotp_set;

	LCD_PRINT("kernel s6d77a1_enter_sleep, is_sleep = %d\n", is_sleep);

	if (is_sleep) {
		sleep_in_out = sleep_in;
		size = ARRAY_SIZE(sleep_in);
	} else {
		sleep_in_out = sleep_out;
		size = ARRAY_SIZE(sleep_out);
	}

	mipi_eotp_set(0, 0);

	for (i = 0; i < size; i++) {
		tag = (sleep_in_out->tag >> 24);
		if (tag & LCM_TAG_SEND)
			mipi_dcs_write(sleep_in_out->data,
					(sleep_in_out->tag & LCM_TAG_MASK));
		else if (tag & LCM_TAG_SLEEP)
			msleep(sleep_in_out->tag & LCM_TAG_MASK);

		sleep_in_out++;
	}
	mipi_eotp_set(0, 0);

	return 0;
}

#ifdef CONFIG_FB_ESD_SUPPORT
static int sprdfb_s6d77a1_dsi_rx_cmds(struct panel_spec *panel, u8 regs,
		u16 get_len, u8 *data, bool in_bllp)
{
	int ret;
	struct ops_mipi *ops = panel->info.mipi->ops;
	u8 pkt_size[2];
	u16 work_mode = panel->info.mipi->work_mode;

	pr_debug("%s, read data len: %d\n", __func__, get_len);
	if (!data) {
		pr_err("Expected registers or data is NULL.\n");
		return -EIO;
	}
	if (!ops->mipi_set_cmd_mode || !ops->mipi_force_read
			|| !ops->mipi_force_write) {
		pr_err("%s: Expected functions are NULL.\n", __func__);
		return -EFAULT;
	}

	if (!in_bllp) {
		if (work_mode == SPRDFB_MIPI_MODE_CMD)
			ops->mipi_set_lp_mode();
		else
			ops->mipi_set_data_lp_mode();
		/*
		 * If cmds is transmitted in LP mode, not in BLLP,
		 * commands mode should be enabled
		 * */
		ops->mipi_set_cmd_mode();
	}

	pkt_size[0] = get_len & 0xff;
	pkt_size[1] = (get_len >> 8) & 0xff;

	ops->mipi_force_write(DATA_TYPE_SET_RX_PKT_SIZE, pkt_size,
			sizeof(pkt_size));

	ret = ops->mipi_force_read(regs, get_len, data);
	if (ret < 0) {
		pr_err("%s: dsi read id fail\n", __func__);
		return -EINVAL;
	}

	if (!in_bllp) {
		if (work_mode == SPRDFB_MIPI_MODE_CMD)
			ops->mipi_set_hs_mode();
		else
			ops->mipi_set_data_hs_mode();
	}

	return 0;
}

static int s6d77a1_dsi_panel_dead(struct panel_spec *panel)
{
	int i, cnt = PANEL_READ_CNT;
	u8 regs = PANEL_POWER_MODE_REG;
	u8 data[PANEL_POWER_MODE_LEN] = { 0 };
	bool in_bllp = false;

#ifdef FB_CHECK_ESD_IN_VFP
	in_bllp = true;
#endif
	do {
		sprdfb_s6d77a1_dsi_rx_cmds(panel, regs, PANEL_POWER_MODE_LEN, data,
				in_bllp);
		i = PANEL_POWER_MODE_OFFSET;
		pr_debug("%s: reading panel power mode(0x%2x) is 0x%x", __func__, regs,
				data[i]);
		if ((data[i] & PANEL_NOT_DEAD_STS) == PANEL_NOT_DEAD_STS)
			return true;
	} while (cnt-- > 0);

	pr_err("panel is dead, read power mode is 0x%x.\n", data[i]);
	return false;
}
#endif

static struct panel_operations lcd_s6d77a1_mipi_operations = {
	.panel_init = s6d77a1_mipi_init,
	.panel_readid = s6d77a1_readid,
	.panel_enter_sleep = s6d77a1_enter_sleep,
#ifdef CONFIG_FB_ESD_SUPPORT
	.panel_esd_check = s6d77a1_dsi_panel_dead,
#endif
};

static struct timing_rgb lcd_s6d77a1_mipi_timing = {
	.hfp = 115,  /* unit: pixel */
	.hbp = 55,
	.hsync = 50,
	.vfp = 8, /* unit: line */
	.vbp = 16,
	.vsync = 4,
};

static struct info_mipi lcd_s6d77a1_mipi_info = {
	.work_mode  = SPRDFB_MIPI_MODE_VIDEO,
	.video_bus_width = 24, /* 18, 16 */
	.lan_number = 2,
	.phy_feq = 466*1000,
	.h_sync_pol = SPRDFB_POLARITY_POS,
	.v_sync_pol = SPRDFB_POLARITY_POS,
	.de_pol = SPRDFB_POLARITY_POS,
	.te_pol = SPRDFB_POLARITY_POS,
	.color_mode_pol = SPRDFB_POLARITY_NEG,
	.shut_down_pol = SPRDFB_POLARITY_NEG,
	.timing = &lcd_s6d77a1_mipi_timing,
	.ops = NULL,
};

struct panel_spec lcd_s6d77a1_mipi_spec = {
	.width = 480,
	.height = 800,
	.width_mm = 52,
	.height_mm = 87,
	.fps	= 60,
	.suspend_mode = SEND_SLEEP_CMD,
	.type = LCD_MODE_DSI,
	.direction = LCD_DIRECT_NORMAL,
	.is_clean_lcd = true,
	.info = {
		.mipi = &lcd_s6d77a1_mipi_info
	},
	.ops = &lcd_s6d77a1_mipi_operations,
};

struct panel_cfg lcd_s6d77a1_mipi = {
	/* this panel can only be main lcd */
	.dev_id = SPRDFB_MAINLCD_ID,
	.lcd_id = 0x55b810,
	.lcd_name = "lcd_s6d77a1_mipi",
	.panel = &lcd_s6d77a1_mipi_spec,
};

static int __init lcd_s6d77a1_mipi_init(void)
{
	return sprdfb_panel_register(&lcd_s6d77a1_mipi);
}

subsys_initcall(lcd_s6d77a1_mipi_init);
