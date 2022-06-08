/*
 * Copyright (C) 2014 Samsung Electronics Co.Ltd
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/ctype.h>
#include <linux/miscdevice.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fb.h>
#include <linux/ioctl.h>
#include <linux/lcd.h>
#include <linux/string.h>
#include "../sprdfb_panel.h"

#include "mdnie_lite_tuning.h"
#include "sprdfb_mdnie_lite_tuning.h"

#define MAX_FILE_NAME 200
#define MAX_NUM_LINES 400
#define TUNING_FILE_PATH "/opt/usr"
#define TUNING_FILE_NAME "tuning.txt"
#define MAX_LUT_SIZE 256

#define MDNIE_LITE_TUN_DEBUG
#ifdef MDNIE_LITE_TUN_DEBUG
#define DEBUG_MDNIE(x...)	pr_info("[mdnie] " x)
#else
#define DEBUG_MDNIE(x...)
#endif

struct mdnie_lite_tun_type mdnie_tun_state = {
	.mdnie_enable = false,
	.scenario = MDNIE_UI_MODE,
	.background = STANDARD_MODE,
	.outdoor = OUTDOOR_OFF_MODE,
};

const char background_name[MAX_BACKGROUND_MODE][16] = {
	"DYNAMIC",
	"STANDARD",
	"MOVIE",
	"NATURAL",
};

const char scenario_name[MAX_MDNIE_MODE][16] = {
	"UI_MODE",
	"GALLERY_MODE",
	"VIDEO_MODE",
	"VT_MODE",
	"CAMERA_MODE",
	"BROWSER_MODE",
	"NEGATIVE_MODE",
	"EMAIL_MODE",
	"EBOOK_MODE",
	"GRAY_MODE",
};

static struct class *mdnie_class;
struct device *tune_mdnie_dev;
static struct panel_spec *panel_info;
struct sprdfb_device *sprd_fb_dev;
static int load_tuning_data(char *filename);
static int parse_text(char *src, int len);
sprd_mdnie_data cur_mode_settings[SPRD_NUM_MDNIE_REGS];

static int parse_text(char *src, int len)
{
	int i, j, count;
	int end_of_reg_set = 0, num_of_regs = 0;
	char *EOR = "*";
	char *str_line[MAX_NUM_LINES], *sstart, *c, *str;
	unsigned int data1;

	c = src;
	count = 0;
	sstart = c;

	for (i = 0; i < len; i++, c++) {
		char a = *c;
		if (a == '\r' || a == '\n') {
			if (c > sstart) {
				str_line[count] = sstart;
				count++;
			}
			/* add "*" token to mark end of register set */
			if (c == sstart) {
				str_line[count] = EOR;
				count++;
			}
			*c = '\0';
			sstart = c + 1;
		}
	}

	if (c > sstart) {
		str_line[count] = sstart;
		count++;
	}

	j = 0; num_of_regs = 0; end_of_reg_set = 0;
	for (i = 0; i <= count; i++) {
		c = str_line[i];
		if (i == count)
			str_line[count] = EOR;
		if (!strcmp(str_line[i], EOR)) {
			if (end_of_reg_set == 2) {
				/* detect end of register set */
				cur_mode_settings[num_of_regs].cmd_type = 0x39;
				cur_mode_settings[num_of_regs].len = j;
				j = 0; ++num_of_regs;
			} else
				++end_of_reg_set;
		} else {
			end_of_reg_set = 0;
			/* Ignore lines starting with // */
			if ((str = strsep((char **)&c, "//"))) {
				if ((str != '\0') && (strlen(str) != 0)) {
					sscanf(str, "0x%X,", &data1);
					cur_mode_settings[num_of_regs].data[j++] = data1;
				} else
					/* detect end of register set */
					cur_mode_settings[num_of_regs].len = j;
			}
		}
	}

#ifdef MDNIE_LITE_TUN_DEBUG
	for (i = 0; i < num_of_regs; i++) {
		DEBUG_MDNIE("\n[%d] ", cur_mode_settings[i].len);
		for (j = 0; j < cur_mode_settings[i].len; j++)
			DEBUG_MDNIE("0x%X ", cur_mode_settings[i].data[j]);
	}
#endif
	return num_of_regs;
}

static int load_tuning_data(char *filename)
{
	char *dp;
	long l;
	int ret, num;
	loff_t pos;
	struct file *filp;
	mm_segment_t fs;

	DEBUG_MDNIE("[%s] loading file : [%s]\n", __func__, filename);

	fs = get_fs();
	set_fs(get_ds());
	filp = filp_open(filename, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		pr_err("[%s] File open failed\n", __func__);
		return -1;
	}

	l = filp->f_path.dentry->d_inode->i_size;
	dp = kmalloc(l + 10, GFP_KERNEL);
	if (dp == NULL) {
		pr_err("[%s] mem alloc fail for tuning file\n", __func__);
		filp_close(filp, current->files);
		return -1;
	}

	pos = 0;
	memset(dp, 0, l);
	ret = vfs_read(filp, (char __user *)dp, l, &pos);
	if (ret != l) {
		pr_err("[mDNIe] : vfs_read() filed ret : %d\n", ret);
		kfree(dp);
		filp_close(filp, current->files);
		return -1;
	}

	filp_close(filp, current->files);

	set_fs(fs);
	num = parse_text(dp, l);
	if (!num) {
		pr_err("[mDNIe]:Nothing to parse\n");
		kfree(dp);
		return -1;
	}

	kfree(dp);
	return num;
}

void sprdfb_dsi_cmds_send(sprd_mdnie_data *tun_data, int size)
{
	int i;

	mipi_write_t mipi_write =
				panel_info->info.mipi->ops->mipi_write;
	mipi_eotp_set_t mipi_eotp_set =
				panel_info->info.mipi->ops->mipi_eotp_set;

	mipi_eotp_set(0, 0);
#ifdef CONFIG_FB_VSYNC_SUPPORT
	sprdfb_dispc_wait_for_vsync(sprd_fb_dev);
#endif
	for (i = 0; i < size; i++) {
		mipi_write(tun_data->cmd_type, tun_data->data, tun_data->len);
		tun_data++;
	}
	mipi_eotp_set(0, 0);
}

void send_tuning_cmd(void)
{
	DEBUG_MDNIE(" send tuning cmd!!\n");
	sprdfb_dsi_cmds_send((sprd_mdnie_data *)&cur_mode_settings,
					ARRAY_SIZE(cur_mode_settings));
}

void mdnie_set_mode(enum lcd_mdnie_mode mode)
{
	char filename[MAX_FILE_NAME];
	DEBUG_MDNIE("mdnie_set_mode start , mode(%d), background(%d)\n",
		mode, mdnie_tun_state.background);

	if (!mdnie_tun_state.mdnie_enable) {
		pr_err("[ERROR] mDNIE engine is OFF.\n");
		return;
	}

/* FIXME:!! Currently use the default mode to set the mdnie values from
	/opt/usr/tuning.txt
	if (mode < MDNIE_UI_MODE || mode >= MAX_MDNIE_MODE) {
		pr_err("wrong Scenario mode value : %d\n",	mode);
		return;
	}
*/

	switch (mode) {
	case MDNIE_UI_MODE:
		DEBUG_MDNIE(" = UI MODE =\n");
		sprdfb_dsi_cmds_send((sprd_mdnie_data *)&ui_mode,
						ARRAY_SIZE(ui_mode));
		break;

	case MDNIE_VIDEO_MODE:
		DEBUG_MDNIE(" = VIDEO MODE =\n");
		sprdfb_dsi_cmds_send((sprd_mdnie_data *)&video_mode,
						ARRAY_SIZE(video_mode));
		break;

	case MDNIE_CAMERA_MODE:
		DEBUG_MDNIE(" = CAMERA MODE =\n");
		sprdfb_dsi_cmds_send((sprd_mdnie_data *)&camera_mode,
						ARRAY_SIZE(camera_mode));
		break;

	case MDNIE_GALLERY:
		DEBUG_MDNIE(" = GALLERY MODE =\n");
		sprdfb_dsi_cmds_send((sprd_mdnie_data *)&gallery_mode,
						ARRAY_SIZE(gallery_mode));
		break;

	case MDNIE_VT_MODE:
		DEBUG_MDNIE(" = VT MODE =\n");
		sprdfb_dsi_cmds_send((sprd_mdnie_data *)&vtcall_mode,
						ARRAY_SIZE(vtcall_mode));
		break;

	case MDNIE_BROWSER_MODE:
		DEBUG_MDNIE(" = BROWSER MODE =\n");
		sprdfb_dsi_cmds_send((sprd_mdnie_data *)&browser_mode,
						ARRAY_SIZE(browser_mode));
		break;

	case MDNIE_EMAIL_MODE:
		DEBUG_MDNIE(" = EMAIL MODE =\n");
		sprdfb_dsi_cmds_send((sprd_mdnie_data *)&email_mode,
						ARRAY_SIZE(email_mode));
		break;

	case MDNIE_EBOOK_MODE:
		DEBUG_MDNIE(" = EBOOK MODE =\n");
		sprdfb_dsi_cmds_send((sprd_mdnie_data *)&ebook_mode,
						ARRAY_SIZE(ebook_mode));
		break;

	case MDNIE_GRAY_MODE:
		DEBUG_MDNIE(" = Ultra low power (gray) MODE =\n");
		sprdfb_dsi_cmds_send((sprd_mdnie_data *)&gray_mode,
						ARRAY_SIZE(gray_mode));
		break;

	default:
		DEBUG_MDNIE("[%s] no option (%d), so default mode\n", __func__, mode);
		sprintf(filename, "%s/%s", TUNING_FILE_PATH, TUNING_FILE_NAME);
		DEBUG_MDNIE("Set from tuning file %s\n", filename);
		if(load_tuning_data(filename) == -1) {
			pr_err("[%s] No tuning file\n", __func__);
			return;
		}
		send_tuning_cmd();
	}

	DEBUG_MDNIE("mdnie_set_mode end , mode(%d), background(%d)\n",
		mode, mdnie_tun_state.background);

	return;
}

void mdnie_set_outdoor_mode(void)
{
	if (!mdnie_tun_state.mdnie_enable) {
		pr_err("[ERROR] mDNIE engine is OFF.\n");

		goto exit;
	}

	DEBUG_MDNIE(" = OUTDOOR MODE =\n");

	sprdfb_dsi_cmds_send((sprd_mdnie_data *) &outdoor_mode,
			ARRAY_SIZE(outdoor_mode));

exit:
	return;
}

static ssize_t scenario_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	DEBUG_MDNIE("called %s\n", __func__);

	DEBUG_MDNIE("Current Scenario Mode %s\n",
			scenario_name[mdnie_tun_state.scenario]);

	return snprintf(buf, 4, "%d\n", mdnie_tun_state.scenario);
}

static ssize_t scenario_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);
/*
	if (value < MDNIE_UI_MODE || value >= MAX_MDNIE_MODE) {
		pr_err("[ERROR] wrong Scenario mode value : %d\n",
			value);
		return size;
	}
*/
	switch (value) {
	case SIG_MDNIE_UI_MODE:
		mdnie_tun_state.scenario = MDNIE_UI_MODE;
		break;

	case SIG_MDNIE_VIDEO_MODE:
		mdnie_tun_state.scenario = MDNIE_VIDEO_MODE;
		break;

	case SIG_MDNIE_CAMERA_MODE:
		mdnie_tun_state.scenario = MDNIE_CAMERA_MODE;
		break;

	case SIG_MDNIE_GALLERY:
		mdnie_tun_state.scenario = MDNIE_GALLERY;
		break;

	case SIG_MDNIE_VT:
		mdnie_tun_state.scenario = MDNIE_VT_MODE;
		break;

	case SIG_MDNIE_BROWSER:
		mdnie_tun_state.scenario = MDNIE_BROWSER_MODE;
		break;

	case SIG_MDNIE_EMAIL:
		mdnie_tun_state.scenario = MDNIE_EMAIL_MODE;
		break;

	case SIG_MDNIE_EBOOK:
		mdnie_tun_state.scenario = MDNIE_EBOOK_MODE;
		break;

	case SIG_MDNIE_GRAY:
		mdnie_tun_state.scenario = MDNIE_GRAY_MODE;
		break;

	default:
		DEBUG_MDNIE("scenario_store value is wrong : value(%d)\n",
		       value);
		mdnie_tun_state.scenario = value;
		break;
	}

	DEBUG_MDNIE(" %s, input value = %d\n", __func__, value);
	mdnie_set_mode(mdnie_tun_state.scenario);

	return size;
}

static ssize_t outdoor_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	DEBUG_MDNIE("called %s\n", __func__);

	return snprintf(buf, 4, "%d\n", mdnie_tun_state.outdoor);
}

static ssize_t outdoor_store(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);

	DEBUG_MDNIE("outdoor value = %d, scenario = %d\n",
		value, mdnie_tun_state.scenario);

	if (value < OUTDOOR_OFF_MODE || value >= MAX_OUTDOOR_MODE) {
		pr_err("wrong outdoor mode value : %d\n", value);

		goto exit;
	}

	mdnie_tun_state.outdoor = value;

	if (mdnie_tun_state.outdoor)
		mdnie_set_outdoor_mode();

exit:
	return size;
}

static ssize_t mdnie_current_settings_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int pos = 0, i, j;
	uint8_t cur_settings[30] = { 0 };
	uint8_t param[2] = { 0 };

	mipi_eotp_set_t mipi_eotp_set =
				panel_info->info.mipi->ops->mipi_eotp_set;
	mipi_force_read_t mipi_force_read =
				panel_info->info.mipi->ops->mipi_force_read;
	mipi_force_write_t mipi_force_write =
				panel_info->info.mipi->ops->mipi_force_write;

	pr_info("%s: MDNIE Current settings\n", __func__);
	mipi_eotp_set(0, 0);
	for (i = 0; i < SPRD_NUM_MDNIE_REGS; i++) {
		pos += sprintf(buf+pos, "\n 0x%x ", cmds[i]);
		param[0] = SPRD_MDNIE_PARAMS[i];
		param[1] = 0x00;
		mipi_force_write(0x37, param, 2);
		mipi_force_read(cmds[i], SPRD_MDNIE_PARAMS[i], cur_settings);
		for (j = 0; j < SPRD_MDNIE_PARAMS[i]; j++)
			pos += sprintf(buf+pos, "0x%X ", cur_settings[j]);
	}
	mipi_eotp_set(0, 0);

	return pos;
}

static struct device_attribute mdnie_attributes[] = {
	__ATTR(current_settings, 0444, mdnie_current_settings_show, NULL),
	__ATTR(scenario, 0664, scenario_show, scenario_store),
	__ATTR(outdoor, 0664, outdoor_show, outdoor_store),
};

void init_mdnie_class(void)
{
	int i;
	DEBUG_MDNIE("start!\n");

	mdnie_class = class_create(THIS_MODULE, "extension");
	if (IS_ERR(mdnie_class))
		pr_err("Failed to create class(mdnie)!\n");

	tune_mdnie_dev = device_create(mdnie_class, NULL, 0, NULL, "mdnie");
	if (IS_ERR(tune_mdnie_dev))
		pr_err("Failed to create device(mdnie)!\n");

	for (i = 0; i < ARRAY_SIZE(mdnie_attributes); i++) {
		if (device_create_file(tune_mdnie_dev, &mdnie_attributes[i]) < 0)
			pr_err("Failed to create device file(%s)!\n",
				mdnie_attributes[i].attr.name);
	}

	mdnie_tun_state.mdnie_enable = true;
	DEBUG_MDNIE("end!\n");
}

void mdnie_lite_tuning_init(struct sprdfb_device *dev)
{
	sprd_fb_dev = dev;
	panel_info = dev->panel;
}
