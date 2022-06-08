/*
 *
 * Zinitix touch driver
 *
 * Copyright (C) 2012 Zinitix Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ioctl.h>
#ifdef CONFIG_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/string.h>
#include <linux/semaphore.h>
#include <linux/kthread.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/input/mt.h>
#include <linux/i2c/zinitix_ts.h>

#include "zxt_bt432_ts.h"
#include "zxt_bt431_fw.h"

#define	ZINITIX_DEBUG		1

#define	TSP_NORMAL_EVENT_MSG	0

#define	I2C_RETRY_CNT	5

u8 m_FirmwareIdx = 0;

#define TSP_INIT_TEST_RATIO	100
#define zinitix_debug_msg(fmt, args...)	\
	if (misc_touch_dev->debug_mode)	\
		printk(KERN_INFO "zinitix : [%-18s:%5d]" fmt, \
		__func__, __LINE__, ## args);

#define zinitix_printk(fmt, args...)	\
		printk(KERN_INFO "zinitix : [%-18s:%5d]" fmt, \
		__func__, __LINE__, ## args);

struct _ts_zinitix_coord {
	u16	x;
	u16	y;
	u8	width;
	u8	sub_status;
#if (TOUCH_POINT_MODE == 2)
	u8	minor_width;
	u8	angle;
#endif
};

struct _ts_zinitix_point_info {
	u16	status;
#if (TOUCH_POINT_MODE == 1)
	u16	event_flag;
#else
	u8	finger_cnt;
	u8	time_stamp;
#endif
	struct _ts_zinitix_coord coord[MAX_SUPPORTED_FINGER_NUM];
};

#define TOUCH_V_FLIP	0x01
#define TOUCH_H_FLIP	0x02
#define TOUCH_XY_SWAP	0x04

 /*Test Mode (Monitoring Raw Data) */
/*---------------------------------------------------------------------*/
/* ITO */
/*---------------------------------------------------------------------*/
#define PDIFF_DEBUG	1

#define SEC_TSP_FACTORY_TEST	1	//for samsung

#define	SEC_DND_N_COUNT			16
#define	SEC_DND_U_COUNT			20
#define	SEC_DND_FREQUENCY		138		//300khz

#define	SEC_HFDND_N_COUNT		16
#define	SEC_HFDND_U_COUNT		20
#define	SEC_HFDND_FREQUENCY		27		//300khz

#define	SEC_SDND_SHIFT_VALUE	3
#define	SEC_SDND_N_COUNT		12
#define	SEC_SDND_U_COUNT		2
#define	SEC_SDND_FREQUENCY		64
/*---------------------------------------------------------------------*/


struct _ts_capa_info {
	u8 is_zmt200;
	u16 ic_revision;
	u16 firmware_version;
	u16 firmware_minor_version;
	u16 reg_data_version;
	u16 x_resolution;
	u16 y_resolution;
	u32 ic_fw_size;
	u32 MaxX;
	u32 MaxY;
	u32 MinX;
	u32 MinY;
	u32 Orientation;
	u8 gesture_support;
	u16 multi_fingers;
	u16 button_num;
	u16 ic_int_mask;
	u16 x_node_num;
	u16 y_node_num;
	u16 total_node_num;
	u16 hw_id;
	u16 afe_frequency;
	u16 shift_value;
	u16 u_count;
	u8	i2s_checksum;
};

enum _ts_work_proceedure {
	TS_NO_WORK = 0,
	TS_NORMAL_WORK,
	TS_ESD_TIMER_WORK,
	TS_IN_EALRY_SUSPEND,
	TS_IN_SUSPEND,
	TS_IN_RESUME,
	TS_IN_LATE_RESUME,
	TS_IN_UPGRADE,
	TS_REMOVE_WORK,
	TS_SET_MODE,
	TS_HW_CALIBRAION,
	TS_GET_RAW_DATA,
};

#if SEC_TSP_FACTORY_TEST

enum {
	BUILT_IN = 0,
	UMS,
	REQ_FW,
};

#define TSP_CMD_STR_LEN 32
#define TSP_CMD_RESULT_STR_LEN 512
#define TSP_CMD_PARAM_NUM 8
#define TSP_BUF_SIZE 1024

extern struct class *sec_class;

#define TSP_CMD(name, func) .cmd_name = name, .cmd_func = func

struct tsp_cmd {
	struct list_head	list;
	const char	*cmd_name;
	void	(*cmd_func)(void *device_data);
};

/* this is using by cmd_state valiable. */
enum {
	WAITING = 0,
	RUNNING,
	OK,
	FAIL,
	NOT_APPLICABLE,
};

static void fw_update(void *device_data);
static void get_fw_ver_bin(void *device_data);
static void get_fw_ver_ic(void *device_data);
static void get_x_num(void *device_data);
static void get_y_num(void *device_data);
static void get_chip_vendor(void *device_data);
static void get_chip_name(void *device_data);
static void get_threshold(void *device_data);
static void get_key_threshold(void *device_data);
//static void get_reference(void *device_data);
static void get_reference_DND(void *device_data);
static void get_reference_RxDND(void *device_data);
static void get_reference_HFDND(void *device_data);
static void get_reference_RxHFDND(void *device_data);
static void get_normal(void *device_data);
static void get_delta(void *device_data);
static void get_tkey_delta(void *device_data);
//static void run_reference_read(void *device_data);
static void run_reference_DND(void *device_data);
static void run_reference_HFDND(void *device_data);
static void run_reference_PDiff(void *device_data);
static void get_reference_V_Diff(void *device_data);
static void get_reference_max_V_Diff(void *device_data);
static void get_reference_H_Diff(void * device_data);
static void get_reference_max_H_Diff(void * device_data);
//static void set_ref_scale_factor(void *device_data);
static void get_max_dnd(void *device_data);
static void get_min_dnd(void *device_data);
static void get_max_hfdnd(void *device_data);
static void get_min_hfdnd(void *device_data);
static void get_max_hdiff(void *device_data);
static void get_min_hdiff(void *device_data);
static void get_max_vdiff(void *device_data);
static void get_min_vdiff(void *device_data);
static void run_normal_read(void *device_data);
static void run_delta_read(void *device_data);
static void not_support_cmd(void *device_data);
static void run_connect_test(void *device_data);

static struct tsp_cmd tsp_cmds[] = {
	{TSP_CMD("fw_update", fw_update),},
	{TSP_CMD("get_fw_ver_bin", get_fw_ver_bin),},
	{TSP_CMD("get_fw_ver_ic", get_fw_ver_ic),},
	{TSP_CMD("get_x_num", get_x_num),},
	{TSP_CMD("get_y_num", get_y_num),},
	{TSP_CMD("get_chip_vendor", get_chip_vendor),},
	{TSP_CMD("get_chip_name", get_chip_name),},
	{TSP_CMD("get_threshold", get_threshold),},
	{TSP_CMD("get_key_threshold", get_key_threshold),},
	{TSP_CMD("module_off_master", not_support_cmd),},
	{TSP_CMD("module_on_master", not_support_cmd),},
	{TSP_CMD("module_off_slave", not_support_cmd),},
	{TSP_CMD("module_on_slave", not_support_cmd),},
//	{TSP_CMD("run_reference_read", run_reference_read),},
	{TSP_CMD("run_normal_read", run_normal_read),},
	{TSP_CMD("run_delta_read", run_delta_read),},
//	{TSP_CMD("get_reference", get_reference),},
	{TSP_CMD("get_reference_DND", get_reference_DND),},
	{TSP_CMD("get_reference_RxDND", get_reference_RxDND),},
	{TSP_CMD("get_reference_HFDND", get_reference_HFDND),},
	{TSP_CMD("get_reference_RxHFDND", get_reference_RxHFDND),},
	{TSP_CMD("get_reference_V_DIFF", get_reference_V_Diff),},
	{TSP_CMD("get_reference_max_V_DIFF", get_reference_max_V_Diff),},
	{TSP_CMD("get_reference_H_DIFF", get_reference_H_Diff),},
	{TSP_CMD("get_reference_max_H_DIFF", get_reference_max_H_Diff),},
	{TSP_CMD("run_reference_HFDND", run_reference_HFDND),},
	{TSP_CMD("run_reference_DND", run_reference_DND),},
	{TSP_CMD("run_reference_diff", run_reference_PDiff),},
//	{TSP_CMD("set_ref_scale_factor", set_ref_scale_factor),},
	{TSP_CMD("get_max_dnd", get_max_dnd),},
	{TSP_CMD("get_min_dnd", get_min_dnd),},
	{TSP_CMD("get_max_hfdnd", get_max_hfdnd),},
	{TSP_CMD("get_min_hfdnd", get_min_hfdnd),},
	{TSP_CMD("get_max_hdiff", get_max_hdiff),},
	{TSP_CMD("get_min_hdiff", get_min_hdiff),},
	{TSP_CMD("get_max_vdiff", get_max_vdiff),},
	{TSP_CMD("get_min_vdiff", get_min_vdiff),},
	{TSP_CMD("get_normal", get_normal),},
	{TSP_CMD("get_delta", get_delta),},
	{TSP_CMD("get_tkey_delta", get_tkey_delta),},
	{TSP_CMD("tsp_connect_test", run_connect_test),},
	{TSP_CMD("not_support_cmd", not_support_cmd),},
};

static ssize_t menu_sensitivity_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t back_sensitivity_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t touchkey_threshold_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firm_version_panel_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firm_version_phone_show(struct device *dev, struct device_attribute *attr, char *buf);

static DEVICE_ATTR(touchkey_menu, S_IRUGO, menu_sensitivity_show, NULL);
static DEVICE_ATTR(touchkey_back, S_IRUGO, back_sensitivity_show, NULL);
static DEVICE_ATTR(touchkey_threshold, S_IRUGO, touchkey_threshold_show, NULL);
static DEVICE_ATTR(touchkey_firm_version_panel, S_IRUGO, firm_version_panel_show, NULL);
static DEVICE_ATTR(touchkey_firm_version_phone, S_IRUGO, firm_version_phone_show, NULL);
#endif

struct bt432_ts_data {
	struct input_dev *input_dev;
	struct input_dev *key_dev;
	struct task_struct *task;
	struct zxt_ts_platform_data	*pdata;
	wait_queue_head_t	wait;
	struct workqueue_struct *esd_wq;
	struct work_struct	tmr_work;
	struct i2c_client *client;
	struct semaphore update_lock;
	u32 i2c_dev_addr;
	struct _ts_capa_info cap_info;
	char	phys[32];

	struct _ts_zinitix_point_info touch_info;
	struct _ts_zinitix_point_info reported_touch_info;
	u16 icon_event_reg;
	u16 event_type;
	u32 int_gpio_num;
	u32 irq;
	u32 gpio_ldo_en_num;
	u8 button[MAX_SUPPORTED_BUTTON_NUM];
	u8 work_proceedure;
	struct semaphore work_proceedure_lock;
	u8 use_esd_timer;

	u16 debug_reg[8];	// for debug

	bool in_esd_timer;
	struct timer_list esd_timeout_tmr;
	struct timer_list *p_esd_timeout_tmr;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	struct semaphore	raw_data_lock;
	u16 touch_mode;
	s16 cur_data[MAX_TRAW_DATA_SZ];
	u8 update;
	int debug_mode;
	bool init_done;
	bool open_done;
	int finger_status[MAX_SUPPORTED_FINGER_NUM];
	int move_count[MAX_SUPPORTED_FINGER_NUM];
	void (*register_cb)(struct tsp_callbacks *);
	struct tsp_callbacks callbacks;
	bool	ta_status;
	int	touch_count;
#if SEC_TSP_FACTORY_TEST
	s16 ref_data[MAX_RAW_DATA_SZ];
	s16 normal_data[MAX_RAW_DATA_SZ];
	s16 delta_data[MAX_RAW_DATA_SZ];
//	u16 dnd_data[MAX_RAW_DATA_SZ];
	u16 pdnd_data[MAX_RAW_DATA_SZ];
	u16 hfdnd_data[MAX_RAW_DATA_SZ];
	u16	PDND2_N_cnt;
	u16	PDND2_u_cnt;
	u16	PDND2_freq;

	s16 ref_scale_factor;
	s16 ref_btn_option;

	s16 dnd_max_x;
	s16 dnd_max_y;
	s16 dnd_max_val;
	s16 dnd_real_max_x;
	s16 dnd_real_max_y;
	s16 dnd_real_max_val;
	s16 dnd_min_x;
	s16 dnd_min_y;
	s16 dnd_min_val;
	s16 dnd_real_min_x;
	s16 dnd_real_min_y;
	s16 dnd_real_min_val;

	s16 hfdnd_max_x;
	s16 hfdnd_max_y;
	s16 hfdnd_max_val;
	s16 hfdnd_min_x;
	s16 hfdnd_min_y;
	s16 hfdnd_min_val;

	s16 hdiff_max_x;
	s16 hdiff_max_y;
	s16 hdiff_max_val;
	s16 hdiff_min_x;
	s16 hdiff_min_y;
	s16 hdiff_min_val;

	s16 vdiff_max_x;
	s16 vdiff_max_y;
	s16 vdiff_max_val;
	s16 vdiff_min_x;
	s16 vdiff_min_y;
	s16 vdiff_min_val;

	struct list_head	cmd_list_head;
	u8	cmd_state;
	char	cmd[TSP_CMD_STR_LEN];
	int		cmd_param[TSP_CMD_PARAM_NUM];
	char	cmd_result[TSP_CMD_RESULT_STR_LEN];
	struct mutex	cmd_lock;
	bool	cmd_is_running;
	bool	dnd_done;
	bool	hfdnd_done;
#endif
};

/* define i2c sub functions*/
static inline s32 ts_retry_send(struct i2c_client *client, u8 *buf, u16 length)
{
	s32 ret;
	int i;

	for (i=0; i<3; i++) {
		ret = i2c_master_send(client , (u8 *)buf , length);
		if (ret >= 0)
			return ret;
		udelay(10);
	}

	return -1;
}
static inline s32 ts_i2c_read(struct i2c_client *client,
	u8 *values, u16 length)
{
	s32 ret;

	ret = i2c_master_recv(client , values , length);
	if (ret < 0)
		return ret;

	return length;
}

static inline s32 ts_read_data(struct i2c_client *client,
	u16 reg, u8 *values, u16 length)
{
	s32 ret;

	/* select register*/
	ret = ts_retry_send(client , (u8 *)&reg, 2);
	if (ret < 0)
		return ret;
	/* for setup tx transaction. */
	udelay(DELAY_FOR_TRANSCATION);
	ret = i2c_master_recv(client , values , length);
	if (ret < 0)
		return ret;

	return length;
}

static inline s32 ts_write_data(struct i2c_client *client,
	u16 reg, u8 *values, u16 length)
{
	s32 ret;
	u8 pkt[10];	//max packet

	pkt[0] = (reg)&0xff;	//reg addr
	pkt[1] = (reg >> 8)&0xff;

	memcpy((u8*)&pkt[2], values, length);

	ret = ts_retry_send(client , (u8 *)pkt, length+2);
	if (ret < 0)
		return ret;

	udelay(DELAY_FOR_POST_TRANSCATION);

	return length;
}

static inline s32 ts_write_reg(struct i2c_client *client, u16 reg, u16 value)
{
	if (ts_write_data(client, reg, (u8 *)&value, 2) < 0)
		return I2C_FAIL;

	return I2C_SUCCESS;
}

static inline s32 ts_write_cmd(struct i2c_client *client, u16 reg)
{
	s32 ret;

	ret = ts_retry_send(client , (u8 *)&reg, 2);
	if (ret < 0)
		return I2C_FAIL;

	udelay(DELAY_FOR_POST_TRANSCATION);

	return I2C_SUCCESS;
}

static inline s32 ts_read_raw_data(struct i2c_client *client,
		u16 reg, u8 *values, u16 length)
{
	s32 ret;

	/* select register */
	ret = ts_retry_send(client , (u8 *)&reg, 2);
	if (ret < 0)
		return ret;

	/* for setup tx transaction. */
	udelay(200);

	ret = i2c_master_recv(client , values , length);
	if (ret < 0)
		return ret;

	udelay(DELAY_FOR_POST_TRANSCATION);

	return length;
}

static inline s32 ts_read_firmware_data(struct i2c_client *client,
	u16 addr, u8 *values, u16 length)
{
	s32 ret;

	/* select register*/
	ret = ts_retry_send(client , (u8 *)&addr, 2);
	if (ret < 0)
		return ret;

	/* for setup tx transaction. */
	mdelay(1);

	ret = i2c_master_recv(client , values , length);
	if (ret < 0)
		return ret;

	udelay(DELAY_FOR_POST_TRANSCATION);

	return length;
}

static int zinitix_touch_probe(struct i2c_client *client,
	const struct i2c_device_id *i2c_id);
static int zinitix_touch_remove(struct i2c_client *client);
static bool init_touch(struct bt432_ts_data *touch_dev);
static void zinitix_clear_report_data(struct bt432_ts_data *touch_dev);
static void zinitix_parsing_data(struct bt432_ts_data *touch_dev);

#ifdef CONFIG_PM_SLEEP
static int zinitix_resume(struct device *dev);
static int zinitix_suspend(struct device *dev);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void zinitix_early_suspend(struct early_suspend *h);
static void zinitix_late_resume(struct early_suspend *h);
#endif

static void ts_esd_timer_start(u16 sec, struct bt432_ts_data *touch_dev);
static void ts_esd_timer_stop(struct bt432_ts_data *touch_dev);
static void ts_esd_timer_init(struct bt432_ts_data *touch_dev);
static void ts_esd_timeout_handler(unsigned long data);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
static int ts_misc_fops_ioctl(struct inode *inode, struct file *filp,
	unsigned int cmd, unsigned long arg);
#else
static long ts_misc_fops_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg);
#endif
static int ts_misc_fops_open(struct inode *inode, struct file *filp);
static int ts_misc_fops_close(struct inode *inode, struct file *filp);

static const struct file_operations ts_misc_fops = {
	.owner = THIS_MODULE,
	.open = ts_misc_fops_open,
	.release = ts_misc_fops_close,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
	.ioctl = ts_misc_fops_ioctl,
#else
	.unlocked_ioctl = ts_misc_fops_ioctl,
#endif
};

static struct miscdevice touch_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "zinitix_touch_misc",
	.fops = &ts_misc_fops,
};

#define TOUCH_IOCTL_BASE	0xbc
#define TOUCH_IOCTL_GET_DEBUGMSG_STATE		_IOW(TOUCH_IOCTL_BASE, 0, int)
#define TOUCH_IOCTL_SET_DEBUGMSG_STATE		_IOW(TOUCH_IOCTL_BASE, 1, int)
#define TOUCH_IOCTL_GET_CHIP_REVISION		_IOW(TOUCH_IOCTL_BASE, 2, int)
#define TOUCH_IOCTL_GET_FW_VERSION		_IOW(TOUCH_IOCTL_BASE, 3, int)
#define TOUCH_IOCTL_GET_REG_DATA_VERSION	_IOW(TOUCH_IOCTL_BASE, 4, int)
#define TOUCH_IOCTL_VARIFY_UPGRADE_SIZE		_IOW(TOUCH_IOCTL_BASE, 5, int)
#define TOUCH_IOCTL_VARIFY_UPGRADE_DATA		_IOW(TOUCH_IOCTL_BASE, 6, int)
#define TOUCH_IOCTL_START_UPGRADE		_IOW(TOUCH_IOCTL_BASE, 7, int)
#define TOUCH_IOCTL_GET_X_NODE_NUM		_IOW(TOUCH_IOCTL_BASE, 8, int)
#define TOUCH_IOCTL_GET_Y_NODE_NUM		_IOW(TOUCH_IOCTL_BASE, 9, int)
#define TOUCH_IOCTL_GET_TOTAL_NODE_NUM		_IOW(TOUCH_IOCTL_BASE, 10, int)
#define TOUCH_IOCTL_SET_RAW_DATA_MODE		_IOW(TOUCH_IOCTL_BASE, 11, int)
#define TOUCH_IOCTL_GET_RAW_DATA		_IOW(TOUCH_IOCTL_BASE, 12, int)
#define TOUCH_IOCTL_GET_X_RESOLUTION		_IOW(TOUCH_IOCTL_BASE, 13, int)
#define TOUCH_IOCTL_GET_Y_RESOLUTION		_IOW(TOUCH_IOCTL_BASE, 14, int)
#define TOUCH_IOCTL_HW_CALIBRAION		_IOW(TOUCH_IOCTL_BASE, 15, int)
#define TOUCH_IOCTL_GET_REG			_IOW(TOUCH_IOCTL_BASE, 16, int)
#define TOUCH_IOCTL_SET_REG			_IOW(TOUCH_IOCTL_BASE, 17, int)
#define TOUCH_IOCTL_SEND_SAVE_STATUS		_IOW(TOUCH_IOCTL_BASE, 18, int)
#define TOUCH_IOCTL_DONOT_TOUCH_EVENT		_IOW(TOUCH_IOCTL_BASE, 19, int)

struct bt432_ts_data *misc_touch_dev;

#ifdef ZINITIX_TA_COVER_REGISTER
static bool ta_connected =0;
static u16 m_optional_mode = 0;
static u16 m_prev_optional_mode = 0;
extern void zinitix_tsp_register_callback(struct tsp_callbacks *cb);

static void zinitix_set_ta_status(struct bt432_ts_data *touch_dev, bool force)
{
	if (touch_dev == NULL) {
		printk("%s: touch_dev is NULL.\n", __func__);
		return;
	}

	if (ta_connected)
		zinitix_bit_set(m_optional_mode, 0);
	else
		zinitix_bit_clr(m_optional_mode, 0);
}

static void zinitix_charger_status_cb(struct tsp_callbacks *cb, bool status)
{
	if (status)
		ta_connected = true;
	else
		ta_connected = false;

	if (misc_touch_dev->open_done)
		zinitix_set_ta_status(misc_touch_dev, true);

	printk("%s: TA %s\n", __func__, status ? "connected" : "disconnected");
}
static void zinitix_set_optional_mode(struct bt432_ts_data *touch_dev, bool force)
{
	u16	reg_val;

	if (m_prev_optional_mode == m_optional_mode && !force)
		return;
	reg_val = m_optional_mode;

	if (ts_write_reg(touch_dev->client, ZINITIX_USB_DETECT, reg_val)==I2C_SUCCESS)
		m_prev_optional_mode = reg_val;
}
#endif

static bool ts_get_raw_data(struct bt432_ts_data *touch_dev)
{
	u32 total_node = touch_dev->cap_info.total_node_num + touch_dev->cap_info.x_node_num;
	u32 sz;

	if (down_trylock(&touch_dev->raw_data_lock)) {
		zinitix_printk("fail to occupy sema(%d)\n", touch_dev->work_proceedure);
		touch_dev->touch_info.status = 0;
		return true;
	}

	zinitix_debug_msg("read raw data\n");
	sz = total_node*2 + sizeof(struct _ts_zinitix_point_info);

	if (ts_read_raw_data(touch_dev->client,
		ZINITIX_RAWDATA_REG,
		(char *)touch_dev->cur_data, sz) < 0) {
		zinitix_printk("error : read zinitix tc raw data\n");
		up(&touch_dev->raw_data_lock);
		return false;
	}

	touch_dev->update = 1;
	memcpy((u8 *)(&touch_dev->touch_info),
		(u8 *)&touch_dev->cur_data[total_node],
		sizeof(struct _ts_zinitix_point_info));
	up(&touch_dev->raw_data_lock);

	return true;
}

static bool i2c_checksum(struct bt432_ts_data *touch_dev, s16 *pChecksum, u16 wlength)
{
	s16 checksum_result;
	s16 checksum_cur;
	int i;

	checksum_cur = 0;
	for (i=0; i<wlength; i++) {
		checksum_cur += (s16)pChecksum[i];
	}

	if (ts_read_data(touch_dev->client,
		ZINITIX_I2C_CHECKSUM_RESULT,
		(u8 *)(&checksum_result), 2) < 0) {
		printk(KERN_INFO "error read i2c checksum rsult.\n");
		return false;
	}

	if (checksum_cur != checksum_result) {
		printk(KERN_INFO "checksum error : %d, %d\n", checksum_cur, checksum_result);
		return false;
	}

	return true;
}

static bool ts_read_coord(struct bt432_ts_data *touch_dev)
{
#if (TOUCH_POINT_MODE == 1)
	int i;
#endif

	if (gpio_get_value(touch_dev->int_gpio_num)) {
		/*interrupt pin is high, not valid data.*/
		touch_dev->touch_info.status = 0;
		zinitix_debug_msg("woops... inturrpt pin is high\n");
		return true;
	}

	//for  Debugging Tool
	if (touch_dev->touch_mode != TOUCH_POINT_MODE) {
		if (ts_get_raw_data(touch_dev) == false)
			return false;
		goto continue_check_point_data;
	}

#if (TOUCH_POINT_MODE == 1)
	memset(&touch_dev->touch_info,
		0x0, sizeof(struct _ts_zinitix_point_info));

	if (touch_dev->cap_info.i2s_checksum)
		if (ts_write_reg(touch_dev->client,
			ZINITIX_I2C_CHECKSUM_WCNT, 2) != I2C_SUCCESS)
			return false;

	if (ts_i2c_read(touch_dev->client,
		(u8 *)(&touch_dev->touch_info), 10) < 0) {
		zinitix_debug_msg("error read point info using i2c.\n");
		return false;
	}

	if (touch_dev->cap_info.i2s_checksum)
		if (i2c_checksum (touch_dev, (s16 *)(&touch_dev->touch_info), 2) == false)
			return false;

	if (touch_dev->touch_info.event_flag == 0 || touch_dev->touch_info.status == 0)
		goto continue_check_point_data;

	if (touch_dev->cap_info.i2s_checksum)
		if (ts_write_reg(touch_dev->client,
			ZINITIX_I2C_CHECKSUM_WCNT, sizeof(struct _ts_zinitix_coord)/2) != I2C_SUCCESS)
			return false;

	for (i = 1; i < touch_dev->cap_info.multi_fingers; i++) {
		if (zinitix_bit_test(touch_dev->touch_info.event_flag, i)) {
			if (ts_read_data(touch_dev->client,
				ZINITIX_POINT_STATUS_REG+2+(i*4),
				(u8 *)(&touch_dev->touch_info.coord[i]),
				sizeof(struct _ts_zinitix_coord)) < 0) {
				zinitix_debug_msg("error read point info\n");
				return false;
			}
			if (touch_dev->cap_info.i2s_checksum)
				if (i2c_checksum (touch_dev, (s16 *)(&touch_dev->touch_info.coord[i]), sizeof(struct _ts_zinitix_coord)/2) == false)
					return false;
		}
	}

#else
	if (touch_dev->cap_info.i2s_checksum)
		if (ts_write_reg(touch_dev->client,
				ZINITIX_I2C_CHECKSUM_WCNT,
				(sizeof(struct _ts_zinitix_point_info)/2)) != I2C_SUCCESS){
		zinitix_debug_msg("error write checksum wcnt.\n");
		return false;
	}

	if (ts_read_data(touch_dev->client,
		ZINITIX_POINT_STATUS_REG,
		(u8 *)(&touch_dev->touch_info),
		sizeof(struct _ts_zinitix_point_info)) < 0) {
		zinitix_debug_msg("error read point info using i2c.\n");
		return false;
	}

	if (touch_dev->cap_info.i2s_checksum)
		if (i2c_checksum (touch_dev, (s16 *)(&touch_dev->touch_info), sizeof(struct _ts_zinitix_point_info)/2) == false)
			return false;

#endif

continue_check_point_data:

#ifdef ZINITIX_TA_COVER_REGISTER
	zinitix_set_optional_mode(touch_dev, false);
#endif

	if (touch_dev->touch_info.status == 0x0) {
		zinitix_debug_msg("periodical esd repeated int occured\n");
		if (ts_write_cmd(touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD) != I2C_SUCCESS)
			return false;
		udelay(DELAY_FOR_SIGNAL_DELAY);
		return true;
	}

	// error
	if (zinitix_bit_test(touch_dev->touch_info.status, BIT_MUST_ZERO)) {
		zinitix_printk("must zero bit must cleared.(%04x)\n", touch_dev->touch_info.status);
		udelay(DELAY_FOR_SIGNAL_DELAY);
		return false;
	}

	if (zinitix_bit_test(touch_dev->touch_info.status, BIT_ICON_EVENT)) {
		udelay(20);
		if (ts_read_data(touch_dev->client,
			ZINITIX_ICON_STATUS_REG,
			(u8 *)(&touch_dev->icon_event_reg), 2) < 0) {
			zinitix_printk("error read icon info using i2c.\n");
			return false;
		}
	}

	if (ts_write_cmd(touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD) != I2C_SUCCESS)
		return false;

	return true;
}

#if 0
static irqreturn_t ts_int_handler(int irq, void *dev)
{
	struct bt432_ts_data *touch_dev = (struct bt432_ts_data *)dev;
	/* zinitix_debug_msg("interrupt occured +\n"); */
	if (touch_dev == NULL)	return IRQ_HANDLED;
	/* remove pending interrupt */
	if (gpio_get_value(touch_dev->int_gpio_num)) {
		zinitix_debug_msg("invalid interrupt occured +\n");
		return IRQ_HANDLED;
	}
#if USE_THREADED_IRQ
	return IRQ_WAKE_THREAD;
#else
	disable_irq_nosync(irq);
	queue_work(zinitix_workqueue, &touch_dev->work);
	return IRQ_HANDLED;
#endif
}
#endif

static bool bt432_power_sequence(struct bt432_ts_data *touch_dev)
{
	int retry = 0;
	u16 chip_code;

retry_power_sequence:
	if (ts_write_reg(touch_dev->client, 0xc000, 0x0001) != I2C_SUCCESS){
		zinitix_printk("power sequence error (vendor cmd enable)\n");
		goto fail_power_sequence;
	}
	udelay(10);
	if (ts_read_data(touch_dev->client, 0xcc00, (u8 *)&chip_code, 2) < 0) {
		zinitix_printk("fail to read chip code\n");
		goto fail_power_sequence;
	}
	zinitix_printk("chip code = 0x%x\n", chip_code);
	if (chip_code == 0xf400)
		touch_dev->cap_info.is_zmt200 = 0;
	else
		touch_dev->cap_info.is_zmt200 = 1;

	udelay(10);
	if (ts_write_cmd(touch_dev->client, 0xc004) != I2C_SUCCESS){
		zinitix_printk("power sequence error (intn clear)\n");
		goto fail_power_sequence;
	}
	udelay(10);
	if (ts_write_reg(touch_dev->client, 0xc002, 0x0001) != I2C_SUCCESS){
		zinitix_printk("power sequence error (nvm init)\n");
		goto fail_power_sequence;
	}
	mdelay(2);
	if (ts_write_reg(touch_dev->client, 0xc001, 0x0001) != I2C_SUCCESS){
		zinitix_printk("power sequence error (program start)\n");
		goto fail_power_sequence;
	}
	msleep(FIRMWARE_ON_DELAY);	//wait for checksum cal
	return true;

fail_power_sequence:
	if (retry++ < 3) {
		msleep(CHIP_ON_DELAY);
		zinitix_printk("retry = %d\n", retry);
		goto retry_power_sequence;
	}
	return false;
}

static bool zinitix_power(bool on)
{
	static struct regulator *touch_regulator_3v0 =  NULL;
	int ret=0;

	if (!touch_regulator_3v0) {
		touch_regulator_3v0 = regulator_get(NULL,"vddsim2");
		if (IS_ERR(touch_regulator_3v0)) {
			zinitix_printk("get touch_regulator_1v8 regulator error\n");
			goto out;
		}
	}

	if (on) {
		ret = regulator_set_voltage(touch_regulator_3v0, 3000000, 3000000);
		if (ret) {
			zinitix_printk("%s: touch_regulator_3v0 set_voltage failed (%d)\n",__func__, ret);
			goto error;
		}

		ret = regulator_enable(touch_regulator_3v0);
		if (ret) {
			zinitix_printk("%s: touch_regulator_3v0 enable failed (%d)\n",__func__, ret);
			goto error;
		}

		ret = regulator_is_enabled(touch_regulator_3v0);
		if (!ret) {
			zinitix_printk("%s: touch_regulator_3v0 is disabled.(%d)\n",__func__, ret);
			goto error;
		}
	} else {
		ret = regulator_disable(touch_regulator_3v0);
		if (ret) {
			zinitix_printk("%s: touch_regulator_3v0 disable failed (%d)\n",__func__, ret);
			goto error;
		}

		ret = regulator_is_enabled(touch_regulator_3v0);
		if (ret) {
			zinitix_printk("%s: touch_regulator_3v0 is enabled.(%d)\n",__func__, ret);
			goto error;
		}
	}

	zinitix_printk("%s [%s]\n", __func__, regulator_is_enabled(touch_regulator_3v0) ? "ON":"OFF");

	return ret;

error:
	regulator_put(touch_regulator_3v0);
out:
	touch_regulator_3v0 = NULL;

	return -1;
}

static bool bt432_power_control(struct bt432_ts_data *touch_dev, u8 ctl)
{
	bool ret = true;

	zinitix_power((bool)ctl);

	if (ctl)
		msleep(CHIP_ON_DELAY);
	else
		msleep(CHIP_OFF_DELAY);

	if (ctl == POWER_ON_SEQUENCE)
		ret = bt432_power_sequence(touch_dev);

	return ret;

#if 0
	if (ctl == POWER_OFF) {
		gpio_set_value(touch_dev->gpio_ldo_en_num, 0);
		msleep(CHIP_OFF_DELAY);
	} else if (ctl == POWER_ON_SEQUENCE) {
		gpio_set_value(touch_dev->gpio_ldo_en_num, 1);
		msleep(CHIP_ON_DELAY);
		//zxt power on sequence
		return bt432_power_sequence(touch_dev);
	} else if (ctl == POWER_ON)
		gpio_set_value(touch_dev->gpio_ldo_en_num, 1);

	return true;
#endif
}

static bool ts_mini_init_touch(struct bt432_ts_data *touch_dev)
{
	int i;
#if USE_CHECKSUM
	u16 chip_check_sum;
#endif

	if (touch_dev == NULL) {
		zinitix_printk("error (touch_dev == NULL?)\n");
		return false;
	}
	touch_dev->ref_scale_factor = TSP_INIT_TEST_RATIO;

#if USE_CHECKSUM
	if (ts_read_data(touch_dev->client,
		ZINITIX_CHECKSUM_RESULT, (u8 *)&chip_check_sum, 2) < 0)
		goto fail_mini_init;
	if ( chip_check_sum != 0x55aa ) {
		zinitix_printk("firmware checksum error(0x%04x)\n", chip_check_sum);
		goto fail_mini_init;
	}
#endif

	if (ts_write_cmd(touch_dev->client,
		ZINITIX_SWRESET_CMD) != I2C_SUCCESS) {
		zinitix_printk("fail to write reset command\n");
		goto fail_mini_init;
	}

	/* initialize */
	if (ts_write_reg(touch_dev->client,
		ZINITIX_X_RESOLUTION,
		(u16)(touch_dev->cap_info.x_resolution)) != I2C_SUCCESS)
		goto fail_mini_init;

	if (ts_write_reg(touch_dev->client,
		ZINITIX_Y_RESOLUTION,
		(u16)( touch_dev->cap_info.y_resolution)) != I2C_SUCCESS)
		goto fail_mini_init;

	if (ts_write_reg(touch_dev->client,
		ZINITIX_BUTTON_SUPPORTED_NUM,
		(u16)touch_dev->cap_info.button_num) != I2C_SUCCESS)
		goto fail_mini_init;

	if (ts_write_reg(touch_dev->client,
		ZINITIX_SUPPORTED_FINGER_NUM,
		(u16)MAX_SUPPORTED_FINGER_NUM) != I2C_SUCCESS)
		goto fail_mini_init;

	if (ts_write_reg(touch_dev->client,
		ZINITIX_INITIAL_TOUCH_MODE, TOUCH_POINT_MODE) != I2C_SUCCESS)
		goto fail_mini_init;

	if (ts_write_reg(touch_dev->client,
		ZINITIX_TOUCH_MODE, touch_dev->touch_mode) != I2C_SUCCESS)
		goto fail_mini_init;

#ifdef ZINITIX_TA_COVER_REGISTER
	zinitix_set_optional_mode(touch_dev, true);
#endif
	/* soft calibration */
	if (ts_write_cmd(touch_dev->client,
		ZINITIX_CALIBRATE_CMD) != I2C_SUCCESS)
		goto fail_mini_init;

	if (ts_write_reg(touch_dev->client,
		ZINITIX_INT_ENABLE_FLAG,
		touch_dev->cap_info.ic_int_mask) != I2C_SUCCESS)
		goto fail_mini_init;

	/* read garbage data */
	for (i = 0; i < 10; i++)	{
		ts_write_cmd(touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
		udelay(10);
	}

	if (touch_dev->touch_mode != TOUCH_POINT_MODE) {
		if (ts_write_reg(touch_dev->client,
			ZINITIX_DELAY_RAW_FOR_HOST,
			RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS){
			zinitix_printk("Fail to set ZINITIX_DELAY_RAW_FOR_HOST.\n");
			goto fail_mini_init;
		}
	}
	if (ts_write_reg(touch_dev->client,
		ZINITIX_PERIODICAL_INTERRUPT_INTERVAL,
		ZINITIX_SCAN_RATE_HZ
		*ZINITIX_ESD_TIMER_INTERVAL) != I2C_SUCCESS)
		goto fail_mini_init;

	if (touch_dev->use_esd_timer) {
		ts_esd_timer_start(ZINITIX_CHECK_ESD_TIMER, touch_dev);
		zinitix_debug_msg("esd timer start\n");
	}

	zinitix_set_ta_status(touch_dev, true);

	zinitix_debug_msg("successfully mini initialized\n");
	return true;

fail_mini_init:

	zinitix_printk("early mini init fail\n");
	bt432_power_control(touch_dev, POWER_OFF);
	bt432_power_control(touch_dev, POWER_ON_SEQUENCE);

	if (init_touch(touch_dev) == false) {
		zinitix_debug_msg("fail initialized\n");
		return false;
	}

	if (touch_dev->use_esd_timer) {
		ts_esd_timer_start(ZINITIX_CHECK_ESD_TIMER, touch_dev);
		zinitix_debug_msg("esd timer start\n");
	}

	return true;
}

static void zinitix_touch_tmr_work(struct work_struct *work)
{
	struct bt432_ts_data *touch_dev =
		container_of(work, struct bt432_ts_data, tmr_work);

	zinitix_printk("tmr queue work ++\n");

	if (touch_dev == NULL) {
		zinitix_printk("touch dev == NULL ?\n");
		goto fail_time_out_init;
	}

	if (down_trylock(&touch_dev->work_proceedure_lock)) {
		zinitix_printk("fail to occupy sema(%d)\n", touch_dev->work_proceedure);
		ts_esd_timer_start(ZINITIX_CHECK_ESD_TIMER, touch_dev);
		return;
	}

	if (touch_dev->work_proceedure != TS_NO_WORK) {
		zinitix_printk("other process occupied (%d)\n",
			touch_dev->work_proceedure);
		up(&touch_dev->work_proceedure_lock);
		return;
	}
	touch_dev->work_proceedure = TS_ESD_TIMER_WORK;

	disable_irq(touch_dev->irq);
	zinitix_printk("error. timeout occured. maybe ts device dead. so reset & reinit.\n");
	bt432_power_control(touch_dev, POWER_OFF);
	bt432_power_control(touch_dev, POWER_ON_SEQUENCE);

	zinitix_debug_msg("clear all reported points\n");
	zinitix_clear_report_data(touch_dev);
	if (ts_mini_init_touch(touch_dev) == false)
		goto fail_time_out_init;

	touch_dev->work_proceedure = TS_NO_WORK;
	enable_irq(touch_dev->irq);
	up(&touch_dev->work_proceedure_lock);
	zinitix_printk("tmr queue work ---\n");

	return;

fail_time_out_init:
	zinitix_printk("tmr work : restart error\n");
	ts_esd_timer_start(ZINITIX_CHECK_ESD_TIMER, touch_dev);
	touch_dev->work_proceedure = TS_NO_WORK;
	enable_irq(touch_dev->irq);
	up(&touch_dev->work_proceedure_lock);
}

static void ts_esd_timer_start(u16 sec, struct bt432_ts_data *touch_dev)
{
	if (touch_dev == NULL)
		return;
	if (touch_dev->p_esd_timeout_tmr != NULL)
		del_timer(touch_dev->p_esd_timeout_tmr);
	touch_dev->p_esd_timeout_tmr = NULL;

	init_timer(&(touch_dev->esd_timeout_tmr));
	touch_dev->esd_timeout_tmr.data = (unsigned long)(touch_dev);
	touch_dev->esd_timeout_tmr.function = ts_esd_timeout_handler;
	touch_dev->esd_timeout_tmr.expires = jiffies + HZ*sec;
	touch_dev->p_esd_timeout_tmr = &touch_dev->esd_timeout_tmr;
	add_timer(&touch_dev->esd_timeout_tmr);
}

static void ts_esd_timer_stop(struct bt432_ts_data *touch_dev)
{
	if (touch_dev == NULL)
		return;
	if (touch_dev->p_esd_timeout_tmr)
		del_timer(touch_dev->p_esd_timeout_tmr);
	touch_dev->p_esd_timeout_tmr = NULL;
}

static void ts_esd_timer_init(struct bt432_ts_data *touch_dev)
{
	if (touch_dev == NULL)
		return;
	init_timer(&(touch_dev->esd_timeout_tmr));
	touch_dev->esd_timeout_tmr.data = (unsigned long)(touch_dev);
	touch_dev->esd_timeout_tmr.function = ts_esd_timeout_handler;
	touch_dev->p_esd_timeout_tmr = NULL;
}

static void ts_esd_timeout_handler(unsigned long data)
{
	struct bt432_ts_data *touch_dev = (struct bt432_ts_data *)data;
	if (touch_dev == NULL)
		return;
	touch_dev->p_esd_timeout_tmr = NULL;
	queue_work(touch_dev->esd_wq, &touch_dev->tmr_work);
}

#if TOUCH_ONESHOT_UPGRADE
static bool ts_check_need_upgrade(struct bt432_ts_data *touch_dev,
	u16 curVersion, u16 curMinorVersion, u16 curRegVersion, u16 curHWID)
{
	u16	newVersion;
	u16	newMinorVersion;
	u16	newRegVersion;
	u16	newChipCode;
	u16	newHWID;
	u8 *firmware_data;

	firmware_data = (u8*)m_firmware_data;

	curVersion = curVersion&0xff;
	newVersion = (u16) (firmware_data[52] | (firmware_data[53]<<8));
	newVersion = newVersion&0xff;
	newMinorVersion = (u16) (firmware_data[56] | (firmware_data[57]<<8));
	newRegVersion = (u16) (firmware_data[60] | (firmware_data[61]<<8));
	newChipCode = (u16) (firmware_data[64] | (firmware_data[65]<<8));
	if (touch_dev->cap_info.is_zmt200 == 0)
		newHWID = (u16) (firmware_data[0x6b12] | (firmware_data[0x6b13]<<8));
	else
		newHWID = (u16) (firmware_data[0x57d2] | (firmware_data[0x57d3]<<8));

#if CHECK_HWID
	zinitix_printk("cur HW_ID = 0x%x, new HW_ID = 0x%x\n",
		curHWID, newHWID);
	if (curHWID != newHWID)
		return false;
#endif

	zinitix_printk("cur version = 0x%x, new version = 0x%x\n",
		curVersion, newVersion);
	if (curVersion < newVersion)
		return true;
	else if (curVersion > newVersion)
		return false;

	zinitix_printk("cur minor version = 0x%x, new minor version = 0x%x\n",
			curMinorVersion, newMinorVersion);
	if (curMinorVersion < newMinorVersion)
		return true;
	else if (curMinorVersion > newMinorVersion)
		return false;

	zinitix_printk("cur reg data version = 0x%x, new reg data version = 0x%x\n",
			curRegVersion, newRegVersion);
	if (curRegVersion < newRegVersion)
		return true;

	return false;
}
#endif

#define TC_SECTOR_SZ		8

static u8 ts_upgrade_firmware(struct bt432_ts_data *touch_dev,
	const u8 *firmware_data, u32 size)
{
	u16 flash_addr;
	u8 *verify_data;
	int retry_cnt = 0;
	int i;
	int page_sz = 64;
	u16 chip_code;

	verify_data = kzalloc(size, GFP_KERNEL);
	if (verify_data == NULL) {
		zinitix_printk(KERN_ERR "cannot alloc verify buffer\n");
		return false;
	}

retry_upgrade:
	bt432_power_control(touch_dev, POWER_OFF);
	bt432_power_control(touch_dev, POWER_ON);
	mdelay(10);

	if (ts_write_reg(touch_dev->client, 0xc000, 0x0001) != I2C_SUCCESS){
		zinitix_printk("power sequence error (vendor cmd enable)\n");
		goto fail_upgrade;
	}
	udelay(10);
	if (ts_read_data(touch_dev->client, 0xcc00, (u8 *)&chip_code, 2) < 0) {
		zinitix_printk("fail to read chip code\n");
		goto fail_upgrade;
	}
	zinitix_printk("chip code = 0x%x\n", chip_code);
	if (chip_code == 0xf400) {
		touch_dev->cap_info.is_zmt200 = 0;
		page_sz = 128;
	} else {
		touch_dev->cap_info.is_zmt200 = 1;
		page_sz = 64;
	}

	udelay(10);
	if (ts_write_cmd(touch_dev->client, 0xc004) != I2C_SUCCESS){
		zinitix_printk("power sequence error (intn clear)\n");
		goto fail_upgrade;
	}
	udelay(10);
	if (ts_write_reg(touch_dev->client, 0xc002, 0x0001) != I2C_SUCCESS){
		zinitix_printk("power sequence error (nvm init)\n");
		goto fail_upgrade;
	}
	mdelay(5);
	zinitix_printk(KERN_INFO "init flash\n");
	if (ts_write_reg(touch_dev->client, 0xc003, 0x0001) != I2C_SUCCESS){
		zinitix_printk("fail to write nvm vpp on\n");
		goto fail_upgrade;
	}

	if (ts_write_reg(touch_dev->client, 0xc104, 0x0001) != I2C_SUCCESS){
		zinitix_printk("fail to write nvm wp disable\n");
		goto fail_upgrade;
	}

	if (ts_write_cmd(touch_dev->client, ZINITIX_INIT_FLASH) != I2C_SUCCESS) {
		zinitix_printk(KERN_INFO "failed to init flash\n");
		goto fail_upgrade;
	}

	zinitix_printk(KERN_INFO "writing firmware data\n");
	for (flash_addr = 0; flash_addr < size; ) {
		for (i = 0; i < page_sz/TC_SECTOR_SZ; i++) {
			//zinitix_debug_msg("write :addr=%04x, len=%d\n",	flash_addr, TC_SECTOR_SZ);
			if (ts_write_data(touch_dev->client,
				ZINITIX_WRITE_FLASH,
				(u8 *)&firmware_data[flash_addr],TC_SECTOR_SZ) < 0) {
				zinitix_printk(KERN_INFO"error : write zinitix tc firmare\n");
				goto fail_upgrade;
			}
			flash_addr += TC_SECTOR_SZ;
			udelay(100);
		}
		if (touch_dev->cap_info.is_zmt200 == 0)
			mdelay(30);	//for fuzing delay
		else
			mdelay(15);
	}

	if (ts_write_reg(touch_dev->client, 0xc003, 0x0000) != I2C_SUCCESS){
		zinitix_printk("nvm write vpp off\n");
		goto fail_upgrade;
	}

	if (ts_write_reg(touch_dev->client, 0xc104, 0x0000) != I2C_SUCCESS){
		zinitix_printk("nvm wp enable\n");
		goto fail_upgrade;
	}

	zinitix_printk(KERN_INFO "init flash\n");
	if (ts_write_cmd(touch_dev->client, ZINITIX_INIT_FLASH) != I2C_SUCCESS) {
		zinitix_printk(KERN_INFO "failed to init flash\n");
		goto fail_upgrade;
	}

	zinitix_printk(KERN_INFO "read firmware data\n");
	for (flash_addr = 0; flash_addr < size; ) {
		for (i = 0; i < page_sz/TC_SECTOR_SZ; i++) {
			//zinitix_debug_msg("read :addr=%04x, len=%d\n", flash_addr, TC_SECTOR_SZ);
			if (ts_read_firmware_data(touch_dev->client,
				ZINITIX_READ_FLASH,
				(u8*)&verify_data[flash_addr], TC_SECTOR_SZ) < 0) {
				zinitix_printk(KERN_INFO "error : read zinitix tc firmare\n");
				goto fail_upgrade;
			}
			flash_addr += TC_SECTOR_SZ;
		}
	}
	/* verify */
	printk(KERN_INFO "verify firmware data\n");
	if (memcmp((u8 *)&firmware_data[0], (u8 *)&verify_data[0], size) == 0) {
		zinitix_printk(KERN_INFO "upgrade finished\n");
		kfree(verify_data);
		bt432_power_control(touch_dev, POWER_OFF);
		bt432_power_control(touch_dev, POWER_ON_SEQUENCE);

		return true;
	}

fail_upgrade:
	bt432_power_control(touch_dev, POWER_OFF);

	if (retry_cnt++ < ZINITIX_INIT_RETRY_CNT) {
		zinitix_printk(KERN_INFO "upgrade fail : so retry... (%d)\n", retry_cnt);
		goto retry_upgrade;
	}

	if (verify_data != NULL)
		kfree(verify_data);

	zinitix_printk(KERN_INFO "upgrade fail..\n");

	return false;
}

static bool ts_hw_calibration(struct bt432_ts_data *touch_dev)
{
	u16	chip_eeprom_info;
	int time_out = 0;

	if (ts_write_reg(touch_dev->client,
		ZINITIX_TOUCH_MODE, 0x07) != I2C_SUCCESS)
		return false;
	mdelay(10);
	ts_write_cmd(touch_dev->client,	ZINITIX_CLEAR_INT_STATUS_CMD);
	mdelay(10);
	ts_write_cmd(touch_dev->client,	ZINITIX_CLEAR_INT_STATUS_CMD);
	mdelay(50);
	ts_write_cmd(touch_dev->client,	ZINITIX_CLEAR_INT_STATUS_CMD);
	mdelay(10);
	if (ts_write_cmd(touch_dev->client,
		ZINITIX_CALIBRATE_CMD) != I2C_SUCCESS)
		return false;
	if (ts_write_cmd(touch_dev->client,
		ZINITIX_CLEAR_INT_STATUS_CMD) != I2C_SUCCESS)
		return false;
	mdelay(10);
	ts_write_cmd(touch_dev->client,	ZINITIX_CLEAR_INT_STATUS_CMD);

	/* wait for h/w calibration*/
	do {
		mdelay(500);
		ts_write_cmd(touch_dev->client,
				ZINITIX_CLEAR_INT_STATUS_CMD);
		if (ts_read_data(touch_dev->client,
			ZINITIX_EEPROM_INFO_REG,
			(u8 *)&chip_eeprom_info, 2) < 0)
			return false;
		zinitix_debug_msg("touch eeprom info = 0x%04X\n",
			chip_eeprom_info);
		if (!zinitix_bit_test(chip_eeprom_info, 0))
			break;
		if (time_out++ == 4){
			ts_write_cmd(touch_dev->client,	ZINITIX_CALIBRATE_CMD);
			mdelay(10);
			ts_write_cmd(touch_dev->client,
				ZINITIX_CLEAR_INT_STATUS_CMD);
			zinitix_printk("h/w calibration retry timeout.\n");
		}
		if (time_out++ > 10){
			zinitix_printk("[error] h/w calibration timeout.\n");
			break;
		}
	} while (1);

	if (ts_write_reg(touch_dev->client,
		ZINITIX_TOUCH_MODE, TOUCH_POINT_MODE) != I2C_SUCCESS)
		return false;

	if (touch_dev->cap_info.ic_int_mask != 0)
		if (ts_write_reg(touch_dev->client,
			ZINITIX_INT_ENABLE_FLAG,
			touch_dev->cap_info.ic_int_mask)
			!= I2C_SUCCESS)
			return false;
	ts_write_reg(touch_dev->client, 0xc003, 0x0001);
	ts_write_reg(touch_dev->client, 0xc104, 0x0001);
	udelay(100);
	if (ts_write_cmd(touch_dev->client,
		ZINITIX_SAVE_CALIBRATION_CMD) != I2C_SUCCESS)
		return false;
	mdelay(1000);
	ts_write_reg(touch_dev->client, 0xc003, 0x0000);
	ts_write_reg(touch_dev->client, 0xc104, 0x0000);

	return true;
}

static int get_chip_fw_ver(struct bt432_ts_data *touch_dev)
{
	u16 major_version;
	u16 minor_version;
	u16 data_version;

	/* get chip firmware version */
	if (ts_read_data(touch_dev->client, ZINITIX_FIRMWARE_VERSION,
		(u8 *)&major_version, 2) < 0) {
		zinitix_printk("touch chip major version read fail.\n");
		return -1;
	} else
		zinitix_printk("touch chip major version = 0x%02x\n",
			major_version);

	if (ts_read_data(touch_dev->client, ZINITIX_MINOR_FW_VERSION,
			(u8 *)&minor_version, 2) < 0) {
		zinitix_printk("touch chip minor version read fail.\n");
		return -1;
	} else
		zinitix_printk("touch chip minor version = 0x%02x\n",
			minor_version);

	if (ts_read_data(touch_dev->client, ZINITIX_DATA_VERSION_REG,
		(u8 *)&data_version, 2) < 0) {
		zinitix_printk("touch chip data version read fail.\n");
		return -1;
	} else
		zinitix_printk("touch reg data version = 0x%02x\n",
			data_version);

	touch_dev->cap_info.firmware_version = (u16)major_version;
	touch_dev->cap_info.firmware_minor_version = (u16)minor_version;
	touch_dev->cap_info.reg_data_version = (u16)data_version;

	return 0;
}

static bool init_touch(struct bt432_ts_data *touch_dev)
{
	u16 reg_val;
	int i;
	u16 ic_revision;
	u16 firmware_version;
	u16 minor_firmware_version;
	u16 reg_data_version;
	u16 chip_eeprom_info;
#if USE_CHECKSUM
	u16 chip_check_sum;
	u8 checksum_err;
#endif
	int retry_cnt = 0;

	if (touch_dev == NULL) {
		zinitix_printk("error touch_dev == null?\n");
		return false;
	}
	touch_dev->ref_scale_factor = TSP_INIT_TEST_RATIO;

retry_init:

	for (i = 0; i < ZINITIX_INIT_RETRY_CNT; i++) {
		if (ts_read_data(touch_dev->client,
				ZINITIX_EEPROM_INFO_REG,
				(u8 *)&chip_eeprom_info, 2) < 0) {
			zinitix_printk("fail to read eeprom info(%d)\n", i);
			mdelay(10);
			continue;
		} else
			break;
	}
	if (i == ZINITIX_INIT_RETRY_CNT)
		goto fail_init;

#if USE_CHECKSUM
	zinitix_debug_msg("check checksum\n");

	checksum_err = 0;

	for (i = 0; i < ZINITIX_INIT_RETRY_CNT; i++) {
		if (ts_read_data(touch_dev->client,
			ZINITIX_CHECKSUM_RESULT, (u8 *)&chip_check_sum, 2) < 0) {
			mdelay(10);
			continue;
		}

		zinitix_debug_msg("0x%04X\n",	chip_check_sum);

		if (chip_check_sum == 0x55aa)
			break;
		else {
			checksum_err = 1;
			break;
		}
	}

	if (i == ZINITIX_INIT_RETRY_CNT || checksum_err) {
		zinitix_printk("fail to check firmware data\n");
		if (checksum_err == 1 && retry_cnt < ZINITIX_INIT_RETRY_CNT)
			retry_cnt = ZINITIX_INIT_RETRY_CNT;
		goto fail_init;
	}

#endif

	if (ts_write_cmd(touch_dev->client,
		ZINITIX_SWRESET_CMD) != I2C_SUCCESS) {
		zinitix_printk("fail to write reset command\n");
		goto fail_init;
	}

	touch_dev->cap_info.button_num = SUPPORTED_BUTTON_NUM;

	reg_val = 0;
	zinitix_bit_set(reg_val, BIT_PT_CNT_CHANGE);
	zinitix_bit_set(reg_val, BIT_DOWN);
	zinitix_bit_set(reg_val, BIT_MOVE);
	zinitix_bit_set(reg_val, BIT_UP);
	if (touch_dev->cap_info.button_num > 0)
		zinitix_bit_set(reg_val, BIT_ICON_EVENT);
	touch_dev->cap_info.ic_int_mask = reg_val;

	if (ts_write_reg(touch_dev->client,
		ZINITIX_INT_ENABLE_FLAG, 0x0) != I2C_SUCCESS)
		goto fail_init;

	zinitix_debug_msg("send reset command\n");
	if (ts_write_cmd(touch_dev->client,
		ZINITIX_SWRESET_CMD) != I2C_SUCCESS)
		goto fail_init;

	/* get chip revision id */
	if (ts_read_data(touch_dev->client,
		ZINITIX_CHIP_REVISION,
		(u8 *)&ic_revision, 2) < 0) {
		zinitix_printk("fail to read chip revision\n");
		goto fail_init;
	}
	zinitix_printk("touch chip revision id = %x\n",
		ic_revision);

	if (touch_dev->cap_info.is_zmt200 == 0)
		touch_dev->cap_info.ic_fw_size = 32*1024;
	else
		touch_dev->cap_info.ic_fw_size = 24*1024;

	//for  Debugging Tool
	if (ts_read_data(touch_dev->client,
		ZINITIX_HW_ID,
		(u8 *)&touch_dev->cap_info.hw_id, 2) < 0)
		goto fail_init;

	zinitix_printk("touch chip hw id = 0x%04x\n",
			touch_dev->cap_info.hw_id);

	if (ts_read_data(touch_dev->client,
		ZINITIX_TOTAL_NUMBER_OF_Y,
		(u8 *)&touch_dev->cap_info.x_node_num, 2) < 0)
		goto fail_init;
	zinitix_printk("touch chip x node num = %d\n",
		touch_dev->cap_info.x_node_num);
	if (ts_read_data(touch_dev->client,
		ZINITIX_TOTAL_NUMBER_OF_X,
		(u8 *)&touch_dev->cap_info.y_node_num, 2) < 0)
		goto fail_init;
	touch_dev->cap_info.y_node_num--;
	zinitix_printk("touch chip y node num = %d\n",
		touch_dev->cap_info.y_node_num);

	touch_dev->cap_info.total_node_num =
		touch_dev->cap_info.x_node_num*touch_dev->cap_info.y_node_num;
	zinitix_printk("touch chip total node num = %d\n",
		touch_dev->cap_info.total_node_num);

	if (ts_read_data(touch_dev->client,
			ZINITIX_AFE_FREQUENCY,
			(u8 *)&touch_dev->cap_info.afe_frequency, 2) < 0)
			goto fail_init;
	zinitix_debug_msg("afe frequency = %d\n",
			touch_dev->cap_info.afe_frequency);

	if (ts_read_data(touch_dev->client,
		ZINITIX_SHIFT_VALUE,
		(u8 *)&touch_dev->cap_info.shift_value, 2) < 0)
		goto fail_init;
	zinitix_debug_msg("afe shift value = %d\n",
		touch_dev->cap_info.shift_value);

	if (ts_read_data(touch_dev->client,
		ZINITIX_U_COUNT,
		(u8 *)&touch_dev->cap_info.u_count, 2) < 0)
		goto fail_init;
	zinitix_debug_msg("afe N shift = %d\n",
		touch_dev->cap_info.u_count);

#if TOUCH_ONESHOT_UPGRADE
	if (!touch_dev->init_done) {
		/* get chip firmware version */
		if (ts_read_data(touch_dev->client,
			ZINITIX_FIRMWARE_VERSION,
			(u8 *)&firmware_version, 2) < 0)
			goto fail_init;
		zinitix_printk("touch chip firmware version = %x\n",
			firmware_version);

		if (ts_read_data(touch_dev->client,
			ZINITIX_MINOR_FW_VERSION,
			(u8 *)&minor_firmware_version, 2) < 0)
			goto fail_init;
		zinitix_printk("touch chip firmware version = %x\n",
			minor_firmware_version);

		if (ts_read_data(touch_dev->client,
			ZINITIX_DATA_VERSION_REG,
			(u8 *)&reg_data_version, 2) < 0)
			goto fail_init;
		zinitix_debug_msg("touch reg data version = %d\n",
			reg_data_version);

		if (ts_check_need_upgrade(touch_dev, firmware_version, minor_firmware_version,
			reg_data_version, touch_dev->cap_info.hw_id) == true) {
			zinitix_printk("start upgrade firmware\n");

			if (ts_upgrade_firmware(touch_dev, m_firmware_data,
				touch_dev->cap_info.ic_fw_size) == false)
				goto fail_init;

			if (ts_hw_calibration(touch_dev) == false)
				goto fail_init;

			/* disable chip interrupt */
			if (ts_write_reg(touch_dev->client,
				ZINITIX_INT_ENABLE_FLAG, 0) != I2C_SUCCESS)
				goto fail_init;

			/* get chip firmware version */
			if (ts_read_data(touch_dev->client,
				ZINITIX_FIRMWARE_VERSION,
				(u8 *)&firmware_version, 2) < 0)
				goto fail_init;
			zinitix_printk("touch chip firmware version = %x\n",
				firmware_version);

			if (ts_read_data(touch_dev->client,
				ZINITIX_MINOR_FW_VERSION,
				(u8 *)&minor_firmware_version, 2) < 0)
				goto fail_init;
			zinitix_printk("touch chip firmware version = %x\n",
				minor_firmware_version);
		} else
			zinitix_printk("TSP firmware upgrade not required.\n");
	}
#endif

	if (ts_read_data(touch_dev->client,
		ZINITIX_DATA_VERSION_REG,
		(u8 *)&reg_data_version, 2) < 0)
		goto fail_init;
	zinitix_debug_msg("touch reg data version = %d\n",
		reg_data_version);

	if (ts_read_data(touch_dev->client,
		ZINITIX_EEPROM_INFO_REG,
		(u8 *)&chip_eeprom_info, 2) < 0)
		goto fail_init;
	zinitix_debug_msg("touch eeprom info = 0x%04X\n", chip_eeprom_info);

	if (zinitix_bit_test(chip_eeprom_info, 0)) { /* hw calibration bit*/

		if (ts_hw_calibration(touch_dev) == false)
			goto fail_init;

		/* disable chip interrupt */
		if (ts_write_reg(touch_dev->client,
			ZINITIX_INT_ENABLE_FLAG, 0) != I2C_SUCCESS)
			goto fail_init;
	}

	touch_dev->cap_info.ic_revision = (u16)ic_revision;
	touch_dev->cap_info.firmware_version = (u16)firmware_version;
	touch_dev->cap_info.firmware_minor_version = (u16)minor_firmware_version;
	touch_dev->cap_info.reg_data_version = (u16)reg_data_version;

	/* initialize */
	touch_dev->cap_info.x_resolution = touch_dev->pdata->x_resolution;
	touch_dev->cap_info.y_resolution = touch_dev->pdata->y_resolution;

	if (ts_write_reg(touch_dev->client,
		ZINITIX_X_RESOLUTION, (u16)(touch_dev->cap_info.x_resolution)) != I2C_SUCCESS)
		goto fail_init;

	if (ts_write_reg(touch_dev->client,
		ZINITIX_Y_RESOLUTION, (u16)(touch_dev->cap_info.y_resolution)) != I2C_SUCCESS)
		goto fail_init;

	zinitix_debug_msg("touch max x = %d\n", touch_dev->cap_info.x_resolution);
	zinitix_debug_msg("touch max y = %d\n", touch_dev->cap_info.y_resolution);

	touch_dev->cap_info.MinX = (u32)0;
	touch_dev->cap_info.MinY = (u32)0;
	touch_dev->cap_info.MaxX = (u32)touch_dev->cap_info.x_resolution;
	touch_dev->cap_info.MaxY = (u32)touch_dev->cap_info.y_resolution;

	if (ts_write_reg(touch_dev->client,
		ZINITIX_BUTTON_SUPPORTED_NUM,
		(u16)touch_dev->cap_info.button_num) != I2C_SUCCESS)
		goto fail_init;

	if (ts_write_reg(touch_dev->client,
		ZINITIX_SUPPORTED_FINGER_NUM,
		(u16)MAX_SUPPORTED_FINGER_NUM) != I2C_SUCCESS)
		goto fail_init;
	touch_dev->cap_info.multi_fingers = MAX_SUPPORTED_FINGER_NUM;

	zinitix_debug_msg("max supported finger num = %d\n",
		touch_dev->cap_info.multi_fingers);
	touch_dev->cap_info.gesture_support = 0;
	zinitix_debug_msg("set other configuration\n");

	if (ts_write_reg(touch_dev->client,
		ZINITIX_INITIAL_TOUCH_MODE, TOUCH_POINT_MODE) != I2C_SUCCESS)
		goto fail_init;

	if (ts_write_reg(touch_dev->client,
		ZINITIX_TOUCH_MODE, touch_dev->touch_mode) != I2C_SUCCESS)
		goto fail_init;

	//==================================================
#ifdef ZINITIX_TA_COVER_REGISTER
	zinitix_set_optional_mode(touch_dev, true);
#endif

	if (ts_read_data(touch_dev->client, ZINITIX_INTERNAL_FLAG_02,
			(u8 *)&reg_val, 2) < 0)
			goto fail_init;

	touch_dev->cap_info.i2s_checksum = !(!zinitix_bit_test(reg_val, 15));
	zinitix_printk("use i2s checksum = %d\n",
			touch_dev->cap_info.i2s_checksum);

//	if (touch_dev->cap_info.i2s_checksum)
//		if (ts_write_reg(touch_dev->client,
//				ZINITIX_I2C_CHECKSUM_WCNT, (sizeof(struct _ts_zinitix_point_info)/2)) != I2C_SUCCESS)
//				goto fail_init;

	/* soft calibration */
	if (ts_write_cmd(touch_dev->client,
		ZINITIX_CALIBRATE_CMD) != I2C_SUCCESS)
		goto fail_init;

	if (ts_write_reg(touch_dev->client,
		ZINITIX_INT_ENABLE_FLAG,
		touch_dev->cap_info.ic_int_mask) != I2C_SUCCESS)
		goto fail_init;

	/* read garbage data */
	for (i = 0; i < 10; i++)	{
		ts_write_cmd(touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
		udelay(10);
	}

	if (touch_dev->touch_mode != TOUCH_POINT_MODE) { /* Test Mode */
		if (ts_write_reg(touch_dev->client,
			ZINITIX_DELAY_RAW_FOR_HOST,
			RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS) {
			zinitix_printk("Fail to set ZINITIX_DELAY_RAW_FOR_HOST.\n");
			goto fail_init;
		}
	}
	if (ts_write_reg(touch_dev->client,
		ZINITIX_PERIODICAL_INTERRUPT_INTERVAL,
		ZINITIX_SCAN_RATE_HZ
		*ZINITIX_ESD_TIMER_INTERVAL) != I2C_SUCCESS)
		goto fail_init;

	ts_read_data(touch_dev->client, ZINITIX_PERIODICAL_INTERRUPT_INTERVAL,
		(u8 *)&reg_val, 2);
	zinitix_debug_msg("esd timer register = %d\n", reg_val);

	zinitix_debug_msg("successfully initialized\n");
	return true;

fail_init:

	if (++retry_cnt <= ZINITIX_INIT_RETRY_CNT) {
		bt432_power_control(touch_dev, POWER_OFF);
		bt432_power_control(touch_dev, POWER_ON_SEQUENCE);

		zinitix_debug_msg("retry to initiallize(retry cnt = %d)\n",
			retry_cnt);
		goto	retry_init;
	} else if (retry_cnt == ZINITIX_INIT_RETRY_CNT+1) {
		if (touch_dev->cap_info.is_zmt200 == 0)
			touch_dev->cap_info.ic_fw_size = 32*1024;
		else
			touch_dev->cap_info.ic_fw_size = 24*1024;

		zinitix_debug_msg("retry to initiallize(retry cnt = %d)\n", retry_cnt);
#if TOUCH_FORCE_UPGRADE
		if (ts_upgrade_firmware(touch_dev, m_firmware_data,
			touch_dev->cap_info.ic_fw_size) == false) {
			zinitix_printk("upgrade fail\n");
			return false;
		}
		mdelay(100);
		// hw calibration and make checksum
		if (ts_hw_calibration(touch_dev) == false) {
			zinitix_printk("failed to initiallize\n");
			return false;
		}
		goto retry_init;
#endif
	}

	zinitix_printk("failed to initiallize\n");
	return false;
}

static void zinitix_clear_report_data(struct bt432_ts_data *touch_dev)
{
	int i;
	u8 reported = 0;
	u8 sub_status;

	touch_dev->touch_count = 0;

	for (i = 0; i < touch_dev->cap_info.button_num; i++) {
		if (touch_dev->button[i] == ICON_BUTTON_DOWN) {
			touch_dev->button[i] = ICON_BUTTON_UP;
			input_report_key(touch_dev->input_dev,
				touch_dev->pdata->key_code[i], 0);
			reported = true;
			zinitix_debug_msg("button up = %d\n", i);
		}
	}

	for (i = 0; i < touch_dev->cap_info.multi_fingers; i++) {
		sub_status = touch_dev->reported_touch_info.coord[i].sub_status;
		if (zinitix_bit_test(sub_status, SUB_BIT_EXIST)) {
			input_mt_slot(touch_dev->input_dev, i);
			input_mt_report_slot_state(touch_dev->input_dev,
				MT_TOOL_FINGER, 0);

			input_report_abs(touch_dev->input_dev,
					ABS_MT_TRACKING_ID, -1);

			reported = true;
			if (!touch_dev->debug_mode && TSP_NORMAL_EVENT_MSG)
				printk(KERN_INFO "[TSP] R %02d\n", i);
		}
		touch_dev->reported_touch_info.coord[i].sub_status = 0;
		touch_dev->finger_status[i] = 0;
		touch_dev->move_count[i] = 0;
	}
	if (reported) {

		input_report_key(touch_dev->input_dev, BTN_TOUCH, 0);
		input_sync(touch_dev->input_dev);
	}
}

#define	PALM_REPORT_WIDTH	200
#define	PALM_REJECT_WIDTH	255

static void zinitix_chip_reset(struct bt432_ts_data *touch_dev)
{
	zinitix_printk("reset TSP device.\n");

	bt432_power_control(touch_dev, POWER_OFF);
	bt432_power_control(touch_dev, POWER_ON_SEQUENCE);
	zinitix_clear_report_data(touch_dev);
	ts_mini_init_touch(touch_dev);
	touch_dev->work_proceedure = TS_NO_WORK;
}

static void zinitix_parsing_data(struct bt432_ts_data *touch_dev)
{
	int i = 0;
	u8 reported = false;
	u8 sub_status;
	u8 prev_sub_status;
	u32 x, y;
	u32 w;
	u32 tmp;
	u8 palm = 0;
	u8 read_result = 1;

	if (touch_dev == NULL) {
		printk(KERN_INFO "zinitix_parsing_data : touch_dev == NULL?\n");
		return;
	}

	if (down_trylock(&touch_dev->work_proceedure_lock)) {
		zinitix_printk("fail to occupy sema(%d)\n", touch_dev->work_proceedure);
		ts_write_cmd(touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
		return;
	}

	if (touch_dev->work_proceedure != TS_NO_WORK) {
		zinitix_debug_msg("zinitix_touch_thread : \
			[warning] other process occupied..(%d)\n", touch_dev->work_proceedure);
		udelay(DELAY_FOR_SIGNAL_DELAY);
		if (!gpio_get_value(touch_dev->int_gpio_num)) {
			ts_write_cmd(touch_dev->client,
				ZINITIX_CLEAR_INT_STATUS_CMD);
			udelay(DELAY_FOR_SIGNAL_DELAY);
		}

		goto end_parsing_data;
	}

	if (touch_dev->use_esd_timer) {
		ts_esd_timer_stop(touch_dev);
		zinitix_debug_msg("esd timer stop\n");
	}

	touch_dev->work_proceedure = TS_NORMAL_WORK;

	if (ts_read_coord(touch_dev) == false || touch_dev->touch_info.status == 0xffff
		|| touch_dev->touch_info.status == 0x1) {
		// more retry
		for (i=1; i< 50; i++) {	// about 10ms
			if (!(ts_read_coord(touch_dev) == false|| touch_dev->touch_info.status == 0xffff
			|| touch_dev->touch_info.status == 0x1))
				break;
		}

		if (i==50)
			read_result = 0;
	}

	if (!read_result) {
		zinitix_debug_msg("couldn't read touch_dev coord. maybe chip is dead\n");
		bt432_power_control(touch_dev, POWER_OFF);
		bt432_power_control(touch_dev, POWER_ON_SEQUENCE);

		zinitix_debug_msg("clear all reported points\n");
		zinitix_clear_report_data(touch_dev);
		ts_mini_init_touch(touch_dev);
		goto continue_parsing_data;
	}

	/* invalid : maybe periodical repeated int. */
	if (touch_dev->touch_info.status == 0x0)
		goto continue_parsing_data;

	// added by sohnet (20130418)
	if (i!= 0 && zinitix_bit_test(touch_dev->touch_info.status, BIT_PT_EXIST)) {
		zinitix_printk("may be corrupt data, so skip this.\n");
		goto continue_parsing_data;
	}

	reported = false;

	if (zinitix_bit_test(touch_dev->touch_info.status, BIT_ICON_EVENT)) {
		for (i = 0; i < touch_dev->cap_info.button_num; i++) {
			if (zinitix_bit_test(touch_dev->icon_event_reg,
				(BIT_O_ICON0_DOWN+i))) {
				touch_dev->button[i] = ICON_BUTTON_DOWN;
				input_report_key(touch_dev->key_dev,
					touch_dev->pdata->key_code[i], 1);
				reported = true;
				zinitix_debug_msg("%s = [P]\n",
					touch_dev->pdata->key_code[i] == KEY_PHONE
					? "KEY_PHONE":"KEY_BACK");
				if (!touch_dev->debug_mode && TSP_NORMAL_EVENT_MSG)
					printk(KERN_INFO "%s = [P]\n",
					touch_dev->pdata->key_code[i] == KEY_PHONE
					? "KEY_PHONE":"KEY_BACK");
			}
		}

		for (i = 0; i < touch_dev->cap_info.button_num; i++) {
			if (zinitix_bit_test(touch_dev->icon_event_reg,
				(BIT_O_ICON0_UP+i))) {
				touch_dev->button[i] = ICON_BUTTON_UP;
				input_report_key(touch_dev->key_dev,
					touch_dev->pdata->key_code[i], 0);
				reported = true;

				zinitix_debug_msg("%s = [R]\n",
					touch_dev->pdata->key_code[i] == KEY_PHONE
					? "KEY_PHONE":"KEY_BACK");
				if (!touch_dev->debug_mode && TSP_NORMAL_EVENT_MSG)
					printk(KERN_INFO "%s = [R]\n",
					touch_dev->pdata->key_code[i] == KEY_PHONE
					? "KEY_PHONE":"KEY_BACK");
			}
		}

		if(true == reported) {
			input_sync(touch_dev->key_dev);
			goto continue_parsing_data;
		}
	}
#if 0
	/* if button press or up event occured... */
	if (reported == true || !zinitix_bit_test(touch_dev->touch_info.status, BIT_PT_EXIST)) {
		for (i = 0; i < touch_dev->cap_info.multi_fingers; i++) {
			prev_sub_status =
				touch_dev->reported_touch_info.coord[i].sub_status;
			if (zinitix_bit_test(prev_sub_status, SUB_BIT_EXIST)) {
				zinitix_debug_msg("finger [%02d] up\n", i);

				input_mt_slot(touch_dev->input_dev, i);
				input_mt_report_slot_state(touch_dev->input_dev,
					MT_TOOL_FINGER, 0);

				input_report_abs(touch_dev->input_dev,
						ABS_MT_TRACKING_ID, -1);

				if (!touch_dev->debug_mode && TSP_NORMAL_EVENT_MSG)
					printk(KERN_INFO "[TSP] R %02d\n", i);
			}
		}
		memset(&touch_dev->reported_touch_info,
			0x0, sizeof(struct _ts_zinitix_point_info));

		input_report_key(touch_dev->input_dev, BTN_TOUCH, 0);

		input_sync(touch_dev->input_dev);
		if (reported == true)	// for button event
			udelay(100);
		goto continue_parsing_data;
	}
#endif
	if (zinitix_bit_test(touch_dev->touch_info.status, BIT_PALM)){
		zinitix_debug_msg("palm report\n");
		palm = 1;
	}
	if (zinitix_bit_test(touch_dev->touch_info.status, BIT_PALM_REJECT)){
		zinitix_debug_msg("palm reject\n");
		palm = 2;
	}

	for (i = 0; i < touch_dev->cap_info.multi_fingers; i++) {
		sub_status = touch_dev->touch_info.coord[i].sub_status;
		prev_sub_status = touch_dev->reported_touch_info.coord[i].sub_status;

		if (zinitix_bit_test(sub_status, SUB_BIT_EXIST)) {
			x = touch_dev->touch_info.coord[i].x;
			y = touch_dev->touch_info.coord[i].y;
			w = touch_dev->touch_info.coord[i].width;

			 /* transformation from touch to screen orientation */
			if (touch_dev->cap_info.Orientation & TOUCH_V_FLIP)
				y = touch_dev->cap_info.MaxY
					+ touch_dev->cap_info.MinY - y;
			if (touch_dev->cap_info.Orientation & TOUCH_H_FLIP)
				x = touch_dev->cap_info.MaxX
					+ touch_dev->cap_info.MinX - x;
			if (touch_dev->cap_info.Orientation & TOUCH_XY_SWAP)
				zinitix_swap_v(x, y, tmp);

			if (x > touch_dev->cap_info.MaxX ||y > touch_dev->cap_info.MaxY) {
				zinitix_printk("invalid coord # %d : x=%d, y=%d\n", i, x, y);
				if (palm == 2) {
					zinitix_chip_reset(touch_dev);
					goto end_parsing_data;
				} else
					continue;
			}

			touch_dev->touch_info.coord[i].x = x;
			touch_dev->touch_info.coord[i].y = y;
			if (!touch_dev->finger_status[i]) {
				zinitix_debug_msg("[P][%d] x=%d, y=%d\n", i, x, y);
				touch_dev->finger_status[i] = true;
				touch_dev->move_count[i] = 0;
				touch_dev->touch_count++;
			} else
				touch_dev->move_count[i]++;

			if (w == 0)
				w = 1;

			input_mt_slot(touch_dev->input_dev, i);
			input_mt_report_slot_state(touch_dev->input_dev,
				MT_TOOL_FINGER, 1);
			input_report_abs(touch_dev->input_dev,
				ABS_MT_POSITION_X, x);
			input_report_abs(touch_dev->input_dev,
				ABS_MT_POSITION_Y, y);
			input_report_abs(touch_dev->input_dev,
				ABS_MT_TOUCH_MAJOR, (u32)w);
			input_report_abs(touch_dev->input_dev,
				ABS_MT_WIDTH_MAJOR, (u32)w);

#if (TOUCH_POINT_MODE == 2)
			if (palm == 0) {
				if (w >= PALM_REPORT_WIDTH)
					w = PALM_REPORT_WIDTH - 10;
			}else if (palm == 1) {	//palm report
				w = PALM_REPORT_WIDTH;
				touch_dev->touch_info.coord[i].minor_width = PALM_REPORT_WIDTH;
			} else {	// palm reject
				x = y = 0;
				w = PALM_REJECT_WIDTH;
				touch_dev->touch_info.coord[i].minor_width = PALM_REJECT_WIDTH;
			}
			input_report_abs(touch_dev->input_dev,
				ABS_MT_TOUCH_MINOR, (u32)touch_dev->touch_info.coord[i].minor_width);
			input_report_abs(touch_dev->input_dev,
				ABS_MT_ANGLE, touch_dev->touch_info.coord[i].angle - 90);
			input_report_abs(touch_dev->input_dev, ABS_MT_PALM, (palm>0)?1:0);
#endif
		}	else if (zinitix_bit_test(sub_status, SUB_BIT_UP)||
			zinitix_bit_test(prev_sub_status, SUB_BIT_EXIST)) {
			zinitix_debug_msg("[R][%d] c=%d\n", i, touch_dev->move_count[i]);
			touch_dev->finger_status[i] = false;
			touch_dev->move_count[i] = 0;
			touch_dev->touch_count--;
			memset(&touch_dev->touch_info.coord[i],
				0x0, sizeof(struct _ts_zinitix_coord));
			input_mt_slot(touch_dev->input_dev, i);
			input_mt_report_slot_state(touch_dev->input_dev,
				MT_TOOL_FINGER, 0);
		} else
			memset(&touch_dev->touch_info.coord[i],
				0x0, sizeof(struct _ts_zinitix_coord));
	}
	memcpy((char *)&touch_dev->reported_touch_info,
		(char *)&touch_dev->touch_info,
		sizeof(struct _ts_zinitix_point_info));

	input_report_key(touch_dev->input_dev, BTN_TOUCH, !!touch_dev->touch_count);

	input_sync(touch_dev->input_dev);

continue_parsing_data:

	/* check_interrupt_pin, if high, enable int & wait signal */
	if (touch_dev->work_proceedure == TS_NORMAL_WORK) {
		if (touch_dev->use_esd_timer) {
			ts_esd_timer_start(ZINITIX_CHECK_ESD_TIMER, touch_dev);
			zinitix_debug_msg("esd tmr start\n");
		}
		touch_dev->work_proceedure = TS_NO_WORK;
	}
end_parsing_data:
	up(&touch_dev->work_proceedure_lock);
}

#if USE_THREADED_IRQ
static irqreturn_t zinitix_touch_work(int irq, void *data)
{
	struct bt432_ts_data* touch_dev = (struct bt432_ts_data*)data;

	if (touch_dev->init_done)
		zinitix_parsing_data(touch_dev);

	return IRQ_HANDLED;
}
#else
static void zinitix_touch_work(struct work_struct *work)
{
	struct bt432_ts_data *touch_dev =
		container_of(work, struct bt432_ts_data, work);

	zinitix_debug_msg("signalled\n");
	zinitix_parsing_data(touch_dev);
	enable_irq(touch_dev->irq);
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void zinitix_late_resume(struct early_suspend *h)
{
	struct bt432_ts_data *touch_dev = misc_touch_dev;
	//touch_dev = container_of(h, struct bt432_ts_data, early_suspend);

	if (touch_dev == NULL)
		return;
	zinitix_printk("late resume++\n");

	down(&touch_dev->work_proceedure_lock);
	if (touch_dev->work_proceedure != TS_IN_RESUME
		&& touch_dev->work_proceedure != TS_IN_EALRY_SUSPEND) {
		zinitix_printk("invalid work proceedure (%d)\n",
			touch_dev->work_proceedure);
		up(&touch_dev->work_proceedure_lock);
		return;
	}
#ifdef CONFIG_PM_SLEEP
	ts_write_cmd(touch_dev->client, ZINITIX_WAKEUP_CMD);
	mdelay(1);
#else
	bt432_power_control(touch_dev, POWER_ON_SEQUENCE);
#endif
	if (ts_mini_init_touch(touch_dev) == false)
		goto fail_late_resume;
	enable_irq(touch_dev->irq);
	touch_dev->work_proceedure = TS_NO_WORK;
	up(&touch_dev->work_proceedure_lock);
	zinitix_printk("late resume-\n");
	return;
fail_late_resume:
	zinitix_printk("failed to late resume\n");
	enable_irq(touch_dev->irq);
	touch_dev->work_proceedure = TS_NO_WORK;
	up(&touch_dev->work_proceedure_lock);
	return;
}

static void zinitix_early_suspend(struct early_suspend *h)
{
	struct bt432_ts_data *touch_dev = misc_touch_dev;
	//touch_dev = container_of(h, struct bt432_ts_data, early_suspend);

	if (touch_dev == NULL)
		return;

	zinitix_printk("early suspend++\n");

	disable_irq(touch_dev->irq);
	if (touch_dev->use_esd_timer)
		flush_work(&touch_dev->tmr_work);

	down(&touch_dev->work_proceedure_lock);
	if (touch_dev->work_proceedure != TS_NO_WORK) {
		zinitix_printk("invalid work proceedure (%d)\n",
			touch_dev->work_proceedure);
		up(&touch_dev->work_proceedure_lock);
		enable_irq(touch_dev->irq);
		return;
	}
	touch_dev->work_proceedure = TS_IN_EALRY_SUSPEND;

	zinitix_debug_msg("clear all reported points\n");
	zinitix_clear_report_data(touch_dev);

	if (touch_dev->use_esd_timer) {
		ts_write_reg(touch_dev->client,
			ZINITIX_PERIODICAL_INTERRUPT_INTERVAL, 0);
		ts_esd_timer_stop(touch_dev);
		zinitix_printk("ts_esd_timer_stop\n");
	}

#ifdef CONFIG_PM_SLEEP
	ts_write_reg(touch_dev->client, ZINITIX_INT_ENABLE_FLAG, 0x0);

	udelay(100);
	if (ts_write_cmd(touch_dev->client, ZINITIX_SLEEP_CMD) != I2C_SUCCESS) {
		zinitix_printk("failed to enter into sleep mode\n");
		up(&touch_dev->work_proceedure_lock);
		return;
	}
#else
	bt432_power_control(touch_dev, POWER_OFF);
#endif
	zinitix_printk("early suspend-\n");
	up(&touch_dev->work_proceedure_lock);
	return;
}

#endif	/* CONFIG_HAS_EARLYSUSPEND */

#ifdef CONFIG_PM_SLEEP
static int zinitix_resume(struct device *dev)
{
	struct bt432_ts_data *touch_dev = misc_touch_dev;

	if (touch_dev == NULL)
		return -1;

	zinitix_printk("%s\n", __func__);

	down(&touch_dev->work_proceedure_lock);
	if (touch_dev->work_proceedure != TS_IN_SUSPEND) {
		zinitix_printk("invalid work proceedure (%d)\n",
			touch_dev->work_proceedure);
		up(&touch_dev->work_proceedure_lock);
		return 0;
	}

	bt432_power_control(touch_dev, POWER_ON_SEQUENCE);

#ifdef CONFIG_HAS_EARLYSUSPEND
	touch_dev->work_proceedure = TS_IN_RESUME;
#else
	touch_dev->work_proceedure = TS_NO_WORK;
	if (ts_mini_init_touch(touch_dev) == false)
		zinitix_printk("failed to resume\n");
	enable_irq(touch_dev->irq);
#endif

	up(&touch_dev->work_proceedure_lock);
	touch_dev->open_done = true;
	zinitix_printk("%s: done.\n", __func__);

	return 0;
}

static int zinitix_suspend(struct device *dev)
{
	struct bt432_ts_data *touch_dev = misc_touch_dev; //i2c_get_clientdata(client);

	if (touch_dev == NULL)
		return -1;

	zinitix_printk("%s\n", __func__);

	touch_dev->open_done = false;

#ifndef CONFIG_HAS_EARLYSUSPEND
	disable_irq(touch_dev->irq);
#endif
	if (touch_dev->use_esd_timer)
		flush_work(&touch_dev->tmr_work);
	down(&touch_dev->work_proceedure_lock);
	if (touch_dev->work_proceedure != TS_NO_WORK
		&& touch_dev->work_proceedure != TS_IN_EALRY_SUSPEND) {
		zinitix_printk("invalid work proceedure (%d)\n",
			touch_dev->work_proceedure);
		up(&touch_dev->work_proceedure_lock);
#ifndef CONFIG_HAS_EARLYSUSPEND
		enable_irq(touch_dev->irq);
#endif
		return 0;
	}

#ifndef CONFIG_HAS_EARLYSUSPEND
	zinitix_clear_report_data(touch_dev);

	if (touch_dev->use_esd_timer) {
		ts_esd_timer_stop(touch_dev);
		zinitix_printk("ts_esd_timer_stop\n");
	}

	ts_write_cmd(touch_dev->client, ZINITIX_SLEEP_CMD);
	udelay(100);
#endif
	bt432_power_control(touch_dev, POWER_OFF);
	touch_dev->work_proceedure = TS_IN_SUSPEND;

	up(&touch_dev->work_proceedure_lock);
	zinitix_printk("%s: done.\n", __func__);

	return 0;
}
#endif

static bool ts_set_touchmode(u16 value)
{
	int i;

	disable_irq(misc_touch_dev->irq);

	down(&misc_touch_dev->work_proceedure_lock);
	if (misc_touch_dev->work_proceedure != TS_NO_WORK) {
		printk(KERN_INFO "other process occupied.. (%d)\n",
			misc_touch_dev->work_proceedure);
		enable_irq(misc_touch_dev->irq);
		up(&misc_touch_dev->work_proceedure_lock);
		return -1;
	}

	misc_touch_dev->work_proceedure = TS_SET_MODE;

	if (value == TOUCH_DND_MODE) {
		if (ts_write_reg(misc_touch_dev->client, ZINITIX_N_COUNT, SEC_DND_N_COUNT)!=I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : Fail to set ZINITIX_DND_N_COUNT %d.\n", SEC_DND_N_COUNT);
		if (ts_write_reg(misc_touch_dev->client, ZINITIX_U_COUNT,SEC_DND_U_COUNT )!=I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : Fail to set ZINITIX_DND_U_COUNT %d.\n", SEC_DND_U_COUNT);
		if (ts_write_reg(misc_touch_dev->client, ZINITIX_AFE_FREQUENCY, SEC_DND_FREQUENCY)!=I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : Fail to set ZINITIX_SDND_AFE_FREQUENCY %d.\n", SEC_DND_FREQUENCY);
	} else if (misc_touch_dev->touch_mode == TOUCH_DND_MODE){
		if (ts_write_reg(misc_touch_dev->client, ZINITIX_AFE_FREQUENCY, misc_touch_dev->cap_info.afe_frequency)!=I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : Fail to set ZINITIX_AFE_FREQUENCY %d.\n", misc_touch_dev->cap_info.afe_frequency);
		if (ts_write_reg(misc_touch_dev->client, ZINITIX_U_COUNT, misc_touch_dev->cap_info.u_count)!=I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : Fail to reset ZINITIX_DND_U_COUNT.\n");
		if (ts_write_reg(misc_touch_dev->client, ZINITIX_SHIFT_VALUE, misc_touch_dev->cap_info.shift_value)!=I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : Fail to reset ZINITIX_SHIFT_VALUE.\n");
	}

	if (value == TOUCH_SEC_NORMAL_MODE)
		misc_touch_dev->touch_mode = TOUCH_POINT_MODE;
	else
		misc_touch_dev->touch_mode = value;

	printk(KERN_INFO "[zinitix_touch] tsp_set_testmode, touchkey_testmode = %d\n", misc_touch_dev->touch_mode);

	if (misc_touch_dev->touch_mode != TOUCH_POINT_MODE) {
		if (ts_write_reg(misc_touch_dev->client,
			ZINITIX_DELAY_RAW_FOR_HOST,
			RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS)
			zinitix_printk("Fail to set ZINITIX_DELAY_RAW_FOR_HOST.\n");
	}

	if (ts_write_reg(misc_touch_dev->client, ZINITIX_TOUCH_MODE, misc_touch_dev->touch_mode)!=I2C_SUCCESS)
		printk(KERN_INFO "[zinitix_touch] TEST Mode : Fail to set ZINITX_TOUCH_MODE %d.\n", misc_touch_dev->touch_mode);

	// clear garbage data
	for (i=0; i < 10; i++) {
		mdelay(20);
		ts_write_cmd(misc_touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
	}

	misc_touch_dev->work_proceedure = TS_NO_WORK;
	enable_irq(misc_touch_dev->irq);
	up(&misc_touch_dev->work_proceedure_lock);
	return 1;
}

static bool ts_set_touchmode2(u16 value){
	int i;

	disable_irq(misc_touch_dev->irq);

	down(&misc_touch_dev->work_proceedure_lock);
	if (misc_touch_dev->work_proceedure != TS_NO_WORK) {
		printk(KERN_INFO "other process occupied.. (%d)\n",
			misc_touch_dev->work_proceedure);
		enable_irq(misc_touch_dev->irq);
		up(&misc_touch_dev->work_proceedure_lock);
		return -1;
	}

	misc_touch_dev->work_proceedure = TS_SET_MODE;

	if (value == TOUCH_DND_MODE) {

		if (ts_write_reg(misc_touch_dev->client, ZINITIX_N_COUNT, SEC_HFDND_N_COUNT)!=I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : Fail to set ZINITIX_HFDND_N_COUNT %d.\n", SEC_HFDND_N_COUNT);
		if (ts_write_reg(misc_touch_dev->client, ZINITIX_U_COUNT, SEC_HFDND_U_COUNT)!=I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : Fail to set ZINITIX_HFDND_U_COUNT %d.\n", SEC_HFDND_U_COUNT);
		if (ts_write_reg(misc_touch_dev->client, ZINITIX_AFE_FREQUENCY, SEC_HFDND_FREQUENCY)!=I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : Fail to set ZINITIX_HFDND_AFE_FREQUENCY HFDND %d.\n", SEC_HFDND_FREQUENCY);
	}
	else
	{
		if (ts_write_reg(misc_touch_dev->client, ZINITIX_AFE_FREQUENCY,
			misc_touch_dev->cap_info.afe_frequency)!=I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : Fail to set ZINITIX_AFE_FREQUENCY %d.\n", misc_touch_dev->cap_info.afe_frequency);
		if (ts_write_reg(misc_touch_dev->client, ZINITIX_U_COUNT,
			misc_touch_dev->cap_info.u_count)!=I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : Fail to reset ZINITIX_DND_U_COUNT.\n");
		if (ts_write_reg(misc_touch_dev->client, ZINITIX_SHIFT_VALUE,
			misc_touch_dev->cap_info.shift_value)!=I2C_SUCCESS)
			printk(KERN_INFO "[zinitix_touch] TEST Mode : Fail to reset ZINITIX_SHIFT_VALUE.\n");
	}


	if (value == TOUCH_SEC_NORMAL_MODE)
		misc_touch_dev->touch_mode = TOUCH_POINT_MODE;
	else
		misc_touch_dev->touch_mode = value;

	printk(KERN_INFO "[zinitix_touch] tsp_set_testmode, touchkey_testmode = %d\n", misc_touch_dev->touch_mode);

	if (misc_touch_dev->touch_mode != TOUCH_POINT_MODE) {
		if (ts_write_reg(misc_touch_dev->client,
			ZINITIX_DELAY_RAW_FOR_HOST,
			RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS)
			zinitix_printk("Fail to set ZINITIX_DELAY_RAW_FOR_HOST.\n");

	}

	if (ts_write_reg(misc_touch_dev->client, ZINITIX_TOUCH_MODE, misc_touch_dev->touch_mode)!=I2C_SUCCESS)
		printk(KERN_INFO "[zinitix_touch] TEST Mode : Fail to set ZINITX_TOUCH_MODE %d.\n", misc_touch_dev->touch_mode);


	// clear garbage data
	for (i=0; i < 10; i++) {
		mdelay(20);
		ts_write_cmd(misc_touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
	}

	misc_touch_dev->work_proceedure = TS_NO_WORK;
	enable_irq(misc_touch_dev->irq);
	up(&misc_touch_dev->work_proceedure_lock);
	return 1;
}

static int ts_upgrade_sequence(const u8 *firmware_data)
{
	disable_irq(misc_touch_dev->irq);
	down(&misc_touch_dev->work_proceedure_lock);
	misc_touch_dev->work_proceedure = TS_IN_UPGRADE;

	if (misc_touch_dev->use_esd_timer)
		ts_esd_timer_stop(misc_touch_dev);
	zinitix_debug_msg("clear all reported points\n");
	zinitix_clear_report_data(misc_touch_dev);

	printk(KERN_INFO "start upgrade firmware\n");
	if (ts_upgrade_firmware(misc_touch_dev,
		firmware_data,
		misc_touch_dev->cap_info.ic_fw_size) == false) {
		enable_irq(misc_touch_dev->irq);
		misc_touch_dev->work_proceedure = TS_NO_WORK;
		up(&misc_touch_dev->work_proceedure_lock);
		return -1;
	}

	if (init_touch(misc_touch_dev) == false) {
		enable_irq(misc_touch_dev->irq);
		misc_touch_dev->work_proceedure = TS_NO_WORK;
		up(&misc_touch_dev->work_proceedure_lock);
		return -1;
	}

	if (misc_touch_dev->use_esd_timer) {
		ts_esd_timer_start(ZINITIX_CHECK_ESD_TIMER,
			misc_touch_dev);
		zinitix_debug_msg("esd timer start\n");
	}

	enable_irq(misc_touch_dev->irq);
	misc_touch_dev->work_proceedure = TS_NO_WORK;
	up(&misc_touch_dev->work_proceedure_lock);
	return 0;
}

#if SEC_TSP_FACTORY_TEST

#define CHIP_INFO		"ZINITIX"

static void set_cmd_result(struct bt432_ts_data *info, char *buff, int len)
{
	strncat(info->cmd_result, buff, len);
}

static void set_default_result(struct bt432_ts_data *info)
{
	char delim = ':';
	memset(info->cmd_result, 0x00, ARRAY_SIZE(info->cmd_result));
	memcpy(info->cmd_result, info->cmd, strlen(info->cmd));
	strncat(info->cmd_result, &delim, 1);
}

static void not_support_cmd(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};

	set_default_result(info);
	sprintf(buff, "%s", "NA");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = NOT_APPLICABLE;
	dev_info(&info->client->dev, "%s: \"%s(%d)\"\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
	return;
}

static void get_fw_ver_bin(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	u16 newVersion, newMinorVersion, newRegVersion, newHWID;
	u32 version;
	char buff[16] = {0};
	u8 *firmware_data;

	set_default_result(info);

	firmware_data = (u8*)m_firmware_data;

	newVersion = (u16) (firmware_data[52] | (firmware_data[53]<<8));
	newMinorVersion = (u16) (firmware_data[56] | (firmware_data[57]<<8));
	newRegVersion = (u16) (firmware_data[60] | (firmware_data[61]<<8));
	newHWID = (u16) (firmware_data[0x57d2] | (firmware_data[0x57d3]<<8));

	version = (u32)((u32)(newHWID&0xff)<<16)|((newVersion&0xf)<<12)|((newMinorVersion&0xf)<<8)|(newRegVersion&0xff);

	snprintf(buff, sizeof(buff), "ZI%06X", version);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_fw_ver_ic(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	struct _ts_capa_info *cap_info = &info->cap_info;
	u32 version;
	char buff[16] = {0};
	u16 firmware_version;
	u16 minor_firmware_version;
	u16 reg_data_version;
	u16 newHWID;
	int read_retry;
	int cmp_retry;

	set_default_result(info);

	for (cmp_retry = 0; cmp_retry < I2C_RETRY_CNT; cmp_retry++) {
		read_retry = I2C_RETRY_CNT;
		do {
			if (ts_read_data(info->client, ZINITIX_FIRMWARE_VERSION,
					(u8 *)&firmware_version, 2)) {
				zinitix_printk("touch chip firmware version = 0x%x\n", firmware_version);
				read_retry = 0;
			} else {
				zinitix_printk("Major version read failed.\n");
				msleep(10);
				read_retry--;
			}
		} while(read_retry);

		if (firmware_version == cap_info->firmware_version)
			break;
		else
			zinitix_printk("firmware_version=[0x%x], cap_info->firmware_version=[0x%x]\n",\
				firmware_version, cap_info->firmware_version);
	}

	for (cmp_retry = 0; cmp_retry < I2C_RETRY_CNT; cmp_retry++) {
		read_retry = I2C_RETRY_CNT;
		do {
			if (ts_read_data(info->client, ZINITIX_MINOR_FW_VERSION,
					(u8 *)&minor_firmware_version, 2)) {
				zinitix_printk("touch chip firmware version = 0x%x\n", minor_firmware_version);
				read_retry = 0;
			} else {
				zinitix_printk("Minor version read failed.\n");
				msleep(10);
				read_retry--;
			}
		} while(read_retry);

		if (minor_firmware_version == cap_info->firmware_minor_version)
			break;
		else
			zinitix_printk("minor_firmware_version=[0x%x], cap_info->firmware_minor_version=[0x%x]\n",\
				minor_firmware_version, cap_info->firmware_minor_version);
	}

	for (cmp_retry = 0; cmp_retry < I2C_RETRY_CNT; cmp_retry++) {
		read_retry = I2C_RETRY_CNT;
		do {
			if (ts_read_data(info->client, ZINITIX_DATA_VERSION_REG,
					(u8 *)&reg_data_version, 2)) {
				zinitix_debug_msg("touch reg data version = 0x%02x\n", reg_data_version);
				read_retry = 0;
			} else {
				zinitix_printk("reg_data version read failed.\n");
				msleep(10);
				read_retry--;
			}
		} while(read_retry);

		if (reg_data_version == cap_info->reg_data_version)
			break;
		else
			zinitix_printk("reg_data_version=[0x%x], cap_info->reg_data_version=[0x%x]\n",\
				reg_data_version, cap_info->reg_data_version);
	}

	for (cmp_retry = 0; cmp_retry < I2C_RETRY_CNT; cmp_retry++) {
		read_retry = I2C_RETRY_CNT;
		do {
			if (ts_read_data(info->client, ZINITIX_HW_ID,
					(u8 *)&newHWID, 2)) {
				zinitix_printk("touch chip hw id = 0x%02x\n",	newHWID);
				read_retry = 0;
			} else {
				zinitix_printk("touch chip hw id read failed.\n");
				msleep(10);
				read_retry--;
			}
		} while(read_retry);

		if (newHWID == cap_info->hw_id)
			break;
		else
			zinitix_printk("HWID=[0x%x], cap_info->hw_id=[0x%x]\n",\
				newHWID, cap_info->hw_id);
	}

	cap_info->firmware_version = firmware_version;
	cap_info->firmware_minor_version = minor_firmware_version;
	cap_info->reg_data_version = reg_data_version;
	cap_info->hw_id = newHWID;

	version = (u32)((u32)(newHWID&0xff)<<16)|((firmware_version&0xf)<<12)|\
		((minor_firmware_version&0xf)<<8)|(reg_data_version&0xff);

	snprintf(buff, sizeof(buff), "ZI%06X", version);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_x_num(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};

	set_default_result(info);

	snprintf(buff, sizeof(buff), "%u", info->cap_info.x_node_num);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_y_num(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};

	set_default_result(info);

	snprintf(buff, sizeof(buff), "%u", info->cap_info.y_node_num);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_chip_vendor(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};

	set_default_result(info);

	snprintf(buff, sizeof(buff), "%s", CHIP_INFO);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_chip_name(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};

	set_default_result(info);

	snprintf(buff, sizeof(buff), "BT432");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_threshold(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	int ret = 0;
	u16 threshold;
	char buff[16] = {0};

	set_default_result(info);

	ret = ts_read_data(info->client, ZINITIX_SENSITIVITY, (u8*)&threshold, 2);

	if (ret < 0) {
		snprintf(buff, sizeof(buff), "%s", "fail");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = FAIL;
		return;
	}

	snprintf(buff, sizeof(buff), "%d", threshold);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void run_connect_test(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	int ret = 0;
	u16 threshold;
	char buff[16] = {0};

	set_default_result(info);

	ret = ts_read_data(info->client, ZINITIX_SENSITIVITY, (u8*)&threshold, 2);

	if (ret < 0) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = FAIL;
		return;
	}

	info->cmd_state = OK;
	snprintf(buff, sizeof(buff) , "%s", "OK");
	set_cmd_result(info, buff,
				strnlen(buff, sizeof(buff)));
}

static void get_key_threshold(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	int ret = 0;
	u16 threshold;
	char buff[16] = {0};

	set_default_result(info);

	ret = ts_read_data(misc_touch_dev->client, ZINITIX_BUTTON_SENSITIVITY, (u8*)&threshold, 2);

	if (ret < 0) {
		snprintf(buff, sizeof(buff), "%s", "fail");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = FAIL;
		return;
	}

	snprintf(buff, sizeof(buff), "%u", threshold);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static bool get_raw_data(struct bt432_ts_data *touch_dev, u8 *buff, int skip_cnt)
{
	u32 total_node = touch_dev->cap_info.total_node_num + touch_dev->cap_info.x_node_num;
	u32 sz;
	int i;

	disable_irq(touch_dev->irq);

	if (touch_dev->work_proceedure != TS_NO_WORK) {
		printk(KERN_INFO "other process occupied.. (%d)\n",
			touch_dev->work_proceedure);
		enable_irq(touch_dev->irq);
		up(&touch_dev->work_proceedure_lock);
		return false;
	}

	touch_dev->work_proceedure = TS_GET_RAW_DATA;

	for (i=0; i<skip_cnt; i++) {
		while (gpio_get_value(touch_dev->int_gpio_num))
			msleep(1);

		ts_write_cmd(touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
		msleep(1);
	}

	zinitix_debug_msg("read raw data\n");
	sz = total_node*2;

	while (gpio_get_value(touch_dev->int_gpio_num))
		msleep(1);

	if (ts_read_raw_data(touch_dev->client,
		ZINITIX_RAWDATA_REG, (char *)buff, sz) < 0) {
		zinitix_printk("error : read zinitix tc raw data\n");
		touch_dev->work_proceedure = TS_NO_WORK;
		enable_irq(touch_dev->irq);
		up(&touch_dev->work_proceedure_lock);
		return false;
	}
	ts_write_cmd(touch_dev->client, ZINITIX_CLEAR_INT_STATUS_CMD);
	touch_dev->work_proceedure = TS_NO_WORK;
	enable_irq(touch_dev->irq);
	return true;
}

#if 0
static void run_reference_read(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};
	unsigned int min, max;
	int x_num =info->cap_info.y_node_num+1, y_num=info->cap_info.x_node_num;
	int i,j;

	set_default_result(info);
	ts_set_touchmode(TOUCH_SDND_MODE);
	get_raw_data(info, (u8 *)info->dnd_data, 10);
	ts_set_touchmode(TOUCH_POINT_MODE);

	min = 0xFFFF;
	max = 0x0000;

	for (i=0; i<x_num-1; i++) {
		for (j=0; j<y_num; j++) {
			printk("%d ", info->dnd_data[j+i*y_num]);

			if ((info->dnd_data[j+i*y_num] < min) && (info->dnd_data[j+i*y_num]!=0)) {
				min = info->dnd_data[j+i*y_num];
			}

			if (info->dnd_data[j+i*y_num] > max) {
				max = info->dnd_data[j+i*y_num];
			}
		}
		printk("\n");
	}

	printk(" raw_min= %u, raw_max= %u", min, max);
	sprintf(buff, "%u,%u\n", min, max);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	info->cmd_state = OK;
}
#endif


static bool run_reference_DND_read(void *device_data, int button0, int button1) //DND
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;

	int i, j, nButton;
	int x_num = info->cap_info.y_node_num+1, y_num = info->cap_info.x_node_num;
	int buttons[2] = {button0,button1};
	bool result = true, item_result= true;

	set_default_result(info);
	if (0 > ts_set_touchmode(TOUCH_DND_MODE)) {
		printk("%s: failed to change TOUCH_DND_MODE\n", __func__);
		return false;
	}

	if (!get_raw_data(info, (u8 *)info->pdnd_data, 5)) {
		printk("%s: failed get_raw_data\n", __func__);
		return false;
	}

	if (0 > ts_set_touchmode(TOUCH_POINT_MODE)) {
		printk("%s: failed to change TOUCH_POINT_MODE\n", __func__);
		return false;
	}

	printk("PDND_data =\n");
	for (i=0; i<x_num; i++) {
		for (j=0; j<y_num; j++) {
		            printk("%5d ", info->pdnd_data[j+i*y_num]);
		}
		printk("\n");
	}

	info->dnd_max_x = info->dnd_max_y = info->dnd_min_x = info->dnd_min_y = 0;
	info->dnd_max_val = -32768;
	info->dnd_min_val = 32767;
	info->dnd_real_max_val = -32768;
	info->dnd_real_min_val = 32767;

	//View area
	for (i=0; i<x_num-1; i++) {
		for (j=0; j<y_num; j++) {
			if (info->dnd_real_min_val > info->pdnd_data[j+i*y_num]) {
				info->dnd_real_min_x = i;
				info->dnd_real_min_y = j;
				info->dnd_real_min_val = info->pdnd_data[j+i*y_num];
			}
			if (info->dnd_real_max_val < info->pdnd_data[j+i*y_num]) {
				info->dnd_real_max_x = i;
				info->dnd_real_max_y = j;
				info->dnd_real_max_val = info->pdnd_data[j+i*y_num];
			}
			if ((info->pdnd_data[j+i*y_num] < pdnd_min[i][j]) && (info->pdnd_data[j+i*y_num]!=0)) {
				printk(" DND View Min fail : NODE = %u, Raw data = %d, SPEC = %u\n",j+i*y_num, info->pdnd_data[j+i*y_num], pdnd_min[i][j]);
				item_result = false;
				result = false;
			}
			if (info->dnd_min_val > info->pdnd_data[j+i*y_num] - pdnd_min[i][j]) {
				info->dnd_min_x = i;
				info->dnd_min_y = j;
				info->dnd_min_val = info->pdnd_data[j+i*y_num] - pdnd_min[i][j];
			}
			if (info->pdnd_data[j+i*y_num] > pdnd_max[i][j]) {
				printk(" DND View Max fail : NODE = %u, Raw data = %d, SPEC = %u\n",j+i*y_num, info->pdnd_data[j+i*y_num], pdnd_max[i][j]);
				item_result = false;
				result = false;
			}
			if (info->dnd_max_val < info->pdnd_data[j+i*y_num] - pdnd_max[i][j]) {
				info->dnd_max_x = i;
				info->dnd_max_y = j;
				info->dnd_max_val = info->pdnd_data[j+i*y_num] - pdnd_max[i][j];
			}
		}
	}

	if (item_result)
		printk("DND View pass\n");
	else
		printk("DND View fail\n");

	//button
	item_result = true;
	if ( info->cap_info.button_num) {
		for (i = 0; i < 2; i++) {
			nButton = buttons[i];

			if (nButton < 0)
				continue;
			if (info->pdnd_data[(x_num-1)*y_num+nButton] < pdnd_min[x_num-1][nButton]) {
				printk(" DND Button Min fail : NODE = %u,  Raw data = %d, SPEC = %u\n", nButton, info->pdnd_data[(x_num-1)*y_num+nButton], pdnd_min[x_num-1][nButton]);
				item_result = false;
				result = false;
			}

			if (info->dnd_min_val > info->pdnd_data[(x_num-1)*y_num+nButton] - pdnd_min[x_num-1][nButton]) {
				info->dnd_min_x = x_num-1;
				info->dnd_min_y = nButton;
				info->dnd_min_val = info->pdnd_data[(x_num-1)*y_num+nButton] - pdnd_min[x_num-1][nButton];
			}

			if (info->pdnd_data[(x_num-1)*y_num+nButton] > pdnd_max[x_num-1][nButton]) {
				printk(" DND Button Max fail : NODE = %u, Raw data = %d, SPEC = %u\n",nButton, info->pdnd_data[(x_num-1)*y_num+nButton], pdnd_max[x_num-1][nButton]);
				item_result = false;
				result = false;
			}

			if (info->dnd_max_val <  info->pdnd_data[(x_num-1)*y_num+nButton] - pdnd_max[x_num-1][nButton]) {
				info->dnd_max_x = x_num-1;
				info->dnd_max_y = nButton;
				info->dnd_max_val =  info->pdnd_data[(x_num-1)*y_num+nButton] - pdnd_max[x_num-1][nButton];
			}
		}
	}

	if (item_result) {
		info->dnd_done = true;
		printk("DND Button pass\n");
	} else {
		info->dnd_done = false;
		printk("DND Butoon fail\n");
	}

	return result;
}

static bool run_reference_HFDND_read(void *device_data, int button0, int button1) //HFDND
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	int x_num = info->cap_info.y_node_num+1, y_num = info->cap_info.x_node_num;
	int buttons[2] = {button0,button1};
	bool result = true, item_result = true;
	int i, j, nButton;

	set_default_result(info);
	ts_set_touchmode2(TOUCH_DND_MODE);
	get_raw_data(info, (u8 *)info->hfdnd_data, 5);
	ts_set_touchmode2(TOUCH_POINT_MODE);

	printk("HFDND_data =\n");
	for (i=0; i<x_num; i++) {
		for (j=0; j<y_num; j++)
			printk("%5d ", info->hfdnd_data[j+i*y_num]);
		printk("\n");
	}

	info->hfdnd_max_x = info->hfdnd_max_y = info->hfdnd_min_x = info->hfdnd_min_y = 0;
	info->hfdnd_max_val = -32768;
	info->hfdnd_min_val = 32767;

	//MAX
	for (i=0; i<x_num-1; i++) {
		for (j=0; j<y_num; j++) {
			if ((info->hfdnd_data[j+i*y_num] < hfdnd_min[i][j]) && (info->hfdnd_data[j+i*y_num]!=0)) {
				printk(" HFDND View Min fail : NODE = %u,Raw data = %d, SPEC = %u\n",j+i*y_num, info->hfdnd_data[j+i*y_num], hfdnd_min[i][j]);
				item_result = false;
				result = false;
			}

			if (info->hfdnd_min_val > info->hfdnd_data[j+i*y_num] - hfdnd_min[i][j]) {
				info->hfdnd_min_x = i;
				info->hfdnd_min_y = j;
				info->hfdnd_min_val = info->hfdnd_data[j+i*y_num] - hfdnd_min[i][j];
			}

			if (info->hfdnd_data[j+i*y_num] > hfdnd_max[i][j]) {
				printk(" HFDND View Max fail : NODE = %u, Raw data = %d, SPEC = %u\n",j+i*y_num, info->hfdnd_data[j+i*y_num], hfdnd_max[i][j]);
				item_result = false;
				result = false;
			}

			if (info->hfdnd_max_val < info->hfdnd_data[j+i*y_num] - hfdnd_max[i][j]) {
				info->hfdnd_max_x = i;
				info->hfdnd_max_y = j;
				info->hfdnd_max_val = info->hfdnd_data[j+i*y_num] - hfdnd_max[i][j];
			}
		}
	}

	if (item_result)
		printk("HFDND View pass\n");
	else
		printk("HFDND View Fail\n");

	//button
	item_result = true;
	if ( info->cap_info.button_num) {
		for (i = 0; i < 2; i++) {
			nButton = buttons[i];
			if (nButton < 0)
				continue;
			if (info->hfdnd_data[(x_num-1)*y_num+nButton] < hfdnd_min[x_num-1][nButton]) {
				printk(" HFDND Button Min fail : NODE = %u, Raw data = %d, SPEC = %u\n",nButton, info->hfdnd_data[(x_num-1)*y_num+nButton], hfdnd_min[x_num-1][nButton]);
				item_result = false;
				result = false;
			}
			if (info->hfdnd_min_val > info->hfdnd_data[(x_num-1)*y_num+nButton] - hfdnd_min[x_num-1][nButton]) {
				info->hfdnd_min_x = x_num-1;
				info->hfdnd_min_y = nButton;
				info->hfdnd_min_val = info->hfdnd_data[(x_num-1)*y_num+nButton]- hfdnd_min[x_num-1][nButton];
			}
			if (info->hfdnd_data[(x_num-1)*y_num+nButton] > hfdnd_max[x_num-1][nButton]) {
				printk(" HFDND Button Max fail : NODE = %u, Raw data = %d, SPEC = %u\n",nButton, info->hfdnd_data[(x_num-1)*y_num+nButton], hfdnd_min[x_num-1][nButton]);
				item_result = false;
				result = false;
			}
			if (info->hfdnd_max_val <  info->hfdnd_data[(x_num-1)*y_num+nButton] -  hfdnd_min[x_num-1][nButton]) {
				info->hfdnd_max_x = x_num-1;
				info->hfdnd_max_y = nButton;
				info->hfdnd_max_val =  info->hfdnd_data[(x_num-1)*y_num+nButton] -  hfdnd_min[x_num-1][nButton];
			}
		}
	}

	if (item_result) {
		info->hfdnd_done = true;
		printk("HFDND Button pass\n");
	} else {
		info->hfdnd_done = false;
		printk("HFDND Button fail\n");
	}

	return result;
}

static void run_reference_DND(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};
	bool temp;

	info->cmd_state = WAITING;
	set_default_result(info);

	temp = run_reference_DND_read(device_data, 4, 7);


	if (temp) {
		printk("DND pass\n");
		sprintf(buff, "%s\n", "OK");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = OK;
	} else {
		printk("DND fail\n");
		sprintf(buff, "%s\n", "NG");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = FAIL;
	}
}


static void run_reference_HFDND(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};
	bool	temp;

	info->cmd_state = WAITING;
	set_default_result(info);

	temp = run_reference_HFDND_read(device_data, 4, 7);

	if (temp) {
		printk("HF DND pass\n");
		sprintf(buff, "%s\n", "OK");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = OK;
	} else {
		printk("HF DND fail\n");
		sprintf(buff, "%s\n", "NG");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = FAIL;
	}
}

static bool run_reference_PDiff_read(void *device_data, int button0, int button1) //DND
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	int print_diff[19*10]={0,};
	int buttons[2] = {button0,button1};
	int i, j, diff_val, pre_val, next_val, nButton;
	int x_num = info->cap_info.y_node_num+1, y_num = info->cap_info.x_node_num;
	bool result = true, item_result = true;

	printk("TSP Diff test scale factor = %d\n", info->ref_scale_factor);
	printk("H Diff start\n");
	//H DIff
	info->hdiff_max_x = info->hdiff_max_y = info->hdiff_min_x = info->hdiff_min_y = 0;
	info->hdiff_max_val = -32768;
	info->hdiff_min_val = 32767;

	for (i = 0; i < x_num - 1; i++) {
		for (j = 0; j <y_num-1; j++) {
			//printk("%d ", info->dnd_data[i*y_num+j]);
			next_val = info->pdnd_data[(i*y_num)+(j+1)];
			pre_val = info->pdnd_data[(i*y_num)+j];
			diff_val = (next_val > pre_val)?(next_val - pre_val):(pre_val - next_val);

			printk("%d ", diff_val);

			pre_val = (info->ref_scale_factor == 100)?((s16)dnd_h_diff[i][j]):((s16)(((s32)dnd_h_diff[i][j] * info->ref_scale_factor) /100));
			if (diff_val > pre_val) {
				result = false;
				item_result = false;
			}
#if PDIFF_DEBUG
			print_diff[i*y_num+j] = diff_val;
#endif
			if (info->hdiff_max_val < diff_val-pre_val) {
				info->hdiff_max_val = diff_val-pre_val;
				info->hdiff_max_x = i;
				info->hdiff_max_y= j;
			}
			if (info->hdiff_min_val > diff_val - pre_val) {
				info->hdiff_min_val =  diff_val - pre_val;
				info->hdiff_min_x = i;
				info->hdiff_min_y= j;
			}
		}
		printk("\n");
	}

	if (item_result)
		printk("H Diff pass\n");
	else
		printk("H Diff fail\n");

	info->hdiff_max_x = info->hdiff_max_y = info->hdiff_min_x = info->hdiff_min_y = 0;
	info->vdiff_max_val = -32768;
	info->vdiff_min_val = 32767;

	//V DIff  View
	item_result = true;
	for (i=0; i < x_num-2; i++) {
		for (j=0; j<y_num; j++) {
			//printk("%d ", info->pdnd_data[i*y_num+j]);
			next_val = info->pdnd_data[(i*y_num)+j];
			pre_val = info->pdnd_data[(i*y_num)+j+y_num];
			diff_val = (next_val > pre_val)?(next_val - pre_val):(pre_val - next_val);

			printk(" %d ", diff_val);

			pre_val = (info->ref_scale_factor == 100)?((s16)dnd_v_diff[i][j]):((s16)(((s32)dnd_v_diff[i][j] * info->ref_scale_factor) /100));
			if (diff_val > pre_val) {
				result = false;
				item_result = false;
			}
#if PDIFF_DEBUG
			print_diff[i*y_num+j] = diff_val;
#endif
			if (info->vdiff_max_val < diff_val - pre_val) {
				info->vdiff_max_val = diff_val -pre_val;
				info->vdiff_max_x = i;
				info->vdiff_max_y= j;
			}
			if (info->vdiff_min_val > diff_val - pre_val) {
				info->vdiff_min_val = diff_val - pre_val;
				info->vdiff_min_x = i;
				info->vdiff_min_y= j;
			}
		}
		printk("\n");
	}

	if (item_result)
		printk("V Diff view pass\n");
	else
		printk("V Diff view fail\n");

//V DIff  button
	item_result = true;
	if ( info->cap_info.button_num) {
		printk("TSP Button scale = %d\n", info->ref_scale_factor);
		printk("TSP Button Diff Spec. = %d %d\n",
			dnd_v_diff[x_num-2][buttons[0]], dnd_v_diff[x_num-2][buttons[1]]);

		for (i = 0; i < 2; i++) {
			nButton = buttons[i];
			if (nButton < 0)
				continue;
			next_val = info->pdnd_data[(x_num-1)*y_num+nButton];
			pre_val = info->pdnd_data[(x_num-2)*y_num+nButton];
			diff_val = (next_val > pre_val)?(next_val - pre_val):(pre_val - next_val);

			pre_val = (info->ref_scale_factor == 100)?( (s16)dnd_v_diff[x_num-2][nButton]+info->ref_btn_option): \
				( (s16)(((s32)dnd_v_diff[x_num-2][nButton]*info->ref_scale_factor)/100)+info->ref_btn_option);

			if (diff_val > pre_val) {
				item_result = false;
				result = false;
			}
			if (info->vdiff_max_val < diff_val - pre_val) {
				info->vdiff_max_val = diff_val - pre_val;
				info->vdiff_max_x = x_num - 1;
				info->vdiff_max_y= nButton;
			}
			if (info->vdiff_min_val	>diff_val - pre_val) {
				info->vdiff_min_val = diff_val-pre_val;
				info->vdiff_min_x = x_num - 1;
				info->vdiff_min_y= nButton;
			}
#if PDIFF_DEBUG
			printk("buttons[%d]'s diff_val is %d\n", i, diff_val);
#endif
		}
	}
	if (item_result)
		printk("Button Diff pass\n");
	else
		printk("Button Diff fail\n");

	return result;
}

static void run_reference_PDiff(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};
	bool	temp;

	info->cmd_state = WAITING;
	set_default_result(info);

	temp = run_reference_PDiff_read(device_data, 4, 7);

	if (temp) {
		printk("diff pass\n");
		sprintf(buff, "%s\n", "OK");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = OK;
	} else {
		printk("diff fail\n");
		sprintf(buff, "%s\n", "NG");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = FAIL;
	}
}

static void get_reference_max_H_Diff(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};
	int H_diff[19*9] = {0};
	int i, j, diff_val, pre_val, next_val;
	int x_num = info->cap_info.y_node_num+1, y_num = info->cap_info.x_node_num;
	int max_hdiff = 0;

	set_default_result(info);

	printk("H Diff start\n");
	//H DIff
	for (i = 0; i < x_num - 1; i++) {
		for (j = 0; j <y_num-1; j++) {
			next_val = info->pdnd_data[(i*y_num)+(j+1)];
			pre_val = info->pdnd_data[(i*y_num)+j];
			diff_val = (next_val > pre_val)?(next_val - pre_val):(pre_val - next_val);

			printk("%4d ", diff_val);
			H_diff[i*(y_num-1)+j] = diff_val;
			if ((i == 0) && (j == 0))
				max_hdiff = diff_val;
			else {
				if (max_hdiff < diff_val)
					max_hdiff = diff_val;
			}
		}
		printk("\n");
	}


	sprintf(buff, "%d", max_hdiff);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
}

static void get_reference_H_Diff(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[19*10] = {0};
	int H_diff[19*9] = {0};
	char tmp[10] = {0};
	int y_node;
	int node_num;
	int i, j, diff_val, pre_val, next_val;
	int x_num = info->cap_info.y_node_num+1, y_num = info->cap_info.x_node_num;

	set_default_result(info);

	printk("H Diff start\n");
	//H DIff
	for (i = 0; i < x_num - 1; i++) {
		for (j = 0; j <y_num-1; j++) {
			next_val = info->pdnd_data[(i*y_num)+(j+1)];
			pre_val = info->pdnd_data[(i*y_num)+j];
			diff_val = (next_val > pre_val)?(next_val - pre_val):(pre_val - next_val);

			printk("%4d ", diff_val);
			H_diff[i*(y_num-1)+j] = diff_val;
		}
		printk("\n");
	}

	y_node = info->cmd_param[0];

	if (y_node < 0 || y_node > y_num-1) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = FAIL;
		return;
	}

	for (i=0; i < x_num-1; i++) {
		node_num = i*(y_num-1) + y_node;
		sprintf(tmp, "%d", H_diff[node_num]);
		strcat(buff, tmp);
		if (i < x_num-2)
			strcat(buff, " ");
	}

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
}

static void get_reference_max_V_Diff(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};
	int V_Diff[19*10] = {0};
	int buttons[2] = {4,7};
	int i, j, pre_val, next_val, nButton;
	int x_num = info->cap_info.y_node_num+1, y_num = info->cap_info.x_node_num;
	int diff_val = 0, max_vdiff = 0;

	set_default_result(info);

	for (i=0; i < x_num-2; i++) {
		for (j=0; j<y_num; j++) {
			next_val = info->pdnd_data[(i*y_num)+j];
			pre_val = info->pdnd_data[(i*y_num)+j+y_num];
			diff_val = (next_val > pre_val)?(next_val - pre_val):(pre_val - next_val);

			printk(" %4d ", diff_val);
			V_Diff[i*y_num+j] = diff_val;
			if ((i == 0) && (j == 0))
				max_vdiff = diff_val;
			else {
				if (max_vdiff < diff_val)
					max_vdiff = diff_val;
			}
		}
		printk("\n");
	}

	//V DIff  button
	if ( info->cap_info.button_num) {
		for (i = 0; i < 2; i++) {
			nButton = buttons[i];
			if (nButton < 0)
				continue;
			next_val = info->pdnd_data[(x_num-1)*y_num+nButton];
			pre_val = info->pdnd_data[(x_num-2)*y_num+nButton];
			diff_val = (next_val > pre_val)?(next_val - pre_val):(pre_val - next_val);
			V_Diff[(x_num-1)*y_num + nButton] = diff_val;
			if (max_vdiff < diff_val)
				max_vdiff = diff_val;
		}
	}

	sprintf(buff, "%d", diff_val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
}



static void get_reference_V_Diff(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[19*10] = {0};
	char tmp[10] = {0};
	int V_Diff[19*10] = {0};
	int y_node;
	int node_num;
	int buttons[2] = {4,7};
	int i, j, diff_val, pre_val, next_val, nButton;
	int x_num = info->cap_info.y_node_num+1, y_num = info->cap_info.x_node_num;

	set_default_result(info);

	for (i=0; i < x_num-2; i++) {
		for (j=0; j<y_num; j++) {
			next_val = info->pdnd_data[(i*y_num)+j];
			pre_val = info->pdnd_data[(i*y_num)+j+y_num];
			diff_val = (next_val > pre_val)?(next_val - pre_val):(pre_val - next_val);

			printk(" %4d ", diff_val);
			V_Diff[i*y_num+j] = diff_val;
		}
		printk("\n");
	}

	//V DIff  button
	if ( info->cap_info.button_num) {
		for (i = 0; i < 2; i++) {
			nButton = buttons[i];
			if (nButton < 0)
				continue;
			next_val = info->pdnd_data[(x_num-1)*y_num+nButton];
			pre_val = info->pdnd_data[(x_num-2)*y_num+nButton];
			diff_val = (next_val > pre_val)?(next_val - pre_val):(pre_val - next_val);
			V_Diff[(x_num-1)*y_num + nButton] = diff_val;
		}
	}

	y_node = info->cmd_param[0];

	if (y_node < 0 || y_node > y_num) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = FAIL;
		return;
	}

	for (i=0; i < x_num-2; i++) {
		node_num = i*y_num + y_node;
		sprintf(tmp, "%d", V_Diff[node_num]);
		strcat(buff, tmp);
		if (i < x_num-3)
			strcat(buff, " ");
	}

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
}

static void get_max_dnd(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};

	set_default_result(info);

	sprintf(buff, "%d", info->dnd_real_max_val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;
}

static void get_min_dnd(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};

	set_default_result(info);

	sprintf(buff, "%d", info->dnd_real_min_val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;
}

static void get_max_hfdnd(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};
	s16 val[3] = {0};

	set_default_result(info);

	val[0] =  info->hfdnd_max_x;
	val[1] =  info->hfdnd_max_y;
	val[2] =  info->hfdnd_max_val;

	memcpy(buff, (char*)val, sizeof(buff));
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;
}

static void get_min_hfdnd(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};
	s16 val[3] = {0};

	set_default_result(info);

	val[0] =  info->hfdnd_min_x;
	val[1] =  info->hfdnd_min_y;
	val[2] =  info->hfdnd_min_val;

	memcpy(buff, (char*)val, sizeof(buff));
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;
}

static void get_max_hdiff(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};
	s16 val[3] = {0};

	set_default_result(info);

	val[0] =  info->hdiff_max_x;
	val[1] =  info->hdiff_max_y;
	val[2] =  info->hdiff_max_val;

	sprintf(buff, "%d", info->hdiff_max_val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;
}

static void get_min_hdiff(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};
	s16 val[3] = {0};

	set_default_result(info);

	val[0] =  info->hdiff_min_x;
	val[1] =  info->hdiff_min_y;
	val[2] =  info->hdiff_min_val;

	sprintf(buff, "%d", info->hdiff_min_val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;
}

static void get_max_vdiff(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};
	s16 val[3] = {0};

	set_default_result(info);

	val[0] =  info->vdiff_max_x;
	val[1] =  info->vdiff_max_y;
	val[2] =  info->vdiff_max_val;

	memcpy(buff, (char*)val, sizeof(buff));
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;
}

static void get_min_vdiff(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};
	s16 val[3] = {0};

	set_default_result(info);

	val[0] =  info->vdiff_min_x;
	val[1] =  info->vdiff_min_y;
	val[2] =  info->vdiff_min_val;

	memcpy(buff, (char*)val, sizeof(buff));
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;
}

static void run_normal_read(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;

	set_default_result(info);
	ts_set_touchmode(TOUCH_NORMAL_MODE);
	get_raw_data(info, (u8 *)info->normal_data, 5);
	ts_set_touchmode(TOUCH_POINT_MODE);
	info->cmd_state = OK;
}

static void run_delta_read(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;

	set_default_result(info);
	ts_set_touchmode(TOUCH_DELTA_MODE);
	get_raw_data(info, (u8 *)info->delta_data, 5);
	ts_set_touchmode(TOUCH_POINT_MODE);
	info->cmd_state = OK;
}

#if 0
static void get_reference(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};
	unsigned int val;
	int x_node, y_node;
	int node_num;
	int x_num = info->cap_info.y_node_num+1, y_num = info->cap_info.x_node_num;

	set_default_result(info);

	x_node = info->cmd_param[0];
	y_node = info->cmd_param[1];

	if (x_node < 0 || x_node > x_num ||
		y_node < 0 || y_node > y_num) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = FAIL;
		return;
	}

	node_num = x_node*y_num + y_node;

	val = info->dnd_data[node_num];
	snprintf(buff, sizeof(buff), "%u", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
}
#endif

static void get_reference_RxDND(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	int y_node, i;
	int node_num;
	int x_num = info->cap_info.y_node_num+1, y_num = info->cap_info.x_node_num;
	s16 val[20] = {0};
	char buff[40] = {0};

	set_default_result(info);

	y_node = info->cmd_param[0];

	if (y_node < 0 || y_node > y_num) {
		snprintf(buff, sizeof(buff), "%s", "abnormal");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = FAIL;
		return;
	}

	for (i=0; i < x_num; i++) {
		node_num = i*y_num + y_node;
		val[i] = info->pdnd_data[node_num];
	}

	memcpy(buff, (char*)val, sizeof(buff));
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
}

static void get_reference_DND(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[19*7] = {0};
	char tmp[16] = {0};
	int y_node, i;
	int node_num;
	int x_num = info->cap_info.y_node_num+1, y_num = info->cap_info.x_node_num;

	set_default_result(info);

	y_node = info->cmd_param[0];

	if (y_node < 0 || y_node > y_num) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = FAIL;
		return;
	}

	run_reference_DND_read(device_data, 4, 7);

	for (i=0; i < x_num-1; i++) {
		node_num = i*y_num + y_node;
		sprintf(tmp, "%d",  info->pdnd_data[node_num]);
		strcat(buff, tmp);
		if (i < x_num-2)
			strcat(buff, " ");
	}
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
				buff, strnlen(buff, sizeof(buff)));

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;
	return;


}

static void get_reference_RxHFDND(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[40] = {0};
	s16 val[20] = {0};
	int y_node, i;
	int node_num;
	int x_num = info->cap_info.y_node_num+1, y_num = info->cap_info.x_node_num;

	set_default_result(info);

	y_node = info->cmd_param[0];

	if (y_node < 0 || y_node > y_num) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = FAIL;
		return;
	}

	for (i=0; i < x_num; i++) {
		node_num = i*y_num + y_node;
		val[i] = info->hfdnd_data[node_num];
	}

	memcpy(buff, (char*)val, sizeof(buff));
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
}

static void get_reference_HFDND(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[512] = {0};
	char tmp[16] = {0};
	int y_node;
	int node_num;
	int x_num = info->cap_info.y_node_num+1, y_num = info->cap_info.x_node_num;
	int i;

	set_default_result(info);

	y_node = info->cmd_param[0];

	if (y_node < 0 || y_node > y_num) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = FAIL;
		return;
	}

	run_reference_HFDND_read(device_data, 4, 7);

	for (i=0; i < x_num-1; i++) {
		node_num = i*y_num + y_node;
		sprintf(tmp, "%d",  info->hfdnd_data[node_num]);
		strcat(buff, tmp);
		if (i < x_num-2)
			strcat(buff, " ");
	}
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
				buff, strnlen(buff, sizeof(buff)));

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;
	return;
}

static void get_normal(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};
	unsigned int val;
	int x_node, y_node;
	int node_num;
	int x_num = info->cap_info.y_node_num+1, y_num = info->cap_info.x_node_num;

	set_default_result(info);

	x_node = info->cmd_param[0];
	y_node = info->cmd_param[1];

	if (x_node < 0 || x_node > x_num||
		y_node < 0 || y_node > y_num) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = FAIL;
		return;
	}

	node_num = x_node*y_num + y_node;

	val = info->normal_data[node_num];
	snprintf(buff, sizeof(buff), "%u", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
}

static void get_delta(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};
	unsigned int val;
	int x_node, y_node;
	int node_num;
	int x_num = info->cap_info.y_node_num+1, y_num = info->cap_info.x_node_num;

	set_default_result(info);

	x_node = info->cmd_param[0];
	y_node = info->cmd_param[1];

	if (x_node < 0 || x_node > x_num ||
		y_node < 0 || y_node > y_num) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = FAIL;
		return;
	}

	node_num = x_node*y_num+ y_node;

	val = info->delta_data[node_num];
	snprintf(buff, sizeof(buff), "%u", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
}

static void get_tkey_delta(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	char buff[16] = {0};
	u16 val;
	int btn_node;
	int ret;

	set_default_result(info);

	btn_node = info->cmd_param[0];

	if (btn_node < 0 || btn_node > MAX_SUPPORTED_BUTTON_NUM)
		goto err_out;

	disable_irq(misc_touch_dev->irq);
	down(&misc_touch_dev->work_proceedure_lock);
	if (misc_touch_dev->work_proceedure != TS_NO_WORK) {
		printk(KERN_INFO "other process occupied.. (%d)\n",
			misc_touch_dev->work_proceedure);
		enable_irq(misc_touch_dev->irq);
		up(&misc_touch_dev->work_proceedure_lock);
		goto err_out;
	}
	misc_touch_dev->work_proceedure = TS_SET_MODE;

	ret = ts_read_data(misc_touch_dev->client, ZINITIX_BTN_WIDTH + btn_node, (u8*)&val, 2);

	if (ret < 0) {
		printk(KERN_INFO "read error..\n");
		enable_irq(misc_touch_dev->irq);
		misc_touch_dev->work_proceedure = TS_NO_WORK;
		up(&misc_touch_dev->work_proceedure_lock);
		goto err_out;
	}
	misc_touch_dev->work_proceedure = TS_NO_WORK;
	enable_irq(misc_touch_dev->irq);
	up(&misc_touch_dev->work_proceedure_lock);

	snprintf(buff, sizeof(buff), "%u", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = OK;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
	return;

err_out:
	snprintf(buff, sizeof(buff), "%s", "NG");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = FAIL;
}

static void fw_update(void *device_data)
{
	struct bt432_ts_data *info = (struct bt432_ts_data *)device_data;
	struct i2c_client *client = info->client;
	int ret = 0;
	const u8 *buff = 0;
	mm_segment_t old_fs = {0};
	struct file *fp = NULL;
	long fsize = 0, nread = 0;
	char fw_path[MAX_FW_PATH+1];
	char result[16] = {0};

	set_default_result(info);

	switch (info->cmd_param[0]) {
	case BUILT_IN:
		ret = ts_upgrade_sequence((u8*)m_firmware_data);
		if (ret<0) {
			info->cmd_state = FAIL;
			return;
		}
		break;

	case UMS:
		old_fs = get_fs();
		set_fs(get_ds());

		snprintf(fw_path, MAX_FW_PATH, "%s", TSP_FW_FILEPATH);
		fp = filp_open(fw_path, O_RDONLY, 0);
		if (IS_ERR(fp)) {
			dev_err(&client->dev,
				"file %s open error:%d\n", fw_path, (s32)fp);
			info->cmd_state = FAIL;
			goto err_open;
		}

		fsize = fp->f_path.dentry->d_inode->i_size;

		if (fsize != info->cap_info.ic_fw_size) {
			dev_err(&client->dev, "invalid fw size!!\n");
			info->cmd_state = FAIL;
			goto err_open;
		}

		buff = kzalloc((size_t)fsize, GFP_KERNEL);
		if (!buff) {
			dev_err(&client->dev, "fail to alloc buffer for fw\n");
			info->cmd_state = FAIL;
			goto err_alloc;
		}

		nread = vfs_read(fp, (char __user *)buff, fsize, &fp->f_pos);
		if (nread != fsize) {
			info->cmd_state = FAIL;
			goto err_fw_size;
		}

		filp_close(fp, current->files);
		set_fs(old_fs);
		dev_info(&client->dev, "ums fw is loaded!!\n");

		ret = ts_upgrade_sequence((u8*)buff);
		if (ret<0) {
			kfree(buff);
			goto not_support;
		}
		break;

	default:
		dev_err(&client->dev, "invalid fw file type!!\n");
		goto not_support;
	}

	get_chip_fw_ver(info);

	info->cmd_state = OK;
	snprintf(result, sizeof(result) , "%s", "OK");
	set_cmd_result(info, result,
				strnlen(result, sizeof(result)));

	if (fp != NULL) {
		kfree(buff);
		filp_close(fp, NULL);
		set_fs(old_fs);
	}

	return;

if (fp != NULL) {
err_fw_size:
	kfree(buff);
err_alloc:
	filp_close(fp, NULL);
err_open:
	set_fs(old_fs);
}
not_support:
	info->cmd_state = FAIL;
	snprintf(result, sizeof(result) , "%s", "NG");
	set_cmd_result(info, result, strnlen(result, sizeof(result)));
	return;
}

static ssize_t store_cmd(struct device *dev, struct device_attribute
				  *devattr, const char *buf, size_t count)
{
	struct bt432_ts_data *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;

	char *cur, *start, *end;
	char buff[TSP_CMD_STR_LEN] = {0};
	int len, i;
	struct tsp_cmd *tsp_cmd_ptr = NULL;
	char delim = ',';
	bool cmd_found = false;
	int param_cnt = 0;
	int ret;

	if (info->cmd_is_running == true) {
		dev_err(&info->client->dev, "tsp_cmd: other cmd is running.\n");
		goto err_out;
	}

	/* check lock  */
	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = true;
	mutex_unlock(&info->cmd_lock);

	info->cmd_state = WAITING;

	for (i = 0; i < ARRAY_SIZE(info->cmd_param); i++)
		info->cmd_param[i] = 0;

	len = (int)count;
	if (*(buf + len - 1) == '\n')
		len--;
	memset(info->cmd, 0x00, ARRAY_SIZE(info->cmd));
	memcpy(info->cmd, buf, len);

	cur = strchr(buf, (int)delim);
	if (cur)
		memcpy(buff, buf, cur - buf);
	else
		memcpy(buff, buf, len);

	/* find command */
	list_for_each_entry(tsp_cmd_ptr, &info->cmd_list_head, list) {
		if (!strcmp(buff, tsp_cmd_ptr->cmd_name)) {
			cmd_found = true;
			break;
		}
	}

	/* set not_support_cmd */
	if (!cmd_found) {
		list_for_each_entry(tsp_cmd_ptr, &info->cmd_list_head, list) {
			if (!strcmp("not_support_cmd", tsp_cmd_ptr->cmd_name))
				break;
		}
	}

	/* parsing parameters */
	if (cur && cmd_found) {
		cur++;
		start = cur;
		memset(buff, 0x00, ARRAY_SIZE(buff));
		do {
			if (*cur == delim || cur - buf == len) {
				end = cur;
				memcpy(buff, start, end - start);
				*(buff + strlen(buff)) = '\0';
				ret = kstrtoint(buff, 10,\
						info->cmd_param + param_cnt);
				start = cur + 1;
				memset(buff, 0x00, ARRAY_SIZE(buff));
				param_cnt++;
			}
			cur++;
		} while (cur - buf <= len);
	}

	dev_info(&client->dev, "cmd = %s\n", tsp_cmd_ptr->cmd_name);
	for (i = 0; i < param_cnt; i++)
		dev_info(&client->dev, "cmd param %d= %d\n", i,
							info->cmd_param[i]);

	tsp_cmd_ptr->cmd_func(info);

err_out:
	return count;
}

static ssize_t show_cmd_status(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct bt432_ts_data *info = dev_get_drvdata(dev);
	char buff[16] = {0};

	dev_info(&info->client->dev, "tsp cmd: status:%d\n",
			info->cmd_state);

	if (info->cmd_state == WAITING)
		snprintf(buff, sizeof(buff), "WAITING");

	else if (info->cmd_state == RUNNING)
		snprintf(buff, sizeof(buff), "RUNNING");

	else if (info->cmd_state == OK)
		snprintf(buff, sizeof(buff), "OK");

	else if (info->cmd_state == FAIL)
		snprintf(buff, sizeof(buff), "FAIL");

	else if (info->cmd_state == NOT_APPLICABLE)
		snprintf(buff, sizeof(buff), "NOT_APPLICABLE");

	return snprintf(buf, TSP_BUF_SIZE, "%s\n", buff);
}

static ssize_t show_cmd_result(struct device *dev, struct device_attribute
				    *devattr, char *buf)
{
	struct bt432_ts_data *info = dev_get_drvdata(dev);

	dev_info(&info->client->dev, "tsp cmd: result: %s\n", info->cmd_result);

	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = false;
	mutex_unlock(&info->cmd_lock);

	info->cmd_state = WAITING;

	return snprintf(buf, TSP_BUF_SIZE, "%s\n", info->cmd_result);
}

static DEVICE_ATTR(cmd, S_IWUSR | S_IWGRP, NULL, store_cmd);
static DEVICE_ATTR(cmd_status, S_IRUGO, show_cmd_status, NULL);
static DEVICE_ATTR(cmd_result, S_IRUGO, show_cmd_result, NULL);

static struct attribute *sec_touch_attributes[] = {
	&dev_attr_cmd.attr,
	&dev_attr_cmd_status.attr,
	&dev_attr_cmd_result.attr,
	NULL,
};

static struct attribute_group sec_touch_attributes_group = {
	.attrs = sec_touch_attributes,
};

static ssize_t firm_version_panel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bt432_ts_data *info = dev_get_drvdata(dev);
	u16 newVersion, newMinorVersion, newRegVersion, newHWID;
	u32 version;

	newVersion = info->cap_info.firmware_version;
	newMinorVersion = info->cap_info.firmware_minor_version;
	newRegVersion = info->cap_info.reg_data_version;
	newHWID = info->cap_info.hw_id;
	version = (u32)((u32)(newHWID&0xff)<<16)|((newVersion&0xf)<<12)|((newMinorVersion&0xf)<<8)|(newRegVersion&0xff);

	dev_info(&info->client->dev, "%s: version: 0x%06x.\n", __func__, version);

	return snprintf(buf, PAGE_SIZE, "0x%06x\n", version);
}

static ssize_t firm_version_phone_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bt432_ts_data *info = dev_get_drvdata(dev);
	u16 newVersion, newMinorVersion, newRegVersion, newHWID;
	u32 version;
	u8 *firmware_data;

	firmware_data = (u8*)m_firmware_data;

	newVersion = (u16) (firmware_data[52] | (firmware_data[53]<<8));
	newMinorVersion = (u16) (firmware_data[56] | (firmware_data[57]<<8));
	newRegVersion = (u16) (firmware_data[60] | (firmware_data[61]<<8));
	newHWID = (u16) (firmware_data[0x57d2] | (firmware_data[0x57d3]<<8));

	version = (u32)((u32)(newHWID&0xff)<<16)|((newVersion&0xf)<<12)|((newMinorVersion&0xf)<<8)|(newRegVersion&0xff);

	dev_info(&info->client->dev, "%s: version: 0x%06x.\n", __func__, version);

	return snprintf(buf, PAGE_SIZE, "0x%06x\n", version);
}

static ssize_t menu_sensitivity_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u16 val;
	int ret;

	disable_irq(misc_touch_dev->irq);
	down(&misc_touch_dev->work_proceedure_lock);
	if (misc_touch_dev->work_proceedure != TS_NO_WORK) {
		printk(KERN_INFO "other process occupied.. (%d)\n",
			misc_touch_dev->work_proceedure);
		enable_irq(misc_touch_dev->irq);
		up(&misc_touch_dev->work_proceedure_lock);
		goto err_out;
	}
	misc_touch_dev->work_proceedure = TS_SET_MODE;

	ret = ts_read_data(misc_touch_dev->client, ZINITIX_BTN_WIDTH + 0, (u8*)&val, 2);

	if (ret < 0) {
		printk(KERN_INFO "read error..\n");
		enable_irq(misc_touch_dev->irq);
		misc_touch_dev->work_proceedure = TS_NO_WORK;
		up(&misc_touch_dev->work_proceedure_lock);
		goto err_out;
	}
	misc_touch_dev->work_proceedure = TS_NO_WORK;
	enable_irq(misc_touch_dev->irq);
	up(&misc_touch_dev->work_proceedure_lock);

	sprintf(buf, "%d\n", val);

	return snprintf(buf, PAGE_SIZE, "%d\n", val);

err_out:
	sprintf(buf, "%s", "NG");
	return 0;
}

static ssize_t back_sensitivity_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u16 val;
	int ret;

	disable_irq(misc_touch_dev->irq);
	down(&misc_touch_dev->work_proceedure_lock);
	if (misc_touch_dev->work_proceedure != TS_NO_WORK) {
		printk(KERN_INFO "other process occupied.. (%d)\n",
			misc_touch_dev->work_proceedure);
		enable_irq(misc_touch_dev->irq);
		up(&misc_touch_dev->work_proceedure_lock);
		goto err_out;
	} else
		printk(KERN_INFO "work_proceedure is no_work.(%d)\n",
			misc_touch_dev->work_proceedure);

	misc_touch_dev->work_proceedure = TS_SET_MODE;

	ret = ts_read_data(misc_touch_dev->client, ZINITIX_BTN_WIDTH + 1, (u8*)&val, 2);
	if (ret < 0) {
		printk(KERN_INFO "read error..\n");
		enable_irq(misc_touch_dev->irq);
		misc_touch_dev->work_proceedure = TS_NO_WORK;
		up(&misc_touch_dev->work_proceedure_lock);
		goto err_out;
	}

	misc_touch_dev->work_proceedure = TS_NO_WORK;
	enable_irq(misc_touch_dev->irq);
	up(&misc_touch_dev->work_proceedure_lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", val);

err_out:
	sprintf(buf, "%s", "NG");
	return 0;
}

static ssize_t touchkey_threshold_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	int ret = 0;
	u16 threshold;
	ret = ts_read_data(misc_touch_dev->client, ZINITIX_BUTTON_SENSITIVITY, (u8*)&threshold, 2);
	if (ret < 0) {
		sprintf(buf, "%s", "fail");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", threshold);
}
#endif

static int ts_misc_fops_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int ts_misc_fops_close(struct inode *inode, struct file *filp)
{
	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
static int ts_misc_fops_ioctl(struct inode *inode,
	struct file *filp, unsigned int cmd,
	unsigned long arg)
#else
static long ts_misc_fops_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg)
#endif
{
	void __user *argp = (void __user *)arg;
	struct _raw_ioctl raw_ioctl;
	u8 *u8Data;
	int ret = 0;
	size_t sz = 0;
	u16 version;
	u16 mode;

	struct _reg_ioctl reg_ioctl;
	u16 val;
	int nval = 0;

	if (misc_touch_dev == NULL)
		return -1;

	/* zinitix_debug_msg("cmd = %d, argp = 0x%x\n", cmd, (int)argp); */

	switch (cmd) {
	case TOUCH_IOCTL_GET_DEBUGMSG_STATE:
		ret = misc_touch_dev->debug_mode;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_SET_DEBUGMSG_STATE:
		if (copy_from_user(&nval, argp, 4)) {
			printk(KERN_INFO "[zinitix_touch] error : copy_from_user\n");
			return -1;
		}
		if (nval)
			printk(KERN_INFO "[zinitix_touch] on debug mode (%d)\n",
				nval);
		else
			printk(KERN_INFO "[zinitix_touch] off debug mode (%d)\n",
				nval);
		misc_touch_dev->debug_mode = nval;
		break;

	case TOUCH_IOCTL_GET_CHIP_REVISION:
		ret = misc_touch_dev->cap_info.ic_revision;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_FW_VERSION:
		ret = misc_touch_dev->cap_info.firmware_version;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_REG_DATA_VERSION:
		ret = misc_touch_dev->cap_info.reg_data_version;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_VARIFY_UPGRADE_SIZE:
		if (copy_from_user(&sz, argp, sizeof(size_t)))
			return -1;

		printk(KERN_INFO "firmware size = %d\n", sz);
		if (misc_touch_dev->cap_info.ic_fw_size != sz) {
			printk(KERN_INFO "firmware size error\n");
			return -1;
		}
		break;

	case TOUCH_IOCTL_VARIFY_UPGRADE_DATA:
		if (copy_from_user((void*)m_firmware_data,
			argp, misc_touch_dev->cap_info.ic_fw_size))
			return -1;

		version = (u16) (m_firmware_data[52] | (m_firmware_data[53]<<8));

		printk(KERN_INFO "firmware version = %x\n", version);

		if (copy_to_user(argp, &version, sizeof(version)))
			return -1;
		break;

	case TOUCH_IOCTL_START_UPGRADE:
		return ts_upgrade_sequence((u8*)&m_firmware_data[0]);

	case TOUCH_IOCTL_GET_X_RESOLUTION:
		ret = misc_touch_dev->cap_info.x_resolution;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_Y_RESOLUTION:
		ret = misc_touch_dev->cap_info.y_resolution;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_X_NODE_NUM:
		ret = misc_touch_dev->cap_info.x_node_num;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_Y_NODE_NUM:
		ret = misc_touch_dev->cap_info.y_node_num;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_TOTAL_NODE_NUM:
		ret = misc_touch_dev->cap_info.total_node_num;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_HW_CALIBRAION:
		ret = -1;
		disable_irq(misc_touch_dev->irq);
		down(&misc_touch_dev->work_proceedure_lock);
		if (misc_touch_dev->work_proceedure != TS_NO_WORK) {
			printk(KERN_INFO"other process occupied.. (%d)\n",
				misc_touch_dev->work_proceedure);
			up(&misc_touch_dev->work_proceedure_lock);
			return -1;
		}
		misc_touch_dev->work_proceedure = TS_HW_CALIBRAION;
		mdelay(100);

		/* h/w calibration */
		if (ts_hw_calibration(misc_touch_dev) == true)
			ret = 0;

		mode = misc_touch_dev->touch_mode;
		if (ts_write_reg(misc_touch_dev->client,
			ZINITIX_TOUCH_MODE, mode) != I2C_SUCCESS) {
			printk(KERN_INFO "fail to set touch mode %d.\n",
				mode);
			goto fail_hw_cal;
		}

		if (ts_write_cmd(misc_touch_dev->client,
			ZINITIX_SWRESET_CMD) != I2C_SUCCESS)
			goto fail_hw_cal;

		enable_irq(misc_touch_dev->irq);
		misc_touch_dev->work_proceedure = TS_NO_WORK;
		up(&misc_touch_dev->work_proceedure_lock);
		return ret;
fail_hw_cal:
		enable_irq(misc_touch_dev->irq);
		misc_touch_dev->work_proceedure = TS_NO_WORK;
		up(&misc_touch_dev->work_proceedure_lock);
		return -1;

	case TOUCH_IOCTL_SET_RAW_DATA_MODE:
		if (misc_touch_dev == NULL) {
			zinitix_debug_msg("misc device NULL?\n");
			return -1;
		}
		if (copy_from_user(&nval, argp, 4)) {
			printk(KERN_INFO "[zinitix_touch] error : copy_from_user\n");
			misc_touch_dev->work_proceedure = TS_NO_WORK;
			return -1;
		}
		ts_set_touchmode((u16)nval);

		return 0;

	case TOUCH_IOCTL_GET_REG:
		if (misc_touch_dev == NULL) {
			zinitix_debug_msg("misc device NULL?\n");
			return -1;
		}
		down(&misc_touch_dev->work_proceedure_lock);
		if (misc_touch_dev->work_proceedure != TS_NO_WORK) {
			printk(KERN_INFO "other process occupied.. (%d)\n",
				misc_touch_dev->work_proceedure);
			up(&misc_touch_dev->work_proceedure_lock);
			return -1;
		}

		misc_touch_dev->work_proceedure = TS_SET_MODE;

		if (copy_from_user(&reg_ioctl,
			argp, sizeof(struct _reg_ioctl))) {
			misc_touch_dev->work_proceedure = TS_NO_WORK;
			up(&misc_touch_dev->work_proceedure_lock);
			printk(KERN_INFO "[zinitix_touch] error : copy_from_user\n");
			return -1;
		}

		if (ts_read_data(misc_touch_dev->client,
			reg_ioctl.addr, (u8 *)&val, 2) < 0)
			ret = -1;

		nval = (int)val;

		if (copy_to_user(reg_ioctl.val, (u8 *)&nval, 4)) {
			misc_touch_dev->work_proceedure = TS_NO_WORK;
			up(&misc_touch_dev->work_proceedure_lock);
			printk(KERN_INFO "[zinitix_touch] error : copy_to_user\n");
			return -1;
		}

		zinitix_debug_msg("read : reg addr = 0x%x, val = 0x%x\n",
			reg_ioctl.addr, nval);

		misc_touch_dev->work_proceedure = TS_NO_WORK;
		up(&misc_touch_dev->work_proceedure_lock);
		return ret;

	case TOUCH_IOCTL_SET_REG:

		if (misc_touch_dev == NULL) {
			zinitix_debug_msg("misc device NULL?\n");
			return -1;
		}
		down(&misc_touch_dev->work_proceedure_lock);
		if (misc_touch_dev->work_proceedure != TS_NO_WORK) {
			printk(KERN_INFO "other process occupied.. (%d)\n",
				misc_touch_dev->work_proceedure);
			up(&misc_touch_dev->work_proceedure_lock);
			return -1;
		}

		misc_touch_dev->work_proceedure = TS_SET_MODE;
		if (copy_from_user(&reg_ioctl,
				argp, sizeof(struct _reg_ioctl))) {
			misc_touch_dev->work_proceedure = TS_NO_WORK;
			up(&misc_touch_dev->work_proceedure_lock);
			printk(KERN_INFO "[zinitix_touch] error : copy_from_user\n");
			return -1;
		}

		if (copy_from_user(&val, reg_ioctl.val, 4)) {
			misc_touch_dev->work_proceedure = TS_NO_WORK;
			up(&misc_touch_dev->work_proceedure_lock);
			printk(KERN_INFO "[zinitix_touch] error : copy_from_user\n");
			return -1;
		}

		if (ts_write_reg(misc_touch_dev->client,
			reg_ioctl.addr, val) != I2C_SUCCESS)
			ret = -1;

		zinitix_debug_msg("write : reg addr = 0x%x, val = 0x%x\n",
			reg_ioctl.addr, val);
		misc_touch_dev->work_proceedure = TS_NO_WORK;
		up(&misc_touch_dev->work_proceedure_lock);
		return ret;

	case TOUCH_IOCTL_DONOT_TOUCH_EVENT:

		if (misc_touch_dev == NULL) {
			zinitix_debug_msg("misc device NULL?\n");
			return -1;
		}
		down(&misc_touch_dev->work_proceedure_lock);
		if (misc_touch_dev->work_proceedure != TS_NO_WORK) {
			printk(KERN_INFO"other process occupied.. (%d)\n",
				misc_touch_dev->work_proceedure);
			up(&misc_touch_dev->work_proceedure_lock);
			return -1;
		}

		misc_touch_dev->work_proceedure = TS_SET_MODE;
		if (ts_write_reg(misc_touch_dev->client,
			ZINITIX_INT_ENABLE_FLAG, 0) != I2C_SUCCESS)
			ret = -1;
		zinitix_debug_msg("write : reg addr = 0x%x, val = 0x0\n",
			ZINITIX_INT_ENABLE_FLAG);

		misc_touch_dev->work_proceedure = TS_NO_WORK;
		up(&misc_touch_dev->work_proceedure_lock);
		return ret;

	case TOUCH_IOCTL_SEND_SAVE_STATUS:
		if (misc_touch_dev == NULL) {
			zinitix_debug_msg("misc device NULL?\n");
			return -1;
		}
		down(&misc_touch_dev->work_proceedure_lock);
		if (misc_touch_dev->work_proceedure != TS_NO_WORK) {
			printk(KERN_INFO"other process occupied.. (%d)\n",
				misc_touch_dev->work_proceedure);
			up(&misc_touch_dev->work_proceedure_lock);
			return -1;
		}
		misc_touch_dev->work_proceedure = TS_SET_MODE;
		ret = 0;
		ts_write_reg(misc_touch_dev->client, 0xc003, 0x0001);
		ts_write_reg(misc_touch_dev->client, 0xc104, 0x0001);
		if (ts_write_cmd(misc_touch_dev->client,
			ZINITIX_SAVE_STATUS_CMD) != I2C_SUCCESS)
			ret =  -1;

		mdelay(1000);	/* for fusing eeprom */
		ts_write_reg(misc_touch_dev->client, 0xc003, 0x0000);
		ts_write_reg(misc_touch_dev->client, 0xc104, 0x0000);

		misc_touch_dev->work_proceedure = TS_NO_WORK;
		up(&misc_touch_dev->work_proceedure_lock);
		return ret;

	case TOUCH_IOCTL_GET_RAW_DATA:
		if (misc_touch_dev == NULL) {
			zinitix_debug_msg("misc device NULL?\n");
			return -1;
		}

		if (misc_touch_dev->touch_mode == TOUCH_POINT_MODE)
			return -1;

		down(&misc_touch_dev->raw_data_lock);
		if (misc_touch_dev->update == 0) {
			up(&misc_touch_dev->raw_data_lock);
			return -2;
		}

		if (copy_from_user(&raw_ioctl,
			argp, sizeof(raw_ioctl))) {
			up(&misc_touch_dev->raw_data_lock);
			printk(KERN_INFO "[zinitix_touch] error : copy_from_user\n");
			return -1;
		}

		misc_touch_dev->update = 0;

		u8Data = (u8 *)&misc_touch_dev->cur_data[0];

		if (copy_to_user(raw_ioctl.buf, (u8 *)u8Data,
			raw_ioctl.sz)) {
			up(&misc_touch_dev->raw_data_lock);
			return -1;
		}

		up(&misc_touch_dev->raw_data_lock);
		return 0;

	default:
		break;
	}
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int zinitix_touch_open(struct input_dev *dev)
{
	struct bt432_ts_data *touch_dev = input_get_drvdata(dev);

	dev_info(&touch_dev->client->dev, "%s.\n", __func__);
	zinitix_resume(&touch_dev->client->dev);

	return 0;
}

static void zinitix_touch_close(struct input_dev *dev)
{
	struct bt432_ts_data *touch_dev = input_get_drvdata(dev);

	dev_info(&touch_dev->client->dev, "%s.\n", __func__);
	zinitix_suspend(&touch_dev->client->dev);
}
#endif

#ifdef CONFIG_OF
static int zinitix_touch_get_keycodes(struct device *dev, char *name,
				struct zxt_ts_platform_data *pdata)
{
	struct property *prop;
	struct device_node *np = dev->of_node;
	int rc = 0;
	size_t keycodes_size;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	keycodes_size = prop->length / sizeof(u32);
	rc = of_property_read_u32_array(np, name, pdata->key_code, keycodes_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "%s: Unable to read %s\n", __func__, name);
		return rc;
	}

	return 0;
}

static int zinitix_touch_parse_dt(struct device *dev,
			struct zxt_ts_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	int rc;

	rc = zinitix_touch_get_keycodes(dev, "tkey-keycodes", pdata);
	if (rc)
		return rc;

	of_property_read_u32(np, "int_gpio", (u32 *)&pdata->gpio_int);
	of_property_read_u32(np, "x_resolution", (u32 *)&pdata->x_resolution);
	of_property_read_u32(np, "y_resolution", (u32 *)&pdata->y_resolution);
	of_property_read_u32(np, "debug_mode", (u32 *)&pdata->debug_mode);

	dev_info(dev, "%s: pdata->gpio_int = %d\n", __func__, pdata->gpio_int);
	dev_info(dev, "%s: pdata->x_resolution = %d\n", __func__, pdata->x_resolution);
	dev_info(dev, "%s: pdata->y_resolution = %d\n", __func__, pdata->y_resolution);
	dev_info(dev, "%s: pdata->debug_mode = %d\n", __func__, pdata->debug_mode);

	return 0;
}
#else
static int zinitix_touch_parse_dt(struct device *dev,
			struct cypress_touchkey_platform_data *pdata)
{
	return -ENODEV;
}
#endif

extern int is_lcd_attached(void);

static int zinitix_touch_probe(struct i2c_client *client,
		const struct i2c_device_id *i2c_id)
{
	struct zxt_ts_platform_data *pdata;
	struct bt432_ts_data *touch_dev;
	struct input_dev *input;
	struct input_dev *key;
	struct device *sec_touchscreen;
	struct device *sec_touchkey;

	int ret = 0;
	int i;

	dev_info(&client->dev, "%s\n", __func__);

	if (!is_lcd_attached()) {
		dev_err(&client->dev, "%s: front panel is not attached.\n", __func__);
		return 0;
	}

	ret = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if (!ret) {
		dev_err(&client->dev, "failed to check i2c functionality\n");
		ret = -EIO;
		goto err_check_functionality;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct zxt_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_info(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = zinitix_touch_parse_dt(&client->dev, pdata);
		if (ret)
			return ret;
	} else
		pdata = client->dev.platform_data;

	touch_dev = kzalloc(sizeof(struct bt432_ts_data), GFP_KERNEL);
	if (!touch_dev) {
		printk(KERN_ERR "unabled to allocate touch data\n");
		goto err_alloc_dev_data;
	}

	i2c_set_clientdata(client, touch_dev);
	touch_dev->client = client;
	touch_dev->pdata = pdata;

	input = input_allocate_device();
	if (input == NULL) {
		printk(KERN_ERR "unabled to allocate input device\n");
		goto err_input_allocate_device;
	}

	touch_dev->input_dev = input;
	touch_dev->int_gpio_num = pdata->gpio_int;
	touch_dev->debug_mode = pdata->debug_mode;

	dev_info(&client->dev, "%s: int_gpio_num[%d], debug_mode[%d]\n",
		__func__, touch_dev->int_gpio_num, touch_dev->debug_mode);

	if (bt432_power_control(touch_dev, POWER_ON_SEQUENCE)== false){
		zinitix_printk("power on sequence error : no ts device?\n");
		goto err_power_sequence;
	}

	memset(&touch_dev->reported_touch_info,
		0x0, sizeof(struct _ts_zinitix_point_info));

	touch_dev->touch_mode = TOUCH_POINT_MODE;
	misc_touch_dev = touch_dev;

	if (init_touch(touch_dev) == false) {
		goto err_input_register_device;
	}

	for (i = 0; i < MAX_SUPPORTED_BUTTON_NUM; i++)
		touch_dev->button[i] = ICON_BUTTON_UNCHANGE;

	touch_dev->use_esd_timer = false;

	INIT_WORK(&touch_dev->tmr_work, zinitix_touch_tmr_work);
	touch_dev->esd_wq =
		create_singlethread_workqueue("touch_dev->esd_wq");
	if (!touch_dev->esd_wq) {
		printk(KERN_ERR "unabled to create touch tmr work queue\n");
		goto err_kthread_create_failed;
	}

	if (ZINITIX_ESD_TIMER_INTERVAL) {
		touch_dev->use_esd_timer = 1;
		ts_esd_timer_init(touch_dev);
		ts_esd_timer_start(ZINITIX_CHECK_ESD_TIMER, touch_dev);
		printk(KERN_INFO " ts_esd_timer_start\n");
	}

	snprintf(touch_dev->phys, sizeof(touch_dev->phys),
		"%s/input0", dev_name(&client->dev));
	touch_dev->input_dev->name = "sec_touchscreen";
	touch_dev->input_dev->id.bustype = BUS_I2C;
	touch_dev->input_dev->phys = touch_dev->phys;
	touch_dev->input_dev->dev.parent = &client->dev;

	key = input_allocate_device();
	if (key == NULL) {
		printk(KERN_ERR "unabled to allocate input device\n");
		goto err_input_allocate_device;
	}

	touch_dev->key_dev = key;

	touch_dev->key_dev->name = "sec_touchkey";
	touch_dev->key_dev->id.bustype = BUS_I2C;
	touch_dev->key_dev->phys = touch_dev->phys;
	touch_dev->key_dev->dev.parent = &client->dev;
	touch_dev->touch_count = 0;

	set_bit(EV_KEY, touch_dev->key_dev->evbit);
	set_bit(EV_SYN, touch_dev->key_dev->evbit);
	for (i = 0; i < MAX_SUPPORTED_BUTTON_NUM; i++)
		set_bit(touch_dev->pdata->key_code[i], touch_dev->key_dev->keybit);

	set_bit(EV_ABS, touch_dev->input_dev->evbit);
	set_bit(EV_SYN, touch_dev->input_dev->evbit);
	set_bit(EV_KEY, touch_dev->input_dev->evbit);
	set_bit(BTN_TOUCH, touch_dev->input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, touch_dev->input_dev->propbit);

	input_mt_init_slots(touch_dev->input_dev, touch_dev->cap_info.multi_fingers, 0);


	if (touch_dev->cap_info.Orientation & TOUCH_XY_SWAP) {
		input_set_abs_params(touch_dev->input_dev, ABS_MT_POSITION_Y,
			touch_dev->cap_info.MinX,
			touch_dev->cap_info.MaxX + ABS_PT_OFFSET,
			0, 0);
		input_set_abs_params(touch_dev->input_dev, ABS_MT_POSITION_X,
			touch_dev->cap_info.MinY,
			touch_dev->cap_info.MaxY + ABS_PT_OFFSET,
			0, 0);
	} else {
		input_set_abs_params(touch_dev->input_dev, ABS_MT_POSITION_X,
			touch_dev->cap_info.MinX,
			touch_dev->cap_info.MaxX + ABS_PT_OFFSET,
			0, 0);
		input_set_abs_params(touch_dev->input_dev, ABS_MT_POSITION_Y,
			touch_dev->cap_info.MinY,
			touch_dev->cap_info.MaxY + ABS_PT_OFFSET,
			0, 0);
	}
	input_set_abs_params(touch_dev->input_dev, ABS_X, 0, touch_dev->cap_info.MaxX, 0, 0);
	input_set_abs_params(touch_dev->input_dev, ABS_Y, 0, touch_dev->cap_info.MaxY, 0, 0);
	input_set_abs_params(touch_dev->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(touch_dev->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

#if (TOUCH_POINT_MODE == 2)
	input_set_abs_params(touch_dev->input_dev, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(touch_dev->input_dev, ABS_MT_ANGLE, -90, 90, 0, 0);
	input_set_abs_params(touch_dev->input_dev, ABS_MT_PALM, 0, 1, 0, 0);
#endif
	input_set_abs_params(touch_dev->input_dev, ABS_MT_TRACKING_ID,
		0, touch_dev->cap_info.multi_fingers, 0, 0);

	zinitix_debug_msg("register %s input device\n",
		touch_dev->input_dev->name);
	input_set_drvdata(touch_dev->input_dev, touch_dev);
	ret = input_register_device(touch_dev->input_dev);
	if (ret) {
		printk(KERN_ERR "unable to register %s input device\n",
			touch_dev->input_dev->name);
		goto err_input_register_device;
	}

	input_set_drvdata(touch_dev->key_dev, touch_dev);
	ret = input_register_device(touch_dev->key_dev);
	if (ret) {
		printk(KERN_ERR "unable to register %s key device\n",
			touch_dev->key_dev->name);
		goto err_input_register_device;
	}

	/* configure touchscreen interrupt gpio */
	ret = gpio_request(touch_dev->int_gpio_num, "zinitix_irq_gpio");
	if (ret) {
		printk(KERN_ERR "unable to request gpio.(%s)\r\n",
			touch_dev->input_dev->name);
		goto err_request_gpio;
	}

	ret = gpio_direction_input(touch_dev->int_gpio_num);
	if (ret) {
		printk(KERN_ERR "unable to configure gpio.(%s)\r\n",
			touch_dev->input_dev->name);
		goto err_configure_gpio;
	}

	touch_dev->irq = gpio_to_irq(pdata->gpio_int);
	if (touch_dev->irq < 0)
		printk(KERN_INFO "error. gpio_to_irq(..) function is not \
			supported? you should define GPIO_TOUCH_IRQ.\n");
	zinitix_debug_msg("request irq (irq = %d, pin = %d)\n",
		touch_dev->irq, touch_dev->int_gpio_num);

	touch_dev->work_proceedure = TS_NO_WORK;
	sema_init(&touch_dev->work_proceedure_lock, 1);

	ret = request_threaded_irq(touch_dev->irq, NULL, zinitix_touch_work,
		IRQF_TRIGGER_LOW | IRQF_ONESHOT , ZINITIX_TS_NAME, touch_dev);

	if (ret) {
		printk(KERN_ERR "unable to register irq.(%s)\n",
			touch_dev->input_dev->name);
		goto err_configure_gpio;
	}

#ifdef CONFIG_PM_SLEEP
	touch_dev->input_dev->open = zinitix_touch_open;
	touch_dev->input_dev->close = zinitix_touch_close;
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	touch_dev->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	touch_dev->early_suspend.suspend = zinitix_early_suspend;
	touch_dev->early_suspend.resume = zinitix_late_resume;
	register_early_suspend(&touch_dev->early_suspend);
#endif

#ifdef ZINITIX_TA_COVER_REGISTER
	touch_dev->callbacks.inform_charger = zinitix_charger_status_cb;
	touch_dev->register_cb = zinitix_tsp_register_callback;
	if (touch_dev->register_cb)
		touch_dev->register_cb(&touch_dev->callbacks);
#endif

	sema_init(&touch_dev->raw_data_lock, 1);

	ret = misc_register(&touch_misc_device);
	if (ret)
		zinitix_debug_msg("Fail to register touch misc device.\n");

#if SEC_TSP_FACTORY_TEST
	INIT_LIST_HEAD(&touch_dev->cmd_list_head);
	for (i = 0; i < ARRAY_SIZE(tsp_cmds); i++)
		list_add_tail(&tsp_cmds[i].list, &touch_dev->cmd_list_head);

	mutex_init(&touch_dev->cmd_lock);
	touch_dev->cmd_is_running = false;

	//sys/class/misc/touch_misc_fops/....
	sec_touchscreen = device_create(sec_class, NULL, 0, touch_dev, "tsp");

	if (IS_ERR(sec_touchscreen))
		dev_err(&client->dev, "Failed to create device for the sysfs\n");

	ret = sysfs_create_group(&sec_touchscreen->kobj, &sec_touch_attributes_group);
	if (ret)
		dev_err(&client->dev, "Failed to create sysfs .\n");

	sec_touchkey = device_create(sec_class, NULL, 0, touch_dev, "sec_touchkey");

	if (device_create_file(sec_touchkey, &dev_attr_touchkey_menu) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_touchkey_menu.attr.name);
	if (device_create_file(sec_touchkey, &dev_attr_touchkey_back) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_touchkey_back.attr.name);
	if (device_create_file(sec_touchkey, &dev_attr_touchkey_threshold) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_touchkey_threshold.attr.name);
	if (device_create_file(sec_touchkey, &dev_attr_touchkey_firm_version_panel) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_touchkey_firm_version_panel.attr.name);
	if (device_create_file(sec_touchkey, &dev_attr_touchkey_firm_version_phone) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_touchkey_firm_version_phone.attr.name);
#endif

	disable_irq(touch_dev->irq);
	bt432_power_control(touch_dev, POWER_OFF);
	touch_dev->work_proceedure = TS_IN_SUSPEND;

	touch_dev->init_done = true;
	dev_info(&client->dev, "zinitix touch probe done.\n");

	return 0;

err_configure_gpio:
	gpio_free(touch_dev->int_gpio_num);
err_request_gpio:
	input_unregister_device(touch_dev->input_dev);
err_input_register_device:
	input_free_device(touch_dev->input_dev);
err_power_sequence:
err_kthread_create_failed:
err_input_allocate_device:
	kfree(touch_dev);
err_alloc_dev_data:
err_check_functionality:
	touch_dev = NULL;
	misc_touch_dev = NULL;
	dev_err(&client->dev, "zinitix touch probe failed.\n");
	return -ENODEV;
}

static int zinitix_touch_remove(struct i2c_client *client)
{
	struct bt432_ts_data *touch_dev = i2c_get_clientdata(client);
	if (touch_dev == NULL)	return 0;

	zinitix_debug_msg("zinitix_touch_remove+\n");
	down(&touch_dev->work_proceedure_lock);
	touch_dev->work_proceedure = TS_REMOVE_WORK;

	if (touch_dev->use_esd_timer != 0) {
		flush_work(&touch_dev->tmr_work);
		ts_write_reg(touch_dev->client,
			ZINITIX_PERIODICAL_INTERRUPT_INTERVAL, 0);
		ts_esd_timer_stop(touch_dev);
		zinitix_debug_msg(KERN_INFO " ts_esd_timer_stop\n");
		destroy_workqueue(touch_dev->esd_wq);
	}

	if (touch_dev->irq)
		free_irq(touch_dev->irq, touch_dev);

	misc_deregister(&touch_misc_device);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&touch_dev->early_suspend);
#endif

	if (gpio_is_valid(touch_dev->int_gpio_num) != 0)
		gpio_free(touch_dev->int_gpio_num);

	input_unregister_device(touch_dev->input_dev);
	input_free_device(touch_dev->input_dev);
	up(&touch_dev->work_proceedure_lock);
	kfree(touch_dev);

	return 0;
}

static struct i2c_device_id zinitix_idtable[] = {
	{ ZINITIX_TS_NAME, 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, zinitix_idtable);

#ifdef CONFIG_OF
static const struct of_device_id zinitix_ts_of_match[] = {
        { .compatible = "Zinitix,BT432", },
        { }
};
#endif

static struct i2c_driver zinitix_touch_driver = {
	.id_table	= zinitix_idtable,
	.probe	= zinitix_touch_probe,
	.remove	= zinitix_touch_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= ZINITIX_TS_NAME,
#ifdef CONFIG_OF
                .of_match_table = zinitix_ts_of_match,
#endif
	},
};

static int __init zinitix_touch_init(void)
{
	return i2c_add_driver(&zinitix_touch_driver);
}

static void __exit zinitix_touch_exit(void)
{
	i2c_del_driver(&zinitix_touch_driver);
}

module_init(zinitix_touch_init);
module_exit(zinitix_touch_exit);

MODULE_DESCRIPTION("zinitix touch-screen device driver using i2c interface");
MODULE_AUTHOR("sohnet <seonwoong.jang@zinitix.com>");
MODULE_LICENSE("GPL");
