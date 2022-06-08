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
#include "sprdfb_panel.h"

#define MAX_NUM_LINES 50
#define LCM_TAG_SHIFT 24
#define LCM_TAG_MASK  ((1 << 24) -1)
#define LCM_SEND(len) ((1 << LCM_TAG_SHIFT)| len)
#define LCM_SLEEP(ms) ((2 << LCM_TAG_SHIFT)| ms)

#define LCM_TAG_SEND  (1<< 0)
#define LCM_TAG_SLEEP (1 << 1)

#define MAX_DATA 150
#define MAX_LCD_REGISTERS 50

#define SHORT_CMD_NOPARAM 3
#define SHORT_CMD_ONE_PARAM 4

#define LCD_TUN_DEBUG
#ifdef LCD_TUN_DEBUG
#define DEBUG_LCD_TUNE(x...)       pr_info("[LCD_TUNE] " x)
#else
#define DEBUG_LCD_TUNE(x...)
#endif

typedef struct panel_init_cmd {
	unsigned int tag;
	unsigned char data[MAX_DATA];
} init_cmd;

init_cmd lcd_init_cmd[MAX_LCD_REGISTERS];
extern struct lcm_init_code *g_init_data;
static int lcd_parse_text(char *src, int len)
{
	int i, j, k, l;
	int num_of_regs = 0;
	char *str_line[MAX_NUM_LINES], *sstart, *c, *str;
	unsigned int data;

	c = src;
	num_of_regs = 0;
	sstart = c;

	for (i = 0; i < len; i++, c++) {
		char a = *c;
		if (a == '\r' || a == '\n') {
			if (c > sstart) {
				str_line[num_of_regs] = sstart;
				num_of_regs++;
			}
			*c = '\0';
			sstart = c + 1;
		}
	}
	for (i = 0, k = 0; i < num_of_regs+2; i++, ++k) {
		if (!((num_of_regs-k) & 0xFE)) {
			DEBUG_LCD_TUNE("Add sleep of 120ms for sleep_out command");
			lcd_init_cmd[i].tag = LCM_SLEEP(120);
			if (k > num_of_regs)
				return num_of_regs;
			++i;
		}
		c = str_line[k];
		j = 2;
		while ((str = strsep((char **)&c, ","))) {
			if (strlen(str) == 0)
				break;
			sscanf(str, "%x", &data);
			lcd_init_cmd[i].data[j++] = data;
		}
		if (j == SHORT_CMD_NOPARAM) {
			lcd_init_cmd[i].tag = LCM_SEND(1);
			lcd_init_cmd[i].data[0] = lcd_init_cmd[i].data[2];
		} else if (j == SHORT_CMD_ONE_PARAM) {
			lcd_init_cmd[i].tag = LCM_SEND(2);
			lcd_init_cmd[i].data[0] = lcd_init_cmd[i].data[2];
			lcd_init_cmd[i].data[1] = lcd_init_cmd[i].data[3];
		} else {
			lcd_init_cmd[i].tag = LCM_SEND(j);
			lcd_init_cmd[i].data[0] = (j-2);
			lcd_init_cmd[i].data[1] = 0;
		}

		DEBUG_LCD_TUNE("[%d] ", i);
		for (l = 0; l < (lcd_init_cmd[i].tag & LCM_TAG_MASK); l++)
			DEBUG_LCD_TUNE("0x%X ", lcd_init_cmd[i].data[l]);
		DEBUG_LCD_TUNE("\n");
	}

	return num_of_regs;
}

static int lcd_load_tune_data(char *filename)
{
	char *dp;
	long l;
	int ret, num;
	loff_t pos;
	struct file *filp;
	mm_segment_t fs;

	DEBUG_LCD_TUNE("[%s] loading file : [%s]\n", __func__, filename);

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
		pr_err("[LCD_TUNE] : vfs_read() filed ret : %d\n", ret);
		kfree(dp);
		filp_close(filp, current->files);
		return -1;
	}

	filp_close(filp, current->files);

	set_fs(fs);
	num = lcd_parse_text(dp, l);
	if (!num) {
		pr_err("[LCD_TUNE]:Nothing to parse\n");
		kfree(dp);
		return -1;
	}

	kfree(dp);
	return num;
}

int lcd_store_panel_init_cmd()
{
	int ret = 0;
	ret = lcd_load_tune_data("/opt/usr/lcd_tune.txt");
	g_init_data = (struct lcm_init_code *)&lcd_init_cmd;

	return ret;
}
