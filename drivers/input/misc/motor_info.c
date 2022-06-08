#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/err.h>

#if defined CONFIG_MACH_KIRAN_2G || defined CONFIG_MACH_VIVALTO
#include "kiran_motor_info.h"
#endif

extern struct class *sec_class;
struct device *motor_info;
static struct platform_device motor_name_dev = {
	.name="motor_name",
	.id = -1,
};

static ssize_t show_motor_name(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	char *mot_name;
#if defined CONFIG_MACH_KIRAN_2G || defined CONFIG_MACH_VIVALTO
	mot_name = get_motor_name();
#else
	mot_name = "NULL";
#endif

	return sprintf(buf, "%s\n", mot_name);
}

static DEVICE_ATTR(motor_name, 0444, show_motor_name, NULL);

static struct device_attribute *motor_class_attr[] ={
	&dev_attr_motor_name,
	NULL,
};

static int create_motor_class(void)
{
	int ret, i;

	motor_info = device_create(sec_class, NULL, 0, NULL, "motor_info");
	if(IS_ERR(motor_info)){
		ret = PTR_ERR(motor_info);
		printk("Failed to create device(motor_info)\n");
		return ret;
	}

	for(i=0; i<ARRAY_SIZE(motor_class_attr); i++){
		if(motor_class_attr[i] == NULL)
			break;

		ret = device_create_file(motor_info, *motor_class_attr);
		if(ret<0){
			printk("%s : create file error\n", __func__);
			return ret;
		}
	}

	return 0;
}

static int __init motor_driver_init(void)
{
	return create_motor_class();
}

static void __exit motor_driver_exit(void)
{
	platform_device_unregister(&motor_name_dev);
	device_unregister(motor_info);
}

module_init(motor_driver_init);
module_exit(motor_driver_exit);

MODULE_AUTHOR("Sanghyeon Lee <sirano06.lee@samsung.com>");
MODULE_DESCRIPTION("motor info class driver");
MODULE_LICENSE("GPL");
