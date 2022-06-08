#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/pm_qos.h>
#include <linux/sched.h>

#include <linux/trm.h>
#define define_one_root_rw(_name)		\
static struct global_attr _name =		\
__ATTR(_name, 0600, show_##_name, store_##_name)

struct pm_qos_lock_tag {
	int min_value;
	int max_value;
};
int min_cpu_freq, max_cpu_freq;

static struct pm_qos_lock_tag cpufreq_pm_qos_lock[NUMBER_OF_LOCK];
static struct pm_qos_request cpufreq_min_qos_array[NUMBER_OF_LOCK];
static struct pm_qos_request cpufreq_max_qos_array[NUMBER_OF_LOCK];
static struct pm_qos_request cpu_online_min_qos_array[NUMBER_OF_LOCK];
static struct pm_qos_request cpu_online_max_qos_array[NUMBER_OF_LOCK];

static int atoi(const char *str)
{
	int result = 0;
	int count = 0;
	if (str == NULL)
		return -1;
	while (str[count] && str[count] >= '0' && str[count] <= '9') {
		result = result * 10 + str[count] - '0';
		++count;
	}
	return result;
}

static int set_pmqos_data(struct pm_qos_request *any_qos_array, int pmqos_type, const char *buf)
{
	int lock_id = KERNEL_RESERVED00, lock_value = 0;
	char *p2 = NULL;

	p2 = strstr(buf, "ID");

	if (p2 == NULL)
		p2 = strstr(buf, "id");

	if (p2 != NULL)
		lock_id = atoi(p2+2);
	else
		lock_id = KERNEL_RESERVED00;

	if (lock_id >= NUMBER_OF_LOCK) {
		pr_err("%s lock_id=%d is wrong", __FUNCTION__ ,lock_id);
		return -EINVAL;
	}

	if (strstr(buf, "-1")!=NULL)
		lock_value = -1;
	else
		lock_value = atoi(buf);

	printk(KERN_DEBUG "%s %s/%d id=%d value=%d\n", __FUNCTION__
		, get_current()->comm ,get_current()->pid ,lock_id ,lock_value);

	if (lock_value == -1) {
		if (pm_qos_request_active(&any_qos_array[lock_id]))
			pm_qos_remove_request(&any_qos_array[lock_id]);

	} else {
		if (!pm_qos_request_active(&any_qos_array[lock_id])) {
			pm_qos_add_request(&any_qos_array[lock_id]
				, pmqos_type, lock_value);
		} else
			pm_qos_update_request(&any_qos_array[lock_id], lock_value);
	}

	return 0;
}


static ssize_t show_cpufreq_max(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	unsigned int ret = 0;
	max_cpu_freq = pm_qos_request(PM_QOS_CPU_FREQ_MAX);
	ret +=  snprintf(buf + ret, PAGE_SIZE - ret, "%d\n", max_cpu_freq);

	return ret;
}

static ssize_t store_cpufreq_max(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	int ret = 0;
	ret = set_pmqos_data(cpufreq_max_qos_array, PM_QOS_CPU_FREQ_MAX, buf);
	if (ret)
		return ret;
	else
		return count;
}
static ssize_t show_cpufreq_min(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	unsigned int ret = 0;
	min_cpu_freq = pm_qos_request(PM_QOS_CPU_FREQ_MIN);
	ret +=  snprintf(buf + ret, PAGE_SIZE - ret, "%d\n", min_cpu_freq);

	return ret;
}

static ssize_t store_cpufreq_min(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	int ret = 0;
	ret = set_pmqos_data(cpufreq_min_qos_array, PM_QOS_CPU_FREQ_MIN, buf);
	if (ret)
		return ret;
	else
		return count;
}

static ssize_t show_cpu_online_max(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	unsigned int ret = 0;
	ret =  sprintf(buf, "%d\n", pm_qos_request(PM_QOS_CPU_ONLINE_MAX));
	return ret;
}

static ssize_t store_cpu_online_max(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	set_pmqos_data(cpu_online_max_qos_array, PM_QOS_CPU_ONLINE_MAX, buf);
	if (num_online_cpus() > pm_qos_request(PM_QOS_CPU_ONLINE_MAX))
		cpu_down(1);

	return count;
}

static ssize_t show_cpu_online_min(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	unsigned int ret = 0;
	ret =  sprintf(buf, "%d\n", pm_qos_request(PM_QOS_CPU_ONLINE_MIN));
	return ret;
}

static ssize_t store_cpu_online_min(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	set_pmqos_data(cpu_online_min_qos_array, PM_QOS_CPU_ONLINE_MIN, buf);

	if (num_online_cpus() < pm_qos_request(PM_QOS_CPU_ONLINE_MIN)) {
		pr_info("%s cpu_up\n", __FUNCTION__);
		cpu_up(1);
	}
	return count;
}
define_one_root_rw(cpufreq_max);
define_one_root_rw(cpufreq_min);
define_one_root_rw(cpu_online_max);
define_one_root_rw(cpu_online_min);


static struct attribute *pmqos_attributes[] = {
	&cpufreq_min.attr,
	&cpufreq_max.attr,
	&cpu_online_min.attr,
	&cpu_online_max.attr,
	NULL
};

static struct attribute_group pmqos_attr_group = {
	.attrs = pmqos_attributes,
	.name = "pmqos",
};


static int __init cpufreq_pmqos_init(void)
{
	int rc;

	rc = sysfs_create_group(cpufreq_global_kobject,
				&pmqos_attr_group);
	if (rc) {
		pr_err("%s create sysfs error rc=%d\n", __FUNCTION__, rc);
		return rc;
	}

	return rc;
}

MODULE_AUTHOR("Yong-U Baek <yu.baek@samsung.com>");
MODULE_DESCRIPTION("cpufreq_pmqos");
MODULE_LICENSE("GPL");

late_initcall(cpufreq_pmqos_init);

