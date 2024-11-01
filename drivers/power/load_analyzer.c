/*
 *  drivers/power/system_load_analyzer.c
 *
 *  Copyright (C)  2011 Samsung Electronics co. ltd
 *    Yong-U Baek <yu.baek@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/reboot.h>
//#include <plat/cpu.h>
#include <linux/delay.h>
#include <linux/pm_qos.h>

#include <linux/vmalloc.h>
#include <linux/cpuidle.h>
#include <linux/load_analyzer.h>

#include <linux/devfreq.h>

#if defined(CONFIG_SLP_CHECK_BUS_LOAD) && defined(CONFIG_CLK_MON)
#define CONFIG_SLP_BUS_CLK_CHECK_LOAD 1
#include <linux/clk_mon.h>
#endif

#define CONFIG_CHECK_WORK_HISTORY	1
#define CONFIG_CHECK_LPA_CAUSE	1
#define CONFIG_DEFAULT_VAL		0

#include <linux/debugfs.h>
#include <linux/uaccess.h>
#define CPU_NUM	NR_CPUS

static int get_index(int cnt, int ring_size, int diff)
{
	int ret = 0, modified_diff;

	if ((diff > ring_size) || (diff * (-1) > ring_size))
		modified_diff = diff % ring_size;
	else
		modified_diff = diff;

	ret = (ring_size + cnt + modified_diff) % ring_size;

	return ret;
}

static int wrapper_for_debug_fs(char __user *buffer, \
						size_t count, loff_t *ppos, int (*fn)(char *, int))
{

	static char *buf = NULL;
	int buf_size = (PAGE_SIZE * 256);
	unsigned int ret = 0, size_for_copy = count;
	static unsigned int rest_size = 0;

	if (*ppos < 0 || !count)
		return -EINVAL;

	if (*ppos == 0) {
		buf = vmalloc(buf_size);

		if (!buf)
			return -ENOMEM;

		ret = fn(buf, buf_size - PAGE_SIZE); /* PAGE_SIZE mean margin */

		if (ret <= count) {
			size_for_copy = ret;
			rest_size = 0;
		} else {
			size_for_copy = count;
			rest_size = ret -size_for_copy;
		}
	} else {
		if (rest_size <= count) {
			size_for_copy = rest_size;
			rest_size = 0;
		} else {
			size_for_copy = count;
			rest_size -= size_for_copy;
		}
	}

	if (size_for_copy >  0) {
		int offset = (int) *ppos;
		if (copy_to_user(buffer, buf + offset , size_for_copy)) {
			vfree(buf);
			return -EFAULT;
		}
		*ppos += size_for_copy;
	} else
		vfree(buf);

	return size_for_copy;
}

unsigned int cpu_load_history_num = 160;

struct cpu_load_freq_history_tag {
	char time[16];
	unsigned long long time_stamp;
	unsigned int cpufreq[CPU_NUM];
	int cpu_max_locked_freq;
	int cpu_min_locked_freq;
	int cpu_max_locked_online;
	int cpu_min_locked_online;

	unsigned int cpu_load[CPU_NUM];
	unsigned int cpu_idle_time[CPU_NUM][3];

	unsigned int touch_event;
	unsigned int nr_onlinecpu;
	unsigned int nr_run_avg;
	unsigned int task_history_cnt[CPU_NUM];
	unsigned int gpu_freq;
	unsigned int gpu_utilization;

	char status;

#if defined (CONFIG_CHECK_WORK_HISTORY)
	unsigned int work_history_cnt[CPU_NUM];
#endif

#if defined(CONFIG_SLP_CHECK_BUS_LOAD)
	unsigned int mif_bus_freq;
	unsigned int mif_bus_load;
#endif
#if defined(CONFIG_SLP_BUS_CLK_CHECK_LOAD)
	unsigned int power_domains[PWR_DOMAINS_NUM];
	unsigned int clk_gates[CLK_GATES_NUM];
#endif

};

static struct cpu_load_freq_history_tag *cpu_load_freq_history;
static struct cpu_load_freq_history_tag *cpu_load_freq_history_view;
#define CPU_LOAD_FREQ_HISTORY_SIZE	\
	(sizeof(struct cpu_load_freq_history_tag) * cpu_load_history_num)

static unsigned int  cpu_load_freq_history_cnt;
static unsigned int  cpu_load_freq_history_view_cnt;

static  int  cpu_load_freq_history_show_cnt = 100;
#if defined(CONFIG_SLP_CHECK_BUS_LOAD)
#define CPU_BUS_VIEW_DEFAULT_VALUE	100
static  int  cpu_bus_load_freq_history_show_cnt = CPU_BUS_VIEW_DEFAULT_VALUE;
#endif
#if defined(CONFIG_SLP_BUS_CLK_CHECK_LOAD)
static  int  cpu_bus_clk_load_freq_history_show_cnt = 100;
#endif
struct cpu_process_runtime_tag temp_process_runtime;

#define CPU_TASK_HISTORY_NUM 200
unsigned int cpu_task_history_num = CPU_TASK_HISTORY_NUM;

struct cpu_task_history_tag {
	unsigned long long time;
	struct task_struct *task;
	unsigned int pid;
};
static struct cpu_task_history_tag (*cpu_task_history)[CPU_NUM];
static struct cpu_task_history_tag	 (*cpu_task_history_view)[CPU_NUM];
#define CPU_TASK_HISTORY_SIZE	(sizeof(struct cpu_task_history_tag) \
					* cpu_task_history_num * CPU_NUM)

bool cpu_task_history_onoff;

struct list_head process_headlist;

static unsigned int  cpu_task_history_cnt[CPU_NUM];
static unsigned int  cpu_task_history_show_start_cnt;
static unsigned int  cpu_task_history_show_end_cnt;
static  int  cpu_task_history_show_select_cpu;

static unsigned long long  total_time, section_start_time, section_end_time;

void __slp_store_task_history(unsigned int cpu, struct task_struct *task)
{
	unsigned int cnt;

	if (cpu_task_history_onoff == 0)
		return ;

	if (++cpu_task_history_cnt[cpu] >= cpu_task_history_num)
		cpu_task_history_cnt[cpu] = 0;
	cnt = cpu_task_history_cnt[cpu];

	cpu_task_history[cnt][cpu].time = cpu_clock(UINT_MAX);
	cpu_task_history[cnt][cpu].task = task;
	cpu_task_history[cnt][cpu].pid = task->pid;
}



#if defined (CONFIG_CHECK_WORK_HISTORY)

#define CPU_WORK_HISTORY_NUM 10
unsigned int cpu_work_history_num = CPU_WORK_HISTORY_NUM;
static unsigned int  cpu_work_history_cnt[CPU_NUM];
static unsigned int  cpu_work_history_show_start_cnt;
static unsigned int  cpu_work_history_show_end_cnt;
static  int  cpu_work_history_show_select_cpu;

static struct list_head work_headlist;

struct cpu_work_history_tag {
	u64 start_time;
	u64 end_time;
//	u64 occup_time;

	struct task_struct *task;
	unsigned int pid;
	struct work_struct *work;
	work_func_t	func;
};
static struct cpu_work_history_tag (*cpu_work_history)[CPU_NUM];
static struct cpu_work_history_tag	 (*cpu_work_history_view)[CPU_NUM];
#define CPU_WORK_HISTORY_SIZE	(sizeof(struct cpu_work_history_tag) \
					* cpu_work_history_num * CPU_NUM)

bool cpu_work_history_onoff;

u64  get_load_analyzer_time(void)
{
	return cpu_clock(UINT_MAX);
}

void __slp_store_work_history(struct work_struct *work, work_func_t func
						, u64 start_time, u64 end_time)
{
	unsigned int cnt, cpu;
	struct task_struct *task;

	if (cpu_work_history_onoff == 0)
		return ;

	cpu = raw_smp_processor_id();
	task = current;

	if (++cpu_work_history_cnt[cpu] >= cpu_work_history_num)
		cpu_work_history_cnt[cpu] = 0;
	cnt = cpu_work_history_cnt[cpu];

	cpu_work_history[cnt][cpu].start_time = start_time;
	cpu_work_history[cnt][cpu].end_time = end_time;
//	cpu_work_history[cnt][cpu].occup_time = end_time - start_time;
	cpu_work_history[cnt][cpu].task = task;
	cpu_work_history[cnt][cpu].pid = task->pid;
	cpu_work_history[cnt][cpu].work = work;
	cpu_work_history[cnt][cpu].func = func;


}
#endif



struct saved_task_info_tag {
	unsigned int pid;
	char comm[TASK_COMM_LEN];
};
#define SAVED_TASK_INFO_NUM 50
#define SAVED_TASK_INFO_SIZE (sizeof(struct saved_task_info_tag)	* SAVED_TASK_INFO_NUM)
static struct saved_task_info_tag *saved_task_info;
static unsigned int  saved_task_info_cnt;


bool saved_task_info_onoff;

static int store_killed_init(void)
{
	saved_task_info = vmalloc(SAVED_TASK_INFO_SIZE);
	if (saved_task_info == NULL)
		return -ENOMEM;

	return 0;
}

static void store_killed_exit(void)
{
	if (saved_task_info != NULL)
		vfree(saved_task_info);
}


static void store_killed_memset(void)
{
	memset(saved_task_info, 0, SAVED_TASK_INFO_SIZE);
}



void store_killed_task(struct task_struct *tsk)
{
	unsigned int cnt;

	if( saved_task_info_onoff == 0)
		return;

	if (++saved_task_info_cnt >= SAVED_TASK_INFO_NUM)
		saved_task_info_cnt = 0;

	cnt = saved_task_info_cnt;

	saved_task_info[cnt].pid = tsk->pid;
	strncpy(saved_task_info[cnt].comm, tsk->comm, TASK_COMM_LEN);
}


static int search_killed_task(unsigned int pid, char *task_name)
{
	unsigned int cnt = saved_task_info_cnt, i;

	for (i = 0; i< SAVED_TASK_INFO_NUM; i++) {
		if (saved_task_info[cnt].pid == pid) {
			strncpy(task_name, saved_task_info[cnt].comm,\
													TASK_COMM_LEN);
			break;
		}
		cnt = get_index(cnt, SAVED_TASK_INFO_NUM, -1);
	}

	if( i == SAVED_TASK_INFO_NUM)
		return -1;
	else
		return 0;
}


struct cpu_process_runtime_tag {
	struct list_head list;
	unsigned long long runtime;
	struct task_struct *task;
	unsigned int pid;
	unsigned int cnt;
	unsigned int usage;
};

#if defined (CONFIG_CHECK_WORK_HISTORY)
struct cpu_work_runtime_tag {
	struct list_head list;
	u64 start_time;
	u64 end_time;
	u64 occup_time;

	struct task_struct *task;
	unsigned int pid;
	unsigned int cnt;

	struct work_struct *work;
	work_func_t func;
};
#endif

static unsigned long long calc_delta_time(unsigned int cpu, unsigned int index)
{
	unsigned long long run_start_time, run_end_time;

	if (index == 0) {
		run_start_time
		    = cpu_task_history_view[cpu_task_history_num-1][cpu].time;
	} else
		run_start_time = cpu_task_history_view[index-1][cpu].time;

	if (run_start_time < section_start_time)
		run_start_time = section_start_time;

	run_end_time = cpu_task_history_view[index][cpu].time;

	if (run_end_time < section_start_time)
		return 0;

	if (run_end_time > section_end_time)
		run_end_time = section_end_time;

	return  run_end_time - run_start_time;
}

static unsigned long long calc_delta_time_work(unsigned int cpu, unsigned int index)
{
	unsigned long long run_start_time, run_end_time;


	run_start_time = cpu_work_history_view[index][cpu].start_time;
	if (run_start_time < section_start_time)
		run_start_time = section_start_time;

	run_end_time = cpu_work_history_view[index][cpu].end_time;

	if (run_end_time < section_start_time)
		return 0;

	if (run_end_time > section_end_time)
		run_end_time = section_end_time;

	return  run_end_time - run_start_time;
}



static void add_process_to_list(unsigned int cpu, unsigned int index)
{
	struct cpu_process_runtime_tag *new_process;

	new_process
		= kmalloc(sizeof(struct cpu_process_runtime_tag), GFP_KERNEL);
	new_process->runtime = calc_delta_time(cpu, index);
	new_process->cnt = 1;
	new_process->task = cpu_task_history_view[index][cpu].task;
	new_process->pid = cpu_task_history_view[index][cpu].pid;

	if (new_process->runtime != 0) {
		INIT_LIST_HEAD(&new_process->list);
		list_add_tail(&new_process->list, &process_headlist);
	} else
		kfree(new_process);

	return;
}

#if defined (CONFIG_CHECK_WORK_HISTORY)
static void add_work_to_list(unsigned int cpu, unsigned int index)
{
	struct cpu_work_runtime_tag *new_work;

	new_work
		= kmalloc(sizeof(struct cpu_work_runtime_tag), GFP_KERNEL);

	new_work->occup_time =  calc_delta_time_work(cpu, index);

	new_work->cnt = 1;
	new_work->task = cpu_work_history_view[index][cpu].task;
	new_work->pid = cpu_work_history_view[index][cpu].pid;

	new_work->work = cpu_work_history_view[index][cpu].work;
	new_work->func = cpu_work_history_view[index][cpu].func;
	pr_info("%s %d\n", __FUNCTION__, __LINE__);

	if (new_work->occup_time != 0) {
		INIT_LIST_HEAD(&new_work->list);
		list_add_tail(&new_work->list, &work_headlist);
	} else
		kfree(new_work);

	return;
}
#endif

static void del_process_list(void)
{
	struct cpu_process_runtime_tag *curr;
	struct list_head *p, *n;

	list_for_each_prev_safe(p, n, &process_headlist) {
		curr = list_entry(p, struct cpu_process_runtime_tag, list);
		kfree(curr);
	}
	process_headlist.prev = NULL;
	process_headlist.next = NULL;

}

#if defined (CONFIG_CHECK_WORK_HISTORY)
static void del_work_list(void)
{
	struct cpu_work_runtime_tag *curr;
	struct list_head *p, *n;

	list_for_each_prev_safe(p, n, &work_headlist) {
		curr = list_entry(p, struct cpu_work_runtime_tag, list);
		kfree(curr);
	}
	work_headlist.prev = NULL;
	work_headlist.next = NULL;

}
#endif

static int comp_list_runtime(struct list_head *list1, struct list_head *list2)
{
	struct cpu_process_runtime_tag *list1_struct, *list2_struct;

	int ret = 0;
	 list1_struct = list_entry(list1, struct cpu_process_runtime_tag, list);
	 list2_struct = list_entry(list2, struct cpu_process_runtime_tag, list);

	if (list1_struct->runtime > list2_struct->runtime)
		ret = 1;
	else if (list1_struct->runtime < list2_struct->runtime)
		ret = -1;
	else
		ret  = 0;

	return ret;
}

#if defined (CONFIG_CHECK_WORK_HISTORY)
static int comp_list_occuptime(struct list_head *list1, struct list_head *list2)
{
	struct cpu_work_runtime_tag *list1_struct, *list2_struct;

	int ret = 0;
	 list1_struct = list_entry(list1, struct cpu_work_runtime_tag, list);
	 list2_struct = list_entry(list2, struct cpu_work_runtime_tag, list);

	if (list1_struct->occup_time > list2_struct->occup_time)
		ret = 1;
	else if (list1_struct->occup_time < list2_struct->occup_time)
		ret = -1;
	else
		ret  = 0;

	return ret;
}
#endif


static void swap_process_list(struct list_head *list1, struct list_head *list2)
{
	struct list_head *list1_prev, *list1_next , *list2_prev, *list2_next;

	list1_prev = list1->prev;
	list1_next = list1->next;

	list2_prev = list2->prev;
	list2_next = list2->next;

	list1->prev = list2;
	list1->next = list2_next;

	list2->prev = list1_prev;
	list2->next = list1;

	list1_prev->next = list2;
	list2_next->prev = list1;

}

static unsigned int view_list(int max_list_num, char *buf, unsigned int buf_size, unsigned int ret)
{
	struct list_head *p;
	struct cpu_process_runtime_tag *curr;
	char task_name[80] = {0,};
	char *p_name = NULL;

	unsigned int cnt = 0, list_num = 0;
	unsigned long long  t, total_time_for_clc;

	bool found_in_killed_process_list = 0;

	list_for_each(p, &process_headlist) {
		curr = list_entry(p, struct cpu_process_runtime_tag, list);
		list_num++;
	}

	for (cnt = 0; cnt < list_num; cnt++) {
		list_for_each(p, &process_headlist) {
		curr = list_entry(p, struct cpu_process_runtime_tag, list);
			if (p->next != &process_headlist) {
				if (comp_list_runtime(p, p->next) == -1)
					swap_process_list(p, p->next);
			}
		}
	}

	total_time_for_clc = total_time;
	do_div(total_time_for_clc, 1000);

	cnt = 1;
	list_for_each(p, &process_headlist) {
		curr = list_entry(p, struct cpu_process_runtime_tag, list);
		t = curr->runtime * 100;
		do_div(t, total_time_for_clc);
		curr->usage = t + 5;

		if ((curr != NULL) && (curr->task != NULL)
			&& (curr->task->pid == curr->pid)) {
			p_name = curr->task->comm;

		} else {
			if(search_killed_task(curr->pid, task_name) >= 0) {
				found_in_killed_process_list = 1;
			} else {
				snprintf(task_name, sizeof(task_name)
							, "NOT found task");
			}
			p_name = task_name;
		}

		if (ret < buf_size - 1) {
			ret +=  snprintf(buf + ret, buf_size - ret,
				"[%3d] %16s(%5d/%s) %6d[sched] %13lld[ns]"\
				" %3d.%02d[%%]\n"
				, cnt++, p_name, curr->pid
				, ((found_in_killed_process_list) == 1) ? "X" : "O"
				, curr->cnt, curr->runtime
				, curr->usage/1000, (curr->usage%1000) /10);
		} else
			break;

		found_in_killed_process_list = 0;

		if (cnt > max_list_num)
			break;
	}

	return ret;
}


static unsigned int view_list_raw(char *buf,unsigned int buf_size, unsigned int ret)
{
	struct list_head *p;
	struct cpu_process_runtime_tag *curr;
	char task_name[80] = {0,};
	char *p_name = NULL;

	unsigned int cnt = 0, list_num = 0;
	unsigned long long  t, total_time_for_clc;

	list_for_each(p, &process_headlist) {
		curr = list_entry(p, struct cpu_process_runtime_tag, list);
		list_num++;
	}

	for (cnt = 0; cnt < list_num; cnt++) {
		list_for_each(p, &process_headlist) {
		curr = list_entry(p, struct cpu_process_runtime_tag, list);
			if (p->next != &process_headlist) {
				if (comp_list_runtime(p, p->next) == -1)
					swap_process_list(p, p->next);
			}
		}
	}

	total_time_for_clc = total_time;
	do_div(total_time_for_clc, 1000);

	cnt = 1;
	list_for_each(p, &process_headlist) {
		curr = list_entry(p, struct cpu_process_runtime_tag, list);
		t = curr->runtime * 100;
		do_div(t, total_time_for_clc);
		curr->usage = t + 5;

		if ((curr != NULL) && (curr->task != NULL)
			&& (curr->task->pid == curr->pid)) {
			p_name = curr->task->comm;

		} else {
			snprintf(task_name, sizeof(task_name)
						, "NOT_FOUND");
			p_name = task_name;
		}

		if (ret < buf_size - 1) {
			ret +=  snprintf(buf + ret, buf_size - ret,
				"%d %s %d %lld %d\n"
				, curr->pid, p_name
				, curr->cnt, curr->runtime
				, curr->usage);
		} else
			break;

	}

	return ret;
}

#if defined (CONFIG_CHECK_WORK_HISTORY)
static unsigned int view_workfn_list(char *buf, unsigned int buf_size, unsigned int ret)
{
	struct list_head *p;
	struct cpu_work_runtime_tag *curr;
	unsigned int cnt = 0, list_num = 0;

	list_for_each(p, &work_headlist) {
		curr = list_entry(p, struct cpu_work_runtime_tag, list);
		list_num++;
	}

	for (cnt = 0; cnt < list_num; cnt++) {
		list_for_each(p, &work_headlist) {
		curr = list_entry(p, struct cpu_work_runtime_tag, list);
			if (p->next != &work_headlist) {
				if (comp_list_occuptime(p, p->next) == -1)
					swap_process_list(p, p->next);
			}
		}
	}

	cnt = 1;
	list_for_each(p, &work_headlist) {
		curr = list_entry(p, struct cpu_work_runtime_tag, list);
		if (ret < buf_size - 1) {
			ret +=  snprintf(buf + ret, buf_size - ret,
			"[%2d] %32pf(%4d) %16s %11lld[ns]\n"
			, cnt++, curr->func, curr->cnt ,curr->task->comm ,curr->occup_time);
		}
	}

	return ret;
}
#endif

static struct cpu_process_runtime_tag *search_exist_pid(unsigned int pid)
{
	struct list_head *p;
	struct cpu_process_runtime_tag *curr;

	list_for_each(p, &process_headlist) {
		curr = list_entry(p, struct cpu_process_runtime_tag, list);
		if (curr->pid == pid)
			return curr;
	}
	return NULL;
}

#if defined (CONFIG_CHECK_WORK_HISTORY)
static struct cpu_work_runtime_tag *search_exist_workfn(work_func_t func)
{
	struct list_head *p;
	struct cpu_work_runtime_tag *curr;

	list_for_each(p, &work_headlist) {
		curr = list_entry(p, struct cpu_work_runtime_tag, list);
		if (curr->func == func)
			return curr;
	}
	return NULL;
}
#endif


static void clc_process_run_time(unsigned int cpu
			, unsigned int start_cnt, unsigned int end_cnt)
{
	unsigned  int cnt = 0,  start_array_num;
	unsigned int end_array_num, end_array_num_plus1;
	unsigned int i, loop_cnt;
	struct cpu_process_runtime_tag *process_runtime_data;
	unsigned long long t1, t2;

	start_array_num
	    = (cpu_load_freq_history_view[start_cnt].task_history_cnt[cpu] + 1)
		% cpu_task_history_num;

	section_start_time
		= cpu_load_freq_history_view[start_cnt].time_stamp;
	section_end_time
		= cpu_load_freq_history_view[end_cnt].time_stamp;

	end_array_num
	= cpu_load_freq_history_view[end_cnt].task_history_cnt[cpu];
	end_array_num_plus1
	= (cpu_load_freq_history_view[end_cnt].task_history_cnt[cpu] + 1)
			% cpu_task_history_num;


	t1 = cpu_task_history_view[end_array_num][cpu].time;
	t2 = cpu_task_history_view[end_array_num_plus1][cpu].time;

	if (t2 < t1)
		end_array_num_plus1 = end_array_num;

	total_time = section_end_time - section_start_time;

	if (process_headlist.next != NULL)
		del_process_list();

	INIT_LIST_HEAD(&process_headlist);

	if (end_array_num_plus1 >= start_array_num)
		loop_cnt = end_array_num_plus1-start_array_num + 1;
	else
		loop_cnt = end_array_num_plus1
				+ cpu_task_history_num - start_array_num + 1;

	for (i = start_array_num, cnt = 0; cnt < loop_cnt; cnt++, i++) {
		if (i >= cpu_task_history_num)
			i = 0;
		process_runtime_data
			= search_exist_pid(cpu_task_history_view[i][cpu].pid);
		if (process_runtime_data == NULL)
			add_process_to_list(cpu, i);
		else {
			process_runtime_data->runtime
				+= calc_delta_time(cpu, i);
			process_runtime_data->cnt++;
		}
	}

}

#if defined (CONFIG_CHECK_WORK_HISTORY)
static void clc_work_run_time(unsigned int cpu
			, unsigned int start_cnt, unsigned int end_cnt)
{
	unsigned  int cnt = 0,  start_array_num;
	unsigned int end_array_num, end_array_num_plus1;
	unsigned int i, loop_cnt;
	struct cpu_work_runtime_tag *work_runtime_data;
	unsigned long long t1, t2;

	start_array_num
	    = (cpu_load_freq_history_view[start_cnt].work_history_cnt[cpu] + 1)
		% cpu_work_history_num;

	section_start_time
		= cpu_load_freq_history_view[start_cnt].time_stamp;
	section_end_time
		= cpu_load_freq_history_view[end_cnt].time_stamp;

	end_array_num
	= cpu_load_freq_history_view[end_cnt].work_history_cnt[cpu];
	end_array_num_plus1
	= (cpu_load_freq_history_view[end_cnt].work_history_cnt[cpu] + 1)
			% cpu_work_history_num;

	t1 = cpu_work_history_view[end_array_num][cpu].end_time;
	t2 = cpu_work_history_view[end_array_num_plus1][cpu].end_time;

	if (t2 < t1)
		end_array_num_plus1 = end_array_num;

	total_time = section_end_time - section_start_time;

	if (work_headlist.next != NULL)
		del_work_list();

	INIT_LIST_HEAD(&work_headlist);

	if (end_array_num_plus1 >= start_array_num)
		loop_cnt = end_array_num_plus1-start_array_num + 1;
	else
		loop_cnt = end_array_num_plus1
				+ cpu_work_history_num - start_array_num + 1;

	for (i = start_array_num, cnt = 0; cnt < loop_cnt; cnt++, i++) {
		if (i >= cpu_work_history_num)
			i = 0;

		work_runtime_data = search_exist_workfn(cpu_work_history_view[i][cpu].func);
		if (work_runtime_data == NULL)
			add_work_to_list(cpu, i);
		else {
			work_runtime_data->occup_time
				+= calc_delta_time_work(cpu, i);
			work_runtime_data->cnt++;
		}
	}

}
#endif

static unsigned int  process_sched_time_view(unsigned int cpu,
			unsigned int start_cnt, unsigned int end_cnt
			, char *buf, unsigned int buf_size, unsigned int ret)
{
	unsigned  int i = 0, start_array_num, data_line, cnt=0;
	unsigned int end_array_num, start_array_num_for_time;

	start_array_num_for_time
	= cpu_load_freq_history_view[start_cnt].task_history_cnt[cpu];
	start_array_num
	= (cpu_load_freq_history_view[start_cnt].task_history_cnt[cpu]+1)
			% cpu_task_history_num;
	end_array_num
		= cpu_load_freq_history_view[end_cnt].task_history_cnt[cpu];

	total_time = section_end_time - section_start_time;

	if (end_cnt == start_cnt+1) {
		ret +=  snprintf(buf + ret, buf_size - ret,
			"[%d] TOTAL SECTION TIME = %lld[ns]\n[%5d]~[%5d]/(%lld~%lld)\n\n"
			, end_cnt, total_time
			, (cpu_load_freq_history_view[start_cnt].task_history_cnt[cpu]\
						+ 1)	% cpu_task_history_num
			, (cpu_load_freq_history_view[end_cnt].task_history_cnt[cpu]\
						+ 1)% cpu_task_history_num
			, cpu_load_freq_history_view[start_cnt].time_stamp
			, cpu_load_freq_history_view[end_cnt].time_stamp);
	} else {
		ret +=  snprintf(buf + ret, buf_size - ret,
			"[%d~%d] TOTAL SECTION TIME = %lld[ns]\n[%5d]~[%5d]/(%lld~%lld)\n\n"
				, start_cnt+1, end_cnt, total_time
			, (cpu_load_freq_history_view[start_cnt].task_history_cnt[cpu]\
						+ 1)% cpu_task_history_num
			, (cpu_load_freq_history_view[end_cnt].task_history_cnt[cpu]\
						+ 1)% cpu_task_history_num
			, cpu_load_freq_history_view[start_cnt].time_stamp
			, cpu_load_freq_history_view[end_cnt].time_stamp);
	}

	end_array_num = get_index(end_array_num, cpu_task_history_num, 2);

	if (end_array_num >= start_array_num_for_time)
		data_line = end_array_num -start_array_num_for_time + 1;
	else {
		data_line = (end_array_num + cpu_task_history_num) \
				-start_array_num_for_time + 1;
	}

	cnt = start_array_num_for_time;

	for (i = 0; i < data_line; i++) {
		u64 delta_time ,cnt_m1;
		char *p_name;
		char task_name[80] = {0,};
		unsigned int pid = 0;

		if (cnt > cpu_task_history_num-1)
			cnt = 0;

		cnt_m1 = get_index(cnt, cpu_task_history_num, -1);
		delta_time = cpu_task_history_view[cnt][cpu].time \
				-cpu_task_history_view[cnt_m1][cpu].time;

		pid = cpu_task_history_view[cnt][cpu].pid;
		if (cpu_task_history_view[cnt][cpu].task->pid == pid) {
			p_name = cpu_task_history_view[cnt][cpu].task->comm;

		} else {
			if(search_killed_task(pid, task_name) < 0) {
				snprintf(task_name, sizeof(task_name) \
							, "NOT found task");
			}
			p_name = task_name;
		}

		if (ret >= buf_size)
			break;

		ret +=  snprintf(buf + ret, buf_size - ret,
			"[%5d] %lld %16s %5d %10lld[ns]\n", cnt
			, cpu_task_history_view[cnt][cpu].time
			, p_name
			, cpu_task_history_view[cnt][cpu].pid
			, delta_time);

		cnt++;
	}

	return ret;

}

#if defined (CONFIG_CHECK_WORK_HISTORY)
static unsigned int  work_time_list_view(unsigned int cpu,
			unsigned int start_cnt, unsigned int end_cnt
			, char *buf, unsigned int buf_size, unsigned int ret)
{
	unsigned  int i = 0, start_array_num, data_line, cnt=0;
	unsigned int end_array_num, start_array_num_for_time;

	start_array_num_for_time
	= cpu_load_freq_history_view[start_cnt].work_history_cnt[cpu];
	start_array_num
	= (cpu_load_freq_history_view[start_cnt].work_history_cnt[cpu]+1)
			% cpu_work_history_num;
	end_array_num
		= cpu_load_freq_history_view[end_cnt].work_history_cnt[cpu];

	total_time = section_end_time - section_start_time;

	if (end_cnt == start_cnt+1) {
		ret +=  snprintf(buf + ret, buf_size - ret,
			"[%d] TOTAL SECTION TIME = %lld[ns]\n[%5d]~[%5d]/(%lld ~ %lld)\n\n"
			, end_cnt, total_time
			, (cpu_load_freq_history_view[start_cnt].work_history_cnt[cpu]\
							+ 1)	% cpu_task_history_num
			, (cpu_load_freq_history_view[end_cnt].work_history_cnt[cpu]\
							+ 1)	% cpu_task_history_num
			, cpu_load_freq_history_view[start_cnt].time_stamp
			, cpu_load_freq_history_view[end_cnt].time_stamp);
	} else {
		ret +=  snprintf(buf + ret, buf_size - ret,
			"[%d~%d] TOTAL SECTION TIME = %lld[ns]\n[%5d]~[%5d]/(%lld ~ %lld)\n\n"
			, get_index(start_cnt, cpu_work_history_num, 1)
			, end_cnt, total_time
			, (cpu_load_freq_history_view[start_cnt].work_history_cnt[cpu]\
							+ 1)	% cpu_task_history_num
			, (cpu_load_freq_history_view[end_cnt].work_history_cnt[cpu]\
							+ 1)	% cpu_task_history_num
			, cpu_load_freq_history_view[start_cnt].time_stamp
			, cpu_load_freq_history_view[end_cnt].time_stamp);
	}

	end_array_num = get_index(end_array_num, cpu_work_history_num, 2);

	if (end_array_num >= start_array_num_for_time)
		data_line = end_array_num -start_array_num_for_time + 1;
	else {
		data_line = (end_array_num + cpu_work_history_num) \
				-start_array_num_for_time + 1;
	}
	cnt = start_array_num_for_time;

	for (i = 0; i < data_line; i++) {
		u64 delta_time;
		char *p_name;
		char task_name[80] = {0,};
		unsigned int pid = 0;

		if (cnt > cpu_work_history_num-1)
			cnt = 0;

		delta_time = cpu_work_history_view[cnt][cpu].end_time \
				-cpu_work_history_view[cnt][cpu].start_time;

		pid = cpu_work_history_view[cnt][cpu].pid;
		if (cpu_work_history_view[cnt][cpu].task->pid == pid) {
			p_name = cpu_work_history_view[cnt][cpu].task->comm;

		} else {
			if(search_killed_task(pid, task_name) < 0) {
				snprintf(task_name, sizeof(task_name) \
							, "NOT found task");
			}
			p_name = task_name;
		}

		if (ret >= buf_size)
			break;

		ret +=  snprintf(buf + ret, buf_size - ret,
			"[%d] %32pf @ %24pf  [%16s]   %lld ~ %lld %10lld[ns] \n", cnt
			, cpu_work_history_view[cnt][cpu].func
			, cpu_work_history_view[cnt][cpu].work
			, p_name
			, cpu_work_history_view[cnt][cpu].start_time
			, cpu_work_history_view[cnt][cpu].end_time
			, delta_time );
		cnt++;
	}

	return ret;

}
#endif

void cpu_load_touch_event(unsigned int event)
{
	unsigned int cnt = 0;

	if (cpu_task_history_onoff == 0)
		return;

	cnt = cpu_load_freq_history_cnt;
	if (event == 0)
		cpu_load_freq_history[cnt].touch_event = 100;
	else if (event == 1)
		cpu_load_freq_history[cnt].touch_event = 1;

}
struct saved_load_factor_tag saved_load_factor;

void store_external_load_factor(int type, unsigned int data)
{
	switch (type) {

	case NR_RUNNING_TASK:
		saved_load_factor.nr_running_task = data;
		break;
	case MIF_BUS_FREQ:
		saved_load_factor.mif_bus_freq = data;
		break;
	case MIF_BUS_LOAD:
		saved_load_factor.mif_bus_load = data;
		break;
	case GPU_FREQ:
		saved_load_factor.gpu_freq = data;
		break;
	case GPU_UTILIZATION:
		saved_load_factor.gpu_utilization = data;
		break;
	default:
		break;
	}
}


u32 emc_clk_get(void);
extern int suspending_flag;
unsigned int cpuidle_get_idle_residency_time(int cpu, int idle_state);

void store_cpu_load(unsigned int cpufreq[], unsigned int cpu_load[])
{
	unsigned int j = 0, cnt = 0;
	unsigned long long t, t_interval;
	unsigned int  t_interval_us;
	static unsigned long long before_t;
	unsigned long  nanosec_rem;
	int cpu_max = 0, cpu_min = 0;
	int cpu_max_online_lock = 0, cpu_min_online_lock = 0;
	struct cpufreq_policy *policy;

	if (cpu_task_history_onoff == 0)
		return;

	if (++cpu_load_freq_history_cnt >= cpu_load_history_num)
		cpu_load_freq_history_cnt = 0;

	cnt = cpu_load_freq_history_cnt;

	policy = cpufreq_cpu_get(0);

	if (policy !=NULL) {
		cpu_min = pm_qos_request(PM_QOS_CPU_FREQ_MIN);
		if (cpu_min < policy->min)
			cpu_min = policy->min;

		cpu_max = pm_qos_request(PM_QOS_CPU_FREQ_MAX);
		if (cpu_max > policy->max)
			cpu_max = policy->max;

		cpufreq_cpu_put(policy);
	}

	cpu_load_freq_history[cnt].cpu_min_locked_freq = cpu_min;
	cpu_load_freq_history[cnt].cpu_max_locked_freq = cpu_max;

	cpu_min_online_lock = pm_qos_request(PM_QOS_CPU_ONLINE_MIN);
	cpu_max_online_lock = pm_qos_request(PM_QOS_CPU_ONLINE_MAX);
	cpu_load_freq_history[cnt].cpu_max_locked_online = cpu_max_online_lock;
	cpu_load_freq_history[cnt].cpu_min_locked_online = cpu_min_online_lock;

	t = cpu_clock(UINT_MAX);

	if (before_t == 0)
		before_t = t;

	for (j = 0; j < CPU_NUM; j++) {
		cpu_load_freq_history[cnt].task_history_cnt[j]
			= cpu_task_history_cnt[j];
	}

#if defined (CONFIG_CHECK_WORK_HISTORY)
	for (j = 0; j < CPU_NUM; j++) {
		cpu_load_freq_history[cnt].work_history_cnt[j]
			= cpu_work_history_cnt[j];
	}
#endif

	t_interval = t -before_t;
	do_div(t_interval, 1000);
	t_interval_us = t_interval;
	before_t = t;

	if (t_interval != 0) {
		cpu_load_freq_history[cnt].cpu_idle_time[0][0]
			= (cpuidle_get_idle_residency_time(0, 0) * 1000 + 5) / t_interval_us;
		cpu_load_freq_history[cnt].cpu_idle_time[0][1]
			= (cpuidle_get_idle_residency_time(0, 1) * 1000 +5) / t_interval_us;
		cpu_load_freq_history[cnt].cpu_idle_time[0][2]
			= (cpuidle_get_idle_residency_time(0, 2) * 1000 +5) / t_interval_us;
		cpu_load_freq_history[cnt].cpu_idle_time[1][0]
			= (cpuidle_get_idle_residency_time(1, 0) * 1000 +5) / t_interval_us;
		cpu_load_freq_history[cnt].cpu_idle_time[1][1]
			= (cpuidle_get_idle_residency_time(1, 1) * 1000 + 5) / t_interval_us;
		cpu_load_freq_history[cnt].cpu_idle_time[1][2]
			= (cpuidle_get_idle_residency_time(1, 2) * 1000 + 5)/ t_interval_us;
	}

	if (suspending_flag == 1)
		cpu_load_freq_history[cnt].status = 'S';
	else if (suspending_flag == 0)
		cpu_load_freq_history[cnt].status = 'N';
	else if (suspending_flag == -1)
		cpu_load_freq_history[cnt].status = 'F';

	cpu_load_freq_history[cnt].time_stamp = t;
	nanosec_rem = do_div(t, 1000000000);
	snprintf(cpu_load_freq_history[cnt].time, sizeof(cpu_load_freq_history[cnt].time),
		"%2lu.%02lu", (unsigned long) t,(unsigned long)nanosec_rem / 10000000);

	for (j = 0; j < CPU_NUM; j++) {
		cpu_load_freq_history[cnt].cpufreq[j] = cpufreq[j];
		cpu_load_freq_history[cnt].cpu_load[j] = cpu_load[j];
	}
	cpu_load_freq_history[cnt].touch_event = 0;
	cpu_load_freq_history[cnt].nr_onlinecpu = num_online_cpus();
	cpu_load_freq_history[cnt].nr_run_avg
		= saved_load_factor.nr_running_task;

	cpu_load_freq_history[cnt].mif_bus_freq = emc_clk_get() * 1000;
	cpu_load_freq_history[cnt].mif_bus_load = saved_load_factor.mif_bus_load;

	cpu_load_freq_history[cnt].gpu_freq = saved_load_factor.gpu_freq;
	cpu_load_freq_history[cnt].gpu_utilization = (saved_load_factor.gpu_utilization * 1000 / 256);

#if defined(CONFIG_SLP_BUS_CLK_CHECK_LOAD)
{
	unsigned int pm_domains[PWR_DOMAINS_NUM];
	unsigned int clk_gates[CLK_GATES_NUM];

	if ((value_for_debug == 2) || (value_for_debug == 3)) {
		clk_mon_power_domain(pm_domains);
		clk_mon_clock_gate(clk_gates);

		memcpy(cpu_load_freq_history[cnt].power_domains
					, pm_domains, PWR_DOMAINS_NUM*sizeof(unsigned int));

		memcpy(cpu_load_freq_history[cnt].clk_gates
					, clk_gates, CLK_GATES_NUM*sizeof(unsigned int));
	} else {
		memset(cpu_load_freq_history[cnt].power_domains
			, 0, PWR_DOMAINS_NUM*sizeof(unsigned int));
		memset(cpu_load_freq_history[cnt].clk_gates
			, 0, CLK_GATES_NUM*sizeof(unsigned int));
	}
}
#endif



}


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

unsigned int show_cpu_load_freq_sub(int cnt, int show_cnt, char *buf, unsigned int buf_size, int ret)
{
	int j, delta = 0;

	if ((cnt - show_cnt) < 0) {
		delta = cnt - show_cnt;
		cnt = cpu_load_history_num + delta;
	} else
		cnt -= show_cnt;

	if ((cnt+1 >= cpu_load_history_num)
			|| (cpu_load_freq_history_view[cnt+1].time == 0))
		cnt = 0;
	else
		cnt++;

	for (j = 0; j < show_cnt; j++) {
		if (cnt > cpu_load_history_num-1)
			cnt = 0;

		ret +=  snprintf(buf + ret, buf_size - ret
		, "%8s    %c    %d.%02d\t%d.%02d/%d.%02d    %d/%d" \
		"    [%5d]\t%3d\t%3d\t%3d \t%2d.%1d\n"
		, cpu_load_freq_history_view[cnt].time
		, cpu_load_freq_history_view[cnt].status
		, cpu_load_freq_history_view[cnt].cpufreq[0]/1000000
		, (cpu_load_freq_history_view[cnt].cpufreq[0]/10000) % 100
		, cpu_load_freq_history_view[cnt].cpu_min_locked_freq/1000000
		, (cpu_load_freq_history_view[cnt].cpu_min_locked_freq/10000) % 100
		, cpu_load_freq_history_view[cnt].cpu_max_locked_freq/1000000
		, (cpu_load_freq_history_view[cnt].cpu_max_locked_freq/10000) % 100
		, cpu_load_freq_history_view[cnt].cpu_min_locked_online
		, cpu_load_freq_history_view[cnt].cpu_max_locked_online
		, cnt
		, cpu_load_freq_history_view[cnt].cpu_load[0]
		, cpu_load_freq_history_view[cnt].cpu_load[1]
		, cpu_load_freq_history_view[cnt].nr_onlinecpu
		, cpu_load_freq_history_view[cnt].nr_run_avg/100
		, cpu_load_freq_history_view[cnt].nr_run_avg%100);

		++cnt;
	}
	return ret;

}

enum {
	NORMAL_MODE,
	RECOVERY_MODE,
};
static int before_cpu_task_history_num;
static int before_cpu_load_history_num;

static int cpu_load_analyzer_init(int mode)
{

	if (mode == RECOVERY_MODE) {
		cpu_task_history_num = before_cpu_task_history_num;
		cpu_load_history_num = before_cpu_load_history_num;
	}

	cpu_load_freq_history = vmalloc(CPU_LOAD_FREQ_HISTORY_SIZE);
	if (cpu_load_freq_history == NULL)
		return -ENOMEM;

	cpu_load_freq_history_view = vmalloc(CPU_LOAD_FREQ_HISTORY_SIZE);
	if (cpu_load_freq_history_view == NULL)
		return -ENOMEM;

	cpu_task_history = vmalloc(CPU_TASK_HISTORY_SIZE);
	if (cpu_task_history == NULL)
		return -ENOMEM;

	cpu_task_history_view = vmalloc(CPU_TASK_HISTORY_SIZE);
	if (cpu_task_history_view == NULL)
		return -ENOMEM;

#if defined (CONFIG_CHECK_WORK_HISTORY)
	cpu_work_history =	vmalloc(CPU_WORK_HISTORY_SIZE);
	if (cpu_work_history == NULL)
		return -ENOMEM;

	cpu_work_history_view = vmalloc(CPU_WORK_HISTORY_SIZE);
	if (cpu_work_history_view == NULL)
		return -ENOMEM;
#endif
	if (store_killed_init() != 0)
		return -ENOMEM;

	memset(cpu_load_freq_history, 0, CPU_LOAD_FREQ_HISTORY_SIZE);
	memset(cpu_load_freq_history_view, 0, CPU_LOAD_FREQ_HISTORY_SIZE);
	memset(cpu_task_history, 0, CPU_TASK_HISTORY_SIZE);
	memset(cpu_task_history_view, 0, CPU_TASK_HISTORY_SIZE);

#if defined (CONFIG_CHECK_WORK_HISTORY)
	memset(cpu_work_history, 0, CPU_WORK_HISTORY_SIZE);
	memset(cpu_work_history_view, 0, CPU_WORK_HISTORY_SIZE);
#endif
	store_killed_memset();

	pr_info("LA size %d %d %d sum %d", CPU_LOAD_FREQ_HISTORY_SIZE,
		CPU_TASK_HISTORY_SIZE, CPU_WORK_HISTORY_SIZE,
		(CPU_LOAD_FREQ_HISTORY_SIZE \
		+ CPU_TASK_HISTORY_SIZE + CPU_WORK_HISTORY_SIZE) * 2);

#if defined (CONFIG_CHECK_WORK_HISTORY)
	cpu_work_history_onoff = 1;
#endif
	cpu_task_history_onoff = 1;

	saved_task_info_onoff = 1;

	return 0;
}



static int cpu_load_analyzer_exit(void)
{

	before_cpu_task_history_num = cpu_task_history_num;
	before_cpu_load_history_num = cpu_load_history_num;

	cpu_task_history_onoff = 0;

#if defined (CONFIG_CHECK_WORK_HISTORY)
	cpu_work_history_onoff = 0;
#endif
	saved_task_info_onoff = 0;

	msleep(1); /* to prevent wrong access to released memory */

	if (cpu_load_freq_history != NULL)
		vfree(cpu_load_freq_history);

	if (cpu_load_freq_history_view != NULL)
		vfree(cpu_load_freq_history_view);

	if (cpu_task_history != NULL)
		vfree(cpu_task_history);
	if (cpu_task_history_view != NULL)
		vfree(cpu_task_history_view);

#if defined (CONFIG_CHECK_WORK_HISTORY)
	if (cpu_work_history != NULL)
		vfree(cpu_task_history);

	if (cpu_work_history_view != NULL)
		vfree(cpu_work_history_view);

#endif

	store_killed_exit();

	return 0;
}

#if defined(CONFIG_SLP_CHECK_BUS_LOAD)
unsigned int show_cpu_bus_load_freq_sub(int cnt, int show_cnt
					, char *buf, unsigned int buf_size, int ret)
{
	int j, delta = 0;

	if ((cnt - show_cnt) < 0) {
		delta = cnt - show_cnt;
		cnt = cpu_load_history_num + delta;
	} else
		cnt -= show_cnt;

	if ((cnt+1 >= cpu_load_history_num)
			|| (cpu_load_freq_history_view[cnt+1].time == 0))
		cnt = 0;
	else
		cnt++;

	for (j = 0; j < show_cnt; j++) {
		if (cnt > cpu_load_history_num-1)
			cnt = 0;

		ret +=  snprintf(buf + ret, buf_size - ret
		, "%8s    %c    %d.%02d\t%d.%02d/%d.%02d    %d/%d" \
		"    [%5d]    %3d (%3d/%3d/%3d)    %3d (%3d/%3d/%3d)\t%3d " \
		"\t%2d.%1d        %3d      %3d       %3d     %3d.%1d\
			\n"
		, cpu_load_freq_history_view[cnt].time
		, cpu_load_freq_history_view[cnt].status
		, cpu_load_freq_history_view[cnt].cpufreq[0]/1000000
		, (cpu_load_freq_history_view[cnt].cpufreq[0]/10000) % 100
		, cpu_load_freq_history_view[cnt].cpu_min_locked_freq/1000000
		, (cpu_load_freq_history_view[cnt].cpu_min_locked_freq/10000) % 100
		, cpu_load_freq_history_view[cnt].cpu_max_locked_freq/1000000
		, (cpu_load_freq_history_view[cnt].cpu_max_locked_freq/10000) % 100
		, cpu_load_freq_history_view[cnt].cpu_min_locked_online
		, cpu_load_freq_history_view[cnt].cpu_max_locked_online
		, cnt
		, cpu_load_freq_history_view[cnt].cpu_load[0]
		, cpu_load_freq_history[cnt].cpu_idle_time[0][0]/10
		, cpu_load_freq_history[cnt].cpu_idle_time[0][1]/10
		, cpu_load_freq_history[cnt].cpu_idle_time[0][2]/10
		, cpu_load_freq_history_view[cnt].cpu_load[1]
		, cpu_load_freq_history[cnt].cpu_idle_time[1][0]/10
		, cpu_load_freq_history[cnt].cpu_idle_time[1][1]/10
		, cpu_load_freq_history[cnt].cpu_idle_time[1][2]/10
		, cpu_load_freq_history_view[cnt].nr_onlinecpu
		, cpu_load_freq_history_view[cnt].nr_run_avg/10
		, cpu_load_freq_history_view[cnt].nr_run_avg%10
		, cpu_load_freq_history_view[cnt].mif_bus_freq/1000
		, cpu_load_freq_history_view[cnt].mif_bus_load
		, cpu_load_freq_history_view[cnt].gpu_freq/1000
		, cpu_load_freq_history_view[cnt].gpu_utilization/10
		, cpu_load_freq_history_view[cnt].gpu_utilization%10);

		++cnt;
	}
	return ret;

}
#endif

#ifdef CONFIG_SLP_BUS_CLK_CHECK_LOAD
unsigned int show_cpu_bus_clk_load_freq_sub(int cnt
					, int show_cnt, char *buf, int ret)
{
	int j, delta = 0;

	if ((cnt - show_cnt) < 0) {
		delta = cnt - show_cnt;
		cnt = cpu_load_history_num + delta;
	} else
		cnt -= show_cnt;

	if ((cnt+1 >= cpu_load_history_num)
			|| (cpu_load_freq_history_view[cnt+1].time == 0))
		cnt = 0;
	else
		cnt++;

	for (j = 0; j < show_cnt; j++) {

		if (cnt > cpu_load_history_num-1)
			cnt = 0;

		if (ret < PAGE_SIZE - 1) {
		ret +=  snprintf(buf + ret, PAGE_SIZE - ret
			,"%8s   %d.%02d  %d.%02d %d.%02d/%d.%02d" \
			"  [%5d]  %3d %3d %3d " \
			" %2d.%1d      %3d        %3d     %3d    %3d.%1d" \
			"  %6x  %6x  %6x  %6x  %6x  %6x   %6x\n"
			, cpu_load_freq_history_view[cnt].time
			, cpu_load_freq_history_view[cnt].cpufreq[0]/1000000
			, (cpu_load_freq_history_view[cnt].cpufreq[0]/10000) % 100
			, cpu_load_freq_history_view[cnt].cpufreq[1]/1000000
			, (cpu_load_freq_history_view[cnt].cpufreq[1]/10000) % 100
			, cpu_load_freq_history_view[cnt].cpu_min_locked_freq/1000000
			, (cpu_load_freq_history_view[cnt].cpu_min_locked_freq/10000) % 100
			, cpu_load_freq_history_view[cnt].cpu_max_locked_freq/1000000
			, (cpu_load_freq_history_view[cnt].cpu_max_locked_freq/10000) % 100
			, cnt
			, cpu_load_freq_history_view[cnt].cpu_load[0]
			, cpu_load_freq_history_view[cnt].cpu_load[1]
			, cpu_load_freq_history_view[cnt].nr_onlinecpu
			, cpu_load_freq_history_view[cnt].nr_run_avg/10
			, cpu_load_freq_history_view[cnt].nr_run_avg%10
			, cpu_load_freq_history_view[cnt].mif_bus_freq/1000
			, cpu_load_freq_history_view[cnt].mif_bus_load
			, cpu_load_freq_history_view[cnt].gpu_freq/1000
			, cpu_load_freq_history_view[cnt].gpu_utilization/10
			, cpu_load_freq_history_view[cnt].gpu_utilization%10
			, cpu_load_freq_history_view[cnt].power_domains[0]
			, cpu_load_freq_history_view[cnt].power_domains[1]
			, cpu_load_freq_history_view[cnt].clk_gates[0]
			, cpu_load_freq_history_view[cnt].clk_gates[1]
			, cpu_load_freq_history_view[cnt].clk_gates[2]
			, cpu_load_freq_history_view[cnt].clk_gates[3]
			, cpu_load_freq_history_view[cnt].clk_gates[4]
			);
		} else
			break;
		++cnt;
	}

	return ret;

}
#endif

static void set_cpu_load_freq_history_array_range(const char *buf)
{
	int show_array_num = 0, select_cpu;
	char cpy_buf[80] = {0,};
	char *p1, *p2, *p_lf;

	p1 = strstr(buf, "-");
	p2 = strstr(buf, "c");
	p_lf = strstr(buf, "\n");

	if (p2 == NULL)
		p2 = strstr(buf, "C");

	if ( (p2 != NULL) && ((p_lf - p2)  > 0)) {
		select_cpu = atoi(p2+1);
		if (select_cpu >= 0 && select_cpu < 4)
			cpu_task_history_show_select_cpu = select_cpu;
		else
			cpu_task_history_show_select_cpu = 0;
	} else
		cpu_task_history_show_select_cpu = -1;

	if (p1 != NULL) {
		strncpy(cpy_buf, buf, sizeof(cpy_buf) - 1);
		*p1 = '\0';
		cpu_task_history_show_start_cnt
			= get_index(atoi(cpy_buf) ,cpu_load_history_num ,-1);
		cpu_task_history_show_end_cnt = atoi(p1+1);

	} else {
		show_array_num = atoi(buf);
		cpu_task_history_show_start_cnt
			= get_index(show_array_num, cpu_load_history_num, -1);
		cpu_task_history_show_end_cnt = show_array_num;
	}

	pr_info("start_cnt=%d end_cnt=%d\n"
		, cpu_task_history_show_start_cnt, cpu_task_history_show_end_cnt);

}

enum {
	WRONG_CPU_NUM = -1,
	WRONG_TIME_STAMP = -2,
	OVERFLOW_ERROR = -3,
};
int check_valid_range(unsigned int cpu, unsigned int start_cnt
						, unsigned int end_cnt)
{
	int ret = 0;
	unsigned long long t1, t2;
	unsigned int end_sched_cnt = 0, end_sched_cnt_margin;
	unsigned int load_cnt, last_load_cnt, overflow = 0;
	unsigned int cnt, search_cnt;
	unsigned int upset = 0;

	unsigned int i;

	if (cpu > CPU_NUM)
		return WRONG_CPU_NUM;

	t1 = cpu_load_freq_history_view[start_cnt].time_stamp;
	t2 = cpu_load_freq_history_view[end_cnt].time_stamp;

	if ((t2 <= t1) || (t1 == 0) || (t2 == 0)) {
		pr_info("[time error] t1=%lld t2=%lld\n", t1, t2);
		return WRONG_TIME_STAMP;
	}

	last_load_cnt = cpu_load_freq_history_view_cnt;

	cnt = cpu_load_freq_history_view[last_load_cnt].task_history_cnt[cpu];
	t1 = cpu_task_history_view[cnt][cpu].time;
	search_cnt = cnt;
	for (i = 0;  i < cpu_task_history_num; i++) {
		search_cnt = get_index(search_cnt, cpu_task_history_num, 1);
		t2 = cpu_task_history_view[search_cnt][cpu].time;

		if (t2 < t1) {
			end_sched_cnt = search_cnt;
			break;
		}

		if (i >= cpu_task_history_num - 1)
			end_sched_cnt = cnt;
	}

	load_cnt = last_load_cnt;
	for (i = 0;  i < cpu_load_history_num; i++) {
		unsigned int sched_cnt, sched_before_cnt;
		unsigned int sched_before_cnt_margin;

		sched_cnt
			= cpu_load_freq_history_view[load_cnt]\
						.task_history_cnt[cpu];
		load_cnt = get_index(load_cnt, cpu_load_history_num, -1);

		sched_before_cnt
			= cpu_load_freq_history_view[load_cnt]\
						.task_history_cnt[cpu];

		if (sched_before_cnt > sched_cnt)
			upset++;

		end_sched_cnt_margin
			= get_index(end_sched_cnt, cpu_task_history_num, 1);
		sched_before_cnt_margin
			= get_index(sched_before_cnt, cpu_task_history_num, -1);

		/* "end_sched_cnt -1" is needed
		  *  because of calulating schedule time */
		if ((upset >= 2) || ((upset == 1)
			&& (sched_before_cnt_margin < end_sched_cnt_margin))) {
			overflow = 1;
			pr_err("[LA] overflow cpu=%d upset=%d sched_before_cnt_margin=%d" \
				"end_sched_cnt_margin=%d end_sched_cnt=%d" \
				"sched_before_cnt=%d sched_cnt=%d load_cnt=%d" \
				, cpu , upset, sched_before_cnt_margin
				, end_sched_cnt_margin, end_sched_cnt
				, sched_before_cnt, sched_cnt, load_cnt);
			break;
		}

		if (load_cnt == start_cnt)
			break;
	}

	if (overflow == 0)
		ret = 0;
	else {
		ret = OVERFLOW_ERROR;
		pr_info("[overflow error]\n");
	}
	return ret;
}

#if defined (CONFIG_CHECK_WORK_HISTORY)
int check_work_valid_range(unsigned int cpu, unsigned int start_cnt
						, unsigned int end_cnt)
{
	int ret = 0;
	unsigned long long t1, t2;
	unsigned int end_sched_cnt = 0, end_sched_cnt_margin;
	unsigned int load_cnt, last_load_cnt, overflow = 0;
	unsigned int cnt, search_cnt;
	unsigned int upset = 0;

	unsigned int i;

	if (cpu > CPU_NUM)
		return WRONG_CPU_NUM;

	t1 = cpu_load_freq_history_view[start_cnt].time_stamp;
	t2 = cpu_load_freq_history_view[end_cnt].time_stamp;

	if ((t2 <= t1) || (t1 == 0) || (t2 == 0)) {
		pr_info("[time error] t1=%lld t2=%lld\n", t1, t2);
		return WRONG_TIME_STAMP;
	}

	last_load_cnt = cpu_load_freq_history_view_cnt;

	cnt = cpu_load_freq_history_view[last_load_cnt].work_history_cnt[cpu];
	t1 = cpu_task_history_view[cnt][cpu].time;
	search_cnt = cnt;
	for (i = 0;  i < cpu_task_history_num; i++) {
		search_cnt = get_index(search_cnt, cpu_task_history_num, 1);
		t2 = cpu_task_history_view[search_cnt][cpu].time;

		if (t2 < t1) {
			end_sched_cnt = search_cnt;
			break;
		}

		if (i >= cpu_task_history_num - 1)
			end_sched_cnt = cnt;
	}

	load_cnt = last_load_cnt;
	for (i = 0;  i < cpu_load_history_num; i++) {
		unsigned int sched_cnt, sched_before_cnt;
		unsigned int sched_before_cnt_margin;

		sched_cnt
			= cpu_load_freq_history_view[load_cnt]\
						.work_history_cnt[cpu];
		load_cnt = get_index(load_cnt, cpu_load_history_num, -1);

		sched_before_cnt
			= cpu_load_freq_history_view[load_cnt]\
						.work_history_cnt[cpu];

		if (sched_before_cnt > sched_cnt)
			upset++;

		end_sched_cnt_margin
			= get_index(end_sched_cnt, cpu_work_history_num, 1);
		sched_before_cnt_margin
			= get_index(sched_before_cnt, cpu_work_history_num, -1);

		/* "end_sched_cnt -1" is needed
		  *  because of calulating schedule time */
		if ((upset >= 2) || ((upset == 1)
			&& (sched_before_cnt_margin < end_sched_cnt_margin))) {
			overflow = 1;
			pr_err("[LA] overflow cpu=%d upset=%d sched_before_cnt_margin=%d" \
				"end_sched_cnt_margin=%d end_sched_cnt=%d" \
				"sched_before_cnt=%d sched_cnt=%d load_cnt=%d" \
				, cpu , upset, sched_before_cnt_margin
				, end_sched_cnt_margin, end_sched_cnt
				, sched_before_cnt, sched_cnt, load_cnt);
			break;
		}

		if (load_cnt == start_cnt)
			break;
	}

	if (overflow == 0)
		ret = 0;
	else {
		ret = OVERFLOW_ERROR;
		pr_info("[overflow error]\n");
	}
	return ret;
}
#endif


int check_running_buf_print(int max_list_num, char *buf, int buf_size, int ret);

void cpu_print_buf_to_klog(char *buffer)
{
	#define MAX_PRINT_LINE_NUM	1000
	char *p = NULL;
	int cnt = 0;

	do {
		p = strsep(&buffer, "\n");
		if (p)
			pr_info("%s\n", p);
	}while ((p!=NULL) && (cnt++ < MAX_PRINT_LINE_NUM));
}


void cpu_last_load_freq(unsigned int range, int max_list_num)
{
	#define BUF_SIZE (1024 * 1024)

	int ret = 0, cnt = 0;
	int start_cnt = 0, end_cnt = 0;
	char *buf;
	char range_buf[64] = {0,};

	buf = vmalloc(BUF_SIZE);

	cpu_load_freq_history_view_cnt = cpu_load_freq_history_cnt;
	memcpy(cpu_load_freq_history_view, cpu_load_freq_history
				, CPU_LOAD_FREQ_HISTORY_SIZE);
	memcpy(cpu_task_history_view, cpu_task_history
					, CPU_TASK_HISTORY_SIZE);
#if defined (CONFIG_CHECK_WORK_HISTORY)
	memcpy(cpu_work_history_view, cpu_work_history
				, CPU_WORK_HISTORY_SIZE);
#endif

	ret +=  snprintf(buf + ret, BUF_SIZE - ret
		, "=======================================" \
		"========================================" \
		"========================================\n");

	ret +=  snprintf(buf + ret, BUF_SIZE - ret
		, "    TIME       CPU0_F  CPU1_F  CPU_LOCK"
		"    [INDEX]\tCPU0 \tCPU1\tONLINE\tNR_RUN\n");


	cnt = cpu_load_freq_history_view_cnt - 1;
 	ret = show_cpu_load_freq_sub(cnt, range, buf, (BUF_SIZE - ret) ,ret);
	ret +=  snprintf(buf + ret, BUF_SIZE - ret, "\n");

	end_cnt = cnt;
	start_cnt = get_index(end_cnt, cpu_load_history_num, (0 -range + 1) );

	sprintf(range_buf, "%d-%d", start_cnt ,end_cnt);
	set_cpu_load_freq_history_array_range(range_buf);

	ret = check_running_buf_print(max_list_num, buf, (BUF_SIZE - ret), ret);

	cpu_print_buf_to_klog(buf);

	vfree(buf);

}


#define CPU_SHOW_LINE_NUM	50
static ssize_t cpu_load_freq_read(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	char *buf;
	static int cnt = 0, show_line_num = 0,  remained_line = 0;

	if (*ppos < 0 || !count)
		return -EINVAL;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (*ppos == 0) {
		cpu_load_freq_history_view_cnt = cpu_load_freq_history_cnt;
		memcpy(cpu_load_freq_history_view, cpu_load_freq_history
					, CPU_LOAD_FREQ_HISTORY_SIZE);
		memcpy(cpu_task_history_view, cpu_task_history
					, CPU_TASK_HISTORY_SIZE);
	#if defined (CONFIG_CHECK_WORK_HISTORY)
		memcpy(cpu_work_history_view, cpu_work_history
					, CPU_WORK_HISTORY_SIZE);
	#endif
		remained_line = cpu_load_freq_history_show_cnt;
		cnt = cpu_load_freq_history_view_cnt - 1;
		cnt = get_index(cnt, cpu_load_history_num, (0 - remained_line));

		ret +=  snprintf(buf + ret, PAGE_SIZE - ret
			, "=============================" \
			"=============================" \
			"=============================\n");

		ret +=  snprintf(buf + ret, PAGE_SIZE - ret
			, "    TIME   STA  CPUF    CPUF_LOCK  ON_LOCK"
			"  [INDEX]\tCPU0 \tCPU1" \
				  " \tONLINE\tNR_RUN\n");
	}

	if (remained_line >= CPU_SHOW_LINE_NUM) {
		show_line_num = CPU_SHOW_LINE_NUM;
		remained_line -= CPU_SHOW_LINE_NUM;

	} else {
		show_line_num = remained_line;
		remained_line = 0;
	}
	cnt = get_index(cnt, cpu_load_history_num, show_line_num);
 	ret = show_cpu_load_freq_sub(cnt, show_line_num, buf, count,ret);

	if (ret >= 0) {
		if (copy_to_user(buffer, buf, ret)) {
			kfree(buf);
			return -EFAULT;
		}
		*ppos += ret;
	}

	kfree(buf);


	return ret;
}

static ssize_t cpu_load_freq_write(struct file *file,
				      const char __user *user_buf, size_t count,
				      loff_t *ppos)
{
	int show_num = 0;

	show_num = atoi(user_buf);
	if (show_num <= cpu_load_history_num)
		cpu_load_freq_history_show_cnt = show_num;
	else
		return -EINVAL;

	return count;
}

#if defined(CONFIG_SLP_CHECK_BUS_LOAD)
#define CPU_BUS_SHOW_LINE_NUM	10
static ssize_t cpu_bus_load_freq_read(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	char *buf;
	static int cnt = 0, show_line_num = 0,  remained_line = 0;

	if (*ppos < 0 || !count)
		return -EINVAL;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (*ppos == 0) {
		cpu_load_freq_history_view_cnt = cpu_load_freq_history_cnt;
		memcpy(cpu_load_freq_history_view, cpu_load_freq_history
					, CPU_LOAD_FREQ_HISTORY_SIZE);
		memcpy(cpu_task_history_view, cpu_task_history
					, CPU_TASK_HISTORY_SIZE);
	#if defined (CONFIG_CHECK_WORK_HISTORY)
		memcpy(cpu_work_history_view, cpu_work_history
					, CPU_WORK_HISTORY_SIZE);
	#endif

		remained_line = cpu_bus_load_freq_history_show_cnt;
		cnt = cpu_load_freq_history_view_cnt - 1;
		cnt = get_index(cnt, cpu_load_history_num, (0 - remained_line));

		ret +=  snprintf(buf + ret, PAGE_SIZE - ret
			, "=====================================" \
			"======================================" \
			"======================================" \
			"=====================================\n");

		ret +=  snprintf(buf + ret, PAGE_SIZE - ret
			, "    TIME   STA   CPUF   CPUF_LOCK  ON_LOCK"
			"  [INDEX]   CPU0 ( C1/ C2/ C3)   CPU1 ( C1/ C2/ C3)" \
			"\tONLINE\tNR_RUN" \
			"   MIF_FREQ  MIF_UTIL  GPUFREQ  GPU_UTIL\n");

	}

	if (remained_line >= CPU_BUS_SHOW_LINE_NUM) {
		show_line_num = CPU_BUS_SHOW_LINE_NUM;
		remained_line -= CPU_BUS_SHOW_LINE_NUM;

	} else {
		show_line_num = remained_line;
		remained_line = 0;
	}
	cnt = get_index(cnt, cpu_load_history_num, show_line_num);

	ret = show_cpu_bus_load_freq_sub(cnt, show_line_num, buf, count, ret);

	if (ret >= 0) {
		if (copy_to_user(buffer, buf, ret)) {
			kfree(buf);
			return -EFAULT;
		}
		*ppos += ret;
	}

	kfree(buf);


	return ret;
}


static ssize_t cpu_bus_load_freq_write(struct file *file,
				      const char __user *user_buf, size_t count,
				      loff_t *ppos)
{
	int show_num = 0;

	show_num = atoi(user_buf);
	if (show_num <= cpu_load_history_num)
		cpu_bus_load_freq_history_show_cnt = show_num;
	else
		return -EINVAL;

	return count;
}

#endif

#if defined(CONFIG_SLP_BUS_CLK_CHECK_LOAD)
#define CPU_BUS_CLK_SHOW_LINE_NUM	20
static ssize_t cpu_bus_clk_read(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	char *buf;
	static int cnt = 0, show_line_num = 0,  remained_line = 0;

	if (*ppos < 0 || !count)
		return -EINVAL;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (*ppos == 0) {

		cpu_load_freq_history_view_cnt = cpu_load_freq_history_cnt;
		memcpy(cpu_load_freq_history_view, cpu_load_freq_history
						, CPU_LOAD_FREQ_HISTORY_SIZE);

		memcpy(cpu_task_history_view, cpu_task_history
						, CPU_TASK_HISTORY_SIZE);

		remained_line = cpu_bus_clk_load_freq_history_show_cnt;
		cnt = cpu_load_freq_history_view_cnt - 1;
		cnt = get_index(cnt, cpu_load_history_num, (0 - remained_line));

		ret +=  snprintf(buf + ret, PAGE_SIZE - ret
			, "============================================="
			"=============================================="
			"=============================================="
			"============================================="
			"==============================\n");

		ret +=  snprintf(buf + ret, PAGE_SIZE - ret
			, "TIME       C0_F  C1_F  CPU_LOCK"
			"  [INDEX]  CPU0  CPU1" \
			" ONL NR_RUN" \
			"   MIF_FREQ  MIF_UTIL  GPUFQ GPU_UT"
			"    PD1       PD2      CLK1    CLK2        CLK3       CLK4	 CLK5\n");
	}

	if (remained_line >= CPU_BUS_CLK_SHOW_LINE_NUM) {
		show_line_num = CPU_BUS_CLK_SHOW_LINE_NUM;
		remained_line -= CPU_BUS_CLK_SHOW_LINE_NUM;

	} else {
		show_line_num = remained_line;
		remained_line = 0;
	}
	cnt = get_index(cnt, cpu_load_history_num, show_line_num);

	ret = show_cpu_bus_clk_load_freq_sub(cnt, show_line_num, buf,  ret);

	if (ret >= 0) {
		if (copy_to_user(buffer, buf, ret)) {
			kfree(buf);
			return -EFAULT;
		}
		*ppos += ret;
	}

	kfree(buf);


	return ret;
}

static ssize_t cpu_bus_clk_write(struct file *file,
				      const char __user *user_buf, size_t count,
				      loff_t *ppos)
{
	int show_num = 0;

	show_num = atoi(user_buf);
	if (show_num <= cpu_load_history_num)
		cpu_bus_clk_load_freq_history_show_cnt = show_num;
	else
		return -EINVAL;

	return count;
}

static unsigned int checking_clk_index;
static ssize_t check_clk_read(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
 	static char *buf = NULL;
	int buf_size = (PAGE_SIZE * 3);
 	unsigned int ret = 0, size_for_copy = count;
	static unsigned int rest_size = 0;

	unsigned int *clk_gates;
	printk(KERN_ALERT "Inside check_clk_read");
	if (*ppos < 0 || !count)
		return -EINVAL;

	if (*ppos == 0) {
		buf = kmalloc(buf_size, GFP_KERNEL);
		if (!buf)
			return -ENOMEM;

		clk_gates = cpu_load_freq_history_view[checking_clk_index].clk_gates;
		printk(KERN_ALERT "Invoking clk_mon_get_clock_info");
		ret = clk_mon_get_clock_info(clk_gates, buf);

		if (ret <= count) {
			size_for_copy = ret;
			rest_size = 0;
		} else {
			size_for_copy = count;
			rest_size = ret -size_for_copy;
		}
	}else {
		if (rest_size <= count) {
			size_for_copy = rest_size;
			rest_size = 0;
		} else {
			size_for_copy = count;
			rest_size -= size_for_copy;
		}
	}

	if (size_for_copy >  0) {
		int offset = (int) *ppos;
		if (copy_to_user(buffer, buf + offset , size_for_copy)) {
			kfree(buf);
			return -EFAULT;
		}
		*ppos += size_for_copy;
	} else
		kfree(buf);

	return size_for_copy;
}

static ssize_t check_clk_write(struct file *file,
				      const char __user *user_buf, size_t count,
				      loff_t *ppos)
{
	int show_num = 0;

	show_num = atoi(user_buf);
	if (show_num <= cpu_load_history_num)
		checking_clk_index = show_num;
	else
		return -EINVAL;

	return count;
}


#endif


int check_running_buf_print(int max_list_num, char *buf, int buf_size, int ret)
{
	int i = 0;
	int ret_check_valid;
	unsigned long long t;
	unsigned int cpu;

	unsigned int start_cnt = cpu_task_history_show_start_cnt;
	unsigned int end_cnt = cpu_task_history_show_end_cnt;
	unsigned long  msec_rem;

	for (i = 0; i < CPU_NUM ; i++) {
		if (cpu_task_history_show_select_cpu != -1)
			if (i != cpu_task_history_show_select_cpu)
				continue;
		cpu = i;
		ret_check_valid = check_valid_range(cpu, start_cnt, end_cnt);
		if (ret_check_valid < 0)	{
			ret +=  snprintf(buf + ret, buf_size - ret
				, "[ERROR] cpu[%d] Invalid range !!! err=%d\n"
				, cpu, ret_check_valid);
			pr_info("[ERROR] cpu[%d] Invalid range !!! err=%d\n"
				, cpu, ret_check_valid);
			continue;
		}

		clc_process_run_time(i, start_cnt, end_cnt);

		t = total_time;
		msec_rem = do_div(t, 1000000);

		if (end_cnt == start_cnt+1) {
			ret +=  snprintf(buf + ret, buf_size - ret,
			"[%d] TOTAL SECTION TIME = %ld.%ld[ms]\n\n"
			, end_cnt	, (unsigned long)t, msec_rem);
		} else {
			ret +=  snprintf(buf + ret, buf_size - ret,
			"[%d~%d] TOTAL SECTION TIME = %ld.%ld[ms]\n\n"
			, start_cnt+1, end_cnt, (unsigned long)t
			, msec_rem);
		}

		ret += snprintf(buf + ret, buf_size - ret,
			"####################################"
			" CPU %d ##############################\n", i);

		if (cpu_task_history_show_select_cpu == -1)
			ret = view_list(max_list_num, buf, buf_size, ret);
		else if (i == cpu_task_history_show_select_cpu)
			ret = view_list(max_list_num, buf, buf_size, ret);

		if (ret < buf_size - 1)
			ret +=  snprintf(buf + ret, buf_size - ret, "\n\n");
	}

	return ret;
}


static ssize_t check_running_read(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
 	static char *buf = NULL;
	int buf_size = (PAGE_SIZE * 256);
 	unsigned int i, ret = 0, size_for_copy = count;
	int ret_check_valid;
	static unsigned int rest_size = 0;

	unsigned int start_cnt = cpu_task_history_show_start_cnt;
	unsigned int end_cnt = cpu_task_history_show_end_cnt;
	unsigned long  msec_rem;
	unsigned long long t;
	unsigned int cpu;

	if (*ppos < 0 || !count)
		return -EINVAL;

	if (*ppos == 0) {
		buf = vmalloc(buf_size);

		if (!buf)
			return -ENOMEM;

		if ((end_cnt) == (start_cnt + 1)) {
			ret +=  snprintf(buf + ret, buf_size - ret
				, "=======================================" \
				"========================================" \
				"========================================\n");

			ret +=  snprintf(buf + ret, buf_size - ret
				, "    TIME       CPU0_F  CPU1_F  CPU_LOCK"
				"    [INDEX]\tCPU0 \tCPU1 \tONLINE\tNR_RUN\n");

			ret = show_cpu_load_freq_sub((int)end_cnt+2
							, 5, buf, buf_size, ret);
			ret +=  snprintf(buf + ret, buf_size - ret, "\n\n");
		}

		for (i = 0; i < CPU_NUM ; i++) {
			if (cpu_task_history_show_select_cpu != -1)
				if (i != cpu_task_history_show_select_cpu)
					continue;
			cpu = i;
			ret_check_valid = check_valid_range(cpu, start_cnt, end_cnt);
			if (ret_check_valid < 0)	{
				ret +=  snprintf(buf + ret, buf_size - ret
					, "[ERROR] cpu[%d] Invalid range !!! err=%d\n"
					, cpu, ret_check_valid);
				pr_info("[ERROR] cpu[%d] Invalid range !!! err=%d\n"
					, cpu, ret_check_valid);
				continue;
			}

			clc_process_run_time(i, start_cnt, end_cnt);

			t = total_time;
			msec_rem = do_div(t, 1000000);

			if (end_cnt == start_cnt+1) {
				ret +=  snprintf(buf + ret, buf_size - ret,
				"[%d] TOTAL SECTION TIME = %ld.%ld[ms]\n\n"
				, end_cnt	, (unsigned long)t, msec_rem);
			} else {
				ret +=  snprintf(buf + ret, buf_size - ret,
				"[%d~%d] TOTAL SECTION TIME = %ld.%ld[ms]\n\n"
				, start_cnt+1, end_cnt, (unsigned long)t
				, msec_rem);
			}

			ret += snprintf(buf + ret, buf_size - ret,
				"####################################"
				" CPU %d ##############################\n", i);

			if (cpu_task_history_show_select_cpu == -1)
				ret = view_list(INT_MAX, buf, buf_size, ret);
			else if (i == cpu_task_history_show_select_cpu)
				ret = view_list(INT_MAX, buf, buf_size, ret);

			if (ret < buf_size - 1)
				ret +=  snprintf(buf + ret, buf_size - ret, "\n\n");
		}

		if (ret <= count) {
			size_for_copy = ret;
			rest_size = 0;
		} else {
			size_for_copy = count;
			rest_size = ret -size_for_copy;
		}
	}else {
		if (rest_size <= count) {
			size_for_copy = rest_size;
			rest_size = 0;
		} else {
			size_for_copy = count;
			rest_size -= size_for_copy;
		}
	}

	if (size_for_copy >  0) {
		int offset = (int) *ppos;
		if (copy_to_user(buffer, buf + offset , size_for_copy)) {
			vfree(buf);
			return -EFAULT;
		}
		*ppos += size_for_copy;
	} else
		vfree(buf);

	return size_for_copy;
}


static ssize_t check_running_raw_read(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	static char *buf = NULL;
	int buf_size = (PAGE_SIZE * 256);
	unsigned int i, ret = 0, size_for_copy = count;
	int ret_check_valid;
	static unsigned int rest_size = 0;

	unsigned int start_cnt = cpu_task_history_show_start_cnt;
	unsigned int end_cnt = cpu_task_history_show_end_cnt;
	unsigned long  msec_rem;
	unsigned long long t;
	unsigned int cpu;

	if (*ppos < 0 || !count)
		return -EINVAL;

	if (*ppos == 0) {
		buf = vmalloc(buf_size);

		if (!buf)
			return -ENOMEM;

		if ((end_cnt) == (start_cnt + 1)) {
			ret +=  snprintf(buf + ret, buf_size - ret
				, "=======================================" \
				"========================================" \
				"========================================\n");

			ret +=  snprintf(buf + ret, buf_size - ret
				, "    TIME       CPU0_F   CPU1_F  CPU_LOCK"
				"    [INDEX]\tCPU0 \tCPU1 \tONLINE\tNR_RUN\n");

			ret = show_cpu_load_freq_sub((int)end_cnt+2
							, 5, buf, buf_size, ret);
			ret +=  snprintf(buf + ret, buf_size - ret, "\n\n");
		}

		for (i = 0; i < CPU_NUM ; i++) {
			if (cpu_task_history_show_select_cpu != -1)
				if (i != cpu_task_history_show_select_cpu)
					continue;
			cpu = i;
			ret_check_valid = check_valid_range(cpu, start_cnt, end_cnt);
			if (ret_check_valid < 0)	{
				ret +=  snprintf(buf + ret, buf_size - ret
					, "[ERROR] cpu[%d] Invalid range !!! err=%d\n"
					, cpu, ret_check_valid);
				pr_info("[ERROR] cpu[%d] Invalid range !!! err=%d\n"
					, cpu, ret_check_valid);
				continue;
			}

			clc_process_run_time(i, start_cnt, end_cnt);

			t = total_time;
			msec_rem = do_div(t, 1000000);

			ret += snprintf(buf + ret, buf_size - ret, "# %d\n", i);

			if (end_cnt == start_cnt+1) {
				ret +=  snprintf(buf + ret, buf_size - ret,
				"[%d] TOTAL SECTION TIME = %ld.%ld[ms]\n"
				, end_cnt	, (unsigned long)t, msec_rem);
			}

			if (cpu_task_history_show_select_cpu == -1)
				ret = view_list_raw(buf, buf_size, ret);
			else if (i == cpu_task_history_show_select_cpu)
				ret = view_list_raw(buf, buf_size, ret);

		}

		if (ret <= count) {
			size_for_copy = ret;
			rest_size = 0;
		} else {
			size_for_copy = count;
			rest_size = ret -size_for_copy;
		}
	}else {
		if (rest_size <= count) {
			size_for_copy = rest_size;
			rest_size = 0;
		} else {
			size_for_copy = count;
			rest_size -= size_for_copy;
		}
	}

	if (size_for_copy >  0) {
		int offset = (int) *ppos;
		if (copy_to_user(buffer, buf + offset , size_for_copy)) {
			vfree(buf);
			return -EFAULT;
		}
		*ppos += size_for_copy;
	} else
		vfree(buf);

	return size_for_copy;
}

static ssize_t check_running_write(struct file *file,
				      const char __user *user_buf, size_t count,
				      loff_t *ppos)
{
	set_cpu_load_freq_history_array_range(user_buf);

#if defined(CONFIG_CHECK_WORK_HISTORY)
		cpu_work_history_show_select_cpu = cpu_task_history_show_select_cpu;
		cpu_work_history_show_start_cnt= cpu_task_history_show_start_cnt;
		cpu_work_history_show_end_cnt = cpu_task_history_show_end_cnt;
#endif

	return count;
}

#if defined (CONFIG_CHECK_WORK_HISTORY)
static ssize_t check_work_read(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	static char *buf = NULL;
	int buf_size = (PAGE_SIZE * 256);
	unsigned int i, ret = 0, size_for_copy = count;
	int ret_check_valid;
	static unsigned int rest_size = 0;

	unsigned int start_cnt = cpu_work_history_show_start_cnt;
	unsigned int end_cnt = cpu_work_history_show_end_cnt;
	unsigned long  msec_rem;
	unsigned long long t;
	unsigned int cpu;

	if (*ppos < 0 || !count)
		return -EINVAL;

	if (*ppos == 0) {
		buf = vmalloc(buf_size);

		if (!buf)
			return -ENOMEM;

		if ((end_cnt) == (start_cnt + 1)) {
			ret +=  snprintf(buf + ret, buf_size - ret
				, "=======================================" \
				"========================================" \
				"========================================\n");

			ret +=  snprintf(buf + ret, buf_size - ret
				, "    TIME       CPU0_F  CPU1_F   CPU_LOCK"
				"    [INDEX]\tCPU0 \tCPU1 \tONLINE \tNR_RUN\n");
			ret = show_cpu_load_freq_sub((int)end_cnt+2
							, 5, buf, buf_size, ret);
			ret +=  snprintf(buf + ret, buf_size - ret, "\n\n");
		}

		for (i = 0; i < CPU_NUM ; i++) {
			if (cpu_work_history_show_select_cpu != -1)
				if (i != cpu_work_history_show_select_cpu)
					continue;

			cpu = i;
			ret_check_valid = check_work_valid_range(cpu, start_cnt, end_cnt);
			if (ret_check_valid < 0)	{
				ret +=  snprintf(buf + ret, buf_size - ret
					, "[ERROR] cpu[%d] Invalid range !!! err=%d\n"
					, cpu, ret_check_valid);
				pr_info("[ERROR] cpu[%d] Invalid range !!! err=%d\n"
					, cpu, ret_check_valid);
				continue;
			}

			clc_work_run_time(i, start_cnt, end_cnt);

			t = total_time;
			msec_rem = do_div(t, 1000000);

			if (end_cnt == start_cnt+1) {
				ret +=  snprintf(buf + ret, buf_size - ret,
				"[%d] TOTAL SECTION TIME = %ld.%ld[ms]\n\n"
				, end_cnt	, (unsigned long)t, msec_rem);
			} else {
				ret +=  snprintf(buf + ret, buf_size - ret,
				"[%d~%d] TOTAL SECTION TIME = %ld.%ld[ms]\n\n"
				, start_cnt+1, end_cnt, (unsigned long)t
				, msec_rem);
			}

			ret += snprintf(buf + ret, buf_size - ret,
				"######################################"
				" CPU %d ###############################\n", i);

			if (cpu_work_history_show_select_cpu == -1)
				ret = view_workfn_list(buf, buf_size, ret);
			else if (i == cpu_work_history_show_select_cpu)
				ret = view_workfn_list(buf, buf_size, ret);

			if (ret < buf_size - 1)
				ret +=  snprintf(buf + ret, buf_size - ret, "\n\n");
		}

		if (ret <= count) {
			size_for_copy = ret;
			rest_size = 0;
		} else {
			size_for_copy = count;
			rest_size = ret -size_for_copy;
		}
	}else {
		if (rest_size <= count) {
			size_for_copy = rest_size;
			rest_size = 0;
		} else {
			size_for_copy = count;
			rest_size -= size_for_copy;
		}
	}

	if (size_for_copy >  0) {
		int offset = (int) *ppos;
		if (copy_to_user(buffer, buf + offset , size_for_copy)) {
			vfree(buf);
			return -EFAULT;
		}
		*ppos += size_for_copy;
	} else
		vfree(buf);

	return size_for_copy;
}

static ssize_t check_work_write(struct file *file,
				      const char __user *user_buf, size_t count,
				      loff_t *ppos)
{
	set_cpu_load_freq_history_array_range(user_buf);

	cpu_work_history_show_select_cpu = cpu_task_history_show_select_cpu;
	cpu_work_history_show_start_cnt= cpu_task_history_show_start_cnt;
	cpu_work_history_show_end_cnt = cpu_task_history_show_end_cnt;

	return count;
}
#endif


enum {
	SCHED_HISTORY_NUM,
	GOVERNOR_HISTORY_NUM,
};

static int load_analyzer_reset(int mode, int value)
{
	int ret = 0;

	cpu_load_analyzer_exit();

	switch (mode) {

	case SCHED_HISTORY_NUM:
		cpu_task_history_num = value;
		break;

	case GOVERNOR_HISTORY_NUM:
		cpu_load_history_num = value;
		break;

	default:
		break;
	}

	 ret = cpu_load_analyzer_init(NORMAL_MODE);
	 if (ret < 0) {
		 cpu_load_analyzer_init(RECOVERY_MODE);
		return -EINVAL;
	 }

	return 0;
}

#define CPU_LOAD_SCHED_NUM_MIN	1000
#define CPU_LOAD_SCHED_NUM_MAX	1000000
static ssize_t saved_sched_num_write(struct file *file,
				      const char __user *user_buf, size_t count,
				      loff_t *ppos)
{
	int saved_sched_num = 0;

	saved_sched_num = atoi(user_buf);
	if ((saved_sched_num < CPU_LOAD_SCHED_NUM_MIN)
		&& (saved_sched_num > CPU_LOAD_SCHED_NUM_MAX)) {

		pr_info("Wrong range value saved_sched_num = %d\n", saved_sched_num);

		return -EINVAL;
	}

	if (load_analyzer_reset(SCHED_HISTORY_NUM, saved_sched_num) < 0)
		return -EINVAL;

	return count;
}


static int saved_sched_num_read_sub(char *buf, int buf_size)
{
	int ret = 0;

	ret +=  snprintf(buf + ret, buf_size - ret, "%d\n", cpu_task_history_num);

	return ret;
}

static ssize_t saved_sched_num_read(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int size_for_copy;

	size_for_copy = wrapper_for_debug_fs(buffer, count, ppos
						,saved_sched_num_read_sub);

	return size_for_copy;
}



static int check_running_detail_sub(char *buf, int buf_size)
{
	int ret = 0, i = 0, ret_check_valid = 0;
	unsigned int start_cnt = cpu_task_history_show_start_cnt;
	unsigned int end_cnt = cpu_task_history_show_end_cnt;
	unsigned int cpu;

	for (i = 0; i < CPU_NUM; i++) {

		if (cpu_task_history_show_select_cpu != -1)
			if (i != cpu_task_history_show_select_cpu)
				continue;
		cpu = i;
		ret_check_valid = check_valid_range(cpu, start_cnt, end_cnt);
		if (ret_check_valid < 0)	{
			ret +=  snprintf(buf + ret, buf_size - ret
				, "[ERROR] cpu[%d] Invalid range !!! err=%d\n"
				, cpu, ret_check_valid);
			pr_info("[ERROR] cpu[%d] Invalid range !!! err=%d\n"
				, cpu, ret_check_valid);
			continue;
		}

		ret += snprintf(buf + ret, buf_size - ret,
			"##########################" \
			   " CPU %d #########################\n", i);

		ret = process_sched_time_view(i, start_cnt, end_cnt, buf, buf_size, ret);

		ret += snprintf(buf + ret, buf_size - ret ,"\n\n");
	}

	return ret;
}

static ssize_t check_running_detail(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int size_for_copy;

	size_for_copy = wrapper_for_debug_fs(buffer, count, ppos,\
													check_running_detail_sub);

	return size_for_copy;
}


#if defined (CONFIG_CHECK_WORK_HISTORY)
static int check_work_detail_sub(char *buf, int buf_size)
{
	int ret = 0, i = 0, ret_check_valid = 0;
	unsigned int start_cnt = cpu_work_history_show_start_cnt;
	unsigned int end_cnt = cpu_work_history_show_end_cnt;
	unsigned int cpu;

	for (i = 0; i < CPU_NUM; i++) {

		if (cpu_work_history_show_select_cpu != -1)
			if (i != cpu_work_history_show_select_cpu)
				continue;
		cpu = i;
		ret_check_valid = check_work_valid_range(cpu, start_cnt, end_cnt);
		if (ret_check_valid < 0)	{
			ret +=  snprintf(buf + ret, buf_size - ret
				, "[ERROR] cpu[%d] Invalid range !!! err=%d\n"
				, cpu, ret_check_valid);
			pr_info("[ERROR] cpu[%d] Invalid range !!! err=%d\n"
				, cpu, ret_check_valid);
			continue;
		}

		ret += snprintf(buf + ret, buf_size - ret,
			"###########################################"
			"################ CPU %d ######################"
			"##########################################\n", i);

		ret = work_time_list_view(i, start_cnt, end_cnt, buf, buf_size, ret);

		ret += snprintf(buf + ret, buf_size - ret ,"\n\n");
	}

	return ret;
}

static ssize_t check_work_detail(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int size_for_copy;

	size_for_copy = wrapper_for_debug_fs(buffer, count, ppos,\
													check_work_detail_sub);

	return size_for_copy;
}
#endif

char *get_not_lpa_cause(void);

#if defined(CONFIG_CHECK_LPA_CAUSE)
static int not_lpa_cause_check_sub(char *buf, int buf_size)
{
	int ret = 0;

	ret += snprintf(buf + ret, buf_size - ret, "%s\n",  get_not_lpa_cause());

	return ret;
}


static ssize_t not_lpa_cause_check(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int size_for_copy;

	size_for_copy = wrapper_for_debug_fs(buffer, count, ppos,\
													not_lpa_cause_check_sub);

	return size_for_copy;
}
#endif


int value_for_debug;
static int debug_value_read_sub(char *buf, int buf_size)
{
	int ret = 0;

	ret +=  snprintf(buf + ret, buf_size - ret, "%d\n", value_for_debug);

	return ret;
}

static ssize_t debug_value_read(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int size_for_copy;

	size_for_copy = wrapper_for_debug_fs(buffer, count, ppos,\
														debug_value_read_sub);

	return size_for_copy;
}

static ssize_t debug_value_write(struct file *file,
				      const char __user *user_buf, size_t count,
				      loff_t *ppos)
{

	value_for_debug = atoi(user_buf);

	return count;
}

extern unsigned int g_cpu_num_limit;

static struct pm_qos_request pm_qos_min_cpu, pm_qos_max_cpu;
static int la_fixed_cpufreq_value;

static int check_valid_cpufreq(unsigned int cpu_freq)
{
	unsigned int i = 0, found = 0;
	struct cpufreq_frequency_table *table;

	table = cpufreq_frequency_get_table(0);

	for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END); i++) {
		if (table[i].frequency == CPUFREQ_ENTRY_INVALID)
			continue;
		if (cpu_freq == table[i].frequency) {
			found = 1;
			break;
		}
	}

	return found;
}

static int fixed_cpu_freq_read_sub(char *buf, int buf_size)
{
	int ret = 0;

	ret +=  snprintf(buf + ret, buf_size - ret, "%d\n", la_fixed_cpufreq_value);

	return ret;
}

static ssize_t fixed_cpu_freq_read(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int size_for_copy;

	size_for_copy = wrapper_for_debug_fs(buffer, count, ppos,\
												fixed_cpu_freq_read_sub);

	return size_for_copy;
}

static ssize_t fixed_cpu_freq_write(struct file *file,
				      const char __user *user_buf, size_t count,
				      loff_t *ppos)
{
	struct cpufreq_policy *policy;

	int fixed_cpufreq_value;

	pr_info("%s user_buf=%s\n", __FUNCTION__, user_buf);

	if ((user_buf[0] == '-') && (user_buf[1] == '1'))
		fixed_cpufreq_value = -1;
	else
		fixed_cpufreq_value = atoi(user_buf);

	if (fixed_cpufreq_value == -1) {
		la_fixed_cpufreq_value = fixed_cpufreq_value;

		pm_qos_update_request(&pm_qos_min_cpu, 0);
		pm_qos_update_request(&pm_qos_max_cpu, LONG_MAX);
	} else {
		policy = cpufreq_cpu_get(0);

		if (check_valid_cpufreq(fixed_cpufreq_value)!=1) {
			pr_err("Invalid cpufreq : %d\n", fixed_cpufreq_value);
			return -EINVAL;
		}

		la_fixed_cpufreq_value = fixed_cpufreq_value;

		pm_qos_update_request(&pm_qos_min_cpu, fixed_cpufreq_value);
		pm_qos_update_request(&pm_qos_max_cpu, fixed_cpufreq_value);
	}

	return count;
}


static int available_cpu_freq_read_sub(char *buf, int buf_size)
{
	int ret = 0, i = 0;
	struct cpufreq_frequency_table *table;

	table = cpufreq_frequency_get_table(0);

	for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END); i++) {
		if (table[i].frequency == CPUFREQ_ENTRY_INVALID)
			continue;
		ret += snprintf(buf + ret, buf_size - ret, "%d ", table[i].frequency);
	}
	ret += snprintf(buf + ret, buf_size - ret, "\n");

	return ret;
}

static ssize_t available_cpu_freq_read(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int size_for_copy;

	size_for_copy = wrapper_for_debug_fs(buffer, count, ppos,\
												available_cpu_freq_read_sub);

	return size_for_copy;
}

static int saved_load_analyzer_data_read_sub(char *buf, int buf_size)
{
	int ret = 0;

	ret +=  snprintf(buf + ret, buf_size - ret, "%d\n", cpu_load_history_num);

	return ret;
}



static ssize_t saved_load_analyzer_data_read(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int size_for_copy;

	size_for_copy = wrapper_for_debug_fs(buffer, count, ppos
					,saved_load_analyzer_data_read_sub);

	return size_for_copy;
}


#define CPU_LOAD_HISTORY_NUM_MIN	1000
#define CPU_LOAD_HISTORY_NUM_MAX	8000
static ssize_t saved_load_analyzer_data_write(struct file *file,
				      const char __user *user_buf, size_t count,
				      loff_t *ppos)
{
	int history_num;
	history_num = atoi(user_buf);

	if ((history_num < CPU_LOAD_SCHED_NUM_MIN)
		&& (history_num > CPU_LOAD_SCHED_NUM_MAX)) {

		pr_info("Wrong range value cpu_load_history_num = %d\n", history_num);

		return -EINVAL;
	}

	if (load_analyzer_reset(GOVERNOR_HISTORY_NUM, history_num) < 0)
		return -EINVAL;
	return count;
}

static ssize_t last_cpu_load_write(struct file *file,
				      const char __user *user_buf, size_t count,
				      loff_t *ppos)
{
#if 0
	#define MAX_RANGE_NUM	1000
	int range_num, max_list_num;

	char *p1 = NULL, *p_lf = NULL;

	p_lf = strstr(user_buf, "\n");

	p1= strstr(user_buf, "TOP");

	if (p1 == NULL)
		p1= strstr(user_buf, "top");

	if (p_lf != NULL) {
		if (p1 - p_lf > 0)
			p1 = NULL;
	}

	if (p1 != NULL)
		max_list_num = atoi(p1+3);
	else
		max_list_num = INT_MAX;

	range_num = atoi(user_buf);

	if ((range_num > 0) && (range_num <= MAX_RANGE_NUM))
		cpu_last_load_freq(range_num, max_list_num);
	else
		return -EINVAL;
#endif

	return count;
}

static unsigned int checking_phy_start_addr, checking_phy_last_addr, checking_phy_length;
static void __iomem *checking_phy_addr_iomem;
static unsigned int last_checking_value;

enum {
	FAR_MODE,
	UNTIL_MODE,
	ONLY_ONE_MODE,
};

int check_input_string_err(int input_format, const char *str) {

	int ret = 0, value;
	char *p_check_str;

	switch (input_format) {

	case FAR_MODE:
		p_check_str = strstr(str, "0x");
		if (p_check_str == NULL)
			goto err;
		ret = sscanf(p_check_str+2, "%x", &value);
		if (ret < 0)
			goto err;


		p_check_str = strstr(str, "++");
		if (p_check_str == NULL)
			goto err;

		p_check_str = strstr(p_check_str+2, "0x");
		if (p_check_str == NULL)
			goto err;
		ret = sscanf(p_check_str+2, "%x", &value);
		if (ret < 0)
			goto err;

		break;
	case UNTIL_MODE:
		p_check_str = strstr(str, "0x");
		if (p_check_str == NULL)
			goto err;
		ret = sscanf(p_check_str+2, "%x", &value);
		if (ret < 0)
			goto err;


		p_check_str = strstr(str, "--");
		if (p_check_str == NULL)
			goto err;

		p_check_str = strstr(p_check_str+2, "0x");
		if (p_check_str == NULL)
			goto err;
		ret = sscanf(p_check_str+2, "%x", &value);
		if (ret < 0)
			goto err;
		break;

	case ONLY_ONE_MODE:
		p_check_str = strstr(str, "0x");
		if (p_check_str == NULL)
			goto err;
		ret = sscanf(p_check_str+2, "%x", &value);
		if (ret < 0)
			goto err;

	}

	return ret;
err:
	return -1;

}

static ssize_t check_phy_addr_write(struct file *file,
				      const char __user *user_buf, size_t count,
				      loff_t *ppos)
{
	int ret = 0;
	char *p_start_addr = NULL, *p_addr_length=NULL;
	int input_format;

	pr_info("user_buf size =%d\n", strlen(user_buf));

	/* STEP 1 : check address mode */
	if (strstr(user_buf, "++") != NULL)
		input_format = FAR_MODE;
	else if (strstr(user_buf, "--") !=NULL)
		input_format = UNTIL_MODE;
	else
		input_format = ONLY_ONE_MODE;

	/* STEP 2 : check error */
	ret = check_input_string_err(input_format, user_buf);
	if (ret < 0)
		goto err;

	/* STEP 2 : get addr info */
	switch (input_format) {

	case FAR_MODE:
		p_start_addr = strstr(user_buf, "0x");
		sscanf(p_start_addr+2, "%x", &checking_phy_start_addr);

		p_addr_length = strstr(p_start_addr+2, "0x");
		sscanf(p_addr_length+2, "%x", &checking_phy_length);
		break;
	case UNTIL_MODE:
		p_start_addr = strstr(user_buf, "0x");
		sscanf(p_start_addr+2, "%x", &checking_phy_start_addr);

		p_addr_length = strstr(p_start_addr+2, "0x");
		sscanf(p_addr_length+2, "%x", &checking_phy_last_addr);

		checking_phy_length = checking_phy_last_addr -checking_phy_start_addr + 4;
		break;

	case ONLY_ONE_MODE:
		p_start_addr = strstr(user_buf, "0x");
		sscanf(p_start_addr+2, "%x", &checking_phy_start_addr);
		checking_phy_length = 4;
		break;

	}

	return count;

err:
	checking_phy_start_addr = 0;
	checking_phy_length = 0;

	return -EINVAL;
}



int phy_addr_read_sub(char *buf, int buf_size)
{
	int ret = 0, offset = 0;

	checking_phy_addr_iomem = ioremap(checking_phy_start_addr, checking_phy_length);
	if (checking_phy_addr_iomem == NULL)
		goto err;

	for (offset = 0; offset < checking_phy_length ; offset+= 4) {

		pr_info("show_addr=0x%x\n", checking_phy_start_addr + offset);

		last_checking_value = readl(checking_phy_addr_iomem + offset);

		ret +=  snprintf(buf + ret, buf_size - ret, "[0x%08x] 0x%08x\n"
			, checking_phy_start_addr + offset, last_checking_value);
	}

	iounmap(checking_phy_addr_iomem);

	return ret;

err:
	return -EINVAL;

}

static ssize_t check_phy_addr_read(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int size_for_copy;

	size_for_copy = wrapper_for_debug_fs(buffer, count, ppos, phy_addr_read_sub);

	return size_for_copy;
}



static const struct file_operations cpu_load_freq_fops = {
	.owner = THIS_MODULE,
	.read = cpu_load_freq_read,
	.write =cpu_load_freq_write,
};

#if defined(CONFIG_SLP_CHECK_BUS_LOAD)
static const struct file_operations cpu_bus_load_freq_fops = {
	.owner = THIS_MODULE,
	.read = cpu_bus_load_freq_read,
	.write =cpu_bus_load_freq_write,
};
#endif


#if defined(CONFIG_SLP_BUS_CLK_CHECK_LOAD)
static const struct file_operations cpu_bus_clk_fops = {
	.owner = THIS_MODULE,
	.read = cpu_bus_clk_read,
	.write =cpu_bus_clk_write,
};
#endif

static const struct file_operations check_running_fops = {
	.owner = THIS_MODULE,
	.read = check_running_read,
	.write =check_running_write,
};

static const struct file_operations check_running_raw_fops = {
	.owner = THIS_MODULE,
	.read = check_running_raw_read,
	.write =check_running_write,
};

#if defined (CONFIG_CHECK_WORK_HISTORY)
static const struct file_operations check_work_fops = {
	.owner = THIS_MODULE,
	.read = check_work_read,
	.write =check_work_write,
};

static const struct file_operations check_work_detail_fops = {
	.owner = THIS_MODULE,
	.read = check_work_detail,
};
#endif

static const struct file_operations check_running_detail_fops = {
	.owner = THIS_MODULE,
	.read = check_running_detail,
};

#if defined(CONFIG_CHECK_LPA_CAUSE)
static const struct file_operations not_lpa_cause_check_fops = {
	.owner = THIS_MODULE,
	.read = not_lpa_cause_check,
};
#endif

static const struct file_operations saved_sched_num_fops = {
	.owner = THIS_MODULE,
	.read = saved_sched_num_read,
	.write =saved_sched_num_write,
};


static const struct file_operations saved_load_analyzer_data_fops = {
	.owner = THIS_MODULE,
	.read = saved_load_analyzer_data_read,
	.write =saved_load_analyzer_data_write,
};

#if defined(CONFIG_SLP_BUS_CLK_CHECK_LOAD)
static const struct file_operations check_clk_fops = {
	.owner = THIS_MODULE,
	.read = check_clk_read,
	.write =check_clk_write,
};
#endif

static const struct file_operations debug_value_fops = {
	.owner = THIS_MODULE,
	.read = debug_value_read,
	.write =debug_value_write,
};



static const struct file_operations fixed_cpu_freq_fops = {
	.owner = THIS_MODULE,
	.read = fixed_cpu_freq_read,
	.write =fixed_cpu_freq_write,
};

static const struct file_operations available_cpu_freq_fops = {
	.owner = THIS_MODULE,
	.read = available_cpu_freq_read,
};

static const struct file_operations last_cpu_load_fops = {
	.owner = THIS_MODULE,
	.write =last_cpu_load_write,

};

static const struct file_operations check_phy_addr_fops = {
	.owner = THIS_MODULE,
	.read = check_phy_addr_read,
	.write =check_phy_addr_write,
};


static int __init system_load_analyzer_init(void)
{
	int ret = 0;
	struct dentry *d;

	d = debugfs_create_dir("load_analyzer", NULL);
	if (d) {
		if (!debugfs_create_file("cpu_load_freq", 0600
			, d, NULL,&cpu_load_freq_fops))   \
				pr_err("%s : debugfs_create_file, error\n", "cpu_load_freq");

#if defined(CONFIG_SLP_CHECK_BUS_LOAD)
		if (!debugfs_create_file("cpu_bus_load_freq", 0600
			, d, NULL,&cpu_bus_load_freq_fops))   \
				pr_err("%s : debugfs_create_file, error\n", "cpu_bus_load_freq");
#endif
		if (!debugfs_create_file("check_running", 0600
			, d, NULL,&check_running_fops))   \
				pr_err("%s : debugfs_create_file, error\n", "check_running");

		if (!debugfs_create_file("check_running_raw", 0600
			, d, NULL,&check_running_raw_fops))   \
				pr_err("%s : debugfs_create_file, error\n", "check_running_raw");
#if defined (CONFIG_CHECK_WORK_HISTORY)
		if (!debugfs_create_file("check_work", 0600
			, d, NULL,&check_work_fops))   \
				pr_err("%s : debugfs_create_file, error\n", "check_work");
		if (!debugfs_create_file("check_work_detail", 0600
			, d, NULL,&check_work_detail_fops))   \
				pr_err("%s : debugfs_create_file, error\n", "check_work_detail");
#endif
		if (!debugfs_create_file("saved_sched_num", 0600
			, d, NULL,&saved_sched_num_fops))   \
				pr_err("%s : debugfs_create_file, error\n", "saved_sched_num");

		if (!debugfs_create_file("saved_load_analyzer_data_num", 0600
			, d, NULL,&saved_load_analyzer_data_fops))   \
				pr_err("%s : debugfs_create_file, error\n", "saved_load_analyzer_data_num");
		if (!debugfs_create_file("check_running_detail", 0600
			, d, NULL,&check_running_detail_fops))   \
				pr_err("%s : debugfs_create_file, error\n", "check_running_detail");

#if defined(CONFIG_CHECK_LPA_CAUSE)
		if (!debugfs_create_file("not_lpa_cause_check", 0600
			, d, NULL,&not_lpa_cause_check_fops))   \
				pr_err("%s : debugfs_create_file, error\n", "not_lpa_cause_check");
#endif

#if defined(CONFIG_SLP_BUS_CLK_CHECK_LOAD)
		if (!debugfs_create_file("cpu_bus_clk_freq", 0600
			, d, NULL,&cpu_bus_clk_fops))   \
				pr_err("%s : debugfs_create_file, error\n", "cpu_bus_clk_freq");

		if (!debugfs_create_file("check_clk", 0600
			, d, NULL,&check_clk_fops))   \
				pr_err("%s : debugfs_create_file, error\n", "check_clk");
#endif
		if (!debugfs_create_file("debug_value", 0600
			, d, NULL,&debug_value_fops))   \
				pr_err("%s : debugfs_create_file, error\n", "debug_value");


		if (!debugfs_create_file("fixed_cpu_freq", 0600
			, d, NULL,&fixed_cpu_freq_fops))   \
				pr_err("%s : debugfs_create_file, error\n", "fixed_cpu_freq");

		if (!debugfs_create_file("available_cpu_freq", 0400
			, d, NULL,&available_cpu_freq_fops))   \
				pr_err("%s : debugfs_create_file, error\n", "available_cpu_freq");

		if (!debugfs_create_file("last_cpu_load", 0200
			, d, NULL,&last_cpu_load_fops))   \
				pr_err("%s : debugfs_create_file, error\n", "last_cpu_load");

		if (!debugfs_create_file("check_phy_addr", 0600
			, d, NULL,&check_phy_addr_fops))   \
				pr_err("%s : debugfs_create_file, error\n", "check_phy_addr");
	}

	pm_qos_add_request(&pm_qos_min_cpu, PM_QOS_CPU_FREQ_MIN, 0);

	pm_qos_add_request(&pm_qos_max_cpu, PM_QOS_CPU_FREQ_MAX, LONG_MAX);

	if (cpu_load_analyzer_init(NORMAL_MODE) != 0)
		pr_info("[%s] cpu_load_analyzer_init ERROR", __func__);

	return ret;
}

static void __exit system_load_analyzer_exit(void)
{
		cpu_load_analyzer_exit();
}

MODULE_AUTHOR("Yong-U, Baek <yu.baek@samsung.com>");
MODULE_DESCRIPTION("'SLP power debuger");
MODULE_LICENSE("GPL");


module_init(system_load_analyzer_init);
module_exit(system_load_analyzer_exit);

