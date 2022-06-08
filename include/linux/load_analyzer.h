
#ifndef _LINUX_SYSTEM_LOAD_ANALYZER_H
#define _LINUX_SYSTEM_LOAD_ANALYZER_H

#define CPU_NUM	NR_CPUS
#define CONFIG_CHECK_WORK_HISTORY	1
#define CONFIG_SLP_CHECK_BUS_LOAD 	1

enum {
	NR_RUNNING_TASK,
	MIF_BUS_FREQ,
	MIF_BUS_LOAD,
	GPU_FREQ,
	GPU_UTILIZATION,
};

struct saved_load_factor_tag {
	unsigned int nr_running_task;
	unsigned int mif_bus_freq;
	unsigned int mif_bus_load;
	unsigned int gpu_freq;
	unsigned int gpu_utilization;
};

extern struct saved_load_factor_tag	saved_load_factor;

extern bool cpu_task_history_onoff;

void store_external_load_factor(int type, unsigned int data);

void store_cpu_load(unsigned int cpufreq[], unsigned int cpu_load[]);

void cpu_load_touch_event(unsigned int event);

void __slp_store_task_history(unsigned int cpu, struct task_struct *task);

static inline void slp_store_task_history(unsigned int cpu
					, struct task_struct *task)
{
	__slp_store_task_history(cpu, task);
}

u64  get_load_analyzer_time(void);

void __slp_store_work_history(struct work_struct *work, work_func_t func
						, u64 start_time, u64 end_time);

void store_killed_task(struct task_struct *tsk);


void cpu_last_load_freq(unsigned int range, int max_list_num);

extern int value_for_debug;

#endif
