#ifndef _LINUX_SHRINKMEM_H
#define _LINUX_SHRINKMEM_H

extern int sysctl_shrink_memory;

extern int sysctl_shrinkmem_handler(struct ctl_table *table, int write,
			void __user *buffer, size_t *length, loff_t *ppos);


#endif
