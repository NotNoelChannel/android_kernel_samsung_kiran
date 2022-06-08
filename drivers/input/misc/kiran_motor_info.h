#ifndef _MOTOR_NAME_H
#define _MOTOR_NAME_H

extern unsigned int system_rev;

enum HW_REV{
	HW_REV_EMUL=0x3,
	HW_REV_00 = 0x0,
	HW_REV_01 = 0x1,
	HW_REV_02 = 0X2,
	HW_REV_03 = 0x4,
};

static char * get_motor_name(void)
{
	switch(system_rev){
		case HW_REV_EMUL:
		case HW_REV_00:
		case HW_REV_01:
		case HW_REV_02:
		case HW_REV_03:
		default :  return "YB0934SMA";
	}
}

#endif
