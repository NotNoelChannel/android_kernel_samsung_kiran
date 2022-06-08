
#ifndef _SPRD27X3_FUELGAUGE4SAMSUNG_H_
#define _SPRD27X3_FUELGAUGE4SAMSUNG_H_

#include <linux/types.h>
#include <linux/battery/sec_charger.h>
#include <linux/battery/sec_fuelgauge.h>
#include <linux/sprd_battery_common.h>



//shashi.ray
#define VG_COMP_NR 4

typedef struct {
        int data[VG_COMP_NR];
} vg_comp_data_t;

typedef struct {
        union {
                int x;
                int voltage;
                int soc;
        };
        union {
                int y;
                int temperature;
        };
        union {
                int data[VG_COMP_NR];
                vg_comp_data_t vg_comp;
                int z;
                int offset;
        };
} data_point_t;



struct battery_data_t {
    struct sprd_battery_platform_data *pdata;
};

#endif

