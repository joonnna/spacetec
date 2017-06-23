#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"


MODULE_AUTHOR("THE MAIN MAN");
MODULE_DESCRIPTION("YAMAN");
MODULE_LICENSE("GPL");


struct hal_counter {
    hal_s32_t *val;
};

typedef struct hal_counter hal_counter_t;

static hal_counter_t *entries;

static int comp_id;


static void
testfunc (void *arg, long period);

static int
export_shit (hal_counter_t *entry)
{   
    int ret;

    printf("%d\n", __LINE__);
    ret = hal_pin_s32_new("0.val", HAL_OUT, &entry->val, comp_id);
    if (ret != 0)
        return ret;

    printf("%d\n", __LINE__);
    ret = hal_export_funct("testfunc", testfunc, entries, 0, 0, comp_id);
    if (ret != 0)
        return ret;

    return 1;
}


int
rtapi_app_main(void)
{
    int a, b, res, ret;
    hal_counter_t *entry;

    comp_id = hal_init("YEP");
    if (comp_id < 0)
    {
        printf("%d\n",__LINE__);
        rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: hal_init() failed\n");
        hal_exit(comp_id);
        return -1;
    }
    
    printf("%d\n", __LINE__);
    entries = hal_malloc(sizeof(hal_counter_t));
    if (entries == NULL) {
        printf("Failed to alloc\n");
        hal_exit(comp_id);
        return -1;
    }
    printf("%d\n", __LINE__);
    ret = export_shit(entries);
    if (ret != 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: exporting failed\n");
        hal_exit(comp_id);
        return -1;
    }

    printf("%d\n", __LINE__);
    
    hal_ready (comp_id);
    
    return 0;
}


void
rtapi_app_exit(void)
{
    hal_exit(comp_id);
}


static void
testfunc (void *arg, long period)
{
    hal_counter_t *args;

    args = arg;

    *args->val++;
}
