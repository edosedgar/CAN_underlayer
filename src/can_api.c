#include <stm32f0xx_ll_system.h>
#include <string.h>
#include "can_api.h"
#include "can_core.h"

/**
 Please don't edit this file unless you
 know what you do
 */

//Private stuff

#define ASSERT(cond)     \
    do {                 \
        __disable_irq(); \
        if (cond)        \
            while (1);   \
    } while (0)

struct cbcks_s cbcks;

// Public stuff

void can_init(void)
{
    memset(&cbcks, 0x00, sizeof(cbcks));
    can_core_config();
}

void can_set_setup(void (*setup_routine)(void))
{
    ASSERT(setup_routine);
    cbcks.setup = setup_routine;
}

void can_set_loop(void (*loop_routine)(void))
{
    ASSERT(loop_routine);
    cbcks.loop = loop_routine;
}

void can_add_get(const char* sensor_name,
                 int (*get_routine)(uint32_t* buf))
{
    ASSERT(cbcks.sens_num == MAX_SENSORS_NUMBER);
    ASSERT(get_routine);
    cbcks.sens_cbcks[cbcks.sens_num].get = get_routine;
    (void)strncpy(cbcks.sens_cbcks[cbcks.sens_num].sens_name,
                  sensor_name,
                  MAX_STRING_LENGTH);
    cbcks.sens_num++;
}

void can_do_loop(void)
{
    (void)cbcks.loop();
}

#undef ASSERT
