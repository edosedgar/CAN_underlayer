#include "stm32f0xx_ll_system.h"
#include "can_api.h"
#include "can_net.h"
#include "xprintf.h"

#include <string.h>
#include <stdio.h>

/**
 Please don't edit this file unless you
 know what you do
 */

//Private stuff

#define ASSERT(cond)     \
    do {                 \
        __disable_irq(); \
        if (!(cond))       \
            while (1);   \
        __enable_irq();  \
    } while (0)

struct cbcks_s cbcks;

// Public stuff

void
can_do_setup(void (*setup_routine)(void)) {
        uint8_t array[] = "Hello world!\n";
        uint8_t array2[20];
        int ret = 0, num_node, id = 0;

        ASSERT(setup_routine);
        memset(&cbcks, 0x00, sizeof(cbcks));
        net_init();
        net_start();
        if (net_wait_join())
                xprintf("Node is joined. Active nodes: %d\n", net_node_num());
        /*while (!(0xFF & ret)) {
                num_node = net_node_num();
                if (num_node) {
                        id = net_get_id(0);
                        ret = net_send(array, sizeof(array), id);
                        xprintf("net_send ended up with %04X code \n", ret);
                }
        }
        return;*/
        while (1) {
                ret = net_recv(array2, RECV_BLOCK);
                xprintf("Msg received(ret = %02X): %s\n", ret, array2);
                memset(array2, 0, 20);
        }

        cbcks.setup = setup_routine;
}

void
can_do_loop(void (*loop_routine)(void)) {
        ASSERT(loop_routine);
        cbcks.loop = loop_routine;
        (void)cbcks.loop();
}

void
can_add_get(const char* sensor_name,
                 int (*get_routine)(uint32_t* buf)) {
        ASSERT(cbcks.sens_num == MAX_SENSORS_NUMBER);
        ASSERT(get_routine);
        cbcks.sens_cbcks[cbcks.sens_num].get = get_routine;
        (void)strncpy(cbcks.sens_cbcks[cbcks.sens_num].sens_name,
                      sensor_name,
                      MAX_NAME_LENGTH);
        cbcks.sens_num++;
}

// Private

void
can_get(char* result) {
        int char_num = 0;
        int temp;
        sprintf(result + char_num,
                "{"
                    "\"%d\":"
                    "{ %n", IDENTIFICATOR, &temp);
        char_num += temp;
        for (int i = 0; i < cbcks.sens_num; i++) {
                sprintf(result + char_num,
                                "{"
                                    "\"sens_name\": \"%s\","
                                    "\"rawdata\": [ %n",
                        cbcks.sens_cbcks[i].sens_name, &temp);
                char_num += temp;
                uint32_t buf[MAX_BUFFER_LENGTH] = {0}; //< Sauron's eye is watching ya
                int buf_size = cbcks.sens_cbcks[i].get(buf);
                for (int j = 0; j < buf_size; j++) {
                        sprintf(result + char_num, "%lu, %n", buf[j], &temp);
                        char_num += temp;
                }
                sprintf(result + char_num,          "]"
                                "}%n" , &temp);
                char_num += temp;
        }
        sprintf(result + char_num,
                    "} "
                "}");
}

#undef ASSERT
