#include "stm32f0xx_ll_system.h"

#include "can_net.h"
#include "can_core.h"
#include "xprintf.h"

#define DEBUG 1

#define dprintf(...)                  \
        if (DEBUG)                    \
                xprintf(__VA_ARGS__); \

uint8_t
net_init(uint32_t node_id, uint32_t brdcst_id) {
        if (can_core_config(CAN_5KBPS))
                dprintf("can_core_config failed\n");
        if (can_set_id(node_id, brdcst_id))
                dprintf("can_set_id failed\n");

        struct can_frame canMsg2;
        xprintf("Waiting for new message\n");
        while (!can_check_new_msg());
        can_read_msg(&canMsg2);
        xprintf("New message! Id: %x msg: %s\n", canMsg2.can_id, canMsg2.data);

        return 0;
}

uint8_t
net_join() {

        return 0;
}

uint8_t
net_ping_nodes() {

        return 0;
}

uint32_t
net_recv(uint32_t* buf, uint8_t blocked) {

        return 0;
}

uint8_t
net_send(uint32_t* buf, uint8_t size, uint8_t ack, uint32_t recipient_id) {

        return 0;
}
