#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_bus.h"

#include "can_net.h"
#include "can_core.h"
#include "xprintf.h"

#include <string.h>

//#define DEBUG

#ifdef DEBUG
#define assert(cond) \
        if (!(cond)) { \
                xprintf("%s failed in %d line\n", #cond, __LINE__ ); \
                *(uint8_t *)0x08000001 = 0x00; \
        }
#else
#define assert(cond) (void)(cond)
#endif

static struct can_frame can_pack;
static struct net_frame net_pack;
/* Actually all nodes have only 16bit ID */
static uint32_t node_id;
static struct net_state net_st = {0};
static struct net_rx_state net_rx_st = {0};
static struct net_tx_state net_tx_st = {0};

static void
embed_net_pack(uint16_t dest_id) {
        can_pack.can_id = dest_id;
        can_pack.can_dlc = CAN_MAX_DLEN;
        memcpy(&(can_pack.data), &net_pack, CAN_MAX_DLEN);

        return;
}

static uint32_t
unload_net_pack() {
        memcpy(&net_pack, &(can_pack.data), CAN_MAX_DLEN);

        return net_pack.source_id;
}

static void
net_header_fill(uint16_t id, uint8_t type, uint8_t size) {
        net_pack.source_id = id;
        net_pack.frame_type = type;
        net_pack.frame_size = size;

        return;
}

static uint8_t
net_add_id(uint32_t id) {
        int i;

        for (i = 0; i < MAX_NODE; i++) {
                if (net_st.nodes[i] == 0) {
                        net_st.nodes[i] = id;
                        net_st.active_node++;
                        return 0;
                } else {
                        if (net_st.nodes[i] == id) {
                                return 1;
                        }
                }
        }
        return 1;
}

static uint8_t
net_del_id(uint32_t id) {
        int i = 0;

        for (i = 0; i < MAX_NODE; i++) {
                if (net_st.nodes[i] == id) {
                        net_st.nodes[i] = 0x0000;
                        net_st.active_node -= 1;
                        return 0;
                }
        }

        return 1;
}

static uint8_t
net_is_node_active(uint32_t id) {
        int i = 0;

        for (i = 0; i < MAX_NODE; i++) {
                if (net_st.nodes[i] == id) {
                        return 0;
                }
        }

        return 1;
}

uint8_t
net_node_num() {

        return net_st.active_node;
}

uint32_t
net_get_id(uint8_t id_num) {
        int i = 0;
        uint8_t id_counter = 0;

        for (i = 0; i < MAX_NODE; i++) {
                if (net_st.nodes[i] != 0) {
                        if (id_num == id_counter) {
                                return net_st.nodes[i];
                        }
                        id_counter++;
                }
        }

        return 0;
}

void
net_start() {
        net_st.poll_en = 1;

        return;
}

uint8_t
net_wait_join() {
        while (net_st.status != ST_READY);

        return net_st.active_node;
}

uint8_t
net_init() {
        int i;

        node_id = LL_GetUID_Word0();
        node_id |= node_id >> 8;
        node_id &= 0xFFFF;
        xprintf("CoreID 0x%04X\n", node_id);

        assert(!can_core_config(CAN_50KBPS));
        assert(!can_set_id(node_id, BRDCST_ID));

        net_st.active_node = 0;
        net_st.status = ST_RESET;
        net_st.sync_timeout = 0;
        net_st.wait_ping = 0;

        for (i = 0; i < MAX_NODE; i++) {
                net_st.nodes[i] = 0x00;
        }

        return 0;
}

static uint8_t
net_fsm() {
        uint32_t new_id;
        uint32_t fr_type;

        if (!can_check_new_msg())
                return 0;
        can_read_msg(&can_pack);
        new_id = unload_net_pack();

        fr_type = net_pack.frame_type;

        if (fr_type == NET_JOIN_ACK) {
                net_add_id(new_id);
                //xprintf("Get acknowledgment from ID: 0x%04x\n", new_id);
        }
        if (fr_type == NET_JOIN) {
                net_header_fill(node_id, NET_JOIN_ACK, 0);
                embed_net_pack(new_id);
                assert(!can_send_msg(&can_pack));
                //xprintf("New node! ID: 0x%04x\n", new_id);
        }
        if (fr_type == NET_SYNCED) {
                net_add_id(new_id);
                xprintf("Node with ID: 0x%04x synced!\n", new_id);
        }
        if (fr_type == NET_PING) {
                net_header_fill(node_id, NET_PING_OK, 0);
                embed_net_pack(new_id);
                assert(!can_send_msg(&can_pack));
        }

        return fr_type;
}

static uint8_t
net_ping_check(uint8_t fsm_ret) {
        uint8_t i = 0;
        static uint8_t flag = 0;

        if (!net_st.active_node) {
                return 0;
        }

        if (!net_st.wait_ping) {
                for (i = 0; i < MAX_NODE; i++) {
                        if (net_st.nodes[i] != 0) {
                                net_st.wait_ping = net_st.nodes[i];
                        }
                }
                flag = 1;
        }

        if (fsm_ret == NET_PING_OK) {
                net_st.sync_timeout = 0;
                while (net_st.nodes[i] != net_st.wait_ping) {
                        i++;
                }
                i++;
                while (net_st.nodes[i] == 0) {
                        i = (i == MAX_NODE) ? 0 : i + 1;
                }
                net_st.wait_ping = net_st.nodes[i];
                flag = 1;
        }

        if (net_st.sync_timeout >= PING_PERIOD_MS) {
                xprintf("Node ID 0x%04x has gone\n", net_st.wait_ping);
                net_st.sync_timeout = 0;
                net_del_id(net_st.wait_ping);
                if (!net_st.active_node) {
                        net_st.wait_ping = 0;
                        return 0;
                }
                for (i = 0; i < MAX_NODE; i++) {
                        if (net_st.nodes[i] != 0) {
                                net_st.wait_ping = net_st.nodes[i];
                        }
                }
                flag = 1;
        }

        if (flag && (net_st.sync_timeout > PING_DELAY)) {
                //xprintf("Ping! ID: 0x%04x\n", net_st.wait_ping);
                net_header_fill(node_id, NET_PING, 0);
                embed_net_pack(net_st.wait_ping);
                assert(!can_send_msg(&can_pack));
                flag = 0;
        }

        net_st.sync_timeout++;

        return 0;
}

uint8_t
net_poll() {
        uint8_t ret = 0;

        if (!(net_st.poll_en))
                return 0;

        switch (net_st.status) {
        case ST_RESET: {
                net_header_fill(node_id, NET_JOIN, 0);
                embed_net_pack(BRDCST_ID);
                assert(!can_send_msg(&can_pack));
                net_st.status = ST_WAIT_JOIN;
                break;
        }
        case ST_WAIT_JOIN: {
                ret = net_fsm();
                if (ret == NET_JOIN_ACK) {
                        net_st.status = ST_JOINED;
                }
                break;
        }
        case ST_JOINED: {
                ret = net_fsm();
                if (ret != NET_JOIN_ACK) {
                        net_st.sync_timeout++;
                }
                if (net_st.sync_timeout >= SYNC_TIMEOUT_MAX) {
                        net_st.status = ST_SYNCED;
                        net_st.sync_timeout = 0;
                }
                break;
        }
        case ST_SYNCED: {
                ret = net_fsm();
                net_header_fill(node_id, NET_SYNCED, 0);
                embed_net_pack(BRDCST_ID);
                assert(!can_send_msg(&can_pack));
                net_st.status = ST_READY;
                break;
        }
        case ST_READY: {
                ret = net_fsm();
                net_ping_check(ret);
                break;
        }
        case ST_RECV_INIT: {
                ret = net_fsm();
                net_ping_check(ret);
                // Handle the first frame
                if (ret == NET_SEND_INIT) {
                        net_rx_st.size = net_pack.frame_size;
                        net_rx_st.source_id = net_pack.source_id;
                        memcpy(net_rx_st.buffer + net_rx_st.offset,
                               net_pack.payload, FRAME_PCAP);
                        net_rx_st.offset = FRAME_PCAP;
                        net_st.status = ST_RECV_INIT_ACK;
                        net_st.recv_wait_ms = 0;
                }
                // Constant timeout check
                net_st.recv_wait_ms++;
                if (net_st.recv_wait_ms >= RECV_TIMEOUT_MS) {
                        net_rx_st.is_rx_filled = 1;
                        net_rx_st.is_wait = 0;
                        net_rx_st.offset = 0;
                        net_rx_st.is_timeout = 1;

                        net_st.recv_wait_ms = 0;
                        net_st.status = ST_READY;
                }
                break;
        }
        case ST_RECV_INIT_ACK: {
                ret = net_fsm();
                net_ping_check(ret);

                net_header_fill(node_id, NET_SEND_INIT_ACK, 0);
                embed_net_pack(net_rx_st.source_id);
                assert(!can_send_msg(&can_pack));
                net_st.status = ST_RECV;

                break;
        }
        case ST_RECV: {
                ret = net_fsm();
                net_ping_check(ret);

                // Handle successively the following frames
                if ((ret == NET_SEND) &&
                    (net_pack.source_id == net_rx_st.source_id)) {
                        net_st.recv_wait_ms = 0;
                        memcpy(net_rx_st.buffer + net_rx_st.offset,
                               net_pack.payload, FRAME_PCAP);
                        net_rx_st.offset += FRAME_PCAP;
                        net_st.status = ST_RECV_ACK;
                }
                // Constant timeout check
                net_st.recv_wait_ms++;
                if (net_st.recv_wait_ms >= RECV_TIMEOUT_MS) {
                        net_rx_st.is_rx_filled = 1;
                        net_rx_st.is_wait = 0;
                        net_rx_st.offset = 0;
                        net_rx_st.source_id = 0;
                        net_rx_st.is_timeout = 1;

                        net_st.recv_wait_ms = 0;
                        net_st.status = ST_READY;
                }

                break;
        }
        case ST_RECV_ACK: {
                ret = net_fsm();
                net_ping_check(ret);

                net_header_fill(node_id, NET_SEND_ACK, 0);
                embed_net_pack(net_rx_st.source_id);
                assert(!can_send_msg(&can_pack));

                net_st.status = ST_RECV;
                // Check for the completion of data receiving
                if (net_rx_st.offset >= net_rx_st.size) {
                        net_rx_st.is_rx_filled = 1;
                        net_rx_st.is_wait = 0;
                        net_rx_st.offset = 0;

                        net_st.recv_wait_ms = 0;
                        net_st.status = ST_READY;
                }

                break;
        }
        case ST_SEND_INIT: {
                ret = net_fsm();
                net_ping_check(ret);

                memcpy(net_pack.payload, net_tx_st.buffer, FRAME_PCAP);
                net_header_fill(node_id, NET_SEND_INIT, net_tx_st.size);
                embed_net_pack(net_tx_st.rec_id);
                assert(!can_send_msg(&can_pack));

                net_tx_st.is_timeout = 0;
                net_tx_st.offset = FRAME_PCAP;
                net_st.status = ST_SEND_INIT_ACK;
                net_st.send_wait_ms = 0;

                break;
        }
        case ST_SEND_INIT_ACK: {
                ret = net_fsm();
                net_ping_check(ret);

                if (ret == NET_SEND_INIT_ACK) {
                        net_st.send_wait_ms = 0;
                        net_st.status = ST_SEND;
                }
                // Constant timeout check
                net_st.send_wait_ms++;
                if (net_st.send_wait_ms >= SEND_TIMEOUT_MS) {
                        net_tx_st.is_tx_done = 1;
                        net_tx_st.is_timeout = 1;
                        net_st.send_wait_ms = 0;
                        net_st.status = ST_READY;
                }

                break;
        }
        case ST_SEND: {
                ret = net_fsm();
                net_ping_check(ret);

                memcpy(net_pack.payload,
                       net_tx_st.buffer + net_tx_st.offset, FRAME_PCAP);
                net_tx_st.offset += FRAME_PCAP;
                net_header_fill(node_id, NET_SEND, FRAME_PCAP);
                embed_net_pack(net_tx_st.rec_id);
                assert(!can_send_msg(&can_pack));
                net_st.status = ST_SEND_ACK;

                break;
        }
        case ST_SEND_ACK: {
                ret = net_fsm();
                net_ping_check(ret);

                if (ret == NET_SEND_ACK) {
                        if (net_tx_st.offset >= net_tx_st.size) {
                                net_st.send_wait_ms = 0;
                                net_tx_st.is_tx_done = 1;
                                net_st.status = ST_READY;
                        } else {
                                net_st.status = ST_SEND;
                        }
                }
                // Constant timeout check
                net_st.send_wait_ms++;
                if (net_st.send_wait_ms >= SEND_TIMEOUT_MS) {
                        net_tx_st.is_tx_done = 1;
                        net_tx_st.is_timeout = 1;
                        net_st.send_wait_ms = 0;
                        net_st.status = ST_READY;
                }

                break;
        }
        }

        return 0;
}

uint32_t
net_recv(uint8_t *buf, uint8_t recv_flag) {
        //Return 0 if data is requested but is not obtained
        if (net_rx_st.is_wait || net_st.status != ST_READY) {
                return 0;
        }
        //Return data obtained
        if (net_rx_st.is_rx_filled) {
                net_rx_st.is_rx_filled = 0;
                if (net_rx_st.is_timeout)
                        return 1;
                return (net_rx_st.source_id << 16) | net_rx_st.size;
        }
        //Request in synchronous mode
        if (recv_flag == RECV_BLOCK) {
                net_st.status = ST_RECV_INIT;
                net_rx_st.is_wait = 1;
                net_rx_st.buffer = buf;
                while (net_rx_st.is_wait);
                net_rx_st.is_rx_filled = 0;
                if (net_rx_st.is_timeout)
                        return 1;
                return (net_rx_st.source_id << 16) | net_rx_st.size;
        }
        //Requst in asynchronous mode
        if (recv_flag == RECV_POLL) {
                net_st.status = ST_RECV_INIT;
                net_rx_st.is_wait = 1;
                net_rx_st.buffer = buf;
                net_rx_st.is_rx_filled = 0;
                return 0;
        }

        return 0;
}

uint16_t
net_send(uint8_t *buf, uint8_t size, uint16_t recipient_id) {
        //Don't allow net_send to be executed while rx is busy
        if (net_rx_st.is_wait || net_st.status != ST_READY)
                return RET_INVAL;
        /* The data can be transmitted only to active node. The active node
           list is sustained automatically by ping mechanism */
        if (net_is_node_active(recipient_id) || !size)
                return RET_INVAL;

        net_tx_st.rec_id = recipient_id;
        net_tx_st.size = size;
        net_tx_st.buffer = buf;
        net_st.status = ST_SEND_INIT;
        net_tx_st.is_tx_done = 0;

        while (!net_tx_st.is_tx_done);
        if (net_tx_st.is_timeout)
                return RET_TIMEOUT;

        return size;
}
#undef assert
