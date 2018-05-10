#ifndef CAN_NET_H
#define CAN_NET_H

#define JOIN_PHASE_1 1
#define JOIN_PHASE_2 2

#define BRDCST_ID 0x71
#define MAX_NODE 0x80

#define SYNC_TIMEOUT_MAX 3000
#define PING_PERIOD_MS   1000
#define PING_DELAY       100

enum NODE_STATUS_T {
        ST_RESET,
        ST_WAIT_JOIN,
        ST_JOINED,
        ST_SYNCED,
        ST_READY,
        ST_READ,
        ST_WRITE
};

enum FRAME_T {
        NET_JOIN        = 0x00,
        NET_JOIN_ACK    = 0x01,
        NET_SYNCED      = 0x02,
        NET_PING        = 0x03,
        NET_PING_OK     = 0x04
};

struct net_frame {
        uint8_t source_id;
        uint8_t frame_type;
        uint8_t frame_size;
        uint8_t payload[5];
} __attribute__((packed));

struct net_state {
        uint8_t active_node;
        uint8_t status;
        uint8_t nodes[MAX_NODE];
        uint32_t poll_en;
        uint32_t sync_timeout;
        uint32_t wait_ping;
};

/*
 * Before start, call this subroutine to init
 * physical underlayer CAN
 */
uint8_t net_init();

/*
 * Before data communication, call this subroutine to
 * run network show
 */
void net_start();

/*
 * The data sent to the node (ID) can be obtained via
 * that subroutine.
 * The *blocked* parameter specifies whether it is required
 * to wait for data or not
 * The highest two bytes of return value are *source ID*,
 * the lowest two bytes are *size of payload*
 *          | 16bit | 16bit |
 * RETVAL = |  ID   | SIZE  |
 */
uint32_t net_recv(uint32_t *buf, uint8_t blocked);

/*
 * To send data to recipient ID.
 * Size units are bytes regardless of 32bitness of buffer
 * If ack is flagges, subroutine will wait for receive
 * confirmation from recipient endpoint
 */
uint8_t net_send(uint32_t *buf, uint8_t size, uint8_t ack,
                 uint32_t recipient_id);

/*
 * This net_poll implements the main algorithm of communication,
 * it is responsible for everything. It is expected to be called
 * from 1ms timer
 */
uint8_t net_poll();

/*
 * Returns the number of active nodes in network
 */
uint8_t net_node_num();

/*
 * Call this routine to make sure that the node is joined network
 */
uint8_t net_wait_join();

#endif
