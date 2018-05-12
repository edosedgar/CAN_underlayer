#ifndef CAN_NET_H
#define CAN_NET_H

#define BRDCST_ID 0x00
#define MAX_NODE 0x20

#define SYNC_TIMEOUT_MAX 3000
#define PING_PERIOD_MS   1000
#define PING_DELAY       100

#define RECV_TIMEOUT_MS  100
#define SEND_TIMEOUT_MS  100

#define FRAME_PCAP       4

enum FRAME_T {
        NET_JOIN          = 0x01,
        NET_JOIN_ACK      = 0x02,
        NET_SYNCED        = 0x03,
        NET_PING          = 0x04,
        NET_PING_OK       = 0x05,
        NET_SEND_INIT     = 0x06,
        NET_SEND_INIT_ACK = 0x07,
        NET_SEND          = 0x08,
        NET_SEND_ACK      = 0x09
};

enum NODE_STATUS_T {
        ST_RESET,
        ST_WAIT_JOIN,
        ST_JOINED,
        ST_SYNCED,
        ST_READY,
        ST_RECV_INIT,
        ST_RECV_INIT_ACK,
        ST_RECV,
        ST_RECV_ACK,
        ST_SEND_INIT,
        ST_SEND_INIT_ACK,
        ST_SEND,
        ST_SEND_ACK
};

enum SEND_T {
        ACK_YES,
        ACK_NO
};

enum RECV_T {
        RECV_BLOCK,
        RECV_POLL
};

enum RET_CODE_T {
        RET_TIMEOUT = 0x0100,
        RET_INVAL   = 0x0200,
        RET_BUSY    = 0x0300
};

struct net_frame {
        uint16_t source_id;
        uint8_t frame_type;
        uint8_t frame_size;
        uint8_t payload[FRAME_PCAP];
} __attribute__((packed));

/*
 * Net controller status structure
 */
struct net_state {
        uint8_t active_node;
        uint8_t status;
        uint16_t nodes[MAX_NODE];
        uint32_t poll_en;
        uint32_t sync_timeout;
        uint32_t wait_ping;
        uint32_t recv_wait_ms;
        uint32_t send_wait_ms;
};

/*
 * RX channel status structure
 */
struct net_rx_state {
        uint32_t source_id;
        uint8_t is_rx_filled;
        uint8_t is_wait;
        uint8_t is_timeout;
        uint8_t size;
        uint8_t *buffer;
        uint8_t offset;
};

/*
 * TX channel status structure
 */
struct net_tx_state {
        uint32_t rec_id;
        uint8_t is_tx_done;
        uint8_t is_timeout;
        uint8_t size;
        uint8_t *buffer;
        uint8_t offset;
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
 * The *recv_flag* parameter specifies whether it is required
 * to wait for data or not:
 *      RECV_BLOCK: spin in the subroutine unless data is received
 *      RECV_POLL: initiate request to accept data
 *              The subroutine can be reentered, so:
 *              If data is being received the net_recv return 0
 *              Once timeout happens, the net_recv exits with 1
 * After successfull receive, the highest two bytes of return
 * value are *source ID*, the lowest two bytes are *size of payload*
 *
 *          | 16bit | 16bit |
 * RETVAL = |  ID   | SIZE  |
 */
uint32_t net_recv(uint8_t *buf, uint8_t recv_flag);

/*
 * To send data to recipient ID.
 * NOTE: if net_recv has been called before hasn't still
 *       finished the net_send will end up with RET_BUSY exit code
 * NOTE: The recipient_id should be only active node ID,
 *       broadcast message is also not permitted
 *       Unless recipient_id is from an active node list,
 *       the net_send will finish with RET_INVAL
 */
uint16_t net_send(uint8_t *buf, uint8_t size, uint16_t recipient_id);

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
 * Call this routine to make sure that the node joined network
 */
uint8_t net_wait_join();

uint32_t net_get_id(uint8_t id_num);

#endif
