#ifndef CAN_NET_H
#define CAN_NET_H

/*
 * Before start, call this subroutine to init
 * physical underlayer CAN
 */
uint8_t net_init(uint32_t node_id, uint32_t brdcst_id);

/*
 * Before data communication, call this subroutine to
 * connect to a network
 */
uint8_t net_join();

/*
 * This routine pings all nodes in a network and update
 * list of the active nodes
 */
uint8_t net_ping_nodes();

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
uint32_t net_recv(uint32_t* buf, uint8_t blocked);

/*
 * To send data to recipient ID.
 * Size units are bytes regardless of 32bitness of buffer
 * If ack is flagges, subroutine will wait for receive
 * confirmation from recipient endpoint.
 */
uint8_t net_send(uint32_t* buf, uint8_t size, uint8_t ack, uint32_t recipient_id);

#endif
