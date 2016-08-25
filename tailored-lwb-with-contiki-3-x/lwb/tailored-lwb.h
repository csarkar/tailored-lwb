#ifndef TAILORED_LWB_H_
#define TAILORED_LWB_H_

#include <stdio.h>
#include <stdlib.h>

#include "contiki.h"

#include "tailored-glossy.h"
#include "slot-def.h"

#include "node-id.h"


/*********************************************************************************************/

PROCESS_NAME(tailored_lwb_process);

/*********************************************************************************************/
enum {
	NO_FAIL = 0, TX_FAIL = 1 
};

enum err_type {
	NO_ERROR = 0, NO_SYNC_PACKET = 1,
	DATA_RX = 4, DATA_SLOT_END = 6, 
	DATA_TX_SUCC = 7, DATA_TX_FAIL = 8, DATA_TX_IGNORE = 9
};

typedef struct {
	uint16_t dst;
	uint16_t slot;
} flow_info_struct;

/*********************************************************************************************************/
/**----------various structures for various types of packets-----------**/


/**
 * \brief Data structure used to send synchronization packet.
 */
typedef struct {
	uint16_t run_time;
	uint8_t  rr_slots;
	uint16_t data_slots;
	uint8_t	 slot_vector[20];
	uint16_t sleep_slots;
} sync_data_struct;

/**
 * \brief Length of sync packet structure.
 */
#define SYNC_LEN                    sizeof(sync_data_struct)


 /**
 * \brief Data structure used to send a data packet. 6 bytes for data packet, and 
 * additional information is piggybacked with it.
 */	
typedef struct {
	uint16_t src;
	uint16_t dst;
	uint8_t  hop;
	uint8_t  data_len;
	uint8_t  data[MAX_PAYLOAD_LEN];
} sensed_data_struct;

/**
 * \brief Length of data packet.
 */
#define SDATA_LEN                   sizeof(sensed_data_struct)


/**
 * \brief Data structure used to send slot request and reply message.
 */	
typedef struct {
	uint16_t src;
	uint16_t dst;
	uint16_t slot;
} request_data_struct;

/**
 * \brief Length of request/reply packet structure.
 */
 #define REQ_LEN                   sizeof(request_data_struct)

/*********************************************************************************************/

/** @} */

/** @} */

#endif /* TAILORED_LWB_H_ */
