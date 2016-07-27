#include "slot-allocation.h"

/*---------------------------------------------------------------------------*/
static uint16_t global_slot_info[MAX_NODE_NUMBER];
static uint16_t global_slot_count;
static uint8_t  slot_granted;

static uint8_t  t_vector[MAX_NODE_NUMBER/8+1];

static int8_t hop_count;
/*---------------------------------------------------------------------------*/

/*------------------------- public functions -------------------------------*/

/**
 * 
 */
uint16_t get_own_slot(void) {
	return slot_granted;
}

/**
 * 
 */
void load_schedule(uint8_t *p_vector) {
	
	memcpy(p_vector, t_vector, sizeof(t_vector));
}

/**
 * Prepare a request message for a node to get a data slot from the sink
 */
int8_t prepare_slot_request(request_data_struct *req_reply) {
	
	if(!IS_SINK() && !slot_granted) {
		req_reply->src  = node_id;
		req_reply->dst  = SINK_NODE_ID;
		req_reply->slot = 0;
		return GLOSSY_INITIATOR;
	} else {
		req_reply->src  = 0;
		req_reply->dst  = 0;
		req_reply->slot = 0;
		return GLOSSY_RECEIVER;
	}
}

/**
 * 
 */
uint16_t handle_slot_request(request_data_struct *req_reply) {
	
	uint16_t src = req_reply->src-1;
	if(global_slot_info[src] == 0) {
		global_slot_info[src] = ++global_slot_count;
	}
	req_reply->slot = global_slot_info[src];

	req_reply->dst  = req_reply->src;
	req_reply->src  = node_id;
	return global_slot_count;
}

/**
 * 
 */
void handle_slot_reply(request_data_struct *req_reply, enum err_type *errno) {
	
	hop_count = get_my_hop();
	
	if(req_reply->dst == node_id) {
		req_reply->src = node_id;
		req_reply->dst = SINK_NODE_ID;		
		req_reply->hop = hop_count+1;
		
		slot_granted = req_reply->slot;
	} 
	else {
		req_reply->src = 0;
		req_reply->dst = 0;
	}
}

/**
 * 
 */
int8_t add_participation(uint8_t data_slot, uint8_t hop) {
	if(!FORWARDER_SELECTION || hop >= hop_count + get_my_hop()) {
		uint8_t i = (data_slot-1)/8;
		t_vector[i] = t_vector[i] | (0x01 <<(data_slot-1)%8);
		return 1;
	}
	return 0;
}
