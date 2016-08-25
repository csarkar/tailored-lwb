#include "slot-participation.h"

/*---------------------------------------------------------------------------*/
static uint8_t forwarder_vector[MAX_NODE_NUMBER/8+1];
static uint8_t participation_vector[MAX_NODE_NUMBER/8+1];
/*---------------------------------------------------------------------------*/

void set_forwarder_vector() {
	
	uint8_t i;
	for(i=0; i<MAX_NODE_NUMBER/8+1; i++) {
		forwarder_vector[i] = 0xFF;
	}
}

/**
 * 
 */
void update_forwarder_selection(uint16_t slot, uint8_t hop_src_to_sink, uint8_t hop_from_src, uint8_t hop_to_sink) {
#if FORWARDER_SELECTION
	if(hop_src_to_sink >= hop_from_src + hop_to_sink) {
		forwarder_vector[(slot-1)/8] |= (0x01 <<(slot-1)%8);
	} else {
		forwarder_vector[(slot-1)/8] &= ~(0x01 <<(slot-1)%8);
	}
#endif
}

/**
 * 
 */
void update_participation_vector(uint8_t vector[], uint16_t data_slots) {
	uint16_t i;
	
	for(i=0; i<=data_slots/8; i++) {
		participation_vector[i] = vector[i] & forwarder_vector[i];
	}
}

/**
 * 
 */
uint8_t decide_participation(uint16_t slot) {
	
	if((participation_vector[(slot-1)/8] & (0x01 <<(slot-1)%8))) {
		return 1;
	} else {
		return 0;
	}
}
