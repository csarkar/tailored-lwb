#include "slot-participation.h"
#include "slot-allocation.h"

/*---------------------------------------------------------------------------*/
static uint8_t  participation_vector[MAX_NODE_NUMBER/8+1];
static uint8_t  energy_aware_vector[MAX_NODE_NUMBER/8+1];
/*---------------------------------------------------------------------------*/

/**
 * 
 */
void update_energy_aware_vector(void) {
	
	uint8_t i;
	
	for(i=0; i<MAX_NODE_NUMBER; i++) {
		energy_aware_vector[i/8] |= (0x01 <<(i%8));
	}
}

/**
 * 
 */
void update_forwarder_selection_vector(uint16_t slot) {
	participation_vector[(slot-1)/8] |= (0x01 <<(slot-1)%8);
}

/**
 * 
 */
void load_forwarder_selection_vector(void) {
	load_schedule(participation_vector);
}

/**
 * 
 */
void update_participation_vector(uint8_t vector[], uint16_t data_slots) {
	uint16_t i;
	
#if FORWARDER_SELECTION
#else
	for(i=0; i<=data_slots/8; i++) {
		participation_vector[i] = vector[i];
	}
#endif
}

/**
 * 
 */
uint8_t decide_participation(uint16_t slot) {
	uint8_t i = (slot-1)/8;
	
	if((participation_vector[i] & (0x01 <<(slot-1)%8))) {// && 
		//(energy_aware_vector[i] & (0x01 <<(slot-1)%8))) {
		return 1;
	} else {
		return 0;
	}
}
