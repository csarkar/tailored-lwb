#ifndef SLOT_PARTICIPATION_H_
#define SLOT_PARTICIPATION_H_

#include "tailored-lwb.h"

/*--------------------------------------------------------------------*/

void set_forwarder_vector();

void update_forwarder_selection(uint16_t slot, uint8_t hop_src_to_sink, uint8_t hop_from_src, uint8_t hop_to_sink);

void update_participation_vector(uint8_t vector[], uint16_t data_slots);

uint8_t decide_participation(uint16_t slot);

/*--------------------------------------------------------------------*/

#endif /* SLOT_PARTICIPATION_H_ */

/** @} */
