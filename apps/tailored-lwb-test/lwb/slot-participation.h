#ifndef SLOT_PARTICIPATION_H_
#define SLOT_PARTICIPATION_H_

#include "contiki.h"

/*--------------------------------------------------------------------*/

void update_energy_aware_vector(void);

void update_forwarder_selection_vector(uint16_t slot);

void load_forwarder_selection_vector(void);

uint8_t decide_participation(uint16_t slot);

/*--------------------------------------------------------------------*/

#endif /* SLOT_PARTICIPATION_H_ */

/** @} */
