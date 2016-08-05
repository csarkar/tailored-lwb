#ifndef SLOT_ALLOCATION_H_
#define SLOT_ALLOCATION_H_

#include "tailored-lwb.h"

/*--------------------------------------------------------------------*/

uint16_t get_own_slot(void);

void load_schedule(uint8_t *p_vector);

int8_t prepare_slot_request(request_data_struct *req_reply);

uint16_t handle_slot_request(request_data_struct *req_reply);

void handle_slot_reply(request_data_struct *req_reply, enum err_type *errno);

int8_t add_participation(uint8_t data_slot, uint8_t hop);

/*--------------------------------------------------------------------*/


#endif /* SLOT_ALLOCATION_H_ */

/** @} */
