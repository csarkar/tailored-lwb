#ifndef ENERGY_STATS_H_
#define ENERGY_STATS_H_

#include "contiki.h"


/*---------------------------------------------------------------------------*/

void energy_update(uint16_t period_ms);

void initiate_energy_accounting(void);

/*---------------------------------------------------------------------------*/

#endif /* ENERGY_STATS_H_ */
