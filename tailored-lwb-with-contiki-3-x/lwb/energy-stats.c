#include "energy-stats.h"

#include <stdio.h>


/*---------------------------------------------------------------------------*/

static int8_t energy_accounting_initiated = 0;

static uint32_t last_cpu, last_lpm, last_transmit, last_listen;

/*---------------------------------------------------------------------------*/

void energy_update(uint16_t period_ms)
{
	uint32_t curr_cpu, curr_lpm, curr_transmit, curr_listen;
	
	if(!energy_accounting_initiated) {
		printf("Please specify platform related parameters in the header file and call initiate_energy_accounting function\n");
		return;
	}

	energest_flush();

	curr_cpu = energest_type_time(ENERGEST_TYPE_CPU);
	curr_lpm = energest_type_time(ENERGEST_TYPE_LPM);
	curr_transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT);
	curr_listen = energest_type_time(ENERGEST_TYPE_LISTEN);

	long tx_time_us = ((long) 1e6 / RTIMER_SECOND) * ((long) (curr_transmit - last_transmit));
	long rx_time_us = ((long) 1e6 / RTIMER_SECOND) * ((long) (curr_listen - last_listen));
	long duty_cycle = 100 * (tx_time_us+rx_time_us) / ((long) period_ms);    // still need to be divided by 1000, because period is in ms.
	
	/* Energy (uJ) = v (V) * current (mA) * time (us) / 1000 */
	/* divide by 10 means scale down current value (eliminating the `dot') */
	/*uint8_t tx_level = cc2420_get_txpower();
	tx_energy = voltage * tx_current_consumption(tx_level) * tx_time_us / 1000 / 10;
	rx_energy = voltage * rx_current_consumption * rx_time_us / 1000 / 10;

	remaining_energy = MAX(remaining_energy - tx_energy, 0);
	remaining_energy = MAX(remaining_energy - rx_energy, 0);*/			
			
	printf("Radio on time %ld ms in %u ms, duty cycle %ld.%01ld %%\n",
				(tx_time_us+rx_time_us)/1000, period_ms, duty_cycle / 1000, duty_cycle % 1000);
	
	last_cpu = curr_cpu;
	last_lpm = curr_lpm;
	last_transmit = curr_transmit;
	last_listen = curr_listen;
}


void initiate_energy_accounting(void) {
	
	energy_accounting_initiated = 1;
	
	energest_flush();

	last_cpu = energest_type_time(ENERGEST_TYPE_CPU);
	last_lpm = energest_type_time(ENERGEST_TYPE_LPM);
	last_transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT);
	last_listen = energest_type_time(ENERGEST_TYPE_LISTEN);
}
