#include "tailored-lwb.h"
	

PROCESS(lwb_test, "LWB test");
AUTOSTART_PROCESSES(&lwb_test);
PROCESS_THREAD(lwb_test, ev, data)
{
	static struct etimer et;
	
	PROCESS_BEGIN();
	
	/* Allow some time for the network to settle. */
	etimer_set(&et, 30 * CLOCK_SECOND);
	PROCESS_WAIT_UNTIL(etimer_expired(&et));
	printf("node_id %d\n", node_id);
	
	process_start(&tailored_lwb_process, NULL);
	
	PROCESS_END();
}


/** @} */
/** @} */
/** @} */
