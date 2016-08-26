#include "tailored-lwb.h"
	

PROCESS(lwb_test, "LWB test");
AUTOSTART_PROCESSES(&lwb_test);
PROCESS_THREAD(lwb_test, ev, data)
{
	PROCESS_BEGIN();
	
	process_start(&tailored_lwb_process, NULL);
	
	PROCESS_END();
}


/** @} */
/** @} */
/** @} */
