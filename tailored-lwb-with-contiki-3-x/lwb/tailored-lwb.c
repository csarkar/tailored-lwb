#include "slot-allocation.h"
#include "slot-participation.h"
#include "energy-stats.h"


/**
 * \defgroup custom-lwb-variables Application variables
 * @{
 */

static sync_data_struct sync_data;     /**< \brief Flooding data. */
static struct rtimer rt;                   /**< \brief Rtimer used to schedule Glossy. */
static struct pt pt;                       /**< \brief Protothread used to schedule Glossy. */
static rtimer_clock_t t_ref_l_old = 0;     /**< \brief Reference time computed from the Glossy
                                                phase before the last one. \sa get_t_ref_l */
static uint8_t skew_estimated = 0;         /**< \brief Not zero if the clock skew over a period of length
                                                \link GLOSSY_PERIOD \endlink has already been estimated. */
static uint8_t sync_missed = 0;            /**< \brief Current number of consecutive phases without
                                                synchronization (reference time not computed). */
static int period_skew = 0;                /**< \brief Current estimation of clock skew over a period
                                                of length \link GLOSSY_PERIOD \endlink. */

/** @} */

/**
 * \defgroup lwb-test-variables-stats Statistics variables
 * @{
 */

/*---------------------------------------------------------------------------*/
static sensed_data_struct  sensed_data;
static request_data_struct req_reply;
/*---------------------------------------------------------------------------*/
static rtimer_clock_t REF_TIME;
static rtimer_clock_t NEXT_SLOT;
/*---------------------------------------------------------------------------*/
static int16_t period = 0;
static int     offset_err;
/*---------------------------------------------------------------------------*/
static uint16_t run_time, last_time;

static uint8_t  SYNC_SLOT = 1;

static uint8_t  RR_SLOTS, rr_slots;

static uint16_t DATA_SLOTS, data_slots;
static uint16_t max_data_slots;

static uint8_t  SLEEP_SLOTS;
/*---------------------------------------------------------------------------*/
static flow_info_struct flow_info;
/*---------------------------------------------------------------------------*/
static int8_t FLOODING_ROLE;
static uint16_t rx_count;
static enum err_type errno;
static uint8_t hop_to_sink;
/*---------------------------------------------------------------------------*/

char tailored_lwb_scheduler(struct rtimer *t, void *ptr);
/*---------------------------------------------------------------------------*/

/** @} */
/** @} */

/**
 * \defgroup lwb-test-processes Application processes and functions
 * @{
 */

static inline void estimate_period_skew(void) {
	// Estimate clock skew over a period only if the reference time has been updated.
	if (GLOSSY_IS_SYNCED()) {
		// Estimate clock skew based on previous reference time and the Glossy period.
		period_skew = get_t_ref_l() - (t_ref_l_old + (rtimer_clock_t)GLOSSY_PERIOD);
		// Update old reference time with the newer one.
		t_ref_l_old = get_t_ref_l();
		// If Glossy is still bootstrapping, count the number of consecutive updates of the reference time.
		if (GLOSSY_IS_BOOTSTRAPPING()) {
			// Increment number of consecutive updates of the reference time.
			skew_estimated++;
			// Check if Glossy has exited from bootstrapping.
			if (!GLOSSY_IS_BOOTSTRAPPING()) {
				// Glossy has exited from bootstrapping.
				leds_off(LEDS_RED);
				// Initialize Energest values.
				energest_init();
#if GLOSSY_DEBUG
				high_T_irq = 0;
				bad_crc = 0;
				bad_length = 0;
				bad_header = 0;
#endif /* GLOSSY_DEBUG */

			}
		}
	}
}

/**
 * 
 */
PROCESS(tailored_lwb_print_process, "Tailored LWB print process");

PROCESS_THREAD(tailored_lwb_print_process, ev, data)
{
	PROCESS_BEGIN();
	
	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
		switch(errno) {
		case NO_SYNC_PACKET:
			printf("Glossy NOT received\n");
			break;
		case NO_ERROR:
			break;
		case DATA_SLOT_END:
			printf("total rx %d nodes\n", rx_count);
			break;
		case DATA_TX_SUCC:
		case DATA_TX_FAIL:
		case DATA_TX_IGNORE:
			energy_update((run_time-last_time)*1000);
			last_time = run_time;
		default:
			break;
		}
		errno   = NO_ERROR;
	}

	PROCESS_END();
}

/**
 * 
 */
static inline int8_t reset_parameters() {
	data_slots  = 0;
	rr_slots = 0;
	rx_count = 0;
	SLEEP_SLOTS	= sync_data.sleep_slots;
	if(sync_data.sleep_slots>1) {
		t_ref_l_old = (rtimer_clock_t)(t_ref_l_old + ((SLEEP_SLOTS-1)%2)*RTIMER_SECOND);
	}
		
	if(get_rx_cnt() == 0) {
		return NO_SYNC_PACKET;
	}
	else {
		run_time = sync_data.run_time;
		RR_SLOTS = sync_data.rr_slots;
		DATA_SLOTS  = sync_data.data_slots;
		return NO_ERROR;
	}
}

/**
 * 
 */
void filled_superframe() {
	
	int16_t i;
	for(i=0; i<max_data_slots; i++) {
		sync_data.slot_vector[i/8] |= 1 << (i%8);
	}
	
	sync_data.data_slots = max_data_slots;
	
	if(IPI <= MINIMUM_LWB_ROUND) {
		sync_data.sleep_slots = MINIMUM_LWB_ROUND;
	} else {
		if(IPI%MINIMUM_LWB_ROUND == 0) {
			sync_data.sleep_slots = MINIMUM_LWB_ROUND;
		} else {
			sync_data.sleep_slots = MINIMUM_LWB_ROUND + IPI%MINIMUM_LWB_ROUND;
		}
	}
}

/**
 * 
 */
void bare_superframe(uint8_t sleep_duration) {
	sync_data.data_slots  = 0;
	sync_data.sleep_slots = sleep_duration;
}

/**
 * 
 */
void prepare_next_superframe() {
		
	sync_data.run_time = run_time;	

	if(run_time < COOLOFF_PERIOD) {
		/* before anything starts, just keep sending sync packet every seconds, 
		 * no other slots in the superframe */
		sync_data.rr_slots = 0;
		bare_superframe(1);
	} else if(run_time < STABILIZATION_PERIOD) {
		sync_data.rr_slots = MAX_RR_SLOTS_P_SECOND;
		/* send sync packets every seconds */
		bare_superframe(1);				
	} else if(run_time%IPI == 0) {
		/* allow only minimum RR slots, if some node is yet to get a slot */
		sync_data.rr_slots = MIN_RR_SLOTS;
		/* fill the superframe with suitable slots */
		filled_superframe();
	} else if(run_time%MINIMUM_LWB_ROUND == 0){
		/* send sync packet every MINIMUM_LWB_ROUND, if IPI is equal to this, then this does not arise */
		sync_data.rr_slots = 0;
		bare_superframe(MINIMUM_LWB_ROUND);
	} else {
		/* in all other cases, send sync packet every second, without any contention slot */
		sync_data.rr_slots = 0;
		bare_superframe(1);
	}
}

/**
 * 
 */
void next_radio_activity_schedule(struct rtimer *t, void *ptr) {
		
	rtimer_clock_t GRACE_PERIOD = 0;
	
	if(data_slots < DATA_SLOTS) {
		NEXT_SLOT = (rtimer_clock_t)(NEXT_SLOT + DATA_SLOT_LEN);
		if(flow_info.slot == data_slots+1) {
			GRACE_PERIOD = TX_GUARD_TIME;
		}
	}
	else if(rr_slots < RR_SLOTS) {
		NEXT_SLOT = (rtimer_clock_t)(NEXT_SLOT + RR_SLOT_LEN);
		
		if(IS_SINK()) {
			if(rr_slots%MIN_RR_SLOTS==0) {
				GRACE_PERIOD = (-1)*TX_GUARD_TIME;
			}
			else {
				GRACE_PERIOD = 2*TX_GUARD_TIME;
			}
		}
	}
	else if(NEXT_SLOT < RTIMER_SECOND) { 
		NEXT_SLOT = RTIMER_SECOND;
	}
	
	/* if a second has been passed, add it to the RX_TIME */
	if(NEXT_SLOT >= RTIMER_SECOND) {
		NEXT_SLOT   = (rtimer_clock_t)(NEXT_SLOT - RTIMER_SECOND);
		REF_TIME    = (rtimer_clock_t)(REF_TIME + RTIMER_SECOND);
		run_time++;
		if(SLEEP_SLOTS > 0) {
			SLEEP_SLOTS--;
		}
		
		if(SLEEP_SLOTS==0) {
			if(!IS_SINK()) {
				//GRACE_PERIOD = GLOSSY_GUARD_TIME * (1 + sync_missed) * (-1);
				GRACE_PERIOD = GLOSSY_GUARD_TIME * (-1);
			}
		}
	}
	
	/* schedule next radio activity */
	rtimer_set(t, (rtimer_clock_t)(REF_TIME + NEXT_SLOT + GRACE_PERIOD), 1, 
				(rtimer_callback_t)tailored_lwb_scheduler, ptr);
				
	if(SLEEP_SLOTS==0) {
		/* print stats after the LWB round */
		process_poll(&tailored_lwb_print_process);
		
		/* indicate there will be a sync slot */
		SYNC_SLOT = 1;
		
		/* sink node decides structure of the next superframe */
		if (IS_SINK()) {
			prepare_next_superframe();
		}
	}
	
}

/**
 * 
 */
void node_init() {
	
	run_time = 0;
	last_time = run_time;
		
	if (IS_SINK()) {
		data_slots	  = 0;
		sync_data.run_time	  = 0;
		sync_data.rr_slots    = 0;
		sync_data.sleep_slots = 1;
	} else {
		flow_info.dst  = SINK_NODE_ID;
		flow_info.slot = 0;
	}
}

/**
 * 
 */
rtimer_clock_t prepare_sync_packet(struct rtimer *t) {
	
	leds_on(LEDS_GREEN);
		
	rtimer_clock_t t_stop;
			
	if (IS_SINK()) {
		t_stop = RTIMER_TIME(t) + GLOSSY_DURATION;
		FLOODING_ROLE = GLOSSY_INITIATOR;
		REF_TIME = RTIMER_TIME(t);
	} else {
		if (GLOSSY_IS_BOOTSTRAPPING()) {
			t_stop = RTIMER_TIME(t) + GLOSSY_INIT_DURATION;
		} else {
			t_stop = RTIMER_TIME(t) + GLOSSY_DURATION;
		}
		FLOODING_ROLE = GLOSSY_RECEIVER;	
	}
	
	return t_stop;
}

/**
 * 
 */
void process_sync_packet(struct rtimer *t, void *ptr) {
	
	leds_off(LEDS_GREEN);
	
	if(IS_SINK()) {
		if (!GLOSSY_IS_BOOTSTRAPPING()) {
			// Glossy has already successfully bootstrapped.
			if (!GLOSSY_IS_SYNCED()) {
				// The reference time was not updated: increment reference time by GLOSSY_PERIOD.
				set_t_ref_l(GLOSSY_REFERENCE_TIME + GLOSSY_PERIOD);
				set_t_ref_l_updated(1);
			}
		}
		
		// Estimate the clock skew over the last period.
		estimate_period_skew();
		
		if (GLOSSY_IS_BOOTSTRAPPING()) {
			rtimer_set(t, REF_TIME + GLOSSY_PERIOD, 1, (rtimer_callback_t)tailored_lwb_scheduler, ptr);
			SYNC_SLOT = 1;
		} else {
			NEXT_SLOT  = GLOSSY_SYNC_GUARD;
			reset_parameters();				
			rtimer_set(t, REF_TIME + NEXT_SLOT, 1, (rtimer_callback_t)tailored_lwb_scheduler, ptr);
			SYNC_SLOT = 0;
			update_participation_vector(sync_data.slot_vector, DATA_SLOTS);
		}
	}
	else {
		if (GLOSSY_IS_BOOTSTRAPPING()) {
			// Glossy is still bootstrapping.
			if (!GLOSSY_IS_SYNCED()) {
				// The reference time was not updated: reset skew_estimated to zero.
				skew_estimated = 0;
			}
		} else {
			// Glossy has already successfully bootstrapped.
			if (!GLOSSY_IS_SYNCED()) {
				// The reference time was not updated:
				// increment reference time by GLOSSY_PERIOD + period_skew.
				set_t_ref_l(GLOSSY_REFERENCE_TIME + GLOSSY_PERIOD + period_skew);
				set_t_ref_l_updated(1);
				// Increment sync_missed.
				sync_missed++;
			} else {
				// The reference time was not updated: reset sync_missed to zero.
				sync_missed = 0;
			}
		}
		// Estimate the clock skew over the last period.
		estimate_period_skew();
		if (GLOSSY_IS_BOOTSTRAPPING()) {
			// Glossy is still bootstrapping.
			if (skew_estimated == 0) {
				rtimer_set(t, RTIMER_TIME(t) + GLOSSY_INIT_PERIOD, 1,
						(rtimer_callback_t)tailored_lwb_scheduler, ptr);
				SYNC_SLOT = 1;
			} else {
				REF_TIME = GLOSSY_REFERENCE_TIME + GLOSSY_PERIOD;
				rtimer_set(t, REF_TIME - GLOSSY_INIT_GUARD_TIME, 1,
							(rtimer_callback_t)tailored_lwb_scheduler, ptr);
				SYNC_SLOT = 0;
			}
		} else {
			offset_err = GLOSSY_REFERENCE_TIME - (rtimer_clock_t)REF_TIME;
			REF_TIME   = GLOSSY_REFERENCE_TIME;
			NEXT_SLOT  = GLOSSY_SYNC_GUARD;
			rtimer_set(t, REF_TIME + NEXT_SLOT, 1, (rtimer_callback_t)tailored_lwb_scheduler, ptr);
			
			errno = reset_parameters();
			hop_to_sink = get_my_hop();
			SYNC_SLOT = 0;
			
			if(errno == NO_SYNC_PACKET) {
				process_poll(&tailored_lwb_print_process);
			}
			
			update_participation_vector(sync_data.slot_vector, DATA_SLOTS);
		}
	}
}

/**
 * 
 */
void prepare_data() {
	
	sensed_data.dst = 0;
	
	if(decide_participation(data_slots)){
		if(flow_info.slot == data_slots) {
			sensed_data.src 	 = node_id;
			sensed_data.dst 	 = flow_info.dst;
			sensed_data.data_len = 1;
			sensed_data.hop		 = hop_to_sink+1;
			errno = DATA_TX_SUCC;
			FLOODING_ROLE = GLOSSY_INITIATOR;
		} else { 
			FLOODING_ROLE = GLOSSY_RECEIVER;
		}
	}
	else {
		FLOODING_ROLE = GLOSSY_NO_FLOODING;
	}
}

/**
 * 
 */
void process_data() {
	
	/* if the data is not sent, set error code to TX_FAIL */
	if(FLOODING_ROLE == GLOSSY_INITIATOR && get_rx_cnt()==0) {
		errno = DATA_TX_FAIL;
	} 
	
#if FORWARDER_SELECTION
	if(FLOODING_ROLE == GLOSSY_RECEIVER && get_rx_cnt()) {
		update_forwarder_selection(data_slots, sensed_data.hop, get_my_hop(), hop_to_sink);
	} 
#endif
	
	/* increase the number of packet received */
	if(sensed_data.dst == node_id) {
		rx_count++;
	}
	
	/* at the end of data slots */
	if(data_slots == DATA_SLOTS) {
		if(IS_SINK()) {
			errno = DATA_SLOT_END;
		}
		DATA_SLOTS = 0;
	}
}

/**
 * 
 */
void proecess_rr_data() {
	
	/* switch the role of INITIATOR and RECEIVER */
	if(req_reply.dst == node_id) {
		FLOODING_ROLE = GLOSSY_INITIATOR;
	} else {
		FLOODING_ROLE = GLOSSY_RECEIVER;
		req_reply.dst = 0;
	}

	/* processing at different slots */
	if(rr_slots%MIN_RR_SLOTS == 1) {
		if(get_rx_cnt()) {
			if(IS_SINK()) {
				max_data_slots = handle_slot_request(&req_reply);
			}
		} else {
			/* if no request is received, skip the reply slot */
			FLOODING_ROLE = GLOSSY_NO_FLOODING;
			rr_slots = RR_SLOTS;
		}
	} 
	else {
		if(get_rx_cnt()) {
			handle_slot_reply(&req_reply);
			if(req_reply.dst == node_id) {
				flow_info.slot = req_reply.slot;
			}
		}
	}
	
	/* at the end of rr slots */
	if(rr_slots == RR_SLOTS) {
		RR_SLOTS = 0;
	}
}

/** @} */

/**
 * \defgroup lwb-test-scheduler Periodic scheduling
 * @{
 */

char tailored_lwb_scheduler(struct rtimer *t, void *ptr) {
	
	PT_BEGIN(&pt);

	node_init();

	uint8_t src, dst;
	
	while (1) {
		if(SYNC_SLOT) {
			/* prepare the sync packet to decide the superframe structure */
			rtimer_clock_t t_stop = prepare_sync_packet(t);
			
			/* every node participate (using flooding) in every sync slot */
			tailored_glossy_start((uint8_t *)&sync_data, SYNC_LEN, FLOODING_ROLE, GLOSSY_SYNC, N_TX,
						APPLICATION_HEADER, t_stop, (rtimer_callback_t)tailored_lwb_scheduler, t, ptr);
			PT_YIELD(&pt);
			tailored_glossy_stop(&src, &dst);
			
			/* process the sync packet and schedule the next radio activity */
			process_sync_packet(t, ptr);				
		}
		else if(DATA_SLOTS) {
			/* account for how many data slots have elapsed */
			data_slots++;
			
			/* prepare the data packet for the global slot */
			prepare_data();
			
			/* selective nodes participate (using flooding) in delivering the data */
			if(FLOODING_ROLE != GLOSSY_NO_FLOODING) {	
				tailored_glossy_start((uint8_t *)&sensed_data, SDATA_LEN, FLOODING_ROLE, GLOSSY_SYNC, D_TX,
						APPLICATION_HEADER, (rtimer_clock_t)(RTIMER_TIME(t) + DATA_SLOT_DURATION), 
						(rtimer_callback_t)tailored_lwb_scheduler, t, ptr);
				PT_YIELD(&pt);
				tailored_glossy_stop(&src, &dst);
			}					
			
			/* schedule the next radio activity */
			next_radio_activity_schedule(t, ptr);
	
			/* process the data received in the global slot */
			process_data();
		}
		else if(RR_SLOTS) {
			/* account for how many contention slots have elapsed */
			rr_slots++;
			
			/* prepare the RR packet in the requesting slot */
			if(rr_slots%MIN_RR_SLOTS ==1) {
				FLOODING_ROLE = prepare_slot_request(&req_reply);
			}
			
			/* even though evry node should participate in every rr slots (with specified role),
			 * if no request is received in the first rr slot, skip the next slot */
			if(FLOODING_ROLE != GLOSSY_NO_FLOODING) {
				tailored_glossy_start((uint8_t *)&req_reply, REQ_LEN, FLOODING_ROLE, GLOSSY_NO_SYNC, D_TX,
						APPLICATION_HEADER, (rtimer_clock_t)(RTIMER_TIME(t) + RR_SLOT_DURATION), 
						(rtimer_callback_t)tailored_lwb_scheduler, t, ptr);
				PT_YIELD(&pt);
				tailored_glossy_stop(&src, &dst);
			}
			
			/* schedule the next radio activity */
			next_radio_activity_schedule(t, ptr);	
			
			/* process the data received in the rr_slot */
			proecess_rr_data();
		}
		else {
			/* in the sleep slot nothing to be done */
			/* schedule the next radio activity */
			next_radio_activity_schedule(t, ptr);
		} 
		
		
		/* Yield the protothread. */
		PT_YIELD(&pt);
	}
	
	PT_END(&pt);
}
	

PROCESS(tailored_lwb_process, "LWB Process");
PROCESS_THREAD(tailored_lwb_process, ev, data)
{
	PROCESS_BEGIN();
	
	initiate_energy_accounting();
	
	set_forwarder_vector();
	
	leds_on(LEDS_RED);
	// Start print stats processes.
	process_start(&tailored_lwb_print_process, NULL);
	// Start Glossy busy-waiting process.
	process_start(&tailored_glossy_process, NULL);
	// Start Glossy experiment in one second.
	rtimer_set(&rt, RTIMER_NOW() + RTIMER_SECOND, 1, (rtimer_callback_t)tailored_lwb_scheduler, NULL);
	
	PROCESS_END();
}

/** @} */
/** @} */
/** @} */
