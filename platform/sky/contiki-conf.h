/* -*- C -*- */
/* @(#)$Id: contiki-conf.h,v 1.76 2010/03/19 13:27:46 adamdunkels Exp $ */

#ifndef CONTIKI_CONF_H
#define CONTIKI_CONF_H

#ifdef PLATFORM_CONF_H
#include PLATFORM_CONF_H
#else
#include "platform-conf.h"
#endif /* PLATFORM_CONF_H */

// set COOJA to 1 for simulating Glossy in Cooja
#define COOJA 1
#define TINYOS_SERIAL_FRAMES 0

#ifndef RF_CHANNEL
#define RF_CHANNEL              26
#endif /* RF_CHANNEL */

#define ENERGEST_CONF_ON 1

#define PROCESS_CONF_NUMEVENTS 8
#define PROCESS_CONF_STATS 1


#ifdef PROJECT_CONF_H
#include PROJECT_CONF_H
#endif /* PROJECT_CONF_H */



#endif /* CONTIKI_CONF_H */
