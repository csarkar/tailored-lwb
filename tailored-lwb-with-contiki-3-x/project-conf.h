
// set COOJA to 1 for simulating Glossy in Cooja
#define COOJA 1
#define WITH_TINYOS_AUTO_IDS 1

#define rtimer_arch_now_dco() (TBR)

#define RTIMER_NOW_DCO() rtimer_arch_now_dco()

#ifndef RF_CHANNEL
#define RF_CHANNEL              26
#endif /* RF_CHANNEL */

#undef NETSTACK_CONF_MAC

#undef NETSTACK_CONF_RDC

#undef NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE

#undef NETSTACK_CONF_RADIO

#undef NETSTACK_CONF_FRAMER

#undef CC2420_CONF_AUTOACK

#undef CONTIKIMAC_CONF_COMPOWER        
#undef XMAC_CONF_COMPOWER              
#undef CXMAC_CONF_COMPOWER             

#undef SICSLOWPAN_CONF_COMPRESSION_THRESHOLD

#undef COLLECT_CONF_ANNOUNCEMENTS
#undef CXMAC_CONF_ANNOUNCEMENTS         
#undef XMAC_CONF_ANNOUNCEMENTS          
#undef CONTIKIMAC_CONF_ANNOUNCEMENTS    

#undef COLLECT_NEIGHBOR_CONF_MAX_COLLECT_NEIGHBORS

#undef QUEUEBUF_CONF_NUM

#undef TIMESYNCH_CONF_ENABLED

#undef CC2420_CONF_SFD_TIMESTAMPS

#undef PACKETBUF_CONF_ATTRS_INLINE

#undef CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT

#undef IEEE802154_CONF_PANID

#undef SHELL_VARS_CONF_RAM_BEGIN
#undef SHELL_VARS_CONF_RAM_END

#undef PROFILE_CONF_ON

#undef ELFLOADER_CONF_TEXT_IN_ROM
#undef ELFLOADER_CONF_DATAMEMORY_SIZE
#undef ELFLOADER_CONF_TEXTMEMORY_SIZE

#undef AODV_COMPLIANCE
#undef AODV_NUM_RT_ENTRIES

#undef WITH_ASCII


#undef UIP_CONF_IP_FORWARD      
#undef UIP_CONF_BUFFER_SIZE     


#undef UIP_CONF_ICMP_DEST_UNREACH

#undef UIP_CONF_DHCP_LIGHT
#undef UIP_CONF_LLH_LEN         
#undef UIP_CONF_RECEIVE_WINDOW
#undef UIP_CONF_TCP_MSS
#undef UIP_CONF_MAX_CONNECTIONS 
#undef UIP_CONF_MAX_LISTENPORTS 
#undef UIP_CONF_UDP_CONNS       
#undef UIP_CONF_FWCACHE_SIZE    
#undef UIP_CONF_BROADCAST       
#undef UIP_ARCH_IPCHKSUM        
#undef UIP_CONF_UDP             
#undef UIP_CONF_UDP_CHECKSUMS   
#undef UIP_CONF_PINGADDRCONF    
#undef UIP_CONF_LOGGING         

#undef UIP_CONF_TCP_SPLIT       

#undef AES_128_CONF
