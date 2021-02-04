#ifndef __DEBUG_H
#define __DEBUG_H


#if (IEEE802154E_AMCA == 1)
#include "IEEE802154AMCA.h"
#elif (IEEE802154E_RITMC == 1)
#include "IEEE802154RITMC.h"
#elif (IEEE802154E_ARM == 1)
#include "IEEE802154ARM.h"
#elif (IEEE802154E_AMAC == 1)
#include "IEEE802154AMAC.h"
#else
#include "IEEE802154E.h"
#endif

#define MAX_ELEM_ROUTE 50
extern uint8_t rffbuf[MAX_ELEM_ROUTE];
extern uint8_t ritroute[MAX_ELEM_ROUTE];
//extern ieee154e_dbg_t     ieee154e_dbg;

//Macros de medicao radio...duty cycle
#define LEDS_TX_ON   //leds_radio_on();
#define LEDS_TX_OFF  //leds_radio_off();
#define LEDS_RX_ON   //leds_radio_on();
#define LEDS_RX_OFF  //leds_radio_off();

#define RECORD_RADIO_TX_INIT   	{  ieee154e_vars.radioTxOnInit=ieee154e_vars.lastCapturedTime; \
		                            ieee154e_vars.radioOnThisSlot=TRUE; }

#define RECORD_RADIO_RX_INIT   	{  ieee154e_vars.radioRxOnInit=ieee154e_vars.lastCapturedTime; \
		                            ieee154e_vars.radioOnThisSlot=TRUE; }

#define RECORD_RADIO_TX_CLEAR   	{  ieee154e_vars.radioTxOnTics=0; \
		                               ieee154e_vars.radioOnThisSlot=FALSE; }

#define RECORD_RADIO_RX_CLEAR   	{  ieee154e_vars.radioRxOnTics=0; \
		                               ieee154e_vars.radioOnThisSlot=FALSE; }

#define RECORD_RADIO_TX_ON   	{  ieee154e_vars.lastCapturedTime = getdeltaslotperiod_ms(); \
								   ieee154e_vars.radioTxOnInit = ieee154e_vars.lastCapturedTime; \
	                               ieee154e_vars.radioOnThisSlot = TRUE; \
	                               LEDS_TX_ON   }

#define RECORD_RADIO_RX_ON   	{ ieee154e_vars.lastCapturedTime = getdeltaslotperiod_ms(); \
                                  ieee154e_vars.radioRxOnInit = ieee154e_vars.lastCapturedTime; \
	                              ieee154e_vars.radioOnThisSlot = TRUE; \
	                              LEDS_RX_ON  }

#define RECORD_RADIO_TX_OFF(a)   	{ (a) = getdeltaslotperiod_ms(); \
									  ieee154e_vars.radioTxOnTics += ((a) - ieee154e_vars.radioTxOnInit); \
									  LEDS_TX_OFF   }

#define RECORD_RADIO_RX_OFF(a)   	{ (a) = getdeltaslotperiod_ms(); \
									  ieee154e_vars.radioRxOnTics += ((a) - ieee154e_vars.radioRxOnInit); \
									  LEDS_RX_OFF   }

#define UPDT_LAST_LQI           	{ ieee154e_vars.lastRSSI = ieee154e_vars.dataReceived->l1_rssi; \
                                      ieee154e_vars.lastlqi = ieee154e_vars.dataReceived->l1_lqi; }


typedef struct {
//	uint16_t countdata;
	uint16_t countdatatx;
	uint16_t countdatatxok;
	uint16_t countdatatxerr;
//	uint16_t countack;
	uint16_t countacktxrx;
	uint16_t countacktxrxok;
	uint16_t countacktxrxerr;
} sRITstat;

typedef struct {
	sRITstat txola;
	sRITstat rxola;
	sRITstat txdio;
	sRITstat txdao;
	uint16_t rxdio;
	uint16_t rxdao;
	uint16_t rxCoapFwd;
    sRITstat txcoap;
} RIT_stats_t;


uint8_t printaddress(open_addr_t addr,uint8_t *buf,uint8_t pos);
uint8_t printvar(uint8_t *var,uint8_t size, uint8_t *buf,uint8_t pos);


void treatdebug(uint8_t macRITstate);
void printroute(uint8_t macRITstate);
void clearritroute(void);
uint8_t incroute(uint8_t element);
void clearstatistics(void);
uint8_t printaddress(open_addr_t addr,uint8_t *buf,uint8_t pos);
#endif
