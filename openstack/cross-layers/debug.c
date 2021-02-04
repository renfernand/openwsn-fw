#include "board.h"
#include "opendefs.h"
#include "idmanager.h"
#include "debug.h"
#include "watchdog.h"
#include "stdlib.h"
#include "neighbors.h"
#include "radio.h"
#include "leds.h"
#include "openserial.h"

#include "debug.h"

#define DBG_CONSUMO     0
#define DBG_THROUGHPUT  0
#define DBG_OWSN        1

#if 0
extern uint16_t ritmc_txolaack;
extern uint16_t ritmc_rxolaack;
extern uint32_t radiotxdutycycle1;
extern uint32_t radiotxdutycycle2;
extern uint32_t radiorxdutycycle1;
extern uint32_t radiorxdutycycle2;
extern uint32_t radiorxdutycycle3;
#endif

#if (IEEE802154E_ARM == 1) || (IEEE802154E_AMAC == 1)
extern uint16_t counttxrts;
extern uint16_t countrxrts;
extern uint16_t counttxcts;
extern uint16_t countrxcts;
extern uint16_t countrxHA;
extern uint16_t counttxcw;
extern radio_vars_t radio_vars;
extern uint8_t RxFlagImprimeRoute;
#endif

extern uint8_t  coappending;

uint8_t rffbuf[MAX_ELEM_ROUTE];
uint8_t ritroute[MAX_ELEM_ROUTE];
uint8_t lastritroutepos = 0;
uint8_t printstattoggle=0;
ieee154e_dbg_t     ieee154e_dbg;
RIT_stats_t ritstat;
uint16_t txidle_dutycle_ticks;          //variavel global pois sera utilizada no frame de diagnostico...no proximo ciclo...ele vai ter o ultimo valor
uint16_t RadioRxOn_dutycycle_ticks;    //variavel global pois sera utilizada no frame de diagnostico...no proximo ciclo...ele vai ter o ultimo valor
uint8_t  dutycyclecount;


extern ieee154e_vars_t    ieee154e_vars;
extern ieee154e_stats_t   ieee154e_stats;
#if (IEEE802154E_TSCH == 0)
//extern macneighbors_vars_t macneighbors_vars;
#endif
extern uint8_t macRIT_Pending_RX_frameType;
extern uint8_t macRIT_Pending_TX_frameType;
extern uint32_t txwaitola;
extern uint32_t radiotxola;
//======= misc



void clearstatistics(void)
{
    //rit statistics - zera as variaveis
	ritstat.txola.countdatatx=0;
	ritstat.txola.countdatatxok=0;
	ritstat.txola.countdatatxerr=0;
	ritstat.txola.countacktxrx=0;
	ritstat.txola.countacktxrxok=0;
	ritstat.txola.countacktxrxerr=0;

	ritstat.rxola.countdatatx=0;
	ritstat.rxola.countdatatxok=0;
	ritstat.rxola.countdatatxerr=0;
	ritstat.rxola.countacktxrx=0;
	ritstat.rxola.countacktxrxok=0;
	ritstat.rxola.countacktxrxerr=0;

	ritstat.txdio.countdatatx=0;
	ritstat.txdio.countdatatxok=0;
	ritstat.txdio.countdatatxerr=0;
	ritstat.txdio.countacktxrx=0;
	ritstat.txdio.countacktxrxok=0;
	ritstat.txdio.countacktxrxerr=0;
	ritstat.rxdio=0;
	ritstat.txdao.countdatatx=0;
	ritstat.txdao.countdatatxok=0;
	ritstat.txdao.countdatatxerr=0;
	ritstat.txdao.countacktxrx=0;
	ritstat.txdao.countacktxrxok=0;
	ritstat.txdao.countacktxrxerr=0;
	ritstat.rxdao=0;
	ritstat.rxCoapFwd=0;

	ritstat.txcoap.countdatatx=0;
	ritstat.txcoap.countdatatxok=0;
	ritstat.txcoap.countdatatxerr=0;
	ritstat.txcoap.countacktxrx=0;
	ritstat.txcoap.countacktxrxok=0;
	ritstat.txcoap.countacktxrxerr=0;


#if (IEEE802154E_RITMC == 1)
	ritmc_txolaack=0;
	ritmc_rxolaack=0;
	coappending = 0;
#endif

}

uint8_t printaddress(open_addr_t addr,uint8_t *buf,uint8_t pos){

	buf[pos++]= 0xAA;
	//buf[pos++]= addr.type;
    if (addr.type == 0x01)
    {
		buf[pos++]= addr.addr_16b[0];
		buf[pos++]= addr.addr_16b[1];
    }
    else if (addr.type == 0x02)	{
		buf[pos++]= addr.addr_64b[6];
		buf[pos++]= addr.addr_64b[7];
    }
	else if (addr.type == 0x03) {
		buf[pos++]= addr.addr_128b[14];
		buf[pos++]= addr.addr_128b[15];
	}

    return pos;

}

uint8_t printvar(uint8_t *var,uint8_t size, uint8_t *buf,uint8_t pos){

	//buf[pos++]= 0xBB;

	switch(size)
	{
		case 1:
			buf[pos++]= *var;
			break;
		case 2: //inverti...agora eh formato big endian
			buf[pos++]= *(var+1);
			buf[pos++]= *(var+0);
			break;
		case 4: //inverti...agora eh formato big endian
			buf[pos++]= *(var+3);
			buf[pos++]= *(var+2);
			buf[pos++]= *(var+1);
			buf[pos++]= *(var+0);
			break;
		default:
			break;
	}

    return pos;
}


#if 0
/**
\brief Trigger this module to print status information, over serial.

debugPrint_* functions are used by the openserial module to continuously print
status information about several modules in the OpenWSN stack.

\returns TRUE if this function printed something, FALSE otherwise.
*/
#if (IEEE802154E_TSCH == 0)
bool debugPrint_asn() {
   asn_t output;
   output.byte4         =  ieee154e_vars.asn.byte4;
   output.bytes2and3    =  ieee154e_vars.asn.bytes2and3;
   output.bytes0and1    =  ieee154e_vars.asn.bytes0and1;
   openserial_printStatus(STATUS_ASN,(uint8_t*)&output,sizeof(output));
   return TRUE;
}

/**
\brief Trigger this module to print status information, over serial.

debugPrint_* functions are used by the openserial module to continuously print
status information about several modules in the OpenWSN stack.

\returns TRUE if this function printed something, FALSE otherwise.
*/
bool debugPrint_isSync() {
   uint8_t output=0;
   output = ieee154e_vars.isSync;
   openserial_printStatus(STATUS_ISSYNC,(uint8_t*)&output,sizeof(uint8_t));
   return TRUE;
}

/**
\brief Trigger this module to print status information, over serial.

debugPrint_* functions are used by the openserial module to continuously print
status information about several modules in the OpenWSN stack.

\returns TRUE if this function printed something, FALSE otherwise.
*/
bool debugPrint_macStats() {
   // send current stats over serial
   openserial_printStatus(STATUS_MACSTATS,(uint8_t*)&ieee154e_stats,sizeof(ieee154e_stats_t));
   return TRUE;
}

#endif

/* ignorar imprimir RX quando entrar nos seguintes casos:
   0x85 - rxwindowend (acontece toda hora)
   0x68 - quando ele receber um frame porem é um ola.
*/
const uint8_t noprint[]={0xE3,0xF4,0x48,0x85};
uint8_t checkimprimir(void)
{
   uint8_t j,i;

    for(j=0;j<sizeof(noprint);j++){
    	for(i=0;i<lastritroutepos;i++){
    		if (ritroute[i] == noprint[j])
    			return 0;
    	}
    }

	return TRUE;
}


void clearritroute(void)
{
	int i;

	lastritroutepos = 0;
	for (i=0;i<MAX_ELEM_ROUTE;i++)
	ritroute[i] = 0;

}

uint8_t incroute(uint8_t element)
{
#if ENABLE_DEBUG_RFF
    //if (macRITstate == S_RIT_TX_state)
    {
		if (lastritroutepos < MAX_ELEM_ROUTE)
		{
			ritroute[lastritroutepos] = element;
			lastritroutepos++;
		}
		else
		{
			lastritroutepos = 0;
			ritroute[lastritroutepos] = 0xCC;
			ritroute[lastritroutepos] = element;
		}
    }

    return lastritroutepos;
#else
    return 0;
#endif

}

#if (ENABLE_DEBUG_RFF == 1)
//imprime as estatisticas
void printstat(void){
	uint8_t pos=0;
	uint8_t i=0;

	if (1) { // (printstattoggle == TRUE) {
		//printstattoggle = FALSE;
		rffbuf[pos++]  = 0x96;   //ESTATISTICAS LIVELIST ATUAL DA VIZINHANÇA
		rffbuf[pos++]= idmanager_getIsDAGroot();   //Mostra se é SINK =1  ou MOTE=0

#if (IEEE802154E_TSCH == 0)
		for (i=0;i<MAXNUMMACNEIGHBORS;i++) {
		  if (macneighbors_vars.neighbors[i].used==TRUE) {
			pos = printaddress(macneighbors_vars.neighbors[i].addr_16b,&rffbuf[0],pos);
			rffbuf[pos++]  = macneighbors_vars.neighbors[i].bestchan;
			rffbuf[pos++]  = macneighbors_vars.neighbors[i].stableNeighbor;
			//rffbuf[pos++]  = macneighbors_vars.neighbors[i].switchStabilityCounter;
			pos = printvar((uint8_t *)&macneighbors_vars.neighbors[i].numTxDIO,sizeof(uint8_t),rffbuf,pos);
			//pos = printvar((uint8_t *)&macneighbors_vars.neighbors[i].numTxDIOErr,sizeof(uint8_t),rffbuf,pos);
			pos = printvar((uint8_t *)&macneighbors_vars.neighbors[i].numTxDAO,sizeof(uint8_t),rffbuf,pos);
			//pos = printvar((uint8_t *)&macneighbors_vars.neighbors[i].numTxDAOErr,sizeof(uint8_t),rffbuf,pos);
		  }
	    }
#endif

	    //Nro de COAP
#if (SINK == 1)
		//rffbuf[pos++] = 0xee;
		//pos = printvar((uint8_t *)&ritstat.txcoap.countdatatxok ,sizeof(uint16_t),rffbuf,pos);
		//pos = printvar((uint8_t *)&ritstat.txcoap.countacktxrxok,sizeof(uint16_t),rffbuf,pos);
#endif
	}
	else {
		printstattoggle = TRUE;
#if 0
		rffbuf[pos++]= 0x94; //ESTATISTICAS EXTERNA DO NR DE FRAMES ENVIADOS E RECEBIDOS
	    pos = printvar((uint8_t *)&ritstat.txdio.countdata,sizeof(uint16_t),rffbuf,pos);
	    pos = printvar((uint8_t *)&ritstat.txdio.countdatatx,sizeof(uint16_t),rffbuf,pos);
	    pos = printvar((uint8_t *)&ritstat.txdio.countdatatxok,sizeof(uint16_t),rffbuf,pos);
	    pos = printvar((uint8_t *)&ritstat.rxdio,sizeof(uint16_t),rffbuf,pos);

	    rffbuf[pos++]= 0x93;   //ESTATISTICAS INTERNA DOS SLOTS PRODUZIDOS E FINALIZADOS
		rffbuf[pos++]= idmanager_getIsDAGroot();

		//pos = printvar((uint8_t *)&ieee154e_dbg.num_newSlot,sizeof(uint32_t),rffbuf,pos);
		pos = printvar((uint8_t *)&ieee154e_dbg.num_txslot,sizeof(uint32_t),rffbuf,pos);
		pos = printvar((uint8_t *)&ieee154e_dbg.num_txend,sizeof(uint32_t),rffbuf,pos);
		pos = printvar((uint8_t *)&ieee154e_dbg.num_rxslot,sizeof(uint32_t),rffbuf,pos);
		pos = printvar((uint8_t *)&ieee154e_dbg.num_rxend,sizeof(uint32_t),rffbuf,pos);
#endif
	}

	//openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
}


#if (DBG_CONSUMO == 1)

void printroute(uint8_t macRITstate)
{
	uint8_t i=0;
	uint8_t pos=0;
	uint8_t imprimir=0;
    uint8_t lastpos=0;
	uint8_t imprimirestatisticas=0;
	uint32_t radioOnTics;

	lastpos = incroute(0xCC);


	if (macRITstate == S_RIT_multichnhello_state) {
		dutycyclecount++;
		//tempo de radio ligado desde o inicio do tx slot
		txidle_dutycle_ticks = (uint16_t) txwaitola;
		//tempo de radio ligado considerando somente apos a recepcao do OLA...
		//auxtxwaitola = (uint16_t) txwaitola;
		radioOnTics = ieee154e_vars.radioRxOnTics + ieee154e_vars.radioTxOnTics;
		RadioRxOn_dutycycle_ticks = (uint16_t) (ieee154e_vars.radioRxOnTics - txwaitola);  //* RADIOTIMER_TICS_MS;
	}
	imprimirestatisticas = 0;
	if ((macRITstate == S_RIT_RX_state) || (macRITstate == S_RIT_RX_livelist)) {
		//imprimir = checkimprimir();
		imprimir = 1;
        if (RxFlagImprimeRoute == 0)
        	imprimirestatisticas = 0;
        else
        	imprimirestatisticas = 0;

    	imprimirestatisticas = 0;
		rffbuf[pos++]= 0x98;  //DEGUB - MOSTRA A ROTA DO SLOT RX
	}
	else if (macRITstate == S_RIT_multichnhello_state){
		imprimir = 1;
		imprimirestatisticas = 0;
		rffbuf[pos++]= 0x97; //DEGUB - MOSTRA A ROTA DO SLOT LIVELIST
	}
	else{
		imprimir = 1;     //DEGUB - MOSTRA A ROTA DO SLOT TX
		if (ieee154e_vars.dataToSend!=NULL) {
			//Quando o datatosend nao esta nulo no endslot significa que algo deu errado...devo fazer um retry
			rffbuf[pos++]= 0xFC;
			rffbuf[pos++]= 0xFC;
			rffbuf[pos++]= 0xFC;
			rffbuf[pos++]= 0xFC;
		}
		else{
			rffbuf[pos++]= 0xFA;
			rffbuf[pos++]= 0xFA;
			rffbuf[pos++]= 0xFA;
			rffbuf[pos++]= 0xFA;
			imprimir = 0;
		}

	}


	if  (imprimir) {

		if (imprimirestatisticas == 0) {
			if (lastpos < MAX_ELEM_ROUTE) {

				if ((macRITstate == S_RIT_RX_state) || (macRITstate == S_RIT_RX_livelist)) {
					switch (macRIT_Pending_RX_frameType) {
						case IANA_ICMPv6_RPL_DIO :
							rffbuf[pos++]=0xD1;
							rffbuf[pos++]=0xD1;
							break;
						case IANA_ICMPv6_RPL_DAO :
							rffbuf[pos++]=0xDA;
							rffbuf[pos++]=0xDA;
							break;
						case IANA_UDP :
							rffbuf[pos++]=0xCA;
							rffbuf[pos++]=0xCA;
							break;
						case IANA_ICMPv6_RA_PREFIX_INFORMATION :
							rffbuf[pos++]=0xAA;
							rffbuf[pos++]=0xAA;
							pos = printvar((uint8_t *)&ritstat.rxola.countdatatxok,sizeof(uint16_t),rffbuf,pos);
							break;
						default:
							rffbuf[pos++]=0xFF;
							rffbuf[pos++]=0xFF;
							break;
					}
				 }
				 else if (macRITstate == S_RIT_TX_state) {
						switch (macRIT_Pending_TX_frameType) {
							case IANA_ICMPv6_RPL_DIO :
								rffbuf[pos++]=0xD1;
								rffbuf[pos++]=0xD1;
								pos = printvar((uint8_t *)&ritstat.txdio.countdatatx,sizeof(uint16_t),rffbuf,pos);
								pos = printvar((uint8_t *)&ritstat.txdio.countdatatxok,sizeof(uint16_t),rffbuf,pos);
								break;
							case IANA_ICMPv6_RPL_DAO :
								rffbuf[pos++]=0xDA;
								rffbuf[pos++]=0xDA;
								pos = printvar((uint8_t *)&ritstat.txdao.countdatatxok,sizeof(uint16_t),rffbuf,pos);
								break;
							case IANA_UDP :
								rffbuf[pos++]=0xCA;
								rffbuf[pos++]=0xCA;
								break;
							case IANA_ICMPv6_RA_PREFIX_INFORMATION :
								rffbuf[pos++]=0xAA;
								rffbuf[pos++]=0xAA;
								break;
							default:
								rffbuf[pos++]=0xFF;
								rffbuf[pos++]=0xFF;
								break;
						}
					 }

				rffbuf[pos++]=0xBB;
				for(i=0;i<lastpos;i++)
				{
					rffbuf[pos++]=ritroute[i];
				}
				if ( pos < (MAX_ELEM_ROUTE)) {
					//rffbuf[pos++]=radio_vars.state;
					if ((macRITstate == S_RIT_TX_state) || (macRITstate == S_RIT_multichnhello_state)) {
#if IEEE802154E_AMAC == 1
						uint16_t txidle= (uint16_t) txidle_dutycle_ticks; //txwaitola - tx04
						uint16_t txola= (uint16_t) radiotxola; //tx07
						uint16_t Rx1= (uint16_t) radiorxdutycycle1-txidle_dutycle_ticks;  //tx09
						uint16_t Rx2= (uint16_t) radiorxdutycycle2 - radiorxdutycycle1;  //tx0d
						uint16_t Tx1= (uint16_t) radiotxdutycycle1-txola;  //tx09  mas nao consigo pegar...
						uint16_t Tx2= (uint16_t) radiotxdutycycle2-radiotxdutycycle1;  //tx0b

						pos = printvar((uint8_t *)&ieee154e_vars.radioRxOnTics,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ieee154e_vars.radioTxOnTics,sizeof(uint16_t),rffbuf,pos);
						//pos = printvar((uint8_t *)&RadioRxOn_dutycycle_ticks,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&txidle,sizeof(uint16_t),rffbuf,pos); //tx04
						pos = printvar((uint8_t *)&txola,sizeof(uint16_t),rffbuf,pos); //tx07
						pos = printvar((uint8_t *)&Rx1,sizeof(uint16_t),rffbuf,pos);  //tx09
						pos = printvar((uint8_t *)&Tx1,sizeof(uint16_t),rffbuf,pos);  //tx09
						pos = printvar((uint8_t *)&Tx2,sizeof(uint16_t),rffbuf,pos);  //tx0b
						pos = printvar((uint8_t *)&Rx2,sizeof(uint16_t),rffbuf,pos);  //tx0d
#elif IEEE802154E_ARM == 1
						uint16_t txidle= (uint16_t) txidle_dutycle_ticks; //txwaitola - tx04
						uint16_t txrts = (uint16_t) radiotxola; //tx06
						uint16_t Rx1 = (uint16_t) radiorxdutycycle1 - txidle_dutycle_ticks;  //tx08
						uint16_t Rx2 = (uint16_t) radiorxdutycycle2 - radiorxdutycycle1;  //tx0c
						uint16_t Tx1 = (uint16_t) radiotxdutycycle1-txrts;  //tx0a
						//uint16_t Tx2 = (uint16_t) radiotxdutycycle2-radiotxdutycycle1;  //tx0b

						pos = printvar((uint8_t *)&ieee154e_vars.radioRxOnTics,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ieee154e_vars.radioTxOnTics,sizeof(uint16_t),rffbuf,pos);
						//pos = printvar((uint8_t *)&RadioRxOn_dutycycle_ticks,sizeof(uint16_t),rffbuf,pos);  //(ieee154e_vars.radioRxOnTics - txwaitola);
						pos = printvar((uint8_t *)&txidle,sizeof(uint16_t),rffbuf,pos);   //tx04
						pos = printvar((uint8_t *)&txrts,sizeof(uint16_t),rffbuf,pos); //tx06
						pos = printvar((uint8_t *)&Rx1,sizeof(uint16_t),rffbuf,pos);  //tx08
						pos = printvar((uint8_t *)&Tx1,sizeof(uint16_t),rffbuf,pos);  //tx0a
						pos = printvar((uint8_t *)&Rx2,sizeof(uint16_t),rffbuf,pos);  //tx0c
#else
						uint16_t txidle= (uint16_t) txidle_dutycle_ticks; //txwaitola - tx04
						uint16_t txola = (uint16_t) radiotxola; //tx06
						uint16_t Rx1 = (uint16_t) radiorxdutycycle1 - txidle_dutycle_ticks;  //tx08
						uint16_t Rx2 = (uint16_t) radiorxdutycycle2 - radiorxdutycycle1;  //tx0c
						uint16_t Tx1 = (uint16_t) radiotxdutycycle1-txola;  //tx0a
						uint16_t Tx2 = (uint16_t) radiotxdutycycle2-radiotxdutycycle1;  //tx0b

						pos = printvar((uint8_t *)&txidle,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ieee154e_vars.radioRxOnTics,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ieee154e_vars.radioTxOnTics,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&RadioRxOn_dutycycle_ticks,sizeof(uint16_t),rffbuf,pos);  //(ieee154e_vars.radioRxOnTics - txwaitola);
						pos = printvar((uint8_t *)&Rx1,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&Rx2,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&txola,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&Tx1,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&Tx2,sizeof(uint16_t),rffbuf,pos);

#endif
					}
					else if ((macRITstate == S_RIT_RX_state) || (macRITstate == S_RIT_RX_livelist)) {
#if IEEE802154E_AMAC == 1
						uint16_t txola= (uint16_t) radiotxola;       //txola
						uint16_t Rx1= (uint16_t) radiorxdutycycle1;  //Rx04
						uint16_t Rx2= (uint16_t) radiorxdutycycle2 - radiorxdutycycle1;  //Rx07
						uint16_t Rx3= (uint16_t) radiorxdutycycle3 - radiorxdutycycle2;  //rx09
						uint16_t Tx1= (uint16_t) radiotxdutycycle1-txola;  //rx06
						uint16_t Tx2= (uint16_t) radiotxdutycycle2 - radiotxdutycycle1;  //rx0d

						pos = printvar((uint8_t *)&ieee154e_vars.radioRxOnTics,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ieee154e_vars.radioTxOnTics,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&txola,sizeof(uint16_t),rffbuf,pos); //rx02
						pos = printvar((uint8_t *)&Rx1,sizeof(uint16_t),rffbuf,pos);   //rx04
						pos = printvar((uint8_t *)&Tx1,sizeof(uint16_t),rffbuf,pos);   //rx06
						pos = printvar((uint8_t *)&Rx2,sizeof(uint16_t),rffbuf,pos);   //rx07
						pos = printvar((uint8_t *)&Rx3,sizeof(uint16_t),rffbuf,pos);   //rx09
						pos = printvar((uint8_t *)&Tx2,sizeof(uint16_t),rffbuf,pos);   //rx0d
#elif IEEE802154E_ARM == 1
						uint16_t txola= (uint16_t) radiotxola;       //rx02
						uint16_t Rx1= (uint16_t) radiorxdutycycle1;  //Rx04
						uint16_t Rx2= (uint16_t) radiorxdutycycle2 - radiorxdutycycle1;  //Rx08
						//uint16_t Rx3= (uint16_t) radiorxdutycycle3 - radiorxdutycycle2;  //rx09
						uint16_t Tx1= (uint16_t) radiotxdutycycle1-txola;   //rx06
						uint16_t Tx2= (uint16_t) radiotxdutycycle2 - radiotxdutycycle1;  //rx0b

						pos = printvar((uint8_t *)&ieee154e_vars.radioRxOnTics,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ieee154e_vars.radioTxOnTics,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&txola,sizeof(uint16_t),rffbuf,pos); //rx02
						pos = printvar((uint8_t *)&Rx1,sizeof(uint16_t),rffbuf,pos);   //rx04
						pos = printvar((uint8_t *)&Tx1,sizeof(uint16_t),rffbuf,pos);   //rx06
						pos = printvar((uint8_t *)&Rx2,sizeof(uint16_t),rffbuf,pos);   //rx08
						pos = printvar((uint8_t *)&Tx2,sizeof(uint16_t),rffbuf,pos);   //rx0b
#else //RITMC
						uint16_t txola= (uint16_t) radiotxola;       //txola
						uint16_t Rx1= (uint16_t) radiorxdutycycle1;  //Rx04
						uint16_t Rx2= (uint16_t) radiorxdutycycle2 - radiorxdutycycle1;  //Rx07
						uint16_t Tx1= (uint16_t) radiotxdutycycle1-txola;  //rx06
						uint16_t Tx2= (uint16_t) radiotxdutycycle2 - radiotxdutycycle1;  //rx0d

						pos = printvar((uint8_t *)&txola,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ieee154e_vars.radioRxOnTics,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ieee154e_vars.radioTxOnTics,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&Rx1,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&Rx2,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&Tx1,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&Tx2,sizeof(uint16_t),rffbuf,pos);
#endif
					}
				}

			}
			else {
				rffbuf[pos++]=0xFF;
				rffbuf[pos++]=0xFF;
				rffbuf[pos++]=0xFF;
				rffbuf[pos++]=0xFF;
			}
		}
		else {  //IMPRIMI ESTATISTICAS
			if (macRITstate == S_RIT_multichnhello_state){
				rffbuf[pos++]=0xAA;
#if (IEEE802154E_RITMC == 1)
				pos = printvar((uint8_t *)&ritstat.txola.countdatatx,sizeof(uint16_t),rffbuf,pos);
				pos = printvar((uint8_t *)&ritmc_txolaack,sizeof(uint16_t),rffbuf,pos);
				pos = printvar((uint8_t *)&ritstat.txola.countdatatxok,sizeof(uint16_t),rffbuf,pos);
				pos = printvar((uint8_t *)&ritstat.txola.countacktxrxok,sizeof(uint16_t),rffbuf,pos);
				rffbuf[pos++]=0xAA;
				pos = printvar((uint8_t *)&ritstat.rxola.countdatatxok,sizeof(uint16_t),rffbuf,pos);
				pos = printvar((uint8_t *)&ritmc_rxolaack,sizeof(uint16_t),rffbuf,pos);
#else
				pos = printvar((uint8_t *)&ritstat.txola.countdatatx,sizeof(uint16_t),rffbuf,pos);
				pos = printvar((uint8_t *)&ritstat.txola.countdatatxok,sizeof(uint16_t),rffbuf,pos);
				pos = printvar((uint8_t *)&ritstat.txola.countacktxrxok,sizeof(uint16_t),rffbuf,pos);
				rffbuf[pos++]=0xAA;
				//pos = printvar((uint8_t *)&ritstat.txola.countdatatxerr,sizeof(uint16_t),rffbuf,pos);
				pos = printvar((uint8_t *)&ritstat.rxola.countdatatxok,sizeof(uint16_t),rffbuf,pos);
				pos = printvar((uint8_t *)&ritstat.rxola.countacktxrxok,sizeof(uint16_t),rffbuf,pos);
#endif
				rffbuf[pos++]=0xEE;
				rffbuf[pos++]=dutycyclecount;
				pos = printvar((uint8_t *)&txidle_dutycle_ticks,sizeof(uint16_t),rffbuf,pos);
				//pos = printvar((uint8_t *)&auxtxwaitola,sizeof(uint16_t),rffbuf,pos);
				pos = printvar((uint8_t *)&RadioRxOn_dutycycle_ticks,sizeof(uint16_t),rffbuf,pos);
			}
			else {
				rffbuf[pos++]=0xEE;
				rffbuf[pos++]=0xEE;
				rffbuf[pos++]=0xEE;
				rffbuf[pos++]=0xEE;

				// pos = printvar((uint8_t *)&ieee154e_dbg.num_txslot,sizeof(uint16_t),rffbuf,pos);
				// pos = printvar((uint8_t *)&ieee154e_dbg.num_txend,sizeof(uint16_t),rffbuf,pos);
				//rffbuf[pos++]=0xEE;
				pos = printvar((uint8_t *)&ritstat.txdio.countdatatxok,sizeof(uint16_t),rffbuf,pos);
				rffbuf[pos++]=0xEE;
				pos = printvar((uint8_t *)&ritstat.rxdio,sizeof(uint16_t),rffbuf,pos);
			}
		}

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	    openserial_startOutput();

   }

}
#endif

#if (DBG_THROUGHPUT == 1)

void printroute(uint8_t macRITstate)
{
	uint8_t i=0;
	uint8_t pos=0;
	uint8_t imprimir=0;
    uint8_t lastpos=0;
	uint8_t imprimirestatisticas=0;
	uint32_t radioOnTics;

	lastpos = incroute(0xCC);


	if (macRITstate == S_RIT_multichnhello_state) {
		dutycyclecount++;
		//tempo de radio ligado desde o inicio do tx slot
		txidle_dutycle_ticks = (uint16_t) txwaitola;
		//tempo de radio ligado considerando somente apos a recepcao do OLA...
		//auxtxwaitola = (uint16_t) txwaitola;
		radioOnTics = ieee154e_vars.radioRxOnTics + ieee154e_vars.radioTxOnTics;
		RadioRxOn_dutycycle_ticks = (uint16_t) (ieee154e_vars.radioRxOnTics - txwaitola);  //* RADIOTIMER_TICS_MS;
	}
	imprimirestatisticas = 0;
	if ((macRITstate == S_RIT_RX_state) || (macRITstate == S_RIT_RX_livelist)) {
		imprimir = checkimprimir();
        if (RxFlagImprimeRoute == 0)
        	imprimirestatisticas = 0;
        else
        	imprimirestatisticas = 0;

    	imprimirestatisticas = 0;
		rffbuf[pos++]= 0x98;  //DEGUB - MOSTRA A ROTA DO SLOT RX
	}
	else if (macRITstate == S_RIT_multichnhello_state){
		imprimir = 1;
		imprimirestatisticas = 0;
		rffbuf[pos++]= 0x97; //DEGUB - MOSTRA A ROTA DO SLOT LIVELIST
	}
	else{
		imprimir = 1;     //DEGUB - MOSTRA A ROTA DO SLOT TX
		if (ieee154e_vars.dataToSend!=NULL) {
			//Quando o datatosend nao esta nulo no endslot significa que algo deu errado...devo fazer um retry
			rffbuf[pos++]= 0xFC;
			rffbuf[pos++]= 0xFC;
			rffbuf[pos++]= 0xFC;
			rffbuf[pos++]= 0xFC;
		}
		else{
			rffbuf[pos++]= 0xFA;
			rffbuf[pos++]= 0xFA;
			rffbuf[pos++]= 0xFA;
			rffbuf[pos++]= 0xFA;
			imprimir = 0;
		}

	}


	if  (imprimir) {

		if (imprimirestatisticas == 0) {
			if (lastpos < MAX_ELEM_ROUTE) {

				if ((macRITstate == S_RIT_RX_state) || (macRITstate == S_RIT_RX_livelist)) {
					switch (macRIT_Pending_RX_frameType) {
						case IANA_ICMPv6_RPL_DIO :
							rffbuf[pos++]=0xD1;
							rffbuf[pos++]=0xD1;
							break;
						case IANA_ICMPv6_RPL_DAO :
							rffbuf[pos++]=0xDA;
							rffbuf[pos++]=0xDA;
							break;
						case IANA_UDP :
							rffbuf[pos++]=0xCA;
							rffbuf[pos++]=0xCA;
							break;
						case IANA_ICMPv6_RA_PREFIX_INFORMATION :
							rffbuf[pos++]=0xAA;
							rffbuf[pos++]=0xAA;
							pos = printvar((uint8_t *)&ritstat.rxola.countdatatxok,sizeof(uint16_t),rffbuf,pos);
							break;
						default:
							rffbuf[pos++]=0xFF;
							rffbuf[pos++]=0xFF;
							break;
					}
				 }
				 else if (macRITstate == S_RIT_TX_state) {
						switch (macRIT_Pending_TX_frameType) {
							case IANA_ICMPv6_RPL_DIO :
								rffbuf[pos++]=0xD1;
								rffbuf[pos++]=0xD1;
								pos = printvar((uint8_t *)&ritstat.txdio.countdatatx,sizeof(uint16_t),rffbuf,pos);
								pos = printvar((uint8_t *)&ritstat.txdio.countdatatxok,sizeof(uint16_t),rffbuf,pos);
								break;
							case IANA_ICMPv6_RPL_DAO :
								rffbuf[pos++]=0xDA;
								rffbuf[pos++]=0xDA;
								pos = printvar((uint8_t *)&ritstat.txdao.countdatatxok,sizeof(uint16_t),rffbuf,pos);
								break;
							case IANA_UDP :
								rffbuf[pos++]=0xCA;
								rffbuf[pos++]=0xCA;
								break;
							case IANA_ICMPv6_RA_PREFIX_INFORMATION :
								rffbuf[pos++]=0xAA;
								rffbuf[pos++]=0xAA;
								break;
							default:
								rffbuf[pos++]=0xFF;
								rffbuf[pos++]=0xFF;
								break;
						}
					 }

				rffbuf[pos++]=0xBB;
				for(i=0;i<lastpos;i++)
				{
					rffbuf[pos++]=ritroute[i];
				}
				if ( pos < (MAX_ELEM_ROUTE)) {
#if (IEEE802154E_AMAC == 1)
					if ((macRITstate == S_RIT_TX_state) || (macRITstate == S_RIT_multichnhello_state)) {
						pos = printvar((uint8_t *)&ritstat.txola.countdatatx,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ritstat.txola.countdatatxok,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ritstat.txola.countacktxrxok,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&counttxrts,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&countrxcts,sizeof(uint16_t),rffbuf,pos);
						rffbuf[pos++]=0xAA;
						//pos = printvar((uint8_t *)&ritstat.txola.countdatatxerr,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ritstat.rxola.countdatatxok,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ritstat.rxola.countacktxrxok,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&countrxrts,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&counttxcts,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&countrxHA,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&counttxcw,sizeof(uint16_t),rffbuf,pos);
					}
					else if ((macRITstate == S_RIT_RX_state) || (macRITstate == S_RIT_RX_livelist)) {
						pos = printvar((uint8_t *)&ritstat.txola.countdatatx,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ritstat.txola.countdatatxok,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ritstat.txola.countacktxrxok,sizeof(uint16_t),rffbuf,pos);
						rffbuf[pos++]=0xAA;
						//pos = printvar((uint8_t *)&ritstat.txola.countdatatxerr,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ritstat.rxola.countdatatxok,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ritstat.rxola.countacktxrxok,sizeof(uint16_t),rffbuf,pos);
					}
#else
					if ((macRITstate == S_RIT_TX_state) || (macRITstate == S_RIT_multichnhello_state)) {
						pos = printvar((uint8_t *)&ritstat.txola.countdatatx,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ritstat.txola.countdatatxok,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ritstat.txola.countacktxrxok,sizeof(uint16_t),rffbuf,pos);
						rffbuf[pos++]=0xAA;
						pos = printvar((uint8_t *)&ritstat.rxola.countdatatxok,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ritstat.rxola.countacktxrxok,sizeof(uint16_t),rffbuf,pos);
					}
					else if ((macRITstate == S_RIT_RX_state) || (macRITstate == S_RIT_RX_livelist)) {
						pos = printvar((uint8_t *)&ritstat.txola.countdatatx,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ritstat.txola.countdatatxok,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ritstat.txola.countacktxrxok,sizeof(uint16_t),rffbuf,pos);
						rffbuf[pos++]=0xAA;
						pos = printvar((uint8_t *)&ritstat.rxola.countdatatxok,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ritstat.rxola.countacktxrxok,sizeof(uint16_t),rffbuf,pos);
					}
#endif

				}

			}
			else {
				rffbuf[pos++]=0xFF;
				rffbuf[pos++]=0xFF;
				rffbuf[pos++]=0xFF;
				rffbuf[pos++]=0xFF;
			}
		}
		else {  //IMPRIMI ESTATISTICAS
			if (macRITstate == S_RIT_multichnhello_state){
				rffbuf[pos++]=0xAA;
#if (IEEE802154E_RITMC == 1)
				pos = printvar((uint8_t *)&ritstat.txola.countdatatx,sizeof(uint16_t),rffbuf,pos);
				pos = printvar((uint8_t *)&ritmc_txolaack,sizeof(uint16_t),rffbuf,pos);
				pos = printvar((uint8_t *)&ritstat.txola.countdatatxok,sizeof(uint16_t),rffbuf,pos);
				pos = printvar((uint8_t *)&ritstat.txola.countacktxrxok,sizeof(uint16_t),rffbuf,pos);
				rffbuf[pos++]=0xAA;
				pos = printvar((uint8_t *)&ritstat.rxola.countdatatxok,sizeof(uint16_t),rffbuf,pos);
				pos = printvar((uint8_t *)&ritmc_rxolaack,sizeof(uint16_t),rffbuf,pos);
#else
				pos = printvar((uint8_t *)&ritstat.txola.countdatatx,sizeof(uint16_t),rffbuf,pos);
				pos = printvar((uint8_t *)&ritstat.txola.countdatatxok,sizeof(uint16_t),rffbuf,pos);
				pos = printvar((uint8_t *)&ritstat.txola.countacktxrxok,sizeof(uint16_t),rffbuf,pos);
				rffbuf[pos++]=0xAA;
				//pos = printvar((uint8_t *)&ritstat.txola.countdatatxerr,sizeof(uint16_t),rffbuf,pos);
				pos = printvar((uint8_t *)&ritstat.rxola.countdatatxok,sizeof(uint16_t),rffbuf,pos);
				pos = printvar((uint8_t *)&ritstat.rxola.countacktxrxok,sizeof(uint16_t),rffbuf,pos);
#endif
				rffbuf[pos++]=0xEE;
				rffbuf[pos++]=dutycyclecount;
				pos = printvar((uint8_t *)&txidle_dutycle_ticks,sizeof(uint16_t),rffbuf,pos);
				//pos = printvar((uint8_t *)&auxtxwaitola,sizeof(uint16_t),rffbuf,pos);
				pos = printvar((uint8_t *)&RadioRxOn_dutycycle_ticks,sizeof(uint16_t),rffbuf,pos);
			}
			else {
				rffbuf[pos++]=0xEE;
				rffbuf[pos++]=0xEE;
				rffbuf[pos++]=0xEE;
				rffbuf[pos++]=0xEE;

				// pos = printvar((uint8_t *)&ieee154e_dbg.num_txslot,sizeof(uint16_t),rffbuf,pos);
				// pos = printvar((uint8_t *)&ieee154e_dbg.num_txend,sizeof(uint16_t),rffbuf,pos);
				//rffbuf[pos++]=0xEE;
				pos = printvar((uint8_t *)&ritstat.txdio.countdatatxok,sizeof(uint16_t),rffbuf,pos);
				rffbuf[pos++]=0xEE;
				pos = printvar((uint8_t *)&ritstat.rxdio,sizeof(uint16_t),rffbuf,pos);
			}
		}

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	    openserial_startOutput();

   }

}
#endif
#if (DBG_OWSN == 1)
void printroute(uint8_t macRITstate)
{
	uint8_t i=0;
	uint8_t pos=0;
	uint8_t imprimir=0;
    uint8_t lastpos=0;
	uint8_t imprimirestatisticas=0;
	uint32_t radioOnTics;

	lastpos = incroute(0xCC);

#if (IEEE802154E_TSCH == 0)
	if (macRITstate == S_RIT_multichnhello_state) {
		dutycyclecount++;
		//tempo de radio ligado desde o inicio do tx slot
		txidle_dutycle_ticks = (uint16_t) txwaitola;
		//tempo de radio ligado considerando somente apos a recepcao do OLA...
		//auxtxwaitola = (uint16_t) txwaitola;

		radioOnTics = ieee154e_vars.radioRxOnTics + ieee154e_vars.radioTxOnTics;
		RadioRxOn_dutycycle_ticks = (uint16_t) (ieee154e_vars.radioRxOnTics - txwaitola);  //* RADIOTIMER_TICS_MS;

	}
	imprimirestatisticas = 0;
	if ((macRITstate == S_RIT_RX_state) || (macRITstate == S_RIT_RX_livelist)) {
		imprimir = checkimprimir();
        if (RxFlagImprimeRoute == 0)
        	imprimirestatisticas = 0;
        else
        	imprimirestatisticas = 0;

    	imprimirestatisticas = 0;
		rffbuf[pos++]= 0x98;  //DEGUB - MOSTRA A ROTA DO SLOT RX
	}
	else if (macRITstate == S_RIT_multichnhello_state){
		imprimir = 1;
		imprimirestatisticas = 1;
		rffbuf[pos++]= 0x97; //DEGUB - MOSTRA A ROTA DO SLOT LIVELIST
	}
	else{
		imprimir = 1;     //DEGUB - MOSTRA A ROTA DO SLOT TX
		if (ieee154e_vars.dataToSend!=NULL) {
			//Quando o datatosend nao esta nulo no endslot significa que algo deu errado...devo fazer um retry
			rffbuf[pos++]= 0xFC;
			rffbuf[pos++]= 0xFC;
			rffbuf[pos++]= 0xFC;
			rffbuf[pos++]= 0xFC;
		}
		else{
			rffbuf[pos++]= 0xFA;
			rffbuf[pos++]= 0xFA;
			imprimir = 1;
		}

	}


	if  (imprimir) {

		if (imprimirestatisticas == 0) {
			if (lastpos < MAX_ELEM_ROUTE) {

				if ((macRITstate == S_RIT_RX_state) || (macRITstate == S_RIT_RX_livelist)) {
					switch (macRIT_Pending_RX_frameType) {
						case IANA_ICMPv6_RPL_DIO :
							rffbuf[pos++]=0xD1;
							rffbuf[pos++]=0xD1;
							break;
						case IANA_ICMPv6_RPL_DAO :
							rffbuf[pos++]=0xDA;
							rffbuf[pos++]=0xDA;
							break;
						case IANA_UDP :
							rffbuf[pos++]=0xCA;
							rffbuf[pos++]=0xCA;
							break;
						case IANA_ICMPv6_RA_PREFIX_INFORMATION :
							rffbuf[pos++]=0xAA;
							rffbuf[pos++]=0xAA;
							pos = printvar((uint8_t *)&ritstat.rxola.countdatatxok,sizeof(uint16_t),rffbuf,pos);
							break;
						default:
							rffbuf[pos++]=0xFF;
							rffbuf[pos++]=0xFF;
							break;
					}
				 }
				 else if (macRITstate == S_RIT_TX_state) {
						switch (macRIT_Pending_TX_frameType) {
							case IANA_ICMPv6_RPL_DIO :
								rffbuf[pos++]=0xD1;
								rffbuf[pos++]=0xD1;
								pos = printvar((uint8_t *)&ritstat.txdio.countdatatx,sizeof(uint16_t),rffbuf,pos);
								pos = printvar((uint8_t *)&ritstat.txdio.countdatatxok,sizeof(uint16_t),rffbuf,pos);
								break;
							case IANA_ICMPv6_RPL_DAO :
								rffbuf[pos++]=0xDA;
								rffbuf[pos++]=0xDA;
								pos = printvar((uint8_t *)&ritstat.txdao.countdatatxok,sizeof(uint16_t),rffbuf,pos);
								break;
							case IANA_UDP :
								rffbuf[pos++]=0xCA;
								rffbuf[pos++]=0xCA;
								break;
							case IANA_ICMPv6_RA_PREFIX_INFORMATION :
								rffbuf[pos++]=0xAA;
								rffbuf[pos++]=0xAA;
								break;
							default:
								rffbuf[pos++]=0xFF;
								rffbuf[pos++]=0xFF;
								break;
						}
					 }

				rffbuf[pos++]=0xBB;
				for(i=0;i<lastpos;i++)
				{
					rffbuf[pos++]=ritroute[i];
				}
				if ( pos < (MAX_ELEM_ROUTE)) {
					if ((macRITstate == S_RIT_TX_state) || (macRITstate == S_RIT_multichnhello_state)) {
						pos = printvar((uint8_t *)&ritstat.txola.countdatatx,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ritstat.txola.countdatatxok,sizeof(uint16_t),rffbuf,pos);
						//pos = printvar((uint8_t *)&ritstat.txola.countacktxrxok,sizeof(uint16_t),rffbuf,pos);
						rffbuf[pos++]=0xD1;
						pos = printvar((uint8_t *)&ritstat.txdio.countdatatx,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ritstat.txdio.countdatatxok,sizeof(uint16_t),rffbuf,pos);
						rffbuf[pos++]=0xDA;
						pos = printvar((uint8_t *)&ritstat.txdao.countdatatx,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ritstat.txdao.countdatatxok,sizeof(uint16_t),rffbuf,pos);
						rffbuf[pos++]=0xCA;
						pos = printvar((uint8_t *)&ritstat.txcoap.countdatatx,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ritstat.txcoap.countdatatxok,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ritstat.rxCoapFwd,sizeof(uint16_t),rffbuf,pos);

					}
					else if ((macRITstate == S_RIT_RX_state) || (macRITstate == S_RIT_RX_livelist)) {
						pos = printvar((uint8_t *)&ritstat.rxola.countdatatxok,sizeof(uint16_t),rffbuf,pos);
						pos = printvar((uint8_t *)&ritstat.rxola.countacktxrxok,sizeof(uint16_t),rffbuf,pos);
					}

				}

			}
			else {
				rffbuf[pos++]=0xFF;
				rffbuf[pos++]=0xFF;
				rffbuf[pos++]=0xFF;
				rffbuf[pos++]=0xFF;
			}
		}
		else {  //IMPRIMI ESTATISTICAS
#if (IEEE802154E_TSCH == 0)
			for (i=0;i<MAXNUMMACNEIGHBORS;i++) {
			  if (macneighbors_vars.neighbors[i].used==TRUE) {
				pos = printaddress(macneighbors_vars.neighbors[i].addr_16b,&rffbuf[0],pos);
				rffbuf[pos++]  = macneighbors_vars.neighbors[i].bestchan;
				rffbuf[pos++]  = macneighbors_vars.neighbors[i].stableNeighbor;
				rffbuf[pos++]  = macneighbors_vars.neighbors[i].broadcastPending;
				//rffbuf[pos++]  = macneighbors_vars.neighbors[i].switchStabilityCounter;
				//pos = printvar((uint8_t *)&macneighbors_vars.neighbors[i].numTxDIO,sizeof(uint8_t),rffbuf,pos);
				//pos = printvar((uint8_t *)&macneighbors_vars.neighbors[i].numTxDIOErr,sizeof(uint8_t),rffbuf,pos);
				//pos = printvar((uint8_t *)&macneighbors_vars.neighbors[i].numTxDAO,sizeof(uint8_t),rffbuf,pos);
				//pos = printvar((uint8_t *)&macneighbors_vars.neighbors[i].numTxDAOErr,sizeof(uint8_t),rffbuf,pos);
			  }
		    }
#endif
		}

		openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
	    openserial_startOutput();
	}
#endif
}

#endif



void treatdebug(uint8_t macRITstate)
{
	if (macRITstate == S_RIT_RX_state)
	    ieee154e_dbg.num_rxend++;
	else
	    ieee154e_dbg.num_txend++;

#if 0
	if ((ieee154e_dbg.num_newSlot % 100) == 0) {
		printstat();
	}
	else{
		printroute(macRITstate);
	}
#else
	printroute(macRITstate);
#endif


}
#endif
#endif
