#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "board.h"
#include "opendefs.h"
#include "opencoap.h"
#include "osens_app.h"
#include "openqueue.h"
//#include "cinfo.h"
#include "osens.h"
#include "packetfunctions.h"
#include "opentimers.h"
#include "scheduler.h"
#include "frmdwn_app.h"
#include "debugpins.h"
#include "openserial.h"
#include "debug.h"
#include "leds.h"

#define TRACE_ON 0

//=========================== variables =======================================

const uint8_t osens_frm_path0 [] = "f";
const uint8_t osens_frm_path1 [] = "i";
coap_resource_desc_t osens_frm_vars;

extern osens_frm_t  osens_frm;

//=========================== prototypes ======================================
owerror_t frmdwn_receive(OpenQueueEntry_t* msg, coap_header_iht*  coap_header, coap_option_iht*  coap_options);
uint8_t *decode_chunk(uint8_t *bufin,uint8_t len,uint8_t *bufout, uint8_t type);
uint8_t *decode_framehexa(uint8_t *bufin,uint8_t len,uint8_t *bufout);

void osens_sendDone(OpenQueueEntry_t* msg, owerror_t error);
void osens_val_sendDone(OpenQueueEntry_t* msg, owerror_t error);
uint8_t osens_get_brd_desc(osens_brd_id_t *brd);

static uint8_t * insert_str(uint8_t *buffer, uint8_t *str, uint32_t len, uint8_t quotes);
static uint8_t * insert_uint(uint8_t *buffer, uint64_t value);

//=========================== public ==========================================

void frmdwn_app_int(void) {
    int32_t i32Res;

    // prepare the resource descriptor for the /d and /s paths
	osens_frm_vars.path0len         = sizeof(osens_frm_path0) -1;
	osens_frm_vars.path0val         = (uint8_t*) (&osens_frm_path0);
	osens_frm_vars.path1len         = sizeof(osens_frm_path1) -1;
	osens_frm_vars.path1val         = (uint8_t*) (&osens_frm_path1);
	osens_frm_vars.componentID      = COMPONENT_CINFO;
	osens_frm_vars.securityContext  = NULL;
	osens_frm_vars.discoverable     = TRUE;
	osens_frm_vars.callbackRx       = &frmdwn_receive;
	osens_frm_vars.callbackSendDone = &osens_sendDone;

	memset(&osens_frm,0,sizeof(osens_frm));

	// register with the CoAP modules
    opencoap_register(&osens_frm_vars);

}

//=========================== private =========================================

owerror_t frmdwn_receive(OpenQueueEntry_t* msg, coap_header_iht*  coap_header, coap_option_iht*  coap_options)
{
	owerror_t outcome = E_FAIL;
	//uint8_t n = 0;
	uint8_t buf[50];
	uint8_t *pucbuf;
	uint8_t *pucAux;
	uint8_t optnum = 0;
	uint8_t optidx = 2;
	uint8_t index=0;
	uint8_t len=0;
	uint8_t type=0;
	uint8_t j=0;
	uint8_t *pu8aux;


    switch (coap_header->Code)
    {
		case COAP_CODE_REQ_GET:
			// reset packet payload
			msg->payload = &(msg->packet[127]);
			msg->length = 0;

			 //=== prepare  CoAP response  - esta resposta eh retirada do comando info
			 if (((coap_options[1].pValue[0] == 'i') || (coap_options[1].pValue[0] == 'I')) &&
					 (coap_options[1].type == COAP_OPTION_NUM_URIPATH)) {

				// stack name and version
				 packetfunctions_reserveHeaderSize(msg,1);
				 msg->payload[0] = '\n';
				 packetfunctions_reserveHeaderSize(msg,sizeof(infoStackName)-1+5);
				 memcpy(&msg->payload[0],&infoStackName,sizeof(infoStackName)-1);
				 msg->payload[sizeof(infoStackName)-1+5-5] = '0'+OPENWSN_VERSION_MAJOR;
				 msg->payload[sizeof(infoStackName)-1+5-4] = '.';
				 msg->payload[sizeof(infoStackName)-1+5-3] = '0'+OPENWSN_VERSION_MINOR;
				 msg->payload[sizeof(infoStackName)-1+5-2] = '.';
				 msg->payload[sizeof(infoStackName)-1+5-1] = '0'+OPENWSN_VERSION_PATCH;

				#if 0 // ENABLE_DEBUG_RFF
				{
					 uint8_t pos=0;

					 rffbuf[pos++]= 0x65;
					 rffbuf[pos++]= 0x81;

					 openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
				}
				#endif

				 // set the CoAP header
				 coap_header->Code                = COAP_CODE_RESP_CONTENT;
				 outcome                          = E_SUCCESS;

			 }
			 else if (((coap_options[1].pValue[0] == 'd') || (coap_options[1].pValue[0] == 'd')) &&
						 (coap_options[1].type == COAP_OPTION_NUM_URIPATH)) {
				 uint8_t *pucaux =  (uint8_t *) &osens_frm.frameID;

				// stack name and version
				 packetfunctions_reserveHeaderSize(msg,4);
/*
				 msg->payload[0] = '0' + *(pucaux+1);
				 msg->payload[1] = '0' + *(pucaux+0);
				 msg->payload[2] = '0' + osens_frm.header;
				 msg->payload[3] = '0' + osens_frm.framestatus;
*/
				 msg->payload[0] =  *(pucaux+1);
				 msg->payload[1] =  *(pucaux+0);
				 msg->payload[2] =  osens_frm.header;
				 msg->payload[3] =  osens_frm.framestatus;

				 // set the CoAP header
				 coap_header->Code                = COAP_CODE_RESP_CONTENT;
				 outcome                          = E_SUCCESS;

			 }
			 break;

		case COAP_CODE_REQ_PUT:
			// reset packet payload

			msg->payload = &(msg->packet[127]);
			msg->length = 0;

			coap_header->Code = COAP_CODE_RESP_VALID;
			outcome = E_SUCCESS;

			 if (((coap_options[1].pValue[0] == 'e') || (coap_options[1].pValue[0] == 'E')) &&
					 (coap_options[1].type == COAP_OPTION_NUM_URIPATH)) {
					//erase the flash firmware new area
					osens_frm.flashnewcmd = iFlashErase;
					coap_header->Code = COAP_CODE_RESP_VALID;
					outcome = E_SUCCESS;

			 }
			 else {
				 //get number of sections
				 optnum = (coap_options[1].pValue[0] - 0x30) + 2;

				if ( (coap_options[1].length == 1)  &&
					 (coap_options[1].type == COAP_OPTION_NUM_URIPATH) &&
					 (optnum > 0)) {

					osens_frm.frameLen = 0;

					//Leitura do buffer da mensagem...o restante da mensagem é codificado 2bytes hexadecimal (AA BB 01 03...)
					if (optnum < 10) {
						 //read payload
						 pucbuf = &buf[0];
						 for (optidx=2;optidx<(optnum+2); optidx++){
							 pucbuf = decode_framehexa(coap_options[optidx].pValue,coap_options[optidx].length,pucbuf);
						 }
					}

					//armazena o nro do frame...
					pucbuf = &buf[0];
					 //get the Frame ID (2 bytes)
				     pu8aux = (uint8_t *) &osens_frm.frameID;
				     *(pu8aux+1) = buf[0];
				     *(pu8aux+0) = buf[1];

				     memcpy((void *) osens_frm.payload,(void *)pucbuf,osens_frm.frameLen);
				     osens_frm.frameLen = osens_frm.frameLen - 1;

					#if 0 // ENABLE_DEBUG_RFF
					{
						 uint8_t pos=0;
						 uint8_t j=0;
						 uint8_t *pucaux = (uint8_t *) osens_frm.payload;
						 uint8_t *pucaux1= (uint8_t *)&osens_frm.frameID;

						 rffbuf[pos++]= RFF_COMPONENT_OPENCOAP_TX+5;
						 rffbuf[pos++]= optnum;
						 rffbuf[pos++]= osens_frm.frameLen;
						 rffbuf[pos++]= *pucaux1++;
						 rffbuf[pos++]= *pucaux1;

						 for (j=0;j<5;j++){
						   rffbuf[pos++] = *pucaux++;
						 }

						 //rffbuf[pos++]= 0xcc;
						 //rffbuf[pos++] = osens_frm.payload [osens_frm.frameLen-4];
						 //rffbuf[pos++] = osens_frm.payload [osens_frm.frameLen-3];
						 //rffbuf[pos++] = osens_frm.payload [osens_frm.frameLen-2];

						 openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
					}
					#endif

					osens_frm.flashnewcmd = iFlashChunck;

				}

				// set the CoAP header
				coap_header->Code  = COAP_CODE_RESP_VALID;
				outcome = E_SUCCESS;
			}


		   break;
		default:
			outcome = E_FAIL;
			break;
	}

    return outcome;
}


uint8_t *decode_framehexa(uint8_t *bufin,uint8_t len,uint8_t *bufout){

	uint8_t *pbuf = bufin;
	uint8_t *pbufout = bufout;
	uint8_t *pbuf_end = bufin + len;
	uint8_t  u8number = 0;
	char bufaux[10];

	while (pbuf < pbuf_end)
	{
		bufaux[0] = *pbuf++;
		bufaux[1] = *pbuf++;
		bufaux[2] = 0x00;

		u8number = strtol((char *) &bufaux[0], NULL, 16);
		*pbufout++ = u8number;
		osens_frm.frameLen++;
	}

	return pbufout;
}
/*
 * transforma o buffer de entrada hexadecimal (ASCII) (2 bytes) em inteiro uint8_t (1,2,4 bytes)
 * ex: bufin = "FFFE0502" -
 *     var1b[0] = 0xFF var1b[1]=FE ....
 *     var2b[0] = 0xFFFE var2b[1] = 0x0502
 *     var4b[0] = 0xFFFE0502
 */

uint8_t *decode_chunk(uint8_t *bufin,uint8_t len,uint8_t *bufout, uint8_t type){

	uint8_t *pbuf = bufin;
	uint8_t *pbufout = bufout;
    uint8_t *pbuf_end = bufin + len;
    uint8_t  u8number = 0;
    uint16_t u16number = 0;
    uint32_t u32number = 0;
    char bufaux[10];

	while (pbuf < pbuf_end)
    {
		switch (type){
		  case iAdLen1B:
				bufaux[0] = *pbuf++;
				bufaux[1] = *pbuf++;
				bufaux[2] = 0x00;

				u8number = strtol((char *) &bufaux[0], NULL, 16);
				*pbufout++ = u8number;
				break;
		  case iAdLen2B:
				bufaux[0] = *pbuf++;
				bufaux[1] = *pbuf++;
				bufaux[2] = *pbuf++;
				bufaux[3] = *pbuf++;
				bufaux[4] = 0x00;

				u16number = strtol((char *) &bufaux[0], NULL, 16);
				*((uint16_t *)pbufout) = u16number;
				pbufout+=2;
				break;
		  case iAdLen4B:
				bufaux[0] = *pbuf++;
				bufaux[1] = *pbuf++;
				bufaux[2] = *pbuf++;
				bufaux[3] = *pbuf++;
				bufaux[4] = *pbuf++;
				bufaux[5] = *pbuf++;
				bufaux[6] = *pbuf++;
				bufaux[7] = *pbuf++;
				bufaux[8] = 0x00;

				u32number = strtol((char *) &bufaux[0], NULL, 16);
				*((uint32_t *)pbufout) = u32number;
				pbufout+=4;
				break;
		  default:
			  break;
		}

    }

	return pbufout;
}

/*
uint8_t osens_get_brd_desc(osens_brd_id_t *brd)
{
	if(sm_state.state >= OSENS_STATE_SEND_PT_DESC)
	{
		memcpy(brd,&board_info,sizeof(osens_brd_id_t));
		return 1;
	}
	else
		return 0;
}
*/


