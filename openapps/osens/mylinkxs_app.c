#include <string.h>
#include <stdint.h>
#include "board.h"
#include "opendefs.h"
#include "opencoap.h"
#include "openqueue.h"
#include "osens.h"
#include "packetfunctions.h"
#include "opentimers.h"
#include "scheduler.h"
#include "osens_app.h"
#include "mylinkxs_app.h"
#include "..\..\bsp\boards\cc2538em\osens_itf\mylinkxs.h"

#if 0

#define TRACE_ON 0

owerror_t osens_desc_receive(OpenQueueEntry_t* msg, coap_header_iht*  coap_header, coap_option_iht*  coap_options);
void osens_desc_sendDone(OpenQueueEntry_t* msg, owerror_t error);
owerror_t osens_val_receive(OpenQueueEntry_t* msg, coap_header_iht*  coap_header, coap_option_iht*  coap_options);
void osens_val_sendDone(OpenQueueEntry_t* msg, owerror_t error);

const uint8_t osens_desc_path0 [] = "d";
coap_resource_desc_t osens_desc_vars;
const uint8_t osens_val_path0 [] = "s";
coap_resource_desc_t osens_val_vars;

static double decode_number(uint8_t *buffer, uint8_t len)
{
    uint8_t *pbuf = buffer;
    uint8_t *pbuf_end = buffer + len;
    uint8_t neg = 0;
    uint8_t frac = 0;
    double div = 10.0;
    double number = 0;

    if (*pbuf == '-')
    {
        neg = 1;
        pbuf++;
    }

    while (pbuf < pbuf_end)
    {
        if (*pbuf == '.')
        {
            frac = frac == 0 ? 1 : frac; // only one period per number
            pbuf++;
            continue;
        }

        if (frac == 1)
        {
            number = number + (*pbuf++ - 0x30) / div;
            div = div * 10;
            // protect div near zero here ?
        }
        else
            number = number * 10 + (*pbuf++ - 0x30);
    }

    if (neg)
        number = -1 * number;

    return number;
}

static uint8_t * insert_str(uint8_t *buffer, uint8_t *str, uint32_t len, uint8_t quotes)
{
    uint8_t *pbuf = buffer;
    if (quotes)
        *pbuf++ = '"';

    memcpy(pbuf, str, len);
    pbuf += len;

    if (quotes)
        *pbuf++ = '"';

    return pbuf;
}

// based on http://stackoverflow.com/questions/9655202/how-to-convert-integer-to-string-in-c
static uint8_t * insert_uint(uint8_t *buffer, uint64_t value)
{
    uint8_t* pbuf = buffer;
    uint8_t *pend;
    uint64_t shifter;
    uint8_t digits[10] = {'0','1','2','3','4','5','6','7','8','9'};

    // count the number of digits
    shifter = value;
    do
    {
        ++pbuf;
        shifter = shifter / 10;
    } while(shifter);
    //*p = '\0';
    pend = pbuf;

    // now fill the digits
    do
    {
        *--pbuf = digits[value % 10];
        value = value / 10;
    } while(value);

    return pend;
}

static uint8_t * insert_int(uint8_t *buffer, int64_t value)
{
    uint8_t* pbuf = buffer;

    // add signal, invert value and call the unsigned version
    if(value < 0)
    {
        *pbuf++ = '-';
        value *= -1;
    }

    return insert_uint(pbuf, (uint64_t) value);
}


static uint8_t * insert_float(uint8_t *buffer, double value)
{
	uint64_t itg, frc;
	uint8_t neg;
	uint8_t* pbuf = buffer;

	neg = value < 0 ? 1 : 0;
	itg = (uint64_t) value;
	frc = (uint64_t) ((value - itg)*100000); // precision

	if(neg)
		*pbuf++ = '-';

	pbuf = insert_uint(pbuf,itg);
	*pbuf++ = '.';
	pbuf = insert_uint(pbuf,frc);

	return pbuf;
}

static uint8_t * insert_point_val(uint8_t *buffer, osens_point_t *point)
{
	uint8_t *pbuf = buffer;
    switch (point->type)
    {
    case OSENS_DT_U8:
    	pbuf = insert_uint(pbuf,(uint64_t) point->value.u8);
        break;
    case OSENS_DT_U16:
    	pbuf = insert_uint(pbuf,(uint64_t) point->value.u16);
        break;
    case OSENS_DT_U32:
    	pbuf = insert_uint(pbuf,(uint64_t) point->value.u32);
        break;
    case OSENS_DT_U64:
    	pbuf = insert_uint(pbuf,(uint64_t) point->value.u64);
        break;
    case OSENS_DT_S8:
    	pbuf = insert_int(pbuf,(uint64_t) point->value.s8);
        break;
    case OSENS_DT_S16:
    	pbuf = insert_int(pbuf,(uint64_t) point->value.s16);
        break;
    case OSENS_DT_S32:
    	pbuf = insert_int(pbuf,(uint64_t) point->value.s32);
        break;
    case OSENS_DT_S64:
    	pbuf = insert_int(pbuf,(uint64_t) point->value.s64);
        break;
    case OSENS_DT_FLOAT:
    	pbuf = insert_float(pbuf,point->value.fp32);
        break;
    case OSENS_DT_DOUBLE:
    	pbuf = insert_float(pbuf,(float) point->value.fp64);
        break;
    default:
        break;
    }

    return pbuf;
}

static void set_point_val(osens_point_t *point, double value)
{
    switch (point->type)
    {
    case OSENS_DT_U8:
    	point->value.u8 = (uint8_t) value;
        break;
    case OSENS_DT_U16:
    	point->value.u16 = (uint16_t) value;
        break;
    case OSENS_DT_U32:
    	point->value.u32 = (uint32_t) value;
        break;
    case OSENS_DT_U64:
    	point->value.u64 = (uint64_t) value;
        break;
    case OSENS_DT_S8:
    	point->value.s8 = (int8_t) value;
        break;
    case OSENS_DT_S16:
    	point->value.s16 = (int16_t) value;
        break;
    case OSENS_DT_S32:
    	point->value.s32 = (int32_t) value;
        break;
    case OSENS_DT_S64:
    	point->value.s64 = (int64_t) value;
        break;
    case OSENS_DT_FLOAT:
    	point->value.fp32 = (float) value;
        break;
    case OSENS_DT_DOUBLE:
    	point->value.fp64 = (double) value;
        break;
    default:
        break;
    }
}

void osens_app_init(void) {

    //osens_init_point_db();
    //osens_mote_init();

    // prepare the resource descriptor for the /d and /s paths
    osens_desc_vars.path0len = sizeof(osens_desc_path0) -1;
    osens_desc_vars.path0val = (uint8_t*) (&osens_desc_path0);
    osens_desc_vars.path1len = 0;
    osens_desc_vars.path1val = NULL;
    osens_desc_vars.componentID = COMPONENT_SENSORS_DESC;
    osens_desc_vars.callbackRx = &osens_desc_receive;
    osens_desc_vars.callbackSendDone = &osens_desc_sendDone;

    osens_val_vars.path0len = sizeof(osens_val_path0) -1;
    osens_val_vars.path0val = (uint8_t*) (&osens_val_path0);
    osens_val_vars.path1len = 0;
    osens_val_vars.path1val = NULL;
    osens_val_vars.componentID = COMPONENT_SENSORS_VAL;
    osens_val_vars.callbackRx = &osens_val_receive;
    osens_val_vars.callbackSendDone = &osens_val_sendDone;

    osens_init();

    // register with the CoAP modules
    opencoap_register(&osens_desc_vars);
    opencoap_register(&osens_val_vars);
}

owerror_t osens_desc_receive(OpenQueueEntry_t* msg, coap_header_iht*  coap_header, coap_option_iht*  coap_options)
{
    owerror_t outcome = E_FAIL;
    uint8_t n = 0;
    static uint8_t buf[128];
    uint8_t *pbuf = &buf[0];

    switch (coap_header->Code)
    {
    case COAP_CODE_REQ_GET:
        // reset packet payload
        msg->payload = &(msg->packet[127]);
        msg->length = 0;

		// /d
        if (((coap_options[1].length == 0) && (coap_options[1].type == COAP_OPTION_NUM_URIPATH)) ||
        				(coap_options[1].type != COAP_OPTION_NUM_URIPATH))
		{
			osens_brd_id_t board_info;

			if(osens_get_brd_desc(&board_info))
			{
				pbuf = insert_str(pbuf, (uint8_t*)"{\"v\":", 5,0);
				pbuf = insert_uint(pbuf,board_info.hardware_revision);
				pbuf = insert_str(pbuf, (uint8_t*)",\"m\":", 5,0);
				pbuf = insert_str(pbuf, board_info.model, strlen((char *) board_info.model),1);
				pbuf = insert_str(pbuf, (uint8_t*)",\"id\":", 6,0);
				pbuf = insert_uint(pbuf,board_info.sensor_id);
				pbuf = insert_str(pbuf, (uint8_t*)",\"npts\":", 8,0);
				pbuf = insert_uint(pbuf,board_info.num_of_points);
				pbuf = insert_str(pbuf, (uint8_t*)"}", 1,0);
			}
			else
				pbuf = insert_str(pbuf,(uint8_t*)"{}",2,0);

			outcome = E_SUCCESS;
		} // /d/pt/1 or /d/pt/12
		else if (coap_options[1].length == 2 &&
				coap_options[1].pValue[0] == 'p' &&
				coap_options[1].pValue[1] == 't' &&
				(coap_options[2].length == 1 || coap_options[2].length == 2))
		{
			osens_point_desc_t pt_desc;
			uint8_t index;

			if(coap_options[2].length == 2)
				index = (coap_options[2].pValue[0] - 0x30) * 10 + (coap_options[2].pValue[1] - 0x30);
			else
				index = coap_options[2].pValue[0] - 0x30;

			if(osens_get_pdesc(index,&pt_desc))
			{
				pbuf = insert_str(pbuf, (uint8_t*)"{\"n\":", 5, 0);
				pbuf = insert_str(pbuf, pt_desc.name, strlen((char*) pt_desc.name), 1);
				pbuf = insert_str(pbuf, (uint8_t*)",\"t\":", 5, 0);
				pbuf = insert_uint(pbuf, pt_desc.type);
				pbuf = insert_str(pbuf, (uint8_t*)",\"u\":", 5, 0);
				pbuf = insert_uint(pbuf, pt_desc.unit);
				pbuf = insert_str(pbuf, (uint8_t*)",\"ar\":", 6, 0);
				pbuf = insert_uint(pbuf, pt_desc.access_rights);
				pbuf = insert_str(pbuf, (uint8_t*)",\"s\":", 5, 0);
				pbuf = insert_uint(pbuf, pt_desc.sampling_time_x250ms);
				pbuf = insert_str(pbuf, (uint8_t*)"}", 1, 0);
			}
			else
				pbuf = insert_str(pbuf,(uint8_t*)"{}",2,0);

			outcome = E_SUCCESS;
		}

		if(outcome == E_SUCCESS)
		{
			n = ((uint32_t)pbuf - (uint32_t)buf);
			packetfunctions_reserveHeaderSize(msg, 1 + n);
			msg->payload[0] = COAP_PAYLOAD_MARKER;
			memcpy(&msg->payload[1], buf, n);
			coap_header->Code = COAP_CODE_RESP_CONTENT;
		}

        break;

    case COAP_CODE_REQ_PUT:
        // reset packet payload
        msg->payload = &(msg->packet[127]);
        msg->length = 0;

        // set the CoAP header
        coap_header->Code = COAP_CODE_RESP_CHANGED;

        outcome = E_SUCCESS;
        break;

    default:
        outcome = E_FAIL;
        break;
    }

    return outcome;
}

owerror_t osens_val_receive(
    OpenQueueEntry_t* msg,
    coap_header_iht*  coap_header,
    coap_option_iht*  coap_options
    ) {
    owerror_t outcome = E_FAIL;
    uint8_t n;
    static uint8_t buf[128];
    uint8_t *pbuf = &buf[0];

    switch (coap_header->Code) {
    case COAP_CODE_REQ_GET:
        // reset packet payload
        msg->payload = &(msg->packet[127]);
        msg->length = 0;
        // /s
		if (((coap_options[1].length == 0) && (coap_options[1].type == COAP_OPTION_NUM_URIPATH)) ||
				(coap_options[1].type != COAP_OPTION_NUM_URIPATH))
		{
		} // /s/1 or /s/12
		else if(((coap_options[1].length == 1 || coap_options[1].length == 2)) &&
				(coap_options[1].type == COAP_OPTION_NUM_URIPATH))
		{

			uint8_t index;
			osens_point_t pt;

			if(coap_options[1].length == 2)
				index = (coap_options[1].pValue[0] - 0x30) * 10 + (coap_options[1].pValue[1] - 0x30);
			else
				index = coap_options[1].pValue[0] - 0x30;

#if 0
			if(get_point(index,&pt))
			{
				pbuf = insert_str(pbuf,(uint8_t*)"{\"v\":",5,0);
				pbuf = insert_point_val(pbuf,&pt);
				pbuf = insert_str(pbuf,(uint8_t*)"}",1,0);
			}
			else
				pbuf = insert_str(pbuf,(uint8_t*)"{}",2,0);
#else

    pt.type = OSENS_DT_U8;
    pt.value.u8 = light_get_value();

	pbuf = insert_str(pbuf,(uint8_t*)"{\"v\":",5,0);
	pbuf = insert_point_val(pbuf,&pt);
	pbuf = insert_str(pbuf,(uint8_t*)"}",1,0);


#endif
			outcome = E_SUCCESS;

		}

		if(outcome == E_SUCCESS)
		{
			n = ((uint32_t)pbuf - (uint32_t)buf);
			packetfunctions_reserveHeaderSize(msg, 1 + n);
			msg->payload[0] = COAP_PAYLOAD_MARKER;
			memcpy(&msg->payload[1], buf, n);
			coap_header->Code = COAP_CODE_RESP_CONTENT;
		}

        break;

    case COAP_CODE_REQ_PUT:
        // reset packet payload
        msg->payload = &(msg->packet[127]);
        msg->length = 0;

        // /s/2/-12.45 or /s/12/12.45
		if((coap_options[1].length == 1 || coap_options[1].length == 2) &&
				(coap_options[2].length > 0) &&
				(coap_options[1].type == COAP_OPTION_NUM_URIPATH) &&
				(coap_options[2].type == COAP_OPTION_NUM_URIPATH))
		{
			uint8_t index;
			double number;
			osens_point_t pt;
#if 0
			if(coap_options[1].length == 2)
				index = (coap_options[1].pValue[0] - 0x30) * 10 + (coap_options[1].pValue[1] - 0x30);
			else
				index = coap_options[1].pValue[0] - 0x30;

			number = decode_number(coap_options[2].pValue,coap_options[2].length);
			pt.type = osens_get_ptype(index);

			if(pt.type >= 0)
			{
				set_point_val(&pt,number);
				if(osens_set_pvalue(index,&pt))
				{
					// set the CoAP header
					coap_header->Code = COAP_CODE_RESP_CHANGED;
					outcome = E_SUCCESS;
				}
			}
#else
		      // switch on the light pulse (50 ms)
		      light_on();

		      opentimers_start(1000,TIMER_ONESHOT,TIME_MS,light_timer);
#endif

		}
        break;

    default:
        outcome = E_FAIL;
        break;
    }

    return outcome;
}

void osens_desc_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
    openqueue_freePacketBuffer(msg);
}

void osens_val_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
    openqueue_freePacketBuffer(msg);
}


void light_timer() {
   // switch off the light pulse
   light_off();
}

#endif

