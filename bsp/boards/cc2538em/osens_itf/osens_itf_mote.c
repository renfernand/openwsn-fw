#include "opendefs.h"
#include "board.h"
#include "stdio.h"
#include <string.h>
#include "hw_ioc.h"             // Access to IOC register defines
#include "hw_ssi.h"             // Access to SSI register defines
#include "hw_sys_ctrl.h"        // Clocking control
#include "hw_memmap.h"
#include "ioc.h"                // Access to driverlib ioc fns
#include "gpio.h"               // Access to driverlib gpio fns
#include "osens.h"
#include "osens_itf.h"
#include "opentimers.h"
#include "scheduler.h"
#include "board.h"
#include "sys_ctrl.h"
#include "osens_itf_mote.h"
#include "uarthal.h"
#include "leds.h"
#include "uart.h"
#include "debugpins.h"
#include "debug.h"
#if SENSOR_ACCEL
#include "acc_bma250.h"
#endif

#define TRACE_ON 0

// minimum tick time is 1
#define MS2TICK(ms) (ms) > OSENS_SM_TICK_MS ? (ms) / OSENS_SM_TICK_MS : 1

#define DBG_READ  0
#define DBG_WRITE 1

#if (MYLINKXS_REMOTE_CONTROL == 1)
uint8_t flagwriteenable=0;
void simula_envio_cmd(osens_point_t *pt);

#endif
uint8_t count = 0;

#if (SONOMA14 == 1)
//enderecos SONOMA
#define SONOMA_ADDR_CMD        0x00
#define SONOMA_ADDR_FWDATE     0x01
#define SONOMA_ADDR_SAMPLES    0x08     //DEFAULT 400
#define SONOMA_ADDR_S0_GAIN    0x0D
#define SONOMA_ADDR_S1_GAIN    0x0C
#define SONOMA_ADDR_S2_GAIN    0x0F
#define SONOMA_ADDR_S3_GAIN    0x0E
#define SONOMA_ADDR_S0_OFFSET  0x10
#define SONOMA_ADDR_S2_OFFSET  0x11
#define SONOMA_ADDR_S1_OFFSET  0x12
#define SONOMA_ADDR_S3_OFFSET  0x13
#define SONOMA_ADDR_T_GAIN     0x14
#define SONOMA_ADDR_T_OFFSET   0x15
#define SONOMA_ADDR_VA_RMS     0x2B
#define SONOMA_ADDR_UART_BAUD  0xB1  //UART Baudrate 9600
#endif

enum {
    OSENS_STATE_INIT = 0,
    OSENS_STATE_SEND_ITF_VER = 1,
    OSENS_STATE_WAIT_ITF_VER_ANS = 2,
    OSENS_STATE_PROC_ITF_VER = 3,
    OSENS_STATE_SEND_BRD_ID = 4,
    OSENS_STATE_WAIT_BRD_ID_ANS = 5,
    OSENS_STATE_PROC_BRD_ID = 6,
    OSENS_STATE_SEND_PT_DESC = 7,
    OSENS_STATE_WAIT_PT_DESC_ANS = 8,
    OSENS_STATE_PROC_PT_DESC = 9,
    OSENS_STATE_BUILD_SCH = 10,
    OSENS_STATE_RUN_SCH = 11,
    OSENS_STATE_SEND_PT_VAL = 12,
    OSENS_STATE_WAIT_PT_VAL_ANS = 13,
    OSENS_STATE_PROC_PT_VAL = 14,
    OSENS_STATE_WR_PT = 15,
    OSENS_STATE_WAIT_WR_PT_ANS = 16,
    OSENS_STATE_PROC_WR_PT_ANS = 17
};

#if TRACE_ON == 1
uint8_t *sm_states_str[] = {
    "INIT",
    "SEND_ITF_VER",
    "WAIT_ITF_VER_ANS",
    "PROC_ITF_VER",
    "SEND_BRD_ID",
    "WAIT_BRD_ID_ANS",
    "PROC_BRD_ID",
    "SEND_PT_DESC",
    "WAIT_PT_DESC_ANS",
    "PROC_PT_DESC",
    "BUILD_SCH",
    "RUN_SCH",
    "SEND_PT_VAL",
    "WAIT_PT_VAL_ANS",
    "PROC_PT_VAL",
    "WR_PT",
    "WAIT_WR_PT_ANS",
    "PROC_WR_PT_ANS"
};
#endif

enum {
	OSENS_STATE_EXEC_OK = 0,
	OSENS_STATE_EXEC_WAIT_OK,
	OSENS_STATE_EXEC_WAIT_STOP,
	OSENS_STATE_EXEC_WAIT_ABORT,
	OSENS_STATE_EXEC_ERROR
};


typedef uint8_t (*osens_mote_sm_func_t)(osens_mote_sm_state_t *st);

typedef struct osens_mote_sm_table_s
{
	osens_mote_sm_func_t func;
	uint8_t next_state;
	uint8_t abort_state; // for indicating timeout or end of cyclic operation
	uint8_t error_state;
} osens_mote_sm_table_t;


typedef struct osens_acq_schedule_s
{
	uint8_t num_of_points;
	struct points_e
	{
		uint8_t index;
		uint32_t sampling_time_x250ms;
        uint32_t counter;
	} points[OSENS_MAX_POINTS];

	struct scan_e
	{
		uint8_t num_of_points;
		uint8_t index[OSENS_MAX_POINTS];
	} scan;

	struct write_e
	{
		uint8_t prod;
		uint8_t cons;
		uint8_t index[OSENS_MAX_POINTS];
	} write;
} osens_acq_schedule_t;

#define PC_INC_QUEUE(v,mv) (((v) + 1) >= (mv)) ? 0 : (v) + 1

static uint8_t osens_mote_sm_func_build_sch(osens_mote_sm_state_t *st);
static uint8_t osens_mote_sm_func_pt_desc_ans(osens_mote_sm_state_t *st);
static uint8_t osens_mote_sm_func_req_pt_desc(osens_mote_sm_state_t *st);
static uint8_t osens_mote_sm_func_run_sch(osens_mote_sm_state_t *st);

const osens_mote_sm_table_t osens_mote_sm_table[];
const uint8_t datatype_sizes[] = { 1, 1, 2, 2, 4, 4, 8, 8, 4, 8 }; // check osens_datatypes_e order

uint8_t num_rx_bytes;
uint8_t tx_data_len;
uint32_t flagErrorOccurred;

uint8_t frame[OSENS_MAX_FRAME_SIZE];

osens_cmd_req_t cmd;
osens_cmd_res_t ans;

osens_mote_sm_state_t sm_state;
osens_point_ctrl_t sensor_points;
osens_brd_id_t board_info;
osens_acq_schedule_t schedule;


//=========================== prototypes =======================================
//=========================== public ==========================================
void sensor_timer(void);

#if (MYLINKXS_LIGHT_CONTROL == 0)
static void osens_mote_tick(void);
#endif

static void buBufFlush(void);
//void bspLedToggle(uint8_t ui8Leds);

#if TRACE_ON == 1
static void osens_mote_show_values(void)
{
    uint8_t n;
    OS_UTIL_LOG(1, ("\n"));
    for (n = 0; n < sensor_points.num_of_points; n++)
    {
        switch (sensor_points.points[n].value.type)
        {
        case OSENS_DT_U8:
            OS_UTIL_LOG(1, ("Point %02d: %d\n", n, sensor_points.points[n].value.value.u8));
            break;
        case OSENS_DT_S8:
            OS_UTIL_LOG(1, ("Point %02d: %d\n", n, sensor_points.points[n].value.value.s8));
            break;
        case OSENS_DT_U16:
            OS_UTIL_LOG(1, ("Point %02d: %d\n", n, sensor_points.points[n].value.value.u16));
            break;
        case OSENS_DT_S16:
            OS_UTIL_LOG(1, ("Point %02d: %d\n", n, sensor_points.points[n].value.value.s16));
            break;
        case OSENS_DT_U32:
            OS_UTIL_LOG(1, ("Point %02d: %d\n", n, sensor_points.points[n].value.value.u32));
            break;
        case OSENS_DT_S32:
            OS_UTIL_LOG(1, ("Point %02d: %d\n", n, sensor_points.points[n].value.value.s32));
            break;
        case OSENS_DT_U64:
            OS_UTIL_LOG(1, ("Point %02d: %d\n", n, sensor_points.points[n].value.value.u64));
            break;
        case OSENS_DT_S64:
            OS_UTIL_LOG(1, ("Point %02d: %d\n", n, sensor_points.points[n].value.value.s64));
            break;
        case OSENS_DT_FLOAT:
            OS_UTIL_LOG(1, ("Point %02d: %f\n", n, sensor_points.points[n].value.value.fp32));
            break;
        case OSENS_DT_DOUBLE:
            OS_UTIL_LOG(1, ("Point %02d: %f\n", n, sensor_points.points[n].value.value.fp64));
            break;
        default:
            break;
       }
    }
}
#endif








static uint8_t osens_mote_pack_send_frame(osens_cmd_req_t *cmd, uint8_t cmd_size)
{
    uint8_t size;

    size = osens_pack_cmd_req(cmd, frame);

    if (size != cmd_size)
        return OSENS_STATE_EXEC_ERROR;

#if (MYLINKXS_REMOTE_CONTROL == 1)
    //Devido ao arduino estou colocando um byte a mais de final de frame 'Z'=0x5A
	frame[cmd_size] = BYTE_END_FRAME;
	cmd_size++;

    DBG_LOG(DBG_READ,("Req=%x %x %x %x\n",frame[0],frame[1],frame[2],frame[3]));


    osens_mote_send_frame(frame, cmd_size);



#else
    	if (osens_mote_send_frame(frame, cmd_size) != cmd_size)
    	return OSENS_STATE_EXEC_ERROR;
#endif

    return OSENS_STATE_EXEC_OK;
}

static uint8_t osens_mote_sm_func_pt_val_ans(osens_mote_sm_state_t *st)
{
	uint8_t point;
	uint8_t size;
    uint8_t ans_size;

    point = schedule.scan.index[st->point_index];
    ans_size = 6 + datatype_sizes[sensor_points.points[point].desc.type];

    size = osens_unpack_cmd_res(&ans, frame, ans_size);

#if (MYLINKXS_REMOTE_CONTROL == 0)

    // retry ?
    if (size != ans_size || ans.hdr.addr != (OSENS_REGMAP_READ_POINT_DATA_1 + point))
        return OSENS_STATE_EXEC_OK;

    // ok, save and go to the next
    memcpy(&sensor_points.points[point].value, &ans.payload.point_value_cmd, sizeof(osens_point_t));
#else
    memcpy(&sensor_points.points[point].value, &ans.payload.point_value_cmd, sizeof(osens_point_t));

    DBG_LOG(DBG_READ,("Res=%d %s %x \n",point,sensor_points.points[point].desc.name, sensor_points.points[point].value.value.u16));
	leds_error_toggle();

#endif  // if (MYLINKXS_REMOTE_CONTROL == 0)

    st->retries = 0;
    st->point_index++;

#if TRACE_ON == 1
    osens_mote_show_values();
#endif

    return OSENS_STATE_EXEC_OK;
}

#if MYLINKXS_REMOTE_CONTROL

void simula_envio_cmd(osens_point_t *pt)
{
    flagwriteenable = 1;
}
#endif

static uint8_t osens_mote_sm_func_req_pt_val(osens_mote_sm_state_t *st)
{
	uint8_t point;
#if MYLINKXS_REMOTE_CONTROL
	uint8_t uc;
	uint8_t ucBotao=0;
	osens_point_t pt;
#endif

	// end of point reading
    if(st->point_index >= schedule.scan.num_of_points)
    	return OSENS_STATE_EXEC_WAIT_ABORT;

    // error condition after 3 retries
	st->retries++;
	if(st->retries > 3)
		return OSENS_STATE_EXEC_ERROR;

	point = schedule.scan.index[st->point_index];
    cmd.hdr.addr = OSENS_REGMAP_READ_POINT_DATA_1 + point;
    st->trmout_counter = 0;
    st->trmout = MS2TICK(5000);

#if MYLINKXS_REMOTE_CONTROL
	//PROVISORIO!!!!
	ucBotao = GPIOPinRead(GPIO_A_BASE, GPIO_PIN_3);
	if ((ucBotao & GPIO_PIN_3) == 0) {
		pt.type = OSENS_DT_U16;
		pt.value.u16 = 0x820;
		sm_state.state = OSENS_STATE_WR_PT;
		simula_envio_cmd(&pt);
		osens_set_pvalue(0,&pt);
	}
#endif

	return osens_mote_pack_send_frame(&cmd, 4);

}

#if 0
static uint8_t osens_mote_sm_func_proc_wr_pt(osens_mote_sm_state_t *st)
{
	uint8_t point;
	uint8_t size;
	uint8_t ans_size = 5;

    point = schedule.write.index[st->point_index];

#if (MYLINKXS_REMOTE_CONTROL == 1)
   if(flagwriteenable)
   {
	   flagwriteenable = 0;
	   st->retries = 0;
   }
#endif

   size = osens_unpack_cmd_res(&ans, frame, ans_size);

	// retry ?
	if (size != ans_size || ans.hdr.addr != (OSENS_REGMAP_WRITE_POINT_DATA_1 + point))
		return OSENS_STATE_EXEC_OK;

    // ok,  go to the next
    schedule.write.cons = PC_INC_QUEUE(schedule.write.cons, OSENS_MAX_POINTS);
    st->retries = 0;

    return OSENS_STATE_EXEC_OK;
}
#else
static uint8_t osens_mote_sm_func_proc_wr_pt(osens_mote_sm_state_t *st)
{
	uint8_t point;
	uint8_t size;
	uint8_t ans_size = 5;

    point = schedule.write.index[st->point_index];
#if (MYLINKXS_REMOTE_CONTROL == 1)
   if(flagwriteenable)
   {
	   flagwriteenable = 0;
	   st->retries = 0;
   }
#endif

   size = osens_unpack_cmd_res(&ans, frame, ans_size);

    // retry ?
    if (size != ans_size || ans.hdr.addr != (OSENS_REGMAP_WRITE_POINT_DATA_1 + point))
    {
	  // DBG_LOG(DBG_WRITE,("WrRes ERROR I:%x R:%x L:%x PAR:%x \n",st->point_index,st->retries,size,ans.hdr.addr));
       return OSENS_STATE_EXEC_OK;
    }

    // ok,  go to the next
    //DBG_LOG(DBG_WRITE,("WrRes OK I:%x R:%x L:%x PAR:%x \n",st->point_index,st->retries,size,ans.hdr.addr));
    schedule.write.cons = PC_INC_QUEUE(schedule.write.cons, OSENS_MAX_POINTS);
    st->retries = 0;

    return OSENS_STATE_EXEC_OK;
}
#endif

static uint8_t osens_mote_sm_func_wr_pt(osens_mote_sm_state_t *st)
{
	uint8_t point;
	uint8_t c = schedule.write.cons;
	uint8_t p = schedule.write.prod;
	uint8_t size;

    //DBG_LOG(0,("WrReq0=%x %x %x\n",c,p,st->retries));
 	// end of point writing
	if(c == p)
		return OSENS_STATE_EXEC_WAIT_ABORT;

    // error condition after 3 retries
	st->retries++;
	if(st->retries > 3)
		return OSENS_STATE_EXEC_ERROR;

#if TRACE_ON == 1
    printf("==> Consuming at position %d\n", c);
#endif

    st->point_index = c;
    point = schedule.write.index[st->point_index];
    cmd.hdr.addr = OSENS_REGMAP_WRITE_POINT_DATA_1 + point;
    st->trmout_counter = 0;
    st->trmout = MS2TICK(5000);

    size = 5 + datatype_sizes[sensor_points.points[point].desc.type];
    memcpy(&cmd.payload.point_value_cmd, &sensor_points.points[point].value, sizeof(osens_point_t));

    //DBG_LOG(DBG_WRITE,("WrReq I:%x T:%x V:%x\n",point, sensor_points.points[point].desc.type, sensor_points.points[point].value.value.u16));

    return osens_mote_pack_send_frame(&cmd,size);
}


#if (USE_SPI_INTERFACE == 0)
static uint8_t osens_mote_sm_func_run_sch(osens_mote_sm_state_t *st){
    uint8_t n;
#if 0
	// priorize writings over data scan/schedule execution
	if(schedule.write.prod != schedule.write.cons)
	{
    	st->retries = 0;
		return OSENS_STATE_EXEC_ERROR;
	}
#endif
    schedule.scan.num_of_points = 0;

    for (n = 0; n < schedule.num_of_points; n++)
    {

	    if(schedule.points[n].counter > 0)
    		schedule.points[n].counter--;

        if (schedule.points[n].counter == 0)
        {
            // n: point index in the schedule database
            // index: point index in the points database
            uint8_t index = schedule.points[n].index;

    	    //DBG_LOG(0,("Sch n:%d idx:%d \n",n,schedule.points[n].index));

            schedule.scan.index[schedule.scan.num_of_points] = index;
        	schedule.scan.num_of_points++;
            // restore counter value for next cycle
            schedule.points[n].counter = schedule.points[n].sampling_time_x250ms;
        }

    }

    if(schedule.scan.num_of_points > 0)
    {
    	st->point_index = 0;
    	st->retries = 0;

     	return OSENS_STATE_EXEC_WAIT_ABORT;
    }
    else
    	return OSENS_STATE_EXEC_WAIT_OK;

}

#else

static uint8_t osens_mote_sm_func_run_sch(osens_mote_sm_state_t *st) {
#if (SONOMA14 == 1)
    uint32_t param[5];
#else
    int8_t i8Data[6] = {0};
    uint16_t u16X, u16Y, u16Z;
#endif

    //leds_debug_toggle();

#if (SONOMA14 == 1)

    param[0] = read_reg(SONOMA_ADDR_VA_RMS);
    param[1] = read_reg(SONOMA_ADDR_S0_GAIN);
    param[2] = read_reg(SONOMA_ADDR_S1_GAIN);
    param[3] = read_reg(SONOMA_ADDR_S0_OFFSET);
    param[4] = read_reg(SONOMA_ADDR_S1_OFFSET);

    sensor_points.points[0].value.value.u32 = param[0];
    sensor_points.points[1].value.value.u32 = param[1];
    sensor_points.points[2].value.value.u32 = param[3];

#endif

#if (SENSOR_ACCEL == 1)
    accReadReg2(ACC_X_LSB, (uint8_t *)i8Data, 6);

    u16X = (((uint16_t)(i8Data[1]) << 2) | ((i8Data[0] >> 6) & 0x03));
    u16Y = (((uint16_t)(i8Data[3]) << 2) | ((i8Data[2] >> 6) & 0x03));
    u16Z = (((uint16_t)(i8Data[5]) << 2) | ((i8Data[4] >> 6) & 0x03));

    sensor_points.points[0].value.value.u16 = u16X;
    sensor_points.points[1].value.value.u16 = u16Y;
    sensor_points.points[2].value.value.u16 = u16Z;
#endif

#if 0 //ENABLE_DEBUG_RFF
	 uint8_t pos=0;
	 uint8_t *paux;

	 rffbuf[pos++]= 0x80;
	 paux = (uint8_t*) &param[0];
	 rffbuf[pos++]= *paux++;
	 rffbuf[pos++]= *paux++;
	 rffbuf[pos++]= *paux++;
	 rffbuf[pos++]= *paux++;
	 paux = (uint8_t*) &param[1];
	 rffbuf[pos++]= *paux++;
	 rffbuf[pos++]= *paux++;
	 rffbuf[pos++]= *paux++;
	 rffbuf[pos++]= *paux++;
	 paux = (uint8_t*) &param[2];
	 rffbuf[pos++]= *paux++;
	 rffbuf[pos++]= *paux++;
	 rffbuf[pos++]= *paux++;
	 rffbuf[pos++]= *paux++;

     openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
#endif

	return OSENS_STATE_EXEC_WAIT_OK;
}
#endif

static uint8_t osens_mote_sm_func_build_sch(osens_mote_sm_state_t *st)
{
	uint8_t n, m;

    schedule.num_of_points = 0;
#if 0
    for (n = 0, m = 0; n < board_info.num_of_points; n++)
    {
        if ((sensor_points.points[n].desc.access_rights & OSENS_ACCESS_READ_ONLY) &&
            (sensor_points.points[n].desc.sampling_time_x250ms > 0))
        {
            schedule.points[m].index = n;
            schedule.points[m].counter = sensor_points.points[n].desc.sampling_time_x250ms;
            schedule.points[m].sampling_time_x250ms = sensor_points.points[n].desc.sampling_time_x250ms;

            m++;
            schedule.num_of_points++;
        }
    }
#endif
#if TRACE_ON == 1
    OS_UTIL_LOG(1,("\n"));
    OS_UTIL_LOG(1, ("Schedule\n"));
    OS_UTIL_LOG(1, ("========\n"));
    for (n = 0; n < schedule.num_of_points; n++)
    {
        OS_UTIL_LOG(1, ("[%d] point %02d at %dms\n", n, schedule.points[n].index, schedule.points[n].sampling_time_x250ms * 250));
    }
#endif

    return OSENS_STATE_EXEC_OK;
}

static uint8_t osens_mote_sm_func_pt_desc_ans(osens_mote_sm_state_t *st)
{
	uint8_t size;
    uint8_t ans_size = 20;

    size = osens_unpack_cmd_res(&ans, frame, ans_size);

    if (size != ans_size || (ans.hdr.addr != OSENS_REGMAP_POINT_DESC_1 + st->point_index))
        return OSENS_STATE_EXEC_ERROR;

    // save description and type, value is not available yet
    memcpy(&sensor_points.points[st->point_index].desc,&ans.payload.point_desc_cmd,sizeof(osens_point_desc_t));
    sensor_points.points[st->point_index].value.type = sensor_points.points[st->point_index].desc.type;

#if TRACE_ON == 1
    {
        uint8_t n = st->point_index;
        OS_UTIL_LOG(1, ("\n"));
        OS_UTIL_LOG(1, ("Point %02d info\n", n));
        OS_UTIL_LOG(1, ("=============\n"));
        OS_UTIL_LOG(1, ("Name     : %-8s\n", sensor_points.points[n].desc.name));
        OS_UTIL_LOG(1, ("Type     : %d\n", sensor_points.points[n].desc.type));
        OS_UTIL_LOG(1, ("Unit     : %d\n", sensor_points.points[n].desc.unit));
        OS_UTIL_LOG(1, ("Rights   : %02X\n", sensor_points.points[n].desc.access_rights));
        OS_UTIL_LOG(1, ("Sampling : %d\n\n", sensor_points.points[n].desc.sampling_time_x250ms));
    }
#endif

    st->retries = 0;
    st->point_index++;
    sensor_points.num_of_points = st->point_index;

    return OSENS_STATE_EXEC_OK;
}

static uint8_t osens_mote_sm_func_req_pt_desc(osens_mote_sm_state_t *st)
{
    if(st->point_index >= board_info.num_of_points)
    	return OSENS_STATE_EXEC_WAIT_ABORT;

    // error condition after 3 retries
    st->retries++;
    if (st->retries > 3)
        return OSENS_STATE_EXEC_ERROR;

    cmd.hdr.addr = OSENS_REGMAP_POINT_DESC_1 + st->point_index;
    st->trmout_counter = 0;
    st->trmout = MS2TICK(5000);
    return osens_mote_pack_send_frame(&cmd, 4);
}

static uint8_t osens_mote_sm_func_proc_brd_id_ans(osens_mote_sm_state_t *st)
{
	uint8_t size;
    uint8_t ans_size = 28;

    st->point_index = 0;

    size = osens_unpack_cmd_res(&ans, frame, ans_size);

    if (size != ans_size)
    	return OSENS_STATE_EXEC_ERROR;

   // memcpy(&board_info, &ans.payload.brd_id_cmd, sizeof(osens_brd_id_t));

    if ((board_info.num_of_points == 0) || (board_info.num_of_points > OSENS_MAX_POINTS))
    	return OSENS_STATE_EXEC_ERROR;

    sensor_points.num_of_points = 0;
    st->retries = 0;

    return OSENS_STATE_EXEC_OK;
}

static uint8_t osens_mote_sm_func_req_brd_id(osens_mote_sm_state_t *st)
{
	cmd.hdr.size = 4;
	cmd.hdr.addr = OSENS_REGMAP_BRD_ID;
    st->trmout_counter = 0;
    st->trmout = MS2TICK(5000);
    return osens_mote_pack_send_frame(&cmd, 4);
}


static uint8_t osens_mote_sm_func_proc_itf_ver_ans(osens_mote_sm_state_t *st)
{
    uint8_t size;
    uint8_t ans_size = 6;

    size = osens_unpack_cmd_res(&ans, frame, ans_size);

	if (size != ans_size)
    	return OSENS_STATE_EXEC_ERROR;

    if ((OSENS_ANS_OK != ans.hdr.status) || (OSENS_LATEST_VERSION != ans.payload.itf_version_cmd.version))
    	return OSENS_STATE_EXEC_ERROR;


    return OSENS_STATE_EXEC_OK;
}


static uint8_t osens_mote_sm_func_wait_ans(osens_mote_sm_state_t *st)
{

	/* TODO!!!! MELHORAR O Tratamento do final de frame
	 * Devido ao problema da UART nao estar reconhecendo o final do frame
	 * a rotina esperar um ciclo de scan (250ms) entao se recebeu algum dado ele diz
	 * que recebeu o frame. Posteriormente sera tratado o frame e verificado se recebeu tudo
	 */
	if ((st->trmout_counter > 0) && (num_rx_bytes > 3))
	{
        st->frame_arrived = 1;
		num_rx_bytes = 0;
	}

	if(st->frame_arrived)
	{
        st->frame_arrived = 0;
		return OSENS_STATE_EXEC_WAIT_STOP;
	}

	st->trmout_counter++;

	if(st->trmout_counter > st->trmout)
	{
		//se ocorreu timeout e existe bytes no buffer rx considero que recebeu msg
		if (num_rx_bytes > 0)
		{
			return OSENS_STATE_EXEC_WAIT_STOP;
		}
		else
		{
			return OSENS_STATE_EXEC_WAIT_ABORT;
		}
	}

	return OSENS_STATE_EXEC_WAIT_OK;
}


static uint8_t osens_mote_sm_func_req_ver(osens_mote_sm_state_t *st)
{
    cmd.hdr.addr = OSENS_REGMAP_ITF_VERSION;
    st->trmout_counter = 0;
    st->trmout = MS2TICK(5000);
    return osens_mote_pack_send_frame(&cmd, 4);
}



uint8_t osens_mote_sm_func_init(void)
{
	uint8_t pointpos=0;

	memset(&cmd, 0, sizeof(cmd));
	memset(&ans, 0, sizeof(ans));
    memset(&sensor_points, 0, sizeof(sensor_points));
	memset(&board_info, 0, sizeof(board_info));
	memset(&schedule, 0, sizeof(schedule));

#if 1 //(SONOMA14 == 1)
    strcpy((void *)board_info.model, (void *) "sonoma ");
    board_info.model[OSENS_MODEL_NAME_SIZE-1] = 0;
    strcpy((void *)board_info.manufactor,(void *) "Maxim  ");
    board_info.model[OSENS_MANUF_NAME_SIZE-1] = 0;
    board_info.sensor_id = 0x11223344;
    board_info.hardware_revision = 0x01;
    board_info.num_of_points = 3;
    board_info.capabilities = 0;

    sensor_points.num_of_points = 3;
    pointpos=0;
    strcpy((char *)sensor_points.points[pointpos].desc.name, "VA_RMS");
    sensor_points.points[pointpos].desc.type = SENS_ITF_DT_U16;
    sensor_points.points[pointpos].desc.unit = 0; // TDB
    sensor_points.points[pointpos].desc.access_rights = SENS_ITF_ACCESS_READ_ONLY;
    sensor_points.points[pointpos].desc.sampling_time_x250ms = 1;
    sensor_points.points[pointpos].value.type = SENS_ITF_DT_FLOAT;
    sensor_points.points[pointpos].value.value.fp32 = 11;


    pointpos++;
    strcpy((char *)sensor_points.points[pointpos].desc.name, "IA_RMS");
    sensor_points.points[pointpos].desc.type = SENS_ITF_DT_U16;
    sensor_points.points[pointpos].desc.unit = 0; // TDB
    sensor_points.points[pointpos].desc.access_rights = SENS_ITF_ACCESS_READ_ONLY;
    sensor_points.points[pointpos].desc.sampling_time_x250ms = 1;
    sensor_points.points[pointpos].value.type = SENS_ITF_DT_FLOAT;
    sensor_points.points[pointpos].value.value.fp32 = 1.5;


    pointpos++;
    strcpy((char *)sensor_points.points[pointpos].desc.name, "WATT_A");
    sensor_points.points[pointpos].desc.type = SENS_ITF_DT_U16;
    sensor_points.points[pointpos].desc.unit = 0; // TDB
    sensor_points.points[pointpos].desc.access_rights = SENS_ITF_ACCESS_READ_ONLY;
    sensor_points.points[pointpos].desc.sampling_time_x250ms = 1;
    sensor_points.points[pointpos].value.type = SENS_ITF_DT_FLOAT;
    sensor_points.points[pointpos].value.value.fp32 = 0;

#if ENABLE_DEBUG_RFF
	 uint8_t pos=0;
	 uint8_t *paux;

	 rffbuf[pos++]= 0x88;
	 rffbuf[pos++]= board_info.hardware_revision;
	 rffbuf[pos++]= board_info.num_of_points;

	 openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
#endif

#else

 // #if MYLINKXS_LIGHT_CONTROL
 //configura os ios do sensor
	//light_init();

	strcpy((void *)board_info.model, (void *) "CC2538EM_");
    strcpy((void *)board_info.manufactor,(void *) "TexasInstr");
    board_info.sensor_id = 0x1;
    board_info.hardware_revision = 0x01;
    board_info.num_of_points = 1;
    board_info.capabilities = 0;

    sensor_points.points[0].desc.type = 1;
    sensor_points.points[0].desc.type = 1;

    strcpy((char *)sensor_points.points[0].desc.name, "LIGHT");
    sensor_points.points[0].desc.type = SENS_ITF_DT_U8;
    sensor_points.points[0].desc.unit = 0; // TDB
    sensor_points.points[0].desc.access_rights = SENS_ITF_ACCESS_READ_WRITE;
    sensor_points.points[0].desc.sampling_time_x250ms = 1;
    sensor_points.points[0].value.type = SENS_ITF_DT_U8;
    sensor_points.points[0].value.value.u8 = 10;
    sensor_points.num_of_points = 1;
#endif

    return 0;
}


void osens_mote_sm(void)
{
	uint8_t ret;

#if TRACE_ON == 1
	uint8_t ls = sm_state.state;
#endif

	ret = osens_mote_sm_table[sm_state.state].func(&sm_state);

	if (flagErrorOccurred)
	{
      //reset uart
	  uart1_clearTxInterrupts();
	  uart1_clearRxInterrupts();      // clear possible pending interrupts
	  uart1_enableInterrupts();       // Enable USCI_A1 TX & RX interrupt

	  flagErrorOccurred = 0;
	}

	switch(ret)
	{
		case OSENS_STATE_EXEC_OK:
		case OSENS_STATE_EXEC_WAIT_STOP:
			sm_state.state = osens_mote_sm_table[sm_state.state].next_state;
			break;
		case OSENS_STATE_EXEC_WAIT_OK:
			// still waiting
			break;
		case OSENS_STATE_EXEC_WAIT_ABORT:
			// wait timeout
			sm_state.state = osens_mote_sm_table[sm_state.state].abort_state;
			break;
		case OSENS_STATE_EXEC_ERROR:
		default:
			sm_state.state = osens_mote_sm_table[sm_state.state].error_state;
			break;
	}

#if TRACE_ON == 1
    printf("[SM]  %llu    (%02d) %-16s -> (%02d) %-16s\n", tick_counter, ls, sm_states_str[ls], sm_state.state,sm_states_str[sm_state.state]);

    {
        uint8_t index = 1;
        osens_point_t point;
        if (sensor_points.points[0].value.value.u8 >= 90 && sensor_points.points[1].value.value.u8 == 0)
        {
            point.value.u8 = 1;
            osens_set_pvalue(index, &point);
}

        if (sensor_points.points[0].value.value.u8 < 90 && sensor_points.points[1].value.value.u8 == 1)
{
            point.value.u8 = 0;
            osens_set_pvalue(index, &point);
        }
}
#endif

}

#if (MYLINKXS_LIGHT_CONTROL == 0)

static void osens_mote_tick(void)
{
    scheduler_push_task((task_cbt) osens_mote_sm, TASKPRIO_OSENS_MAIN);
}
#endif

#if ((SENSOR_ACCEL == 1) || (SONOMA14 == 1))
const osens_mote_sm_table_t osens_mote_sm_table[] =
{     //{ func,                                next_state,                   abort_state,              error_state       }
		{ osens_mote_sm_func_init,             OSENS_STATE_BUILD_SCH,        OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_INIT
		{ osens_mote_sm_func_req_ver,          OSENS_STATE_WAIT_ITF_VER_ANS, OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_SEND_ITF_VER
		{ osens_mote_sm_func_wait_ans,         OSENS_STATE_PROC_ITF_VER,     OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_WAIT_ITF_VER_ANS
		{ osens_mote_sm_func_proc_itf_ver_ans, OSENS_STATE_SEND_BRD_ID,      OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_PROC_ITF_VER
		{ osens_mote_sm_func_req_brd_id,       OSENS_STATE_WAIT_BRD_ID_ANS,  OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_SEND_BRD_ID
		{ osens_mote_sm_func_wait_ans,         OSENS_STATE_PROC_BRD_ID,      OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_WAIT_BRD_ID_ANS
		{ osens_mote_sm_func_proc_brd_id_ans,  OSENS_STATE_SEND_PT_DESC,     OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_PROC_BRD_ID
		{ osens_mote_sm_func_req_pt_desc,      OSENS_STATE_WAIT_PT_DESC_ANS, OSENS_STATE_BUILD_SCH,    OSENS_STATE_INIT  }, // OSENS_STATE_SEND_PT_DESC
        { osens_mote_sm_func_wait_ans,         OSENS_STATE_PROC_PT_DESC,     OSENS_STATE_SEND_PT_DESC, OSENS_STATE_INIT  }, // OSENS_STATE_WAIT_PT_DESC_ANS
		{ osens_mote_sm_func_pt_desc_ans,      OSENS_STATE_SEND_PT_DESC,     OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_PROC_PT_DESC
		{ osens_mote_sm_func_build_sch,        OSENS_STATE_RUN_SCH,          OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_BUILD_SCH
		{ osens_mote_sm_func_run_sch,          OSENS_STATE_RUN_SCH,          OSENS_STATE_SEND_PT_VAL,  OSENS_STATE_WR_PT }, // OSENS_STATE_RUN_SCH
		{ osens_mote_sm_func_req_pt_val,       OSENS_STATE_WAIT_PT_VAL_ANS,  OSENS_STATE_RUN_SCH,      OSENS_STATE_INIT  }, // OSENS_STATE_SEND_PT_VAL
		{ osens_mote_sm_func_wait_ans,         OSENS_STATE_PROC_PT_VAL,      OSENS_STATE_SEND_PT_VAL,  OSENS_STATE_INIT  }, // OSENS_STATE_WAIT_PT_VAL_ANS
		{ osens_mote_sm_func_pt_val_ans,       OSENS_STATE_SEND_PT_VAL,      OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_PROC_PT_VAL
		{ osens_mote_sm_func_wr_pt,            OSENS_STATE_WAIT_WR_PT_ANS,   OSENS_STATE_RUN_SCH,      OSENS_STATE_INIT  }, // OSENS_STATE_WR_PT
		{ osens_mote_sm_func_wait_ans,         OSENS_STATE_PROC_WR_PT_ANS,   OSENS_STATE_WR_PT,        OSENS_STATE_INIT  }, // OSENS_STATE_WAIT_WR_PT_ANS
		{ osens_mote_sm_func_proc_wr_pt,       OSENS_STATE_WR_PT,            OSENS_STATE_INIT,         OSENS_STATE_INIT } // OSENS_STATE_PROC_WR_PT_ANS
};
#elif (MYLINKXS_REMOTE_CONTROL == 1)
const osens_mote_sm_table_t osens_mote_sm_table[] =
{     //{ func,                                next_state,                   abort_state,              error_state       }
		{ osens_mote_sm_func_init,             OSENS_STATE_BUILD_SCH,        OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_INIT
		{ osens_mote_sm_func_req_ver,          OSENS_STATE_WAIT_ITF_VER_ANS, OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_SEND_ITF_VER
		{ osens_mote_sm_func_wait_ans,         OSENS_STATE_PROC_ITF_VER,     OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_WAIT_ITF_VER_ANS
		{ osens_mote_sm_func_proc_itf_ver_ans, OSENS_STATE_SEND_BRD_ID,      OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_PROC_ITF_VER
		{ osens_mote_sm_func_req_brd_id,       OSENS_STATE_WAIT_BRD_ID_ANS,  OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_SEND_BRD_ID
		{ osens_mote_sm_func_wait_ans,         OSENS_STATE_PROC_BRD_ID,      OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_WAIT_BRD_ID_ANS
		{ osens_mote_sm_func_proc_brd_id_ans,  OSENS_STATE_SEND_PT_DESC,     OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_PROC_BRD_ID
		{ osens_mote_sm_func_req_pt_desc,      OSENS_STATE_WAIT_PT_DESC_ANS, OSENS_STATE_BUILD_SCH,    OSENS_STATE_INIT  }, // OSENS_STATE_SEND_PT_DESC
        { osens_mote_sm_func_wait_ans,         OSENS_STATE_PROC_PT_DESC,     OSENS_STATE_SEND_PT_DESC, OSENS_STATE_INIT  }, // OSENS_STATE_WAIT_PT_DESC_ANS
		{ osens_mote_sm_func_pt_desc_ans,      OSENS_STATE_SEND_PT_DESC,     OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_PROC_PT_DESC
		{ osens_mote_sm_func_build_sch,        OSENS_STATE_RUN_SCH,          OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_BUILD_SCH
		{ osens_mote_sm_func_run_sch,          OSENS_STATE_RUN_SCH,          OSENS_STATE_SEND_PT_VAL,  OSENS_STATE_WR_PT }, // OSENS_STATE_RUN_SCH
		{ osens_mote_sm_func_req_pt_val,       OSENS_STATE_WAIT_PT_VAL_ANS,  OSENS_STATE_RUN_SCH,      OSENS_STATE_INIT  }, // OSENS_STATE_SEND_PT_VAL
		{ osens_mote_sm_func_wait_ans,         OSENS_STATE_PROC_PT_VAL,      OSENS_STATE_SEND_PT_VAL,  OSENS_STATE_INIT  }, // OSENS_STATE_WAIT_PT_VAL_ANS
		{ osens_mote_sm_func_pt_val_ans,       OSENS_STATE_SEND_PT_VAL,      OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_PROC_PT_VAL
		{ osens_mote_sm_func_wr_pt,            OSENS_STATE_WAIT_WR_PT_ANS,   OSENS_STATE_RUN_SCH,      OSENS_STATE_INIT  }, // OSENS_STATE_WR_PT
		{ osens_mote_sm_func_wait_ans,         OSENS_STATE_PROC_WR_PT_ANS,   OSENS_STATE_WR_PT,        OSENS_STATE_INIT  }, // OSENS_STATE_WAIT_WR_PT_ANS
		{ osens_mote_sm_func_proc_wr_pt,       OSENS_STATE_WR_PT,            OSENS_STATE_INIT,         OSENS_STATE_INIT } // OSENS_STATE_PROC_WR_PT_ANS
};
#else
/*
const osens_mote_sm_table_t osens_mote_sm_table[] =
{     //{ func,                                next_state,                   abort_state,              error_state       }
		{ osens_mote_sm_func_init,             OSENS_STATE_SEND_ITF_VER,     OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_INIT
		{ osens_mote_sm_func_req_ver,          OSENS_STATE_WAIT_ITF_VER_ANS, OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_SEND_ITF_VER
		{ osens_mote_sm_func_wait_ans,         OSENS_STATE_PROC_ITF_VER,     OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_WAIT_ITF_VER_ANS
		{ osens_mote_sm_func_proc_itf_ver_ans, OSENS_STATE_SEND_BRD_ID,      OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_PROC_ITF_VER
		{ osens_mote_sm_func_req_brd_id,       OSENS_STATE_WAIT_BRD_ID_ANS,  OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_SEND_BRD_ID
		{ osens_mote_sm_func_wait_ans,         OSENS_STATE_PROC_BRD_ID,      OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_WAIT_BRD_ID_ANS
		{ osens_mote_sm_func_proc_brd_id_ans,  OSENS_STATE_SEND_PT_DESC,     OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_PROC_BRD_ID
		{ osens_mote_sm_func_req_pt_desc,      OSENS_STATE_WAIT_PT_DESC_ANS, OSENS_STATE_BUILD_SCH,    OSENS_STATE_INIT  }, // OSENS_STATE_SEND_PT_DESC
        { osens_mote_sm_func_wait_ans,         OSENS_STATE_PROC_PT_DESC,     OSENS_STATE_SEND_PT_DESC, OSENS_STATE_INIT  }, // OSENS_STATE_WAIT_PT_DESC_ANS
		{ osens_mote_sm_func_pt_desc_ans,      OSENS_STATE_SEND_PT_DESC,     OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_PROC_PT_DESC
		{ osens_mote_sm_func_build_sch,        OSENS_STATE_RUN_SCH,          OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_BUILD_SCH
		{ osens_mote_sm_func_run_sch,          OSENS_STATE_RUN_SCH,          OSENS_STATE_SEND_PT_VAL,  OSENS_STATE_WR_PT }, // OSENS_STATE_RUN_SCH
		{ osens_mote_sm_func_req_pt_val,       OSENS_STATE_WAIT_PT_VAL_ANS,  OSENS_STATE_RUN_SCH,      OSENS_STATE_INIT  }, // OSENS_STATE_SEND_PT_VAL
		{ osens_mote_sm_func_wait_ans,         OSENS_STATE_PROC_PT_VAL,      OSENS_STATE_SEND_PT_VAL,  OSENS_STATE_INIT  }, // OSENS_STATE_WAIT_PT_VAL_ANS
		{ osens_mote_sm_func_pt_val_ans,       OSENS_STATE_SEND_PT_VAL,      OSENS_STATE_INIT,         OSENS_STATE_INIT  }, // OSENS_STATE_PROC_PT_VAL
		{ osens_mote_sm_func_wr_pt,            OSENS_STATE_WAIT_WR_PT_ANS,   OSENS_STATE_RUN_SCH,      OSENS_STATE_INIT  }, // OSENS_STATE_WR_PT
		{ osens_mote_sm_func_wait_ans,         OSENS_STATE_PROC_WR_PT_ANS,   OSENS_STATE_WR_PT,        OSENS_STATE_INIT  }, // OSENS_STATE_WAIT_WR_PT_ANS
		{ osens_mote_sm_func_proc_wr_pt,       OSENS_STATE_WR_PT,            OSENS_STATE_INIT,         OSENS_STATE_INIT } // OSENS_STATE_PROC_WR_PT_ANS
};
*/
#endif




uint8_t osens_set_pvalue(uint8_t index, osens_point_t *point)
{

    if (point->value.u8 > 0){
    	leds_radio_on();
    }
    else{
    	leds_radio_off();
    }

	return 1;
}

// Provisorio liga lampada
uint8_t osens_liga_lampada_local(void)
{
	osens_point_t point;

	point.type=0;
    point.value.u8=1;

	osens_set_pvalue(0, &point);

	return 0;
}


uint8_t osens_get_num_points(void)
{
	if(sm_state.state >= OSENS_STATE_SEND_PT_DESC)
	{
		return board_info.num_of_points;
	}
	else
		return 0;
}

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

uint8_t osens_get_pdesc(uint8_t index, osens_point_desc_t *desc)
{

#if (SONOMA14 == 1)
    if (index < 3){
    	memcpy(desc,&sensor_points.points[index].desc,sizeof(osens_point_desc_t));
    	return 1;
    }
    else
    	return 0;

#else
	if((sm_state.state >= OSENS_STATE_RUN_SCH) && (index <= sensor_points.num_of_points))
	{
		memcpy(desc,&sensor_points.points[index].desc,sizeof(osens_point_desc_t));
		return 1;
	}
	else
		return 0;
#endif

#if (MYLINKXS_LIGHT_CONTROL == 1)
	memcpy(desc,&sensor_points.points[0].desc,sizeof(osens_point_desc_t));
	return 1;
#endif
}

int8_t osens_get_ptype(uint8_t index)
{
#if 0
	if((sm_state.state >= OSENS_STATE_RUN_SCH) && (index <= sensor_points.num_of_points))
	{
		return sensor_points.points[index].value.type;
	}
	else
		return -1;
#else
	return sensor_points.points[0].value.type;
#endif

}

uint8_t osens_get_point(uint8_t index, osens_point_t *point)
{
#if 0
	if((sm_state.state >= OSENS_STATE_RUN_SCH) && (index <= sensor_points.num_of_points))
	{
		memcpy(point,&sensor_points.points[0].value,sizeof(osens_point_t));
		return 1;
	}
	else
		return 0;
#else
	memcpy(point,&sensor_points.points[0].value,sizeof(osens_point_t));
	return 1;

#endif

}

/**************************************************************************//**
* @brief    This function puts up to \e size bytes into the BSP UART TX
*           buffer and starts to transfer data over UART.
*
*           If \b BSP_UART_ALL_OR_NOTHING is defined, data is put into the
*           TX buffer only if there is room for all \e size bytes.
*
* @param    frame            is a pointer to the source buffer.
* @param    size            is the number of bytes to transfer.
*
* @return   Returns the number of bytes actually copied to the TX buffer.
******************************************************************************/
#if (USE_SPI_INTERFACE == 0)
uint8_t osens_mote_send_frame(uint8_t *frame, uint8_t size)
{
 	uint16_t ui16Length=size;
    register uint16_t ui16Idx = 0;
#if 0
    DISABLE_INTERRUPTS();
    while((ui16Idx < ui16Length))
    {
        UARTCharPut(BSP_UART_BASE, frame[ui16Idx++]);
    }
    ENABLE_INTERRUPTS();
#endif
    return ((uint8_t) ui16Length);
}
#endif
/**************************************************************************//**
* @brief    This function pushes a byte from the UART RX FIFO to the BSP UART
*           RX buffer. The function handles RX buffer wrap-around, but does not
*           handle RX buffer overflow. The function should only be called if
*           there is data in the UART RX FIFO. It modifies volatile variables
*           and should only be called when interrupts are disabled.
*
* @brief    None
******************************************************************************/
static void buBufPushByte(void)
{
    uint32_t value;

    // Push byte from RX FIFO to buffer
    value = UARTCharGetNonBlocking(OSENS_UART_BASE);

    if (num_rx_bytes < OSENS_MAX_FRAME_SIZE)
    	frame[num_rx_bytes] = (uint8_t) value;

    num_rx_bytes++;
    if (num_rx_bytes >= OSENS_MAX_FRAME_SIZE)
        num_rx_bytes = 0;
}


/**************************************************************************//**
* @brief    This function flushes the ringbuffer control structure specified
*           by \e psBuf.
*
* @param    psBuf       is a pointer to a \e tBuBuf ringbuffer structure.
*
* @return   None
******************************************************************************/
static void buBufFlush(void)
{
    // Start of critical section
    bool bIntDisabled = IntMasterDisable();

    num_rx_bytes = 0;

    if(!bIntDisabled)
    {
        IntMasterEnable();
    }
}


void uart1_isr_private(void)
{
    uint32_t ui32IntBm = UARTIntStatus(OSENS_UART_BASE, 1);
    //uint8_t dummy=0;

    UARTIntClear(OSENS_UART_BASE, (ui32IntBm & 0xF0));

    //ERROR LINE
    if (ui32IntBm & (UART_INT_BE | UART_INT_PE | UART_INT_FE))
	{
    	UARTRxErrorClear(OSENS_UART_BASE);
    	buBufFlush();
    	flagErrorOccurred = 1;
        //while(UARTCharsAvail(OSENS_UART_BASE))
        //{
        //	dummy = UARTCharGetNonBlocking(OSENS_UART_BASE);
        //}
	}
    else if(ui32IntBm & (UART_INT_OE))
	{    //OVERRUN
    	UARTRxErrorClear(OSENS_UART_BASE);
        buBufFlush();
    	flagErrorOccurred = 1;
    	//dummy = UARTCharGetNonBlocking(OSENS_UART_BASE);
        //while(UARTCharsAvail(OSENS_UART_BASE))
        //{
        //	dummy = UARTCharGetNonBlocking(OSENS_UART_BASE);
        //}
	}
    else if(ui32IntBm & (UART_INT_RX | UART_INT_RT))
    {
        //
        // Put received bytes into buffer
        //
        //while(UARTCharsAvail(BSP_UART_BASE) && !BU_IS_BUF_FULL(&sBuBufRx))
        while(UARTCharsAvail(OSENS_UART_BASE))
        {
            buBufPushByte();
        }


        //if (ui32IntBm & UART_INT_RT)
        //{
        //	sm_state.frame_arrived = true;
        //}

    }
    else if(ui32IntBm & UART_INT_TX)
    {
        //while(UARTSpaceAvail(BSP_UART_BASE) && (!BU_IS_BUF_EMPTY(&sBuBufTx)))
        //{
        //    buBufPopByte();
        //}
    }
}

//Este eh o que esta rodando agora 241119
void osens_sm_func_run_sch(void){

    uint32_t param[3];
    float fval[3];
#if (SENSOR_ACCEL == 1)
    int8_t i8Data[6] = {0};
    uint16_t u16X, u16Y, u16Z;
#endif

    if (sm_state.state != OSENS_STATE_SEND_PT_DESC) {
  	    sm_state.state = OSENS_STATE_SEND_PT_DESC;
		osens_mote_sm_func_init();
    }

#if 0 //SONOMA14
    //param[0] = read_reg(SONOMA_ADDR_VA_RMS);
    //param[1] = read_reg(SONOMA_ADDR_IA_RMS);
    //param[2] = read_reg(SONOMA_ADDR_WATT_A);
    param[0] = 0x00000A0D;
    param[1] = 0x0018A6AF;
    param[2] = 0x00700000;

    fval[0] = CalculaS23((uint8_t *) &param[0]) / SONOMA_TRIMER_VA_RMS;
    fval[1] = CalculaS23((uint8_t *) &param[1]) / SONOMA_TRIMER_VA_RMS;
    fval[2] = CalculaS23((uint8_t *) &param[2]) / SONOMA_TRIMER_VA_RMS;
#endif
    //sensor_points.points[0].value.value.u16 = 11;
    sensor_points.points[0].value.value.fp32 += count;
    sensor_points.points[1].value.value.fp32 = 22;
    sensor_points.points[2].value.value.fp32 = 33;
    count =1;

#if (SENSOR_ACCEL == 1)
    accReadReg2(ACC_X_LSB, (uint8_t *)i8Data, 6);

    u16X = (((uint16_t)(i8Data[1]) << 2) | ((i8Data[0] >> 6) & 0x03));
    u16Y = (((uint16_t)(i8Data[3]) << 2) | ((i8Data[2] >> 6) & 0x03));
    u16Z = (((uint16_t)(i8Data[5]) << 2) | ((i8Data[4] >> 6) & 0x03));

    sensor_points.points[0].value.value.u16 = u16X;
    sensor_points.points[1].value.value.u16 = u16Y;
    sensor_points.points[2].value.value.u16 = u16Z;
#endif

#if 0 //ENABLE_DEBUG_RFF
	 uint8_t pos=0;
	 uint8_t *paux;

	 rffbuf[pos++]= 0x80;
	 rffbuf[pos++]= board_info.hardware_revision;
	 rffbuf[pos++]= board_info.num_of_points;
	 pos = printvar((uint8_t *)&sensor_points.points[0].value.value.u16,sizeof(uint16_t),rffbuf,pos);
	 pos = printvar((uint8_t *)&sensor_points.points[1].value.value.u16,sizeof(uint16_t),rffbuf,pos);
	 pos = printvar((uint8_t *)&sensor_points.points[2].value.value.u16,sizeof(uint16_t),rffbuf,pos);

	 openserial_printStatus(STATUS_RFF,(uint8_t*)&rffbuf,pos);
#endif

}

#if SONOMA14
float CalculaS23(uint8_t *paux)
{
	float retorno = 0;
	uint8_t auxbuf[3];
    uint8_t *rx= &auxbuf[0];

	auxbuf[2]= *paux++;
	auxbuf[1]= *paux++;
	auxbuf[0]= *paux++;

	retorno += rx[0] & 64 ? 0.5 : 0; // 1/2
	retorno += rx[0] & 32 ? 0.25 : 0; // 1/4
	retorno += rx[0] & 16 ? 0.125 : 0; // 1/8
	retorno += rx[0] & 8  ? 0.0625 : 0; // 1/16
	retorno += rx[0] & 4  ? 0.03125 : 0; // 1/32
	retorno += rx[0] & 2  ? 0.015625 : 0; // 1/64
	retorno += rx[0] & 1  ? 0.0078125 : 0; // 1/128

	retorno += rx[1] & 128 ? 0.00390625 : 0; // 1/256
	retorno += rx[1] & 64  ? 0.001953125 : 0; // 1/512
	retorno += rx[1] & 32  ? 0.0009765625 : 0; // 1/1024
	retorno += rx[1] & 16  ? 0.00048828125 : 0; // 1/2048
	retorno += rx[1] & 8   ? 0.000244140625 : 0; // 1/4096
	retorno += rx[1] & 4   ? 0.0001220703125 : 0; // 1/8192
	retorno += rx[1] & 2   ? 0.00006103515625 : 0; // 1/16384
	retorno += rx[1] & 1   ? 0.00003051757813 : 0; // 1/32768

	retorno += rx[2] & 128 ? 0.00001525878906 : 0; // 1/65536
	retorno += rx[2] & 64  ? 0.000007629394531 : 0; // 1/131072
	retorno += rx[2] & 32  ? 0.000003814697266 : 0; // 1/262144
	retorno += rx[2] & 16  ? 0.000001907348633 : 0; // 1/524288
	retorno += rx[2] & 8   ? 0.0000009536743164 : 0; // 1/1048576
	retorno += rx[2] & 4   ? 0.0000004768371582 : 0; // 1/2097152
	retorno += rx[2] & 2   ? 0.0000002384185791 : 0; // 1/4194304
	retorno += rx[2] & 1   ? 0.0000001192092896 : 0; // 1/8388608

	retorno = rx[0] & 128 ? (-1) * retorno : retorno; // Verifica o sinal

	return retorno;
}
#endif
