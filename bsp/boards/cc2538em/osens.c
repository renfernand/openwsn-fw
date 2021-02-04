#include <stdint.h>
#include <stdio.h>
#include "opendefs.h"
#include "osens.h"
#include "osens_itf.h"
#include "opentimers.h"
#include "scheduler.h"
#include "debug.h"
#include "udma.h"
#include "hw_udma.h"
#include "hw_flash_ctrl.h"
#include "openserial.h"
#include "leds.h"
#if SENSOR_ACCEL
#include "acc_bma250.h"
#endif

osens_brd_id_t board_info;

//=========================== prototypes ======================================
void osens_mote_sm_func_run_sch_cb(opentimers_id_t id);
void osens_sm_func_run_sch (void);
float CalculaS23(uint8_t *paux);

uint8_t reset_timerId;

uint8_t osens_init(void)
{

#if	 (MYLINKXS_SENSORS == 1)
	memset(&board_info, 0, sizeof(board_info));
	
	
    //
    // Fill Source buffer (to be copied to flash) with some data
    //
    reset_timerId  = opentimers_create(TIMER_GENERAL_PURPOSE, TASKPRIO_OPENSERIAL);

    osens_mote_sm_func_init();

    opentimers_scheduleIn(
    	reset_timerId,
		100,
        TIME_MS,
        TIMER_PERIODIC,
		osens_mote_sm_func_run_sch_cb
    );

#endif

    return 1;
}


void osens_mote_sm_func_run_sch_cb (opentimers_id_t id){

	scheduler_push_task((task_cbt) osens_sm_func_run_sch, TASKPRIO_OSENS_MAIN);
}
