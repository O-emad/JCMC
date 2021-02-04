
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "systick.h"
#include "controller.h"
#include "system.h"
#include "canonical.h"
#include "gcode_parser.h"
#include "planner.h"
#include "switch.h"
#include "debugging.h"
#include "HAL.h"



static cont_t cs;

char string1[][100]={
	"n0000 g17 g21 g54 g60 g80 g90 g94",
	"n0010 g10 p1 x0 y0 z0",
	"n0015 g10 p2 x0 y-20 z0",
	"n0020 g10 p3 x0 y-40 z0",
	"n0025 g10 p4 x0 y-60 z0",
	"n0005 g54 g01 x0 y0 z0 f1500",
	"n0009 g03 x0 y0 z-30 p5 i-70 f1500",
	"n0030 g54 g01 x0 y0 z0 f1500",
	"n0035 g01 y-20",
	"n0040 x-20",
	"n0045 y0",
	"n0050 x0",
	"n0055 g55 z40",
	"n0060 g01 x0 y0 f1500",
	"n0061 z0",
	"n0065 y-20",
	"n0070 x-20",
	"n0075 y0",
	"n0080 x0",
	"n0085 g56 z40",
	"n0090 g01 x0 y0 f1500",
	"n0091 z0",
	"n0095 y-20",
	"n0100 x-20",
	"n0105 y0",
	"n0110 x0",
	"n0115 g57 z40",
	"n0120 g01 x0 y0 f1500",
	"n0121 z0",
	"n0125 y-20",
	"n0130 x-20",
	"n0135 y0",
	"n0140 x0",
	"n0145 m5 z40",
	"n0150 m30",
	0
};

char string2[][100]={
	"g01 x-100   f4000",
	"g01 x-50 ",
	"g01 x-25 ",
	"g01 x-12.5  ",
	"g01 x-6.25  ",
	"g01 x-3.125  ",
	"g01 x-1.675  ",
	"g01 x-0.8375 ",
	"M30",
	0
};

char string[][100]={
	"g01 x-100 y-100  f3000",
	"g01 x-200 y-200  f1800",
	"g01 x-300 y-300  f5000",
	"g01 x0 y0 ",
	"g01 x100 y100 ",
	"g01 x200 y200 ",
	"g01 x300 y300 ",
	"g01 x0 y0 ",
	"M30",
	0
};

#define _CODE_SIZE 34

static void _controller_HSM(void);
static stat_t _sync_to_planner(void);
static stat_t _command_dispatch(void);
static stat_t _normal_idler(void);
static stat_t _limit_switch_handler(void);

void controller_run(void){
	while(1){
		_controller_HSM();
	}
}


#define DISPATCH(func) if(func == STAT_RC) return;

static void _controller_HSM(void){
	
	//----- kernel level ISR handlers ----(flags are set in ISRs)------------------------//
												// Order is important:
	//DISPATCH(hw_hard_reset_handler());			// 1. handle hard reset requests
	//DISPATCH(hw_bootloader_handler());			// 2. handle requests to enter bootloader
	//DISPATCH(_shutdown_idler());				// 3. idle in shutdown state
  //DISPATCH( poll_switches());					// 4. run a switch polling cycle
	DISPATCH(_limit_switch_handler());			// 5. limit switch has been thrown

	//DISPATCH(cm_feedhold_sequencing_callback());// 6a. feedhold state machine runner
	//DISPATCH(mp_plan_hold_callback());			// 6b. plan a feedhold from line runtime
	//DISPATCH(_system_assertions());				// 7. system integrity assertions

//----- planner hierarchy for gcode and cycles ---------------------------------------//

	//DISPATCH(st_motor_power_callback());		// stepper motor power sequencing
	//DISPATCH(switch_debounce_callback());		// debounce switches
	//DISPATCH(sr_status_report_callback());		// conditionally send status report
	//DISPATCH(qr_queue_report_callback());		// conditionally send queue report
	//DISPATCH(rx_report_callback());             // conditionally send rx report
	db_start_session(ARC_CALLBACK);
	DISPATCH(cm_arc_callback());				// arc generation runs behind lines
	db_end_session(ARC_CALLBACK);
	DISPATCH(cm_homing_callback());				// G28.2 continuation
	//DISPATCH(cm_jogging_callback());			// jog function
	//DISPATCH(cm_probe_callback());				// G38.2 continuation
	//DISPATCH(cm_deferred_write_callback());		// persist G10 changes when not in machining cycle

//----- command readers and parsers --------------------------------------------------//

	DISPATCH(_sync_to_planner());				// ensure there is at least one free buffer in planning queue
	DISPATCH(_command_dispatch());				// read and execute next command
	DISPATCH(_normal_idler());					// blink LEDs slowly to show everything is OK
}



static stat_t _sync_to_planner(void){
	if(mp_get_available_buffers() < PLANNER_BUFFER_LIMIT){
		return STAT_RC;
	}
	return STAT_OK;
}

static stat_t _command_dispatch(void){
	//if((GPIO_PORTF_DATA_R&0x00000010) == 0){
	if(*string2[cs.block]==0){
		return STAT_NOOP;
	}
		gc_gcode_parser(string2[cs.block]);
		cs.block++;
		//delay_m(500);
	//}
	return STAT_OK;
}

static stat_t _normal_idler(void){
	return STAT_OK;
}

static stat_t _limit_switch_handler(void)
{
	if (cm_get_machine_state() == MACHINE_ALARM) { return (STAT_NOOP);}

	if (get_limit_switchs_thrown() == 0){
		return (STAT_NOOP);
	}else{
		return(cm_hard_alarm(STAT_LIMIT_SWITCH_HIT));
	}
}




