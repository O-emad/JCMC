
#include <stdint.h>
#include <stdbool.h>
#include "system.h"
#include "planner.h"
#include "loader.h"
#include "stepper.h"
#include "timers.h"


load_t ld;



void ld_init(void){

#if defined(__STEPPER)
	ld.buffer_state = PREP_BUFFER_OWNED_BY_EXEC;
	ld.move_type = MOVE_TYPE_NULL;
	ld.actuator_runtime_isbusy = st_runtime_isbusy;
	ld.get_target_units = st_inverse_kinematics;
	ld.prep_line = st_prep_line;
	ld.load_move = st_load_move;
#endif

}

void ld_request_exe(void){
	if(ld.buffer_state == PREP_BUFFER_OWNED_BY_EXEC){
		//execute_timer_enable();
		TIMER5_CTL_R |= TIMER_CTL_TAEN;
	}
}

void TIMER5A_Handler(void){			//LOWEST_PRIORITY interrupt
	execute_timer_acknowledge();
	if(ld.buffer_state == PREP_BUFFER_OWNED_BY_EXEC){
		if((mp_exec_move())!=STAT_NOOP){
			//you have something to execute
			ld.buffer_state = PREP_BUFFER_OWNED_BY_LOADER;
			ld_request_load();
		} 
	}
}


void ld_request_load(void){
	if(ld.actuator_runtime_isbusy()){
		return;
	}
	if(ld.buffer_state == PREP_BUFFER_OWNED_BY_LOADER){
		load_timer_enable();
	}
}

void TIMER5B_Handler(void){		//LOW_PRIORITY interrupt
	load_timer_acknowledge();
	ld.load_move();
}
