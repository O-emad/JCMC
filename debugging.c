
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "debugging.h"

debug_t db;

void db_init(void){
	WTIMER0_CTL_R &=~ TIMER_CTL_TAEN;
	WTIMER0_CTL_R |= TIMER_CTL_TASTALL;
	WTIMER0_CFG_R = TIMER_CFG_16_BIT;
	WTIMER0_TAMR_R |= TIMER_TAMR_TACDIR|TIMER_TAMR_TAMR_PERIOD;
	WTIMER0_TAILR_R = 0xFFFFFFFF;
	WTIMER0_TAPR_R = 0;
	WTIMER0_TAV_R = 0;
	WTIMER0_CTL_R |= TIMER_CTL_TAEN;
	for(uint8_t i=0; i<EVENTS; ++i){
		db.event[i].event_min_time = 0xFFFFFFFF;
	}
}
