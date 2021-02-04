//debugging.h
//July 21, 2018
//Runs on tm4c123

/*
	this unit uses a wide timer ... WTIMER0A specifically but can be changed to collect events data
	the timer in 32bits timer with 16 bits prescaller.
	it's designed to be able to collect the time of specific events and how long it takes to be executed
	this method is non-intrusive takes around 4 cycles to start the cycle and 20 to finish it so that's a
	total of 300ns on 80Mhz clock
	the maximum event time in 53secs "53.68709 secs" .... that's more than enough for any embedded application
	it can collect up to "3518437.209 per event" but that requires double calculation and that will be intrusive.
*/


#ifndef DEBUG_H
#define DEBUG_H
#include "tm4c123gh6pm.h"
#ifndef INLINE
#define INLINE extern inline
#endif


#define maxl(a,b) a>b? a:b
#define minl(a,b) a<b? a:b

enum debuggingevents{
	BLOCK_PREPARE_TIME = 0,
	GCODE_PARSER_TIME,
	CANONICAL_TIME,
	PLAN_LINE_TIME,
	PLAN_MOTION_JERK,
	PLAN_JUNCTION_VELOCITY,
	PLAN_BLOCK_LIST_TIME,
	PLAN_MOTION_PLANNING,
	PLAN_GET_ASSYMETRIC_VELOCITY,
	STEPPER_PREP_LINE,
	STEPPER_LOAD_MOVE,
	DDA_OVERFLOW,
	DDA_MATCH,
	ARC_CANONICAL,
	ARC_COMPUTE,
	ARC_CALLBACK,
	LAST_DB_EVENT
};

#define EVENTS LAST_DB_EVENT

typedef struct debuggingevent{
	uint32_t timer_value;
	uint32_t event_time;
	uint32_t event_min_time;
	uint32_t event_max_time;
	uint32_t event_recalls;
}dbEvent_t;

typedef struct debug{
	dbEvent_t event[EVENTS];
}debug_t;

extern debug_t db;

void db_init(void);
INLINE void db_start_session(uint8_t event) __attribute__((always_inline));
INLINE void db_end_session(uint8_t event) __attribute__((always_inline));



inline void db_start_session(uint8_t event){
	db.event[event].timer_value = WTIMER0_TAV_R;
}

inline void db_end_session(uint8_t event){
	if(event > EVENTS)
		return;
	db.event[event].event_time = (WTIMER0_TAV_R-db.event[event].timer_value)&0xFFFFFFFF;
	db.event[event].event_max_time = maxl(db.event[event].event_time,db.event[event].event_max_time);
	db.event[event].event_min_time = minl(db.event[event].event_time,db.event[event].event_min_time);
	db.event[event].event_recalls++;
}


#endif

