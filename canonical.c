/*
 * canonical.c
 * This file is part of the X project
 *
 * Omar Emad El-Deen
 * Yossef Mohammed Hassanin
 * Mars, 2018
 */
/*
 * this code is a loose implementation of g-code interpreter based on Kramer, Proctor and Messina's
 * canonical machine as described in the NIST RS274/NGC v3
 * the canonical machine is a layer between gcode parser and motion controller
 * It keeps state and executes commands - passing the
 * stateless commands to the motion planning layer.
 */
/* --- System state contexts - Gcode models ---
 *
 *	Useful reference for doing C callbacks http://www.newty.de/fpt/fpt.html
 *
 *	There are 3 temporal contexts for system state:
 *	  - The gcode model in the canonical machine (the MODEL context, held in gm)
 *	  - The gcode model used by the planner (PLANNER context, held in bf's and mm)
 *	  - The gcode model used during motion for reporting (RUNTIME context, held in mr)
 *
 *	It's a bit more complicated than this. The 'gm' struct contains the core Gcode model
 *	context. This originates in the canonical machine and is copied to each planner buffer
 *	(bf buffer) during motion planning. Finally, the gm context is passed to the runtime
 *	(mr) for the RUNTIME context. So at last count the Gcode model exists in as many as
 *	30 copies in the system. (1+28+1)
 *
 *	Depending on the need, any one of these contexts may be called for reporting or by
 *	a function. Most typically, all new commends from the gcode parser work form the MODEL
 *	context, and status reports pull from the RUNTIME while in motion, and from MODEL when
 *	at rest. A convenience is provided in the ACTIVE_MODEL pointer to point to the right
 *	context.
 */
/* --- Synchronizing command execution ---
 *
 *	Some gcode commands only set the MODEL state for interpretation of the current Gcode
 *	block. For example, cm_set_feed_rate(). This sets the MODEL so the move time is
 *	properly calculated for the current (and subsequent) blocks, so it's effected
 *	immediately.
 *
 *	"Synchronous commands" are commands that affect the runtime need to be synchronized
 *	with movement. Examples include G4 dwells, program stops and ends, and most M commands.
 *	These are queued into the planner queue and execute from the queue. Synchronous commands
 *	work like this:
 *
 *	  - Call the cm_xxx_xxx() function which will do any input validation and return an
 *		error if it detects one.
 *
 *	  - The cm_ function calls mp_queue_command(). Arguments are a callback to the _exec_...()
 *		function, which is the runtime execution routine, and any arguments that are needed
 *		by the runtime. See typedef for *exec in planner.h for details
 *
 *	  - mp_queue_command() stores the callback and the args in a planner buffer.
 *
 *	  - When planner execution reaches the buffer it executes the callback w/ the args.
 *		Take careful note that the callback executes under an interrupt, so beware of
 *		variables that may need to be volatile.
 *
 *	Note:
 *	  - The synchronous command execution mechanism uses 2 vectors in the bf buffer to store
 *		and return values for the callback. It's obvious, but impractical to pass the entire
 *		bf buffer to the callback as some of these commands are actually executed locally
 *		and have no buffer.
 */
 
 
//******includes*******//
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "system.h"
#include "canonical.h"
#include "planner.h"
#include "stepper.h"
#include "debugging.h"
#include "util.h"
//////////////////////

//******globals*******//
cmSingleton_t cm;
/////////////////////

//cm_get_active_coord_offset//
//input : axis number
//output : coordinate offset of an axis
//fuction : 
//notes : 
//additions: 
//
float cm_get_active_coord_offset(uint8_t axis){
	if(cm.gm.absolute_override == true){return 0;}
	return (cm.origin_offset_enable ? cm.coord_offset[cm.gm.coordinate_system][axis] + cm.origin_offset[axis] :
																		cm.coord_offset[cm.gm.coordinate_system][axis]);
}

void cm_set_work_offsets(GState_t *gcode_state)
{
	for (uint8_t axis = X_AXIS; axis < AXES; ++axis) {
		gcode_state->work_offset[axis] = cm_get_active_coord_offset(axis);
	}
}
////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
void cm_set_model_target(float values[], float flags[]){
	for(uint8_t axis = X_AXIS; axis < AXES; ++axis){
		if(flags[axis] <= 0.0f){		//or axis state is disabled
			continue;
		}else{
			if(cm.gm.distance_mode == ABSOLUTE_MODE){
				cm.gm.target[axis] = cm_get_active_coord_offset(axis) + _TO_MILLI(values[axis]);
			}else{
				cm.gm.target[axis] += _TO_MILLI(values[axis]);
			}
		}
	}
}
////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
void canonical_init(void){
	memset(&cm.gm,0,sizeof(GState_t));
	memset(&cm.gn,0,sizeof(GIn_t));
	memset(&cm.gf,0,sizeof(GIn_t));
	
	ACTIVE_MODEL = MODEL;
	// set gcode defaults
	cm_select_unit_mode(GCODE_DEFAULT_UNITS);
	cm_set_coord_system(G54);
	cm_select_plane(GCODE_DEFAULT_PLANE);
	cm_select_path_control(GCODE_DEFAULT_PATH_CONTROL);
	cm_select_distance_mode(GCODE_DEFAULT_DISTANCE_MODE);
	cm_set_feed_rate_mode(UNITS_PER_MINUTE_MODE);// always the default
	// never start a machine in a motion mode
	cm.gm.motion_mode = MOTION_MODE_CANCEL_MOTION_MODE;
	cm.machine_state = MACHINE_READY;
	
}
////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
void cm_set_model_linenum(uint32_t linenum){
	cm.gm.linenum = linenum;
}
////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
uint8_t cm_get_motion_mode(GState_t *gcode_state)
{
	return gcode_state->motion_mode;
}

////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
stat_t cm_set_feed_rate_mode(uint8_t mode){
	cm.gm.feedrate_mode = mode;
	return STAT_OK;
}
////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
stat_t cm_set_feed_rate(float feed_rate){
	if(cm.gm.feedrate_mode == INVERSE_TIME_MODE){
		cm.gm.feedrate = 1/cm.gn.feedrate;  //total time to finish the move in minutes
	}
	else{
		cm.gm.feedrate = _TO_MILLI(feed_rate);
	}
	return STAT_OK;
}
////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
stat_t cm_select_plane(uint8_t plane){
	cm.gm.plane_select = plane;
	return STAT_OK;
}

////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
stat_t cm_select_unit_mode(uint8_t mode){
	cm.gm.units_mode = mode;
	return STAT_OK;
}
////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
stat_t cm_select_path_control(uint8_t path){
	cm.gm.path_control = path;
	return STAT_OK;
}
////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
stat_t cm_select_distance_mode(uint8_t mode){
	cm.gm.distance_mode = mode;
	return STAT_OK;
}
////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
stat_t cm_straight_traverse(float target[],float flags[]){
	cm.gm.motion_mode = MOTION_MODE_STRAIGHT_TRAVERSE;
	cm_set_model_target(target,flags);
	//check soft limits
	//return if soft limit alerted
	//start cycle
	mp_plan_line(&cm.gm);
	//finalize the move
	memcpy(&cm.position,&cm.gm.target,sizeof(cm.gm.target));
	return STAT_OK;
}
////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
stat_t cm_straight_feed(float target[],float flags[]){
	//if in inverse time feed mode and the feed rate is omitted its an error and a return is issued
	db_start_session(CANONICAL_TIME);
	if ((cm.gm.feedrate_mode != INVERSE_TIME_MODE) && (fp_ZERO(cm.gm.feedrate))) {
		return (STAT_GCODE_FEEDRATE_NOT_SPECIFIED);
	}
	cm.gm.motion_mode = MOTION_MODE_STRAIGHT_FEED;
	cm_set_model_target(target,flags);
	//check soft limits
	//return if soft limit alerted
	
	cm_set_work_offsets(&cm.gm);
	cm_cycle_start();
	mp_plan_line(&cm.gm);
	cm_finalize_move();
	db_end_session(CANONICAL_TIME);
	return STAT_OK;
}

////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
stat_t cm_set_coord_offsets(uint8_t coord_system, float target[], float flags[]){
	if((coord_system<G54)||(coord_system>G59)){
		return STAT_INPUT_VALUE_OUT_OF_RANGE;
	}
	for(uint8_t axis = X_AXIS; axis<AXES; ++axis){
		if(flags[axis] > 0.0f){
			cm.coord_offset[coord_system][axis] = _TO_MILLI(target[axis]);
		}
	}
	return STAT_OK;
}
////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
stat_t cm_set_coord_system(uint8_t coord){
	cm.gm.coordinate_system = coord;
	return STAT_OK;
}
////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
void cm_set_absolute_override(uint8_t state){
	cm.gm.absolute_override = state;
	//reset offsets here
}
////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
stat_t cm_set_origin_offsets(float values[], float flags[]){
	cm.origin_offset_enable = 1;
	for(uint8_t axis = X_AXIS; axis<AXES; ++axis){
		if(flags[axis] > 0.0f){
			cm.origin_offset[axis] = cm.position[axis] - cm.coord_offset[cm.gm.coordinate_system][axis] - _TO_MILLI(values[axis]);
		}
	}
	return STAT_OK;
}
////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
stat_t cm_reset_origin_offsets(void){
	cm.origin_offset_enable = 0;
	for(uint8_t axis = X_AXIS; axis<AXES; ++axis){
			cm.origin_offset[axis] = 0;
		}
	return STAT_OK;
}
////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
stat_t cm_suspend_origin_offsets(void){
	cm.origin_offset_enable = 0;
	return STAT_OK;
}
////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
stat_t cm_resume_origin_offsets(void){
	cm.origin_offset_enable = 1;
	return STAT_OK;
}



void cm_cycle_start(void)
{
	cm.machine_state = MACHINE_CYCLE;
	if (cm.cycle_state == CYCLE_OFF) {					// don't (re)start homing, probe or other canned cycles
		cm.cycle_state = CYCLE_MACHINING;
	}
}

void cm_finalize_move(void) {
	copy_vector(cm.position, cm.gm.target);		// update model position

	// if in ivnerse time mode reset feed rate so next block requires an explicit feed rate setting
	if ((cm.gm.feedrate_mode == INVERSE_TIME_MODE) && (cm.gm.motion_mode == MOTION_MODE_STRAIGHT_FEED)) {
		cm.gm.feedrate = 0;
	}
}


stat_t cm_soft_alarm(stat_t status)
{
	cm.machine_state = MACHINE_ALARM;
	return (status);						// NB: More efficient than inlining rpt_exception() call.
}

stat_t cm_hard_alarm(stat_t status)
{
	// stop the motors and the spindle
	st_init();							// hard stop
	//turn off spindle
	cm.machine_state = MACHINE_SHUTDOWN;
	return (status);
}

uint8_t cm_get_machine_state(void) { return cm.machine_state;};

uint8_t cm_get_runtime_busy(void) {return mp_get_runtime_busy();}

///getters
uint8_t cm_get_coordinate_system(GState_t* gstate){
	return gstate->coordinate_system;
}
uint8_t cm_get_units_mode(GState_t* gstate){
	return gstate->units_mode;
}
uint8_t cm_get_distance_mode(GState_t* gstate){
	return gstate->distance_mode;
}
uint8_t cm_get_feedrate_mode(GState_t* gstate){
	return gstate->feedrate_mode;
}
uint8_t cm_get_feedrate(GState_t* gstate){
	return gstate->feedrate;
}

void cm_set_motion_mode(GState_t* gstate, uint8_t motion_mode){
	gstate->motion_mode = motion_mode;
}

float cm_get_axis_jerk(uint8_t axis){
	return cm.a[axis].max_jerk;
}

void cm_set_axis_jerk(uint8_t axis, float jerk){
	cm.a[axis].max_jerk = jerk;
	cm.a[X_AXIS].jerk_recip = 1.0f/((cm.a[X_AXIS].max_jerk)*JERK_MULTI);
}
void cm_request_cycle_start(void) { cm.cycle_start_requested = true; }

void cm_set_position(uint8_t axis, float position)
{
	// TODO: Interlock involving runtime_busy test
	cm.position[axis] = position;
	cm.gm.target[axis] = position;
	mp_set_planner_position(axis, position);
	mp_set_runtime_position(axis, position);
	mp_set_steps_to_runtime_position();
}


void cm_set_motion_state(uint8_t motion_state)
{
	cm.motion_state = motion_state;

	switch (motion_state) {
		case (MOTION_STOP): { ACTIVE_MODEL = MODEL; break; }
		case (MOTION_RUN):  { ACTIVE_MODEL = RUNTIME; break; }
		case (MOTION_HOLD): { ACTIVE_MODEL = RUNTIME; break; }
	}
}

float cm_get_absolute_position(GState_t *gcode_state, uint8_t axis)
{
	if (gcode_state == MODEL) return (cm.position[axis]);
	return (mp_get_runtime_absolute_position(axis));
}
