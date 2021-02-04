
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "system.h"
#include "canonical.h"
#include "planner.h"
#include "switch.h"
#include "util.h"

struct homingSingleton{
	// state saved from gcode model
	uint8_t saved_units_mode;		// G20,G21 global setting
	uint8_t saved_coord_system;		// G54 - G59 setting
	uint8_t saved_distance_mode;	// G90,G91 global setting
	uint8_t saved_feed_rate_mode;   // G93,G94 global setting
	float saved_feed_rate;			// F setting
	float saved_jerk;				// saved and restored for each axis homed
	
	float direction;
	float search_travel;
	float search_velocity;
	float latch_velocity;
	float latch_backoff;
	float zero_backoff;
	
	uint8_t homing_closed;			// 0=open, 1=closed
	uint8_t limit_closed;			// 0=open, 1=closed
	uint8_t set_coordinates;		// G28.4 flag. true = set coords to zero at the end of homing cycle
	stat_t (*func)(int8_t axis);	// binding for callback function state machine
	
	int8_t axis;					// axis currently being homed
	uint8_t min_mode;				// mode for min switch for this axis
	uint8_t max_mode;				// mode for max switch for this axis
	
	int8_t homing_switch_axis;
	int8_t limit_switch_axis;
	int8_t homing_switch_position;
	int8_t limit_switch_position;

};

static struct homingSingleton hm;

stat_t _homing_axis_start(int8_t axis);
static stat_t _homing_finalize_exit(int8_t axis);
static stat_t _homing_error_exit(stat_t status);
static int8_t _get_next_axis(int8_t axis);
static stat_t _set_homing_func(stat_t (*func)(int8_t axis));
static stat_t _homing_axis_clear(int8_t axis);
static stat_t _homing_axis_move(int8_t axis, float target, float velocity);
static stat_t _homing_axis_set_zero(int8_t axis);
static stat_t _homing_axis_search(int8_t axis);
static stat_t _homing_axis_latch(int8_t axis);
static stat_t _homing_axis_zero_backoff(int8_t axis);
static stat_t _homing_abort(int8_t axis);

stat_t cm_cycle_homing_start(void){
	////save current state of the machince
	hm.saved_coord_system = cm_get_coordinate_system(ACTIVE_MODEL);
	hm.saved_distance_mode = cm_get_distance_mode(ACTIVE_MODEL);
	hm.saved_feed_rate_mode = cm_get_feedrate_mode(ACTIVE_MODEL);
	hm.saved_feed_rate = cm_get_feedrate(ACTIVE_MODEL);
	hm.saved_units_mode = cm_get_units_mode(ACTIVE_MODEL);
	
	//set up homing state
	cm_select_unit_mode(MILLIMETERS);
	cm_select_distance_mode(INCREMENTAL_MODE);
	cm_set_coord_system(ABSOLUTE_COORDS);	// homing is done in machine coordinates
	cm_set_feed_rate_mode(UNITS_PER_MINUTE_MODE);
	hm.set_coordinates = true;
	
	hm.axis = -1;
	hm.func = _homing_axis_start;
	cm.cycle_state = CYCLE_HOMING;
	cm.homing_state = HOMING_NOT_HOMED;
	return STAT_OK;
}


//step zero: homing intialization for an axis
static stat_t _homing_axis_start(int8_t axis){
	
	//get next axis .. if it's back to -1 then all axes are homed
	//if -2 then none of the axes are requested to be homed
	if ((axis = _get_next_axis(axis)) < 0) { 				// axes are done or error
		if (axis == -1) {									// -1 is done
			cm.homing_state = HOMING_HOMED;
			return (_set_homing_func(_homing_finalize_exit));
		} else if (axis == -2) { 							// -2 is error
			return (_homing_error_exit(STAT_HOMING_BAD_OR_NO_AXIS_WORDS));
		}
	}
	
	cm.homed[axis] = false;
	
	if(fp_ZERO(cm.a[axis].search_velocity)){return _homing_error_exit(STAT_HOMING_ZERO_SEARCH_VELOCITY);}
	if(fp_ZERO(cm.a[axis].latch_velocity)){return _homing_error_exit(STAT_HOMING_ZERO_LATCH_VELOCITY);}
	if (cm.a[axis].latch_backoff < 0) return (_homing_error_exit(STAT_HOMING_ERROR_NEGATIVE_LATCH_BACKOFF));
	
	float travel_distance = fabs(cm.a[axis].max_travel - cm.a[axis].min_travel) + cm.a[axis].latch_backoff;
	if (fp_ZERO(travel_distance)) return (_homing_error_exit(STAT_HOMING_ERROR_TRAVEL_MIN_MAX_IDENTICAL));
	
	hm.min_mode = get_switch_mode(axis, SW_MIN);
	hm.max_mode = get_switch_mode(axis, SW_MAX);
	
	//can't have two switches homing ... one limit and one home/r home&limit
	if(((hm.min_mode&HOMING_BIT)^(hm.max_mode&HOMING_BIT)) == 0){
		return _homing_error_exit(STAT_HOMING_SWITCH_MISCONFIGURATION);
	}
	hm.axis = axis;											// persist the axis
	hm.search_velocity = fabsf(cm.a[axis].search_velocity);	// search velocity is always positive
	hm.latch_velocity = fabsf(cm.a[axis].latch_velocity);	// latch velocity is always positive
	//min switch is the homing switch 
	if(hm.min_mode&HOMING_BIT){
		hm.homing_switch_axis = axis;				// the min is the homing switch
		hm.limit_switch_axis = axis;					// the max would be the limit switch
		hm.homing_switch_position = SW_MIN;
		hm.limit_switch_position = SW_MAX;
		hm.search_travel = travel_distance;
		hm.latch_backoff = -cm.a[axis].latch_backoff;
		hm.zero_backoff = -cm.a[axis].zero_backoff;
	}else{
		hm.homing_switch_axis = axis;				// the min is the homing switch
		hm.limit_switch_axis = axis;					// the max would be the limit switch
		hm.homing_switch_position = SW_MAX;
		hm.limit_switch_position = SW_MIN;
		hm.search_travel = -travel_distance;
		hm.latch_backoff = cm.a[axis].latch_backoff;
		hm.zero_backoff = cm.a[axis].zero_backoff;
	}
	
	uint8_t sw_mode = get_switch_mode(hm.homing_switch_axis,hm.homing_switch_position);
	if ((sw_mode != SW_MODE_HOMING) && (sw_mode != SW_MODE_HOMING_LIMIT)) {
		//if the homing switch is not in homing mode.. skip it to the next axis
		return (_set_homing_func(_homing_axis_start));
	}
	// disable the limit switch parameter if there is no limit switch
	if (get_switch_mode(hm.limit_switch_axis,hm.limit_switch_position) == SW_MODE_DISABLED) hm.limit_switch_axis = -1;
	
	hm.saved_jerk = cm_get_axis_jerk(axis);					// save the max jerk value
	return (_set_homing_func(_homing_axis_clear));			// start the clear
}

//step one : clearing switches if it has been invoked before starting homing cycle
static stat_t _homing_axis_clear(int8_t axis){
	if (get_switch_state(hm.homing_switch_axis,hm.homing_switch_position) == SW_CLOSED) {
		_homing_axis_move(axis, hm.latch_backoff, hm.search_velocity);
	}else if(get_switch_state(hm.limit_switch_axis,hm.limit_switch_position) == SW_CLOSED) {
		_homing_axis_move(axis, -hm.latch_backoff, hm.search_velocity);
	}
	return (_set_homing_func(_homing_axis_search));
}

//step two: search for homing switch
static stat_t _homing_axis_search(int8_t axis){
	cm_set_axis_jerk(axis, cm.a[axis].homing_jerk);			// use the homing jerk for search onward
	_homing_axis_move(axis, hm.search_travel, hm.search_velocity);
    return (_set_homing_func(_homing_axis_latch));
}

//step three: move away from switch at latch velocity
static stat_t _homing_axis_latch(int8_t axis){
	if (get_switch_state(hm.homing_switch_axis,hm.homing_switch_position) != SW_CLOSED)
		return (_set_homing_func(_homing_abort));
	_homing_axis_move(axis, hm.latch_backoff, hm.latch_velocity);
	return (_set_homing_func(_homing_axis_zero_backoff));
}

//step four: zero backoff from switch
static stat_t _homing_axis_zero_backoff(int8_t axis)		// backoff to zero position
{
	_homing_axis_move(axis, hm.zero_backoff, hm.search_velocity);
	return (_set_homing_func(_homing_axis_set_zero));
}

//save the current position as origin if in SET cycle and skip it if in search cycle
static stat_t _homing_axis_set_zero(int8_t axis)			// set zero and finish up
{
	if (hm.set_coordinates != false) {
		cm_set_position(axis,0);
		cm.homed[axis] = true;
	} else {
        // do not set axis if in G28.4 cycle
		cm_set_position(axis, mp_get_runtime_position(axis));
	}
	cm_set_axis_jerk(axis, hm.saved_jerk);					// restore the max jerk value

	return (_set_homing_func(_homing_axis_start));
}

static stat_t _homing_axis_move(int8_t axis, float target, float velocity){
	float vect[] = {0,0,0,0,0,0};
	float flags[] = {false, false, false, false, false, false};

	vect[axis] = target;
	flags[axis] = true;
	cm.gm.feedrate = velocity;
	mp_flush_planner();										
	cm_request_cycle_start();
	stat_t status = cm_straight_feed(vect, flags);
	if(status!= STAT_OK) return status;
	return (STAT_RC);
}

static stat_t _homing_abort(int8_t axis)
{
	cm_set_axis_jerk(axis, hm.saved_jerk);					// restore the max jerk value
	_homing_finalize_exit(axis);
	return (STAT_HOMING_CYCLE_FAILED);						// homing state remains HOMING_NOT_HOMED
}

static stat_t _set_homing_func(stat_t (*func)(int8_t axis)){
	hm.func = func;
	return STAT_RC;
}

static stat_t _homing_finalize_exit(int8_t axis){
	mp_flush_planner(); 					// should be stopped, but in case of switch closure.
													

	cm_set_coord_system(hm.saved_coord_system);				// restore to work coordinate system
	cm_select_unit_mode(hm.saved_units_mode);
	cm_select_distance_mode(hm.saved_distance_mode);
	cm_set_feed_rate_mode(hm.saved_feed_rate_mode);
	cm.gm.feedrate = hm.saved_feed_rate;
	cm_set_motion_mode(MODEL, MOTION_MODE_CANCEL_MOTION_MODE);
	//cm_cycle_end();
	cm.cycle_state = CYCLE_OFF;
	return (STAT_OK);
}

static stat_t _homing_error_exit(stat_t status){
	_homing_finalize_exit(0);
	return (status);						// homing state remains HOMING_NOT_HOMED
}


static int8_t _get_next_axis(int8_t axis){
	if (axis == -1) {	// inelegant brute force solution
		if (fp_TRUE(cm.gf.target[Z_AXIS])) return (Z_AXIS);
		if (fp_TRUE(cm.gf.target[X_AXIS])) return (X_AXIS);
		if (fp_TRUE(cm.gf.target[Y_AXIS])) return (Y_AXIS);
		return (-2);	// error
	} else if (axis == Z_AXIS) {
		if (fp_TRUE(cm.gf.target[X_AXIS])) return (X_AXIS);
		if (fp_TRUE(cm.gf.target[Y_AXIS])) return (Y_AXIS);
	} else if (axis == X_AXIS) {
		if (fp_TRUE(cm.gf.target[Y_AXIS])) return (Y_AXIS);
	}
	return (-1);	// done
}


stat_t cm_homing_callback(void){
	if (cm.cycle_state != CYCLE_HOMING) { return (STAT_NOOP);} 	// exit if not in a homing cycle
	if (cm_get_runtime_busy() == true) { return (STAT_RC);}	// sync to planner move ends
	return (hm.func(hm.axis));									// execute the current homing move
}
