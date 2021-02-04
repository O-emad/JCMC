
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "system.h"
#include "canonical.h"
#include "planner.h"
#include "switch.h"
#include "util.h"



struct probeSingleton{
	
	uint8_t saved_coordinate;
	uint8_t saved_distance_mode;
	uint8_t saved_units_mode;
	uint8_t saved_feedrate_mode;
	uint8_t saved_feedrate;
	float saved_jerk[AXES];
	
	stat_t (*func)(void);
	
	float start_position[AXES];
	float target[AXES];
	float flags[AXES];
};

static struct probeSingleton pb;

static uint8_t _probing_init(void);
static stat_t _probing_start(void);
static stat_t _probing_finish(void);
static void	_probe_restore_settings(void);
static stat_t _probing_error_exit(stat_t status);
static stat_t _probing_finalize(void);

stat_t _set_pb_function(stat_t (*func)(void)){
	pb.func = func;
	return STAT_RC;
}

stat_t cm_straight_probe(float target[],float flags[]){
	
	if(cm.gm.feedrate_mode == INVERSE_TIME_MODE){
		return STAT_PROBE_INVERSE_TIME_FEEDRATE;
	}
	
	if ((cm.gm.feedrate_mode != INVERSE_TIME_MODE) && (fp_ZERO(cm.gm.feedrate))) {
		return (STAT_GCODE_FEEDRATE_NOT_SPECIFIED);
	}
	
	if(fp_ZERO(flags[X_AXIS]) && fp_ZERO(flags[Y_AXIS]) && fp_ZERO(flags[Z_AXIS]))
		return STAT_PROBE_ALL_AXES_OMITTED;
	
	copy_vector(pb.target, target);		// set probe move endpoint
	copy_vector(pb.flags, flags);		// set axes involved on the move
	clear_vector(cm.probe_results);		// clear the old probe position.
										// NOTE: relying on probe_result will not detect a probe to 0,0,0.

	cm.probe_state = PROBE_WAITING;		// wait until planner queue empties before completing initialization
	pb.func = _probing_init; 			// bind probing initialization function
	return (STAT_OK);
}

stat_t cm_probe_callback(void)
{
	if ((cm.cycle_state != CYCLE_PROBE) && (cm.probe_state != PROBE_WAITING)) {
		return (STAT_NOOP);				// exit if not in a probe cycle or waiting for one
	}
	if (cm_get_runtime_busy() == true) { return (STAT_RC);}	// sync to planner move ends
	return (pb.func());                                         // execute the current homing move
}


static stat_t _probing_init(void){
	cm.cycle_state = CYCLE_PROBE;
	cm.probe_state = PROBE_FAILED;
	
	for(uint8_t axis=X_AXIS; axis<AXES; ++axis){
		pb.saved_jerk[axis] = cm_get_axis_jerk(axis);
		pb.start_position[axis] = cm_get_absolute_position(ACTIVE_MODEL, axis);
		cm_set_axis_jerk(axis,cm.a[axis].homing_jerk);
	}
	
	if(get_vector_length(pb.target,pb.start_position) < MINIMUM_PROBE_DISTANCE){
		_probing_error_exit(STAT_MINIMUM_PROBE_DISTANCE);
	}
	
	pb.saved_coordinate = cm_get_coordinate_system(ACTIVE_MODEL);
	pb.saved_distance_mode = cm_get_distance_mode(ACTIVE_MODEL);
	cm_select_distance_mode(ABSOLUTE_MODE);
	cm_set_coord_system(ABSOLUTE_COORDS);

	//cm_spindle_control(SPINDLE_OFF);
	return (_set_pb_function(_probing_start));							// start the move
}

static stat_t _probing_start(void){
	if(get_probe_state() == SW_OPEN){
		uint8_t status = cm_straight_feed(pb.target,pb.flags);
		if(status != STAT_OK) return _probing_error_exit(status);
	}
	return (_set_pb_function(_probing_finish));
}

static stat_t _probing_finish(void){
	cm.probe_state = (get_probe_state() == SW_CLOSED)? PROBE_SUCCEEDED:PROBE_FAILED;
	for( uint8_t axis=0; axis<AXES; axis++ ) {
		// if we got here because of a limit switch being hit we need to keep the model position correct
		cm_set_position(axis, mp_get_runtime_work_position(axis));

		// store the probe results
		cm.probe_results[axis] = cm_get_absolute_position(ACTIVE_MODEL, axis);
	}
	return (_set_pb_function(_probing_finalize));
}


static stat_t _probing_finalize(void){
	_probe_restore_settings();
	return STAT_OK;
}

static stat_t _probing_error_exit(stat_t status){
	_probe_restore_settings();
	return status;
}

static void	_probe_restore_settings(void){
	
	mp_flush_planner();
	
	for(uint8_t axis=X_AXIS; axis<AXES; ++axis){
		cm_set_axis_jerk(axis,pb.saved_jerk[axis]);
	}
	cm_select_distance_mode(pb.saved_distance_mode);
	cm_set_coord_system(pb.saved_coordinate);
	cm_set_motion_mode(MODEL, MOTION_MODE_CANCEL_MOTION_MODE);
	//cm_cycle_end();
	cm.cycle_state = CYCLE_OFF;
}

