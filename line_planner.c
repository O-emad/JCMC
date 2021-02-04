/*
 * gcode_parser.c
 * This file is part of the X project
 *
 * Omar Emad El-Deen
 * Yossef Mohammed Hassanin
 * Mars, 2018
 */
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "system.h"
#include "canonical.h"
#include "planner.h"
#include "loader.h"
#include "debugging.h"
#include "util.h"




////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
stat_t mp_plan_line(GState_t *gmod){
	
	mpBuf_t *bf;
	
	float axis_length[AXES];
	float axis_length_square[AXES];
  float length = 0.0f;
  float length_square = 0.0f;
	float length_square_cbrt = 0.0f;
	//float length_square_cbrt = 0;
	
	float exact_stop = 0;
	volatile float junction_velocity = 8675309;
	db_start_session(PLAN_LINE_TIME);
	for(uint8_t axis = X_AXIS; axis<AXES; ++axis){
		axis_length[axis] = gmod->target[axis]-mm.position[axis]; //check which is better cm.position or mm.position
		axis_length_square[axis] = square(axis_length[axis]);
		length_square +=	axis_length_square[axis]; 
	}
	length = sqrtf(length_square);
	length_square_cbrt = cbrtf(length_square);
	if(fp_ZERO(length)){		//1-length have been checked during planning in here 
		return STAT_OK;			//there's nothing to plan it's a 0 length motion
	}
	//2- if the passed the following .. then this move have appropriate feedrate mode, resolved coordinates, and length
	//now check if this length is executable in at least 1 segement
	_calc_move_time(gmod,length,axis_length);
	//this section is all about estimation, mm.jerk_cbrt is the cubic root of the last jerk value within tolerance
	//we can use this for estimation, but it's not entirely true
	//what happens if your first move when mm.jerk_cbrt is actually short on time
	//that's 0 jerk , 0 dv, 0entry velocity ,, hooray INF move time,, so a check on the INF move time here sounds great...
	//it's correct if we calculate the move jerk .. then the velocity change ..then the move time 
	
	//////////////////
	if(gmod->move_time < MIN_BLOCK_TIME){
		float delta_velocity = mp_get_deltav_max(length_square_cbrt,mm.jerk_cbrt);//mm.jerk_cbrt_recip should be given a value at starting
		float entry_velocity = 0;
		bf = mp_get_latest_queued_buffer();
		if(bf->replanned == true){								  	//this move isn't optimally planned so assume its exit velocity 
			entry_velocity = bf->entry_velocity + 			//equals its entry velocity plus its deltav_max
											 bf->delta_vmax;
		
		}else{									//this move is optimally planned, the entry velocity is this blocks out velocity 
			entry_velocity = bf->exit_velocity;
		}
		float move_time = (2*length)/((2*entry_velocity)+delta_velocity);
		if(isinf(move_time)){
			return STAT_ZERO_VELOCITY_MOVE;
		}
		if(move_time < MIN_BLOCK_TIME){
			return STAT_MINIMUM_TIME_MOVE;
		}
	}
	//try to force your code out of this section during testing
	//////////////////
	if((bf = mp_get_write_buffer()) == NULL){
		return STAT_BUFFER_FULL;				//this actually should never happen as the planner sync will assure
	}																	//always that there's empty buffers
	bf->bf_fun = mp_exec_line;
	bf->length = length;
	bf->length_sqr_cbrt = length_square_cbrt;
	memcpy(&bf->gm,gmod,sizeof(GState_t)); //bf->gm now holds the MODEL
	db_start_session(PLAN_MOTION_JERK);
	mp_set_motion_jerk(axis_length,axis_length_square,length_square,bf); //combined operation sets unit vector, jerk and its components
	db_end_session(PLAN_MOTION_JERK);
	if(bf->gm.path_control != PATH_EXACT_STOP){
		bf->replanned = true;
		exact_stop = 8675309;
		junction_velocity=_get_junction_vmax(bf->pv->unit,bf->unit);
	}
	bf->cruise_vmax = bf->length/bf->gm.move_time;
	bf->entry_vmax = min3(bf->cruise_vmax,exact_stop,junction_velocity);
	bf->delta_vmax = mp_get_deltav_max(bf->length_sqr_cbrt,bf->jerk_cbrt);
	bf->exit_vmax = min3((bf->entry_vmax + bf->delta_vmax),exact_stop,bf->cruise_vmax);
	bf->braking_velocity = bf->delta_vmax;
	_plan_block_list(bf);
	copy_vector(mm.position,bf->gm.target);
	//mp_commit_write_buffer(MOVE_TYPE_ALINE);
	db_end_session(PLAN_LINE_TIME);
	return STAT_OK;
}


////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
static void _calc_move_time(GState_t *gmod, float length, float axis_length[]){
	float xyz_time = 0;
	float tmp_time = 0;
	float max_time = 0;
	if(gmod->motion_mode != MOTION_MODE_STRAIGHT_TRAVERSE){
		if(gmod->feedrate_mode == INVERSE_TIME_MODE){
			xyz_time = gmod->feedrate;
			gmod->feedrate_mode = UNITS_PER_MINUTE_MODE;
		}else{
			xyz_time = length/gmod->feedrate;
		}
	}
	for(uint8_t axis = 0; axis<AXES; ++axis){
		if(gmod->motion_mode == MOTION_MODE_STRAIGHT_TRAVERSE){
			tmp_time = axis_length[axis]/cm.a[axis].max_velocity;
		}else{
			tmp_time = axis_length[axis]/cm.a[axis].max_feedrate;
		}
		max_time = max(tmp_time,max_time);
	}
	gmod->move_time = max(max_time,xyz_time);
}


////
//input : 
//output : 
//fuction : 
//notes : 
//additions: 
//
static void _plan_block_list(mpBuf_t *bf){
	db_start_session(PLAN_BLOCK_LIST_TIME);
	mpBuf_t* bp = bf;
	while ((bp = mp_get_prev_buffer(bp)) != bf) {
		if (bp->replanned == false) { break; }
		bp->braking_velocity = (bp->nx->braking_velocity) + bp->delta_vmax;
	}
	while((bp = mp_get_next_buffer(bp)) != bf){
		if (bp->pv == bf )  {
			bp->entry_velocity = bp->entry_vmax;		// first block in the list
		} else {
			bp->entry_velocity = bp->pv->exit_velocity;	// other blocks in the list
		}
		bp->cruise_velocity = bp->cruise_vmax;
		bp->exit_velocity = min4(bp->exit_vmax,(bp->entry_velocity + bp->delta_vmax),bp->nx->entry_vmax,bp->nx->braking_velocity);
		db_start_session(PLAN_MOTION_PLANNING);
		mp_motion_planning(bp);
		db_end_session(PLAN_MOTION_PLANNING);
		//check replanning condition
		if(fp_Equal(bp->exit_velocity,bp->exit_vmax)||
			 fp_Equal(bp->exit_velocity,bp->nx->entry_vmax)||
			 ((bp->pv->replanned == false) && fp_Equal(bp->exit_velocity,(bp->entry_velocity+bp->delta_vmax)))
			){
			bp->replanned = false;
		}
	}
	bp->entry_velocity = bp->pv->exit_velocity;
	bp->cruise_velocity = bp->cruise_vmax;
	bp->exit_velocity = 0;
	db_start_session(PLAN_MOTION_PLANNING);
	mp_motion_planning(bp);
	db_end_session(PLAN_MOTION_PLANNING);
	db_end_session(PLAN_BLOCK_LIST_TIME);
}


/*
 * _get_junction_vmax() - Sonny's algorithm - simple
 *
 *  Computes the maximum allowable junction speed by finding the velocity that will yield
 *	the centripetal acceleration in the corner_acceleration value. The value of delta sets
 *	the effective radius of curvature. Here's Sonny's (Sungeun K. Jeon's) explanation
 *	of what's going on:
 *
 *	"First let's assume that at a junction we only look a centripetal acceleration to simply
 *	things. At a junction of two lines, let's place a circle such that both lines are tangent
 *	to the circle. The circular segment joining the lines represents the path for constant
 *	centripetal acceleration. This creates a deviation from the path (let's call this delta),
 *	which is the distance from the junction to the edge of the circular segment. Delta needs
 *	to be defined, so let's replace the term max_jerk (see note 1) with max_junction_deviation,
 *	or "delta". This indirectly sets the radius of the circle, and hence limits the velocity
 *	by the centripetal acceleration. Think of the this as widening the race track. If a race
 *	car is driving on a track only as wide as a car, it'll have to slow down a lot to turn
 *	corners. If we widen the track a bit, the car can start to use the track to go into the
 *	turn. The wider it is, the faster through the corner it can go.
 *
 * (Note 1: "max_jerk" refers to the old grbl/marlin max_jerk" approximation term, not the
 *	tinyG jerk terms)
 *
 *	If you do the geometry in terms of the known variables, you get:
 *		sin(theta/2) = R/(R+delta)  Re-arranging in terms of circle radius (R)
 *		R = delta*sin(theta/2)/(1-sin(theta/2).
 *
 *	Theta is the angle between line segments given by:
 *		cos(theta) = dot(a,b)/(norm(a)*norm(b)).
 *
 *	Most of these calculations are already done in the planner. To remove the acos()
 *	and sin() computations, use the trig half angle identity:
 *		sin(theta/2) = +/- sqrt((1-cos(theta))/2).
 *
 *	For our applications, this should always be positive. Now just plug the equations into
 *	the centripetal acceleration equation: v_c = sqrt(a_max*R). You'll see that there are
 *	only two sqrt computations and no sine/cosines."
 *
 *	How to compute the radius using brute-force trig:
 *		float theta = acos(costheta);
 *		float radius = delta * sin(theta/2)/(1-sin(theta/2));
 */
/*  This version extends Chamnit's algorithm by computing a value for delta that takes
 *	the contributions of the individual axes in the move into account. This allows the
 *	control radius to vary by axis. This is necessary to support axes that have
 *	different dynamics; such as a Z axis that doesn't move as fast as X and Y (such as a
 *	screw driven Z axis on machine with a belt driven XY - like a Shapeoko), or rotary
 *	axes ABC that have completely different dynamics than their linear counterparts.
 *
 *	The function takes the absolute values of the sum of the unit vector components as
 *	a measure of contribution to the move, then scales the delta values from the non-zero
 *	axes into a composite delta to be used for the move. Shown for an XY vector:
 *
 *	 	U[i]	Unit sum of i'th axis	fabs(unit_a[i]) + fabs(unit_b[i])
 *	 	Usum	Length of sums			Ux + Uy
 *	 	d		Delta of sums			(Dx*Ux+DY*UY)/Usum
 */

static float _get_junction_vmax(float a_unit[], float b_unit[])
{
	float costheta = - (a_unit[X_AXIS] * b_unit[X_AXIS])
					 - (a_unit[Y_AXIS] * b_unit[Y_AXIS])
					 - (a_unit[Z_AXIS] * b_unit[Z_AXIS]);

	if (costheta < -0.99f) { return (10000000.0f); } 		// straight line cases
	if (costheta > 0.99f)  { return (0.0f); } 				// reversal cases

	// Fuse the junction deviations into a vector sum
	float a_delta = square(a_unit[X_AXIS] * cm.a[X_AXIS].junction_dev);
	a_delta += square(a_unit[Y_AXIS] * cm.a[Y_AXIS].junction_dev);
	a_delta += square(a_unit[Z_AXIS] * cm.a[Z_AXIS].junction_dev);


	float b_delta = square(b_unit[X_AXIS] * cm.a[X_AXIS].junction_dev);
	b_delta += square(b_unit[Y_AXIS] * cm.a[Y_AXIS].junction_dev);
	b_delta += square(b_unit[Z_AXIS] * cm.a[Z_AXIS].junction_dev);


	float delta = (sqrtf(b_delta) + sqrtf(a_delta))/2.0f; //in worst case this will be 0 so no errors to be checked
	float sintheta_over2 = _sqrtf((1.0f - costheta)/2.0f);	//costheta won't ever get above 1 .. no errors to be checked
	float radius = delta * sintheta_over2 / (1.0f-sintheta_over2);//sintheta_over2 is positive and < 1 ..no error
	float velocity = _sqrtf(radius * cm.junction_acceleration);
	return (velocity);
}

uint8_t mp_get_runtime_busy(void){
	if ((ld.actuator_runtime_isbusy() == true) || (mr.move_state == MOVE_RUN)) return (true);
	return (false);
}


float mp_get_runtime_position(uint8_t axis){
	float position = mr.position[axis]-mr.gm.work_offset[axis];		//came out of here in mm
	if(mr.gm.units_mode == INCHES) position /= MM_PER_INCH;
	return position;
}

void mp_set_planner_position(uint8_t axis, float position){mm.position[axis]=position;}
void mp_set_runtime_position(uint8_t axis, float position){mr.position[axis]=position;}

