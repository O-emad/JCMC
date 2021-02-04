
#ifndef CANONICAL_H
#define CANONICAL_H

#define MODEL 	(GState_t *)&cm.gm		// absolute pointer from canonical machine gm model
#define PLANNER (GState_t *)&bf->gm		// relative to buffer *bf is currently pointing to
#define RUNTIME (GState_t *)&mr.gm		// absolute pointer from runtime mm struct
#define ACTIVE_MODEL cm.am					// active model pointer is maintained by state management

#define _TO_MILLI(a) ((cm.gm.units_mode == INCHES) ? (a * MM_PER_INCH) : a)

typedef struct GCodeInput{
	uint32_t linenum;			//N code
	
	uint8_t programflow;	//ending and stopping program
	
	uint8_t next_action;	//non-modal group 0
	uint8_t motion_mode;	//modal group 1
	
	uint8_t feedrate_mode;		//feedrate setting
	float feedrate;						//F in millimeter per minute
	
	
	float target[AXES];		//target point
	
	uint8_t plane_select;			//modal group 2
	uint8_t units_mode;				//modal group 6
	uint8_t distance_mode;		//modal group 3
	uint8_t coordinate_system;		//modal group 12
	uint8_t absolute_override;		//G53 flag, for the current block only
	uint8_t path_control;			//modal group 13
	
	uint8_t tool;					//tool after declaring the tool with T and assigning it with M6
	uint8_t tool_select;	//tool declaring T value
	uint8_t tool_change;	//tool assigning M6, assigning tool_select value to tool
	
	uint8_t mist_coolant;		//TRUE=MIST ON, FALSE=MIST OFF M7,M9
	uint8_t flood_coolant;	//TRUE=FLOOD ON, FALSE=FLOOOD OFF M8,M9
	uint8_t coolant_off;
	
	float parameter;				//P- parameter
	float radius;						//R, radius/R value
	float center_offsets[3];		//IJK
	
	float spindle_speed;		//S in RPM
	uint8_t spindle_mode;		//spindle setting
	//u have 3 chars that won't affect size
	
}GIn_t;


typedef struct GCodeState{
	
	uint32_t linenum;			//N code
	
	float move_time;

	float target[AXES];		//target point
	float work_offset[AXES];
	float spindle_speed;		//S in RPM
	float feedrate;						//F in millimeter per minute
	float parameter;				//P- parameter
	
	uint8_t motion_mode;	//modal group 1
	uint8_t feedrate_mode;		//feedrate setting
	uint8_t plane_select;			//modal group 2
	uint8_t units_mode;				//modal group 6
	uint8_t distance_mode;		//modal group 3
	uint8_t coordinate_system;		//modal group 12
	uint8_t absolute_override;		//G53 flag, for the current block only
	uint8_t path_control;			//modal group 13
	uint8_t tool;					//tool after declaring the tool with T and assigning it with M6
	uint8_t tool_select;	//tool declaring T value
	uint8_t mist_coolant;		//TRUE=MIST ON, FALSE=MIST OFF M7,M9
	uint8_t flood_coolant;	//TRUE=FLOOD ON, FALSE=FLOOOD OFF M8,M9
	uint8_t spindle_mode;		//spindle setting
	//u have 3 chars that won't affect size
}GState_t;

typedef struct AxisConfig{
	float max_velocity;		//max velocity in mm/min for traverse motion
	float max_feedrate;		//max velocity in mm/min for feed motion
	float max_jerk;				//max jerk
	float jerk_recip;
	float jerk_sqrt;
	float jerk_sqrt_recip;
	float max_travel;			//max work envelope for soft limits test
	float min_travel;			//min work envelope for soft limits test
	float junction_dev;
	float search_velocity;				// homing search velocity
	float latch_velocity;				// homing latch velocity
	float latch_backoff;				// backoff from switches prior to homing latch movement
	float zero_backoff;					// backoff from switches for machine zero
	float homing_jerk;
}Axiscfg_t;


typedef struct cmSingleton{
	float coord_offset[COORDS+1][AXES];
	float origin_offset[AXES];
	float position[AXES];
	
	float junction_acceleration;
	
	Axiscfg_t a[AXES];
	
	uint8_t origin_offset_enable;
	
	uint8_t homed[AXES];
	uint8_t machine_state;
	uint8_t cycle_state;
	uint8_t motion_state;
	uint8_t homing_state;
	uint8_t probe_state;
	
	uint8_t cycle_start_requested;
	
	float probe_results[AXES];
	float chordal_tolerance;
	float arc_segment_len;
	GState_t *am;
	
	GState_t gm;		//gcode model
	GIn_t gn;				//gcode input values
	GIn_t gf;				//gcode input flags
}cmSingleton_t;

extern cmSingleton_t cm;


enum MachineState {
	MACHINE_INITIALIZING = 0,		// machine is initializing
	MACHINE_READY,					// machine is ready for use
	MACHINE_ALARM,					// machine in soft alarm state
	MACHINE_PROGRAM_STOP,			// program stop or no more blocks
	MACHINE_PROGRAM_END,			// program end
	MACHINE_CYCLE,					// machine is running (cycling)
	MACHINE_SHUTDOWN,				// machine in hard alarm state (shutdown)
};

enum CycleState {
	CYCLE_OFF = 0,					// machine is idle
	CYCLE_MACHINING,				// in normal machining cycle
	CYCLE_PROBE,					// in probe cycle
	CYCLE_HOMING,					// homing is treated as a specialized cycle
	CYCLE_JOG						// jogging is treated as a specialized cycle
};

enum MotionState {
	MOTION_STOP = 0,				// motion has stopped
	MOTION_RUN,						// machine is in motion
	MOTION_HOLD						// feedhold in progress
};

enum HomingState{
	HOMING_NOT_HOMED =0,
	HOMING_HOMED,
	HOMING_WAITING
};

enum cmProbeState {					// applies to cm.probe_state
	PROBE_FAILED = 0,				// probe reached endpoint without triggering
	PROBE_SUCCEEDED = 1,			// probe was triggered, cm.probe_results has position
	PROBE_WAITING					// probe is waiting to be started
};

enum NEXT_ACTION{
	ACTION_DEFAULT = 0,
	ACTION_DWELL,
	ACTION_SET_COORD_DATA,
	ACTION_SET_AXIS_OFFSETS,
	ACTION_RESET_AXIS_OFFSETS,
	ACTION_SUSPEND_AXIS_OFFSETS,
	ACTION_RESUME_AXIS_OFFSETS,
	ACTION_GOTO_G28_POSITION,
	ACTION_SET_G28_POSITION,
	ACTION_SEARCH_HOME,
	ACTION_SET_ABSOLUTE_ORIGIN,
	ACTION_HOMING_NO_SET,
	ACTION_GOTO_G30_POSITION,
	ACTION_SET_G30_POSITION,
	ACTION_STRAIGHT_PROBE
	
};

enum MOTION_MODE{
	MOTION_MODE_STRAIGHT_TRAVERSE = 0, //G0
	MOTION_MODE_STRAIGHT_FEED, //G1
	MOTION_MODE_CW_ARC,					//G2
	MOTION_MODE_CCW_ARC,				//G3
	MOTION_MODE_CANCEL_MOTION_MODE,			//G80
	MOTION_MODE_CANNED_81,			//G81
	MOTION_MODE_CANNED_82,			//G82
	MOTION_MODE_CANNED_83,			//G83
	MOTION_MODE_CANNED_84,			//G84
	MOTION_MODE_CANNED_85,			//G85
	MOTION_MODE_CANNED_86,			//G86
	MOTION_MODE_CANNED_87,			//G87
	MOTION_MODE_CANNED_88,			//G88
	MOTION_MODE_CANNED_89			//G89
};

enum FEED_RATE_MODE{
	INVERSE_TIME_MODE = 0,  //G93
	UNITS_PER_MINUTE_MODE,	//G94
	UNITS_PER_REV_MODE			//G95
};

enum PLANE_SELECT{
	XY_PLANE = 0,		//G17
	XZ_PLANE,				//G18
	YZ_PLANE				//G19
};

enum UNIT_SYSTEM{
	INCHES = 0,			//G20
	MILLIMETERS,		//G21
};

enum COORDINATE_SYSTEM{
	ABSOLUTE_COORDS = 0,			//machine coordinate system
	G54,											//G54 coordinate
	G55,											//G55 coordinate
	G56,											//G56 coordinate
	G57,											//G57 coordinate
	G58,											//G58 coordinate
	G59,											//G59 coordinate
};

enum PATH_CONTROL{
	PATH_EXACT_PATH = 0,					//G61
	PATH_EXACT_STOP,							//G61.1
	PATH_CONTINUOUS								//G64
};

enum DISTANCE_SYSTEM{
	ABSOLUTE_MODE = 0,			//G90
	INCREMENTAL_MODE				//G91
};

enum SPINDLE_MODE{
	SPINDLE_OFF = 0,					//M5
	SPINDLE_CW,								//M3
	SPINDLE_CCW								//M4
};

enum COOLANT_MODE{
	COOLANT_OFF = 0,			//M9
	COOLANT_ON,
	COOLANT_MIST,					//M7
	COOLANT_FLOOD					//M8
};

enum PROGRAM_FLOW{
	PROGRAM_STOP = 0,
	PROGRAM_END
};

//for now we will be supporting modal groups G(1,2,3,5,6,12,13) M(4,6,7,8) and non-modal group 0
enum MODAL_GROUP{
	MODAL_GROUP_G0 = 0,
	MODAL_GROUP_G1,
	MODAL_GROUP_G2,
	MODAL_GROUP_G3,
	MODAL_GROUP_G5,
	MODAL_GROUP_G6,
	MODAL_GROUP_G12,
	MODAL_GROUP_G13,
	MODAL_GROUP_M4,
	MODAL_GROUP_M6,
	MODAL_GROUP_M7,
	MODAL_GROUP_M8,
};

void cm_set_work_offsets(GState_t *gcode_state);
void cm_set_model_target(float values[], float flags[]);
void canonical_init(void);
void cm_set_model_linenum(uint32_t linenum);
uint8_t cm_get_motion_mode(GState_t *gcode_state);
stat_t cm_set_feed_rate_mode(uint8_t mode);
stat_t cm_set_feed_rate(float feed_rate);
stat_t cm_select_plane(uint8_t plane);
stat_t cm_select_unit_mode(uint8_t mode);
stat_t cm_select_path_control(uint8_t path);
stat_t cm_select_distance_mode(uint8_t mode);
stat_t cm_set_coord_offsets(uint8_t coord_system, float target[], float flags[]);
stat_t cm_set_coord_system(uint8_t coord);
void cm_set_absolute_override(uint8_t state);
stat_t cm_set_origin_offsets(float values[], float flags[]);
stat_t cm_reset_origin_offsets(void);
stat_t cm_suspend_origin_offsets(void);
stat_t cm_resume_origin_offsets(void);
stat_t cm_straight_traverse(float target[],float flags[]);
stat_t cm_straight_feed(float target[],float flags[]);
stat_t cm_arc_feed(float target[], float flags[],float offsets[],float radius);
stat_t cm_arc_callback(void);
void cm_cycle_start(void);
void cm_finalize_move(void);
stat_t cm_soft_alarm(stat_t status);
stat_t cm_hard_alarm(stat_t status);
uint8_t cm_get_machine_state(void);
uint8_t cm_get_runtime_busy(void);
stat_t cm_homing_callback(void);
uint8_t cm_get_coordinate_system(GState_t* gstate);
uint8_t cm_get_units_mode(GState_t* gstate);
uint8_t cm_get_distance_mode(GState_t* gstate);
uint8_t cm_get_feedrate_mode(GState_t* gstate);
uint8_t cm_get_feedrate(GState_t* gstate);
void cm_set_motion_mode(GState_t* gstate, uint8_t motion_mode);
float cm_get_axis_jerk(uint8_t axis);
void cm_set_axis_jerk(uint8_t axis, float jerk);
void cm_request_cycle_start(void);
void cm_set_position(uint8_t axis, float position);
void cm_abort_arc(void);
void cm_set_motion_state(uint8_t motion_state);
float cm_get_absolute_position(GState_t *gcode_state, uint8_t axis);
stat_t cm_cycle_homing_start(void);
stat_t cm_straight_probe(float target[],float flags[]);
#endif

