/*
 * gcode_parser.c
 * This file is part of the X project
 *
 * Omar Emad El-Deen
 * Yossef Mohammed Hassanin
 * Mars, 2018
 */
/*
 * this code is an implementation of g-code interpreter based on Kramer, Proctor and Messina's
 * interpreter as described in the NIST RS274/NGC v3
 *
 * g-code interpreter is the first layer of the controller it interprets the incoming g-code from recevied 
 * from a the main controller into a set of machine states.
 * the interpretation process is as follows:
 * 1- normalization of the g-code block : the block is changed into all caps form, all the white spaces are removed,
 * deleting leading zeros and signal a block delete if provided
 * 2- parsing : in the parsing process a set of functions are invoked to change the transational cononical machine inputs based
 * on the normalized block 
 * 3- execution : the execution follows the order of execution as provided by NIST RS274NGC, a set of functions are invoked
 * to alter the canonical machine state and start the planning process of coordinated motion
 */


//*******includes*******//
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include "system.h"
#include "canonical.h"
#include "debugging.h"
/////////////////////////

//******Prototypes******//
static void _normalize_gcode_block(char *str, uint8_t *d_flag);
static stat_t _parse_gcode_block(char *block);
static stat_t _execute_gcode_block(void);
//////////////////////////


//******Globals********//
struct gcodeparseSingleton{
	uint8_t MODAL[MODAL_GROUP_M8+1];
};

struct gcodeparseSingleton gc; //will be used for G-code validation
/////////////////////////


//gc_gcode_parser//
//input : a gcode block in the form of string
//output : state based on the execution of the process
//fuction : interpretes a gcode block into a machine state or a coordinated motion
//notes : ONLY THE MAIN CONTROLLER CAN INVOKE THIS FUNCTION
//additions: if the machine is alarmed don't process the following g code
//
stat_t gc_gcode_parser(char *block){
		
	char *block_cpy = block;
	uint8_t block_delete_flag;
	db_start_session(BLOCK_PREPARE_TIME);
	db_start_session(GCODE_PARSER_TIME);
	if(cm.machine_state == MACHINE_ALARM){return STAT_MACHINE_ALARMED;}
	
	_normalize_gcode_block(block_cpy, &block_delete_flag);
	
	//if the block delete flag is on which is a / in the first space 
	//ignor the block and return
	if (block_delete_flag == true) {
		return (STAT_NOOP);
	}
	return (_parse_gcode_block(block));
}

//_normalize_gcode_block//
//input : a gcode block in the form of string
//output : none
//fuction : normalize the g-code block
//the block is changed into all caps form, all the white spaces are removed,
//deleting leading zeros and signal a block delete if provided
//notes : ONLY ACCESSED BY gc_gcode_parser
//additions: 
//
static void _normalize_gcode_block(char *str, uint8_t *d_flag){

	char *read_p = str;
	char *write_p = str;
	
//signaling block delete flag
	if(*read_p == '/'){
	*d_flag = true;
	return;// why would you actually continue .. u will ignor the block
	}
	else{*d_flag = false;}
	
//deleting invalid charachters and whitespaces by ignoring them while changing the block into upper case
	for(; *write_p != NUL; read_p++){
		if(*read_p == NUL) *write_p = NUL;
		else if(isalnum(*read_p)||strchr(".-",*read_p)){
			*(write_p++) = (char)toupper(*read_p);
		}
	}
//deleting leading zeros
	read_p = str;
	while(*read_p != NUL){
		if(*read_p == '.'){
			read_p++;
			continue;
		}
		else if((!isdigit(*read_p))&&(*(read_p+1) == '0')&&(isdigit(*(read_p+2)))){
			write_p = read_p+1;
			while(*write_p != NUL){
				*write_p = *(write_p+1);
				write_p++;
			}
			continue;
		}
		read_p++;
	}
}

//_get_next_code_word//
//input : pointers to a char, float and an array of charachters
//output : state based on the word interpretation
//fuction : interpretes a word "N,M,G,X,Y,...." 
//notes : 
//additions: 
//
static stat_t _get_next_code_word(char **strp, char *letter, float *value){
	char *end;
	if(**strp == NUL){
		return STAT_COMPLETE;
	}
	if(islower(**strp)){
		return STAT_INVALID_CODE_FORM;
	}
	
	*letter = **strp;
	(*strp)++;
	//special case when G0 is followed by X-axis word e.g G0X20 the normal get value routine
	//will treat 0x20 as a hex decimal value and return 32
	if((**strp == '0')&&(*(*strp+1) == 'X')){
		*value = 0;
		(*strp)++;
		return STAT_OK;
	}
	// get value from string 
	*value = strtof(*strp,&end);
	if(*strp == end){
		//no conversion has been done
		return STAT_INVALID_NUMBER_FORM;
	}
	*strp = end;
	return STAT_OK;
}

//_point//
//input : a float value
//output : the point value "G60.1  ---> point value 1"
//fuction : returns the point value
//notes : 
//additions: 
//
static uint8_t _point(float value){
	return (uint8_t)(((value*10)-(truncf(value)*10))+0.5f);
}



#define SET_MODAL(m,param,val) {cm.gn.param = val; cm.gf.param = 1; gc.MODAL[m]++; break;}
#define SET_NON_MODAL(param,val) {cm.gn.param = val; cm.gf.param = 1; break;}

//_parse_gcode_block//
//input : normalized gcode block in the form of string
//output : state based on the execution of parsing
//fuction : invokes SET_MODAL and SET_NON_MODAL to set the cannonical machine gcode values and flags 
//notes : 
//additions: 
//
static stat_t _parse_gcode_block(char *block){
	char* strp = block;
	char letter;
	float value = 0;
	stat_t status = STAT_OK;
	
	memset(&gc,0,sizeof(gc));
	memset(&cm.gf,0,sizeof(GIn_t));
	memset(&cm.gn,0,sizeof(GIn_t));
	cm.gn.motion_mode = cm_get_motion_mode(&cm.gm);
	
	while((status = _get_next_code_word(&strp,&letter,&value)) == STAT_OK){
		switch(letter){
			case 'N': SET_NON_MODAL(linenum,(uint32_t)value);
			case 'G':{
				switch((uint8_t)value){
					case 0: SET_MODAL(MODAL_GROUP_G1,motion_mode,MOTION_MODE_STRAIGHT_TRAVERSE);
					case 1: SET_MODAL(MODAL_GROUP_G1,motion_mode,MOTION_MODE_STRAIGHT_FEED);
					case 2: SET_MODAL(MODAL_GROUP_G1,motion_mode,MOTION_MODE_CW_ARC);
					case 3: SET_MODAL(MODAL_GROUP_G1,motion_mode,MOTION_MODE_CCW_ARC);
					case 4: SET_NON_MODAL(next_action,ACTION_DWELL);
					case 10: SET_MODAL(MODAL_GROUP_G0,next_action,ACTION_SET_COORD_DATA);
					case 17: SET_MODAL(MODAL_GROUP_G2,plane_select,XY_PLANE);
					case 18: SET_MODAL(MODAL_GROUP_G2,plane_select,XZ_PLANE);
					case 19: SET_MODAL(MODAL_GROUP_G2,plane_select,YZ_PLANE);
					case 20: SET_MODAL(MODAL_GROUP_G6,units_mode,INCHES);
					case 21: SET_MODAL(MODAL_GROUP_G6,units_mode,MILLIMETERS);
					case 28: {
						switch(_point(value)){
							case 0: SET_MODAL (MODAL_GROUP_G0, next_action, ACTION_GOTO_G28_POSITION);
							case 1: SET_MODAL (MODAL_GROUP_G0, next_action, ACTION_SET_G28_POSITION);
							case 2: SET_NON_MODAL (next_action, ACTION_SEARCH_HOME);
							case 3: SET_NON_MODAL (next_action, ACTION_SET_ABSOLUTE_ORIGIN);
							case 4: SET_NON_MODAL (next_action, ACTION_HOMING_NO_SET);
							default: status = STAT_UNSUPPORTED_GCODE;
						}
						break;
					}
					case 30: {
						switch (_point(value)) {
							case 0: SET_MODAL (MODAL_GROUP_G0, next_action, ACTION_GOTO_G30_POSITION);
							case 1: SET_MODAL (MODAL_GROUP_G0, next_action, ACTION_SET_G30_POSITION);
							default: status = STAT_UNSUPPORTED_GCODE;
						}
						break;
					}
					case 38: {
						switch (_point(value)) {
							case 2: SET_NON_MODAL (next_action, ACTION_STRAIGHT_PROBE);
							default: status = STAT_UNSUPPORTED_GCODE;
						}
						break;
					}
					//case 40
					//case 41
					//case 42
					//case 43
					//case 49
					case 53: SET_NON_MODAL(absolute_override,true);
					case 54: SET_MODAL(MODAL_GROUP_G12,coordinate_system,G54);
					case 55: SET_MODAL(MODAL_GROUP_G12,coordinate_system,G55);
					case 56: SET_MODAL(MODAL_GROUP_G12,coordinate_system,G56);
					case 57: SET_MODAL(MODAL_GROUP_G12,coordinate_system,G57);
					case 58: SET_MODAL(MODAL_GROUP_G12,coordinate_system,G58);
					case 59: SET_MODAL(MODAL_GROUP_G12,coordinate_system,G59);
					case 60:{ 
						switch(_point(value)){
							case 0: SET_MODAL(MODAL_GROUP_G13,path_control,PATH_EXACT_PATH);
							case 1: SET_MODAL(MODAL_GROUP_G13,path_control,PATH_EXACT_STOP);
							default: status = STAT_UNSUPPORTED_GCODE;
						}
						break;
					}
					case 80: SET_MODAL(MODAL_GROUP_G1,motion_mode,MOTION_MODE_CANCEL_MOTION_MODE);
					case 81: SET_MODAL(MODAL_GROUP_G1,motion_mode,MOTION_MODE_CANNED_81);
					case 82: SET_MODAL(MODAL_GROUP_G1,motion_mode,MOTION_MODE_CANNED_82);
					case 83: SET_MODAL(MODAL_GROUP_G1,motion_mode,MOTION_MODE_CANNED_83);
					case 84: SET_MODAL(MODAL_GROUP_G1,motion_mode,MOTION_MODE_CANNED_84);
					case 85: SET_MODAL(MODAL_GROUP_G1,motion_mode,MOTION_MODE_CANNED_85);
					case 86: SET_MODAL(MODAL_GROUP_G1,motion_mode,MOTION_MODE_CANNED_86);
					case 87: SET_MODAL(MODAL_GROUP_G1,motion_mode,MOTION_MODE_CANNED_87);
					case 88: SET_MODAL(MODAL_GROUP_G1,motion_mode,MOTION_MODE_CANNED_88);
					case 89: SET_MODAL(MODAL_GROUP_G1,motion_mode,MOTION_MODE_CANNED_89);
					case 90: SET_MODAL(MODAL_GROUP_G3,distance_mode,ABSOLUTE_MODE);
					case 91: SET_MODAL(MODAL_GROUP_G3,distance_mode,INCREMENTAL_MODE);
					case 92:{
						switch(_point(value)){
							case 0: SET_MODAL(MODAL_GROUP_G0,next_action,ACTION_SET_AXIS_OFFSETS);
							case 1: SET_NON_MODAL(next_action,ACTION_RESET_AXIS_OFFSETS);
							case 2: SET_NON_MODAL(next_action,ACTION_SUSPEND_AXIS_OFFSETS);
							case 3: SET_NON_MODAL(next_action,ACTION_RESUME_AXIS_OFFSETS);
						}
						break;
					}
					case 93: SET_MODAL(MODAL_GROUP_G5,feedrate_mode,INVERSE_TIME_MODE);
					case 94: SET_MODAL(MODAL_GROUP_G5,feedrate_mode,UNITS_PER_MINUTE_MODE);
					//case 98
					//case 99
					default: status = STAT_UNSUPPORTED_GCODE;
				}
				break;
			}
			case 'M':{
				switch((uint8_t)value){
					case 0: SET_MODAL(MODAL_GROUP_M4,programflow,PROGRAM_STOP);
					case 1: SET_MODAL(MODAL_GROUP_M4,programflow,PROGRAM_STOP);
					case 2: SET_MODAL(MODAL_GROUP_M4,programflow,PROGRAM_END);
					case 3: SET_MODAL(MODAL_GROUP_M7,spindle_mode,SPINDLE_CW);
					case 4: SET_MODAL(MODAL_GROUP_M7,spindle_mode,SPINDLE_CCW);
					case 5: SET_MODAL(MODAL_GROUP_M7,spindle_mode,SPINDLE_OFF);
					case 6: SET_MODAL(MODAL_GROUP_M6,tool_change,true);
					case 7: SET_MODAL(MODAL_GROUP_M8,mist_coolant,true);
					case 8:	SET_MODAL(MODAL_GROUP_M8,flood_coolant,true);
					case 9: SET_MODAL(MODAL_GROUP_M8,coolant_off,true);
					case 30: SET_MODAL(MODAL_GROUP_M4,programflow,PROGRAM_END);
					//case 48:
					//case 49:
					default: status = STAT_UNSUPPORTED_MCODE;
				}
				break;
			}
			case 'X': SET_NON_MODAL(target[X_AXIS],value);
			case 'Y': SET_NON_MODAL(target[Y_AXIS],value);
			case 'Z': SET_NON_MODAL(target[Z_AXIS],value);
			case 'I': SET_NON_MODAL(center_offsets[0],value);
			case 'J': SET_NON_MODAL(center_offsets[1],value);
			case 'K': SET_NON_MODAL(center_offsets[2],value);
			case 'F': SET_NON_MODAL(feedrate,value);
			case 'S': SET_NON_MODAL(spindle_speed,value);
			case 'R': SET_NON_MODAL(radius,value);
			case 'P': SET_NON_MODAL(parameter,value);
			case 'T': SET_NON_MODAL(tool_select,(uint8_t)(value+0.5f));
			case 'L': break;
			default: status = STAT_UNSUPPORTED_GCODE;
				
		}
		if(status != STAT_OK) break;
	}
	if(status != STAT_OK && status != STAT_COMPLETE) return status;
	//validation of modals 
	return _execute_gcode_block();
}


//_execute_gcode_block//
//input : none
//output : state based on the execution of the execution process ^^
//fuction : invokes EXEC_FUNC and cannonical machine function to set the gcode model of the cannonical machine
// and starts the palanning of coordinated motions and dwells
//notes : 
//additions: 
//
/*
order of execution as provided by NIST RS274NGC
0.record line number //optional
1. comment (includes message).
2. set feed rate mode (G93, G94 — inverse time or per minute).
3. set feed rate (F).
//override
4. set spindle speed (S).
//override
5. select tool (T).
6. change tool (M6).
7. spindle on or off (M3, M4, M5).
8. coolant on or off (M7, M8, M9).
9. enable or disable overrides (M48, M49).
10. dwell (G4).
11. set active plane (G17, G18, G19).
12. set length units (G20, G21).
13. cutter radius compensation on or off (G40, G41, G42)
14. cutter length compensation on or off (G43, G49)
15. coordinate system selection (G54, G55, G56, G57, G58, G59, G59.1, G59.2, G59.3).
16. set path control mode (G61, G61.1, G64)
17. set distance mode (G90, G91).
18. set retract mode (G98, G99).
19. home (G28, G30) or
change coordinate system data (G10) or
set axis offsets (G92, G92.1, G92.2, G94).
20. perform motion (G0 to G3, G80 to G89), as modified (possibly) by G53.
21. stop (M0, M1, M2, M30, M60).
*/
#define EXEC_FUNC(f,v) if(((uint8_t)(cm.gf.v)) != false) { status = f(cm.gn.v);}
static stat_t _execute_gcode_block(void){
	stat_t status = STAT_OK;
	cm_set_model_linenum(cm.gn.linenum);
	EXEC_FUNC(cm_set_feed_rate_mode,feedrate_mode);
	EXEC_FUNC(cm_set_feed_rate,feedrate);
	//feed and traverse override factor 
	//spindle speed
	//spindle override enable and factor
	//tool select T
	//tool change M6
	//spindle control
	//coolant control
	//overrides enable
	if(cm.gn.next_action == ACTION_DWELL){
		//dwell execution request
	}
	EXEC_FUNC(cm_select_plane,plane_select);
	EXEC_FUNC(cm_select_unit_mode,units_mode);
	//cutter radius compensation
	//cutter length compensation
	EXEC_FUNC(cm_set_coord_system,coordinate_system);
	EXEC_FUNC(cm_select_path_control,path_control);
	EXEC_FUNC(cm_select_distance_mode,distance_mode);
	//retract mode
	switch(cm.gn.next_action){
		case ACTION_SET_COORD_DATA:{status = cm_set_coord_offsets(cm.gn.parameter,cm.gn.target,cm.gf.target); break;}
		case ACTION_SET_AXIS_OFFSETS: {status = cm_set_origin_offsets(cm.gn.target,cm.gf.target); break;}
		case ACTION_RESET_AXIS_OFFSETS: {status = cm_reset_origin_offsets(); break;}
		case ACTION_SUSPEND_AXIS_OFFSETS: {status = cm_suspend_origin_offsets(); break;}
		case ACTION_RESUME_AXIS_OFFSETS: {status = cm_resume_origin_offsets(); break;}
		case ACTION_GOTO_G28_POSITION:
		case ACTION_SET_G28_POSITION:
		case ACTION_SEARCH_HOME:	{status = cm_cycle_homing_start(); break;}
		case ACTION_SET_ABSOLUTE_ORIGIN:
		case ACTION_HOMING_NO_SET:
		case ACTION_GOTO_G30_POSITION:
		case ACTION_SET_G30_POSITION:
		case ACTION_STRAIGHT_PROBE:	{status = cm_straight_probe(cm.gn.target,cm.gf.target); break;}
		case ACTION_DEFAULT: //it's a motion
			cm_set_absolute_override(cm.gn.absolute_override);
			db_end_session(GCODE_PARSER_TIME);
			switch(cm.gn.motion_mode){
				case MOTION_MODE_CANCEL_MOTION_MODE: {cm.gm.motion_mode = cm.gn.motion_mode; break;}
				case MOTION_MODE_STRAIGHT_TRAVERSE:{status = cm_straight_traverse(cm.gn.target,cm.gf.target); break;}
				case MOTION_MODE_STRAIGHT_FEED:{status = cm_straight_feed(cm.gn.target,cm.gf.target); break;}
				case MOTION_MODE_CW_ARC : case MOTION_MODE_CCW_ARC :{
					cm_arc_feed(cm.gn.target,cm.gf.target,cm.gn.center_offsets,cm.gn.radius);
				}
			}
	}
	cm_set_absolute_override(false);
	if (cm.gf.programflow == true) {
		if (cm.gn.programflow == PROGRAM_STOP) {
			//cm_program_stop();
		} else {
			//cm_program_end();
		}
	}
	db_end_session(BLOCK_PREPARE_TIME);
	return status;
}

