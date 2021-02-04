
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "systick.h"
#include "controller.h"
#include "system.h"
#include "canonical.h"
#include "gcode_parser.h"
#include "planner.h"
#include "loader.h"
#include "HAL.h"
#include "stepper.h"
#include "encoder.h"
#include "switch.h"
#include "debugging.h"

uint32_t value;
//char string[]= "n0001 m7 m30 m5 m6 g17 g21 g60.1 g54 g90 g94 g 001 x000.200023 y 003000.2000012 z 00030.00300232 R 200 i 30.4334 j 323 k 3432 f 1400 s2400 p 500 t 6";
/*char string[][100] = 
{
	"n0005 g19 g21g54g60g80g90g94",											
	"n010 g10 l2p1x10y10z10",
	"n15 g10 l2p2 x-5 y-5.5 z-1.123",
	"n020 G92 x7 y8.5 z-1.5",
	"n25 G0 x10 y10 z10",
	"n0030 G53 x0y0z0",
	"n035 g92.2 ",
	"n036 x10 y10",
	"n037g92.3",
	"n40 g55  x5 y5 z5",
	"n041g92.1",
	"n045 x3 y25 z-1.5"
};*/

void FPU_Enable(void){
	NVIC_CPAC_R |= 0x00F00000 ;
}

void SystemInit(){
	FPU_Enable();
}


int main(){

	PLL_Init();
	SysTick_Init();
	peripherals_init();
	sw_init();
	db_init();
	mp_init_buffers();
	canonical_init();
	cm.a[X_AXIS].max_feedrate = 5000.0f;
	cm.a[Y_AXIS].max_feedrate = 5000.0f;
	cm.a[Z_AXIS].max_feedrate = 5000.0f;
	cm.a[X_AXIS].max_velocity = 8000.0f;
	cm.a[Y_AXIS].max_velocity = 8000.0f;
	cm.a[Z_AXIS].max_velocity = 8000.0f;
	mm.jerk = 540;
	mm.jerk_cbrt = cbrtf(mm.jerk);
	cm.a[X_AXIS].max_jerk = 340.0f;
	cm.a[Y_AXIS].max_jerk = 340.0f;
	cm.a[Z_AXIS].max_jerk = 340.0f;
	cm.a[X_AXIS].jerk_recip = 1/((cm.a[X_AXIS].max_jerk)*1000000);
	cm.a[Y_AXIS].jerk_recip = 1/((cm.a[Y_AXIS].max_jerk)*1000000);
	cm.a[Z_AXIS].jerk_recip = 1/((cm.a[Z_AXIS].max_jerk)*1000000);
	cm.a[X_AXIS].junction_dev = 0.05f;
	cm.a[Y_AXIS].junction_dev = 0.05f;
	cm.a[Z_AXIS].junction_dev = 0.05f;
	cm.junction_acceleration = 20000.0f;
	st_cfg.mot[MOTOR_1].step_per_unit = 40;
	st_cfg.mot[MOTOR_2].step_per_unit = 40;
	st_cfg.mot[MOTOR_3].step_per_unit = 40;
	st_cfg.mot[MOTOR_1].motor_map = X_AXIS;
	st_cfg.mot[MOTOR_2].motor_map = Y_AXIS;
	st_cfg.mot[MOTOR_3].motor_map = Z_AXIS;
	st_cfg.mot[MOTOR_1].dir_bit = 0x00000010;
	st_cfg.mot[MOTOR_2].dir_bit = 0x00000020;
	st_cfg.mot[MOTOR_3].dir_bit = 0x00000040;
	st_cfg.mot[MOTOR_1].enable_bit = 0x00000010;
	st_cfg.mot[MOTOR_2].enable_bit = 0x00000020;
	st_cfg.mot[MOTOR_3].enable_bit = 0x00000040;
	st_cfg.mot[MOTOR_1].step_bit = 0x00000010;
	st_cfg.mot[MOTOR_2].step_bit = 0x00000020;
	st_cfg.mot[MOTOR_3].step_bit = 0x00000040;
	cm.chordal_tolerance = CHORDAL_TOLERANCE;
	cm.arc_segment_len = ARC_SEGMENT_LENGTH;
	ld_init();
	encoder_init();
	//start_micro();
	//db_start_session(BLOCK_PREPARE_TIME);
	//db_end_session(BLOCK_PREPARE_TIME);
	//value = 0x00fffffe - current();
	controller_run();

}


