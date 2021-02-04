

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "HAL.h"
#include "system.h"
#include "timers.h"

/* the pins are as described here
PORTA (P0,P1 UART)  (P2,P3 coolant control) (P4-P7 Driver Enable control)
PORTB (P0,P1 USB)   (P4-P7 Driver Step control) (P2,P3 can be used for coolant control if it requires PWM)
PORTC (PC4-PC7 Driver Direction Control)
PORTD (P0,P1 Spindle control)  (P2-P5 USB) (P6,P7 can be used for optical encoder of spindle)
PORTE (P0-P2 limits)	(P3-P5 can support CAN network between machines ,, analog feed back,, I2C communication)
PORTF (P0-P2 limits/homing)	(P3 blinking led)	(P4 probe)


TIMER5 for execute and load queue
WTIMER0 for debugger
WTIMER1 for tick_clock
WTIMER5 for DDA
*/
static void stepper_init(void);
static void limit_init(void);
static void emergency_init(void);
static void execute_timer_init(void);
static void load_timer_init(void);
static void dda_timer_init(void);
static void tick_timer_init(void);


void peripherals_init(void){
	SYSCTL_RCGCGPIO_R |= 0x0000003F;
	while((SYSCTL_PRGPIO_R&0x0000003F) != 0x0000003F ){}
	GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
	GPIO_PORTF_CR_R |= 0x00000001;		//enable writting F0
	GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY;
	GPIO_PORTD_CR_R |= 0x00000080;		//enable writting D7
	SYSCTL_RCGCTIMER_R |= 0x00000020;
	while((SYSCTL_PRTIMER_R&0x00000020) != 0x00000020 ){}
	SYSCTL_RCGCWTIMER_R |= 0x00000023;
	while((SYSCTL_PRWTIMER_R&0x00000023) != 0x00000023 ){}
	emergency_init();
	limit_init();
	stepper_init();
	execute_timer_init();
	load_timer_init();
	dda_timer_init();
	tick_timer_init();
}


//stepper module
static void stepper_init(void){
	///////////////////ENABLE PINS//////////////////////////////////
	GPIO_PORTA_DR8R_R |= 0xF0;
	GPIO_PORTA_DATA_R &=~ 0x000000F0;		//make the pins low
	GPIO_PORTA_DIR_R |= 0x000000F0;			//make the pins output
	GPIO_PORTA_DEN_R |= 0x000000F0;			//make them digital
	GPIO_PORTA_AFSEL_R &=~ 0x000000F0;	//turn off alternate functions
	GPIO_PORTA_PCTL_R &=~ 0xFFFF0000;		//make then normal port pins
	GPIO_PORTA_AMSEL_R &=~ 0x000000F0;	//turn off analog
	////////////////////////////////////////////////////////////////
	////////////////////CLK PINS///////////////////////////////////
	GPIO_PORTB_DR8R_R |= 0xF0;
	GPIO_PORTB_DATA_R &=~ 0x000000F0;		//make the pins low
	GPIO_PORTB_DIR_R |= 0x000000F0;			//make the pins output
	GPIO_PORTB_DEN_R |= 0x000000F0;			//make them digital
	GPIO_PORTB_AFSEL_R &=~ 0x000000F0;	//turn off alternate functions
	GPIO_PORTB_PCTL_R &=~ 0xFFFF0000;		//make then normal port pins
	GPIO_PORTB_AMSEL_R &=~ 0x000000F0;	//turn off analog
	///////////////////////////////////////////////////////////////
	///////////////////DIR PINS////////////////////////////////////
	GPIO_PORTC_DR8R_R |= 0xF0;
	GPIO_PORTC_DATA_R &=~ 0x000000F0;		//make the pins low
	GPIO_PORTC_DIR_R |= 0x000000F0;			//make the pins output
	GPIO_PORTC_DEN_R |= 0x000000F0;			//make them digital
	GPIO_PORTC_AFSEL_R &=~ 0x000000F0;	//turn off alternate functions
	GPIO_PORTC_PCTL_R &=~ 0xFFFF0000;		//make then normal port pins
	GPIO_PORTC_AMSEL_R &=~ 0x000000F0;	//turn off analog
}

//the rest of the module are inline functions in the stepper header file
///////////////////////////////////////////////////////////////////

//limit switches module

static void limit_init(void){
	///////////////// linear limits //////////////////////////////
	GPIO_PORTE_DATA_R &=~ 0x00000007;		//clear data
	GPIO_PORTE_DIR_R &=~ 0x00000007;		//input
	GPIO_PORTE_DEN_R |= 0x00000007;			//digital
	GPIO_PORTE_AFSEL_R &=~ 0x00000007;	//non_alternate
	GPIO_PORTE_PCTL_R &=~ 0x00000FFF;		//normal port
	GPIO_PORTE_AMSEL_R &=~ 0x00000007;	//non_analog
	GPIO_PORTE_IS_R &=~ 0x00000007;		//edge triggered interrupt
	GPIO_PORTE_IBE_R |= 0x00000007;		//interrupt both edges
	GPIO_PORTE_ICR_R |= 0x00000007;		//clear interrupts
	NVIC_PRI1_R = (NVIC_PRI1_R&0xFFFFFF00)|(LIMITS_PRIORITY<<L_LIMITS_PRIORITY_BITS);		//set to priority 1
	NVIC_EN0_R |= L_LIMITS_ENABLE_BIT;
	GPIO_PORTE_IM_R |= 0x00000007;		//mask interrupt
	///////////////////////////////////////////////////////////////
	//////////////////angular limits//////////////////////////////
	GPIO_PORTF_DATA_R &=~ 0x00000007;		//clear data
	GPIO_PORTF_DIR_R &=~ 0x00000007;		//input
	GPIO_PORTF_DEN_R |= 0x00000007;			//digital
	GPIO_PORTF_AFSEL_R &=~ 0x00000007;	//non_alternate
	GPIO_PORTF_PCTL_R &=~ 0x00000FFF;		//normal port
	GPIO_PORTF_AMSEL_R &=~ 0x00000007;	//non_analog
	GPIO_PORTF_IS_R &=~ 0x00000007;		//edge triggered interrupt
	GPIO_PORTF_IBE_R |= 0x00000007;		//interrupt BOTH Edges
	//GPIO_PORTF_IEV_R &=~ 0x00000007;		//interrupt on falling edge
	GPIO_PORTF_ICR_R |= 0x00000007;		//clear interrupts
	NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|(LIMITS_PRIORITY<<A_LIMITS_PRIORITY_BITS);		//set to priority 1
	NVIC_EN0_R |= A_LIMITS_ENABLE_BIT;
	GPIO_PORTF_IM_R |= 0x00000007;		//mask interrupt
}
//the rest are inline functions in homing header file 
//////////////////////////////////////////////////////////////////

//emergency

static void emergency_init(void){
	GPIO_PORTB_DATA_R &=~ 0x00000008;		//clear data
	GPIO_PORTB_DIR_R &=~ 0x00000008;		//input
	GPIO_PORTB_PUR_R |= 0x00000008;
	GPIO_PORTB_DEN_R |= 0x00000008;			//digital
	GPIO_PORTB_AFSEL_R &=~ 0x00000008;	//non_alternate
	GPIO_PORTB_PCTL_R &=~ 0x0000F000;		//normal port
	GPIO_PORTB_AMSEL_R &=~ 0x00000008;	//non_analog
	GPIO_PORTB_IS_R &=~ 0x00000008;		//edge triggered interrupt
	GPIO_PORTB_IBE_R &=~ 0x00000008;		//interrupt selected by IEV
	GPIO_PORTB_IEV_R &=~ 0x00000008;		//interrupt on falling edge
	GPIO_PORTB_ICR_R |= 0x00000008;		//clear interrupts
	NVIC_PRI0_R = (NVIC_PRI0_R&0xFFFF00FF)|(EMERGENCY_PRIORITY);		//set to priority 1
	NVIC_EN0_R |= EMERGENCY_ENABLE_BIT;
	GPIO_PORTB_IM_R |= 0x00000008;		//mask interrupt
}
//the rest are inline functions in emergency header file
///////////////////////////////////////////////////////////////



static void execute_timer_init(void){
	TIMER5_CTL_R &=~ TIMER_CTL_TAEN;
	TIMER5_CTL_R |= TIMER_CTL_TASTALL;
	TIMER5_CFG_R = TIMER_CFG_16_BIT;
	TIMER5_TAMR_R |= TIMER_TAMR_TACDIR|TIMER_TAMR_TAMR_1_SHOT;
	TIMER5_TAILR_R = EXECUTE_TIMER_PERIOD;
	TIMER5_TAPR_R = 0;
	TIMER5_TAV_R = 0;
	TIMER5_ICR_R |= TIMER_ICR_TATOCINT;
	NVIC_PRI23_R = (NVIC_PRI23_R&0xFFFFFF00)|(EXECUTE_TIMER_PRIORITY);
	NVIC_EN2_R |= EXECUTE_ENABLE_BIT;
	TIMER5_IMR_R |= TIMER_IMR_TATOIM;
}

static void load_timer_init(void){
	TIMER5_CTL_R &=~ TIMER_CTL_TBEN;
	TIMER5_CTL_R |= TIMER_CTL_TBSTALL;
	TIMER5_TBMR_R |= TIMER_TBMR_TBCDIR|TIMER_TBMR_TBMR_1_SHOT;
	TIMER5_TBILR_R = LOAD_TIMER_PERIOD;
	TIMER5_TBPR_R = 0;
	TIMER5_TBV_R = 0;
	TIMER5_ICR_R |= TIMER_ICR_TBTOCINT;
	NVIC_PRI23_R = (NVIC_PRI23_R&0xFFFF00FF)|(LOAD_TIMER_PRIORITY);
	NVIC_EN2_R |= LOAD_ENABLE_BIT;
	TIMER5_IMR_R |= TIMER_IMR_TBTOIM;
}

static void dda_timer_init(void){
	WTIMER5_CTL_R &=~ TIMER_CTL_TAEN;
	WTIMER5_CTL_R |= TIMER_CTL_TASTALL;
	WTIMER5_CFG_R = TIMER_CFG_16_BIT;
	WTIMER5_TAMR_R |= TIMER_TAMR_TACDIR|TIMER_TAMR_TAMR_PERIOD|TIMER_TAMR_TAMIE;
	WTIMER5_TAILR_R = EXECUTE_TIMER_PERIOD;
	WTIMER5_TAPR_R = 0;
	WTIMER5_TAV_R = 0;
	WTIMER5_TAPMR_R = 0;
	WTIMER5_ICR_R |= TIMER_ICR_TATOCINT|TIMER_ICR_TAMCINT;
	NVIC_PRI26_R = (NVIC_PRI26_R&0xFFFFFF00)|(DDA_TIMER_PRIORITY);
	NVIC_EN3_R |= DDA_ENABLE_BIT;
	WTIMER5_IMR_R |= TIMER_IMR_TATOIM|TIMER_IMR_TAMIM;
}


static void tick_timer_init(void){
	WTIMER1_CTL_R &=~ TIMER_CTL_TAEN|TIMER_CTL_TBEN;
	WTIMER1_CTL_R |= TIMER_CTL_TASTALL|TIMER_CTL_TBSTALL;
	WTIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
	WTIMER1_TAMR_R |= TIMER_TAMR_TACDIR|TIMER_TAMR_TAMR_PERIOD;
	WTIMER1_TAILR_R = FCPU/1000;			//this will always give 1ms interval no matter the frequency
	WTIMER1_TAV_R = 0;
	WTIMER1_TBV_R = 0;
	WTIMER1_CTL_R |= TIMER_CTL_TAEN|TIMER_CTL_TBEN;
}

