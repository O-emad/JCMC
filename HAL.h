#ifndef HAL_H
#define HAL_H

#ifndef INLINE
#define INLINE extern inline
#endif

/* the pins are as described here
PORTA (P0,P1 UART)  (P2,P3 coolant control) (P4-P7 Driver Enable control)
PORTB (P0,P1 USB)   (P4-P7 Driver Step control) (P2,P3 can be used for coolant control if it requires PWM) (P3 emergency stop)
PORTC (PC4-PC7 Driver Direction Control)
PORTD (P0,P1 Spindle control)  (P2-P5 USB) (P6,P7 can be used for optical encoder of spindle)
PORTE (P0-P2 limits)	(P3-P5 can support CAN network between machines ,, analog feed back,, I2C communication)
PORTF (P0-P2 limits)	(P3 blinking led)	
*/

//emergency stop takes priority 0
//limit switches takes priority 1
//priority 2,3 are left for other uses
//priority 4-6 used by loader and DDA
//priority 7 left for other uses

INLINE uint32_t tick_get_count(void);

#define EMERG_PRIORITY 0UL
#define LIMITS_PRIORITY 1UL

#define L_LIMITS_IRQ 4
#define L_LIMITS_PRIORITY_BITS 5
#define L_LIMITS_ENABLE_BIT 0x00000010
#define L_LIMITS_PRIORITY (LIMITS_PRIORITY<<L_LIMITS_PRIORITY_BITS)

#define A_LIMITS_IRQ 30
#define A_LIMITS_PRIORITY_BITS 21
#define A_LIMITS_ENABLE_BIT 0x40000000
#define A_LIMITS_PRIORITY (LIMITS_PRIORITY<<A_LIMITS_PRIORITY_BITS)

#define EMERGENCY_IRQ 1
#define EMERGENCY_PRIORITY_BITS 13
#define EMERGENCY_ENABLE_BIT 0x00000002
#define EMERGENCY_PRIORITY (EMERG_PRIORITY<<EMERGENCY_PRIORITY_BITS)

void peripherals_init(void);

inline uint32_t tick_get_count(void){return WTIMER1_TBR_R;}
#endif



