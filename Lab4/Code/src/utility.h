#include "stdint.h"
#include "CU_TM4C123.h"

/*-----------Functions for Push Button functionality-----------------------*/

/* Initializes SW1 (PF4) as interrupt enabled push button. */
void init_pushButton(void);

/* Sets the NVIC for push button interrupts */
static void pushButton_nvic_init(void);

/* Note the Handler for GPIOF pins (push buttons) has been relocated into
 * lab4_main.c. This is so that the main function can control interrupt response
 */

 
 
/*-----------Functions for timer functionality-----------------------*/
 
/* Configures TIMER0A one-shot and counts down */
void init_timer_0A(void);
	
/* Delay timer. Input is a multiple of 562us (1T) 
 * Reload value is (SYS_FREQ/256)*562.5us-1 = 34 for 1 period. Don't calculate for wasting processor
 */
void delay_timer_0A(uint16_t num_periods);
	
/* Configures TIMER0B periodic and counts down 
 *	Used for generating modulation every 562us
 */
void init_timer_0B(void);

/* Sets the NVIC for timer_0B interrupts */
static void timer_0B_nvic_init(void);
