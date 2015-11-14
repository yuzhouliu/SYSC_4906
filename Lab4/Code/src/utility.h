#ifndef _UTILITY_H
#define _UTILITY_H

#include "stdint.h"
#include "CU_TM4C123.h"

typedef struct{
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	char time_string[11];
}Time;

static Time system_time;

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


/*----------Functions for string manipulation-----------------------*/
/* Copies string from src to dest up to the input size
 */
void my_strncpy(char *dest, char *src, uint16_t size);

/*----------Functions for time -------------------------------------*/

/* Reset system time to 0
 */
void reset_time(void);

/* Increments the system time
 * Returns current time
 */
Time increment_time(void);

#endif /* _UTILITY_H */
