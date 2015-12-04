#ifndef UTILITY_H
#define UTILITY_H

#include "CU_TM4C123.h"
#include "stdint.h"

/* Configures TIMER0A periodic and counts down 
 *		Used for triggering ADC
 */
void init_timer_0A(void);

/* Sets the frequency output of PWM signal 
 */
void set_frequency(uint16_t frequency);

/* Sets the PWM CMPA in order to get the appropriate duty cycle 
 * Input: ADC voltage: int between 0 and 4096, 0V -> 3.3V
 */
void set_voltage(uint16_t adc_voltage);

static void PWM_GPIO_init(void);

/* Initializes PWM signal on PE4 for output to speaker 
 */
void PWM_init(void);

#endif /* UTILITY_H */
