#include "CU_TM4C123.h"

#include "main.h"
#include "led.h"
#include "adc_driver.h"
#include "utility.h"

#define VOLTAGE_ADC_OFFSET 1850		// Input signal at 1.5V offset, so 0V of sine wave is sampled as 1850 by ADC
#define FIX_SHIFT 8;	// Fixed point represenation 8.8: 00000000.00000000

volatile uint16_t error;
volatile uint8_t state;
signed long int weight_coeff[FILTER_LENGTH];
circular_buffer input_ref;

uint16_t mu = 1;		// Fixed point representation 8.8: 00000000.00000001 = 0.004

int main(void)
{
	set_circ_buffer_0(&input_ref);
	
	LED_init();
	PWM_init();
	init_timer_0A();	
	init_ADC();
	
	while(1) {
		switch(state) {
			case WAITING_FOR_ADC:
				break;
			case NOISE_REF_READY:				// If the noise ref is ready, output anti-noise
				output_anti_noise();
				state = WAITING_FOR_ADC;
				break;
			case ERROR_READY:						// If the error is ready, update LMS filter weights
				update_LMS_weights();
				state = WAITING_FOR_ADC;
				break;
			default:
				turn_on_red_LED();
				break;
		}	
	}	
}

/* Initialize all variables in circular buffer to 0
 */
void set_circ_buffer_0(circular_buffer *buff)
{
	int i;
	
	buff->head = 0;
	buff->tail = FILTER_LENGTH-1;
	for(i = 0; i < FILTER_LENGTH; i++) {
		buff->buffer[i] = 0;
	}
}

/* Calculates and outputs the anti-noise
 */
void output_anti_noise()
{	
	uint16_t out = VOLTAGE_ADC_OFFSET;
	uint8_t w_i, input_i;
	
	// anti-noise = Sum Of w[i]*x[i], where w is weight coeff, and x is noise reference input
	for(w_i = 0, input_i = input_ref.head; w_i < FILTER_LENGTH; w_i++, input_i+=(input_i+1)%FILTER_LENGTH) {
		out += (weight_coeff[w_i] * input_ref.buffer[input_i]) >> FIX_SHIFT;		// Convert from 00000000.00000000 -> 00000000.
	}
		
	set_voltage(out);
}

/* Updates the weights used for the LMS adaptation filter
 */
void update_LMS_weights()
{
	uint8_t w_i, input_i;
	
	// If the error is greater than 1.5V (1850 in ADC), then positive error so: w[n] = w[n] + mu*input
	if(error > VOLTAGE_ADC_OFFSET) {		
		for(w_i = 0, input_i = input_ref.head; w_i < FILTER_LENGTH; w_i++, input_i+=(input_i+1)%FILTER_LENGTH) {
			weight_coeff[w_i] += mu*input_ref.buffer[input_i];
		}
	}
	// If the error is less than 1.5V (1850 in ADC), then negative error so: w[n] = w[n] - mu*input
	else if (error < VOLTAGE_ADC_OFFSET) {
		for(w_i = 0, input_i = input_ref.head; w_i < FILTER_LENGTH; w_i++, input_i+=(input_i+1)%FILTER_LENGTH) {
			weight_coeff[w_i] -= mu*input_ref.buffer[input_i];
		}
	}
	// If no error, then don't do anything
	else {}
}
