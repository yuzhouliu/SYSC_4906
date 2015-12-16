#include "utility.h"

#define SYS_FREQ 16000000UL
#define PRESCALE 255UL

#define SAMPLING_RATE 1952		//Sampling rate of 488Hz

static uint32_t frequency = 62500;
static uint16_t pwm_load = 0;

/* Configures TIMER0A periodic and counts down 
 *		Used for triggering ADC
 */
void init_timer_0A(void)
{
  SYSCTL->RCGCTIMER |= 1UL << 0;  //Enable TIMER0
  if(SYSCTL->RCGCTIMER) {}      //Dummy read to give TIMER time to respond
  TIMER0->CTL &= ~(1UL << 0);      //Disable TIMERA in TIMER0 during config
  TIMER0->CFG = 0x4UL;             //Independent 16-bit timers
  TIMER0->TAMR = 0x2UL;            //Periodic Timer mode
  TIMER0->TAPR = PRESCALE;         //Prescale value
	
	TIMER0->CTL |= (1UL << 1);				//TASTALL
	TIMER0->CTL |= (1UL << 5);		// Allow timer to trigger ADC
	
	TIMER0->CTL &= ~(1UL << 0);	//Disable timer while configuring
	TIMER0->TAILR = SYS_FREQ/((PRESCALE+1)*SAMPLING_RATE)-1;		//Set timer reload to be 
  TIMER0->CTL |= (1UL << 0);   //Enable TIMER0A
  return;
}

/* Sets the frequency output of PWM signal 
 */
void set_frequency(uint16_t frequency)
{
	pwm_load = (SYS_FREQ/frequency) - 1;
	PWM0->_2_LOAD = pwm_load;
}

/* Sets the PWM CMPA in order to get the appropriate duty cycle 
 * Input: ADC voltage: int between 0 and 4096, 0V -> 3.3V
 */
void set_voltage(uint16_t adc_voltage)
{
	/*
	uint16_t ticks_cmpa;
	ticks_cmpa = adc_voltage/16;
	//ticks_cmpa = (4096-adc_voltage)/16;
	
	if (ticks_cmpa == pwm_load)
		ticks_cmpa--;
	*/
	//PWM0->_2_CMPA = ticks_cmpa;
	PWM0->_2_CMPA = adc_voltage>>4;			// adc_voltage/16
}

static void PWM_GPIO_init(void)
{
	// Enable GPIOE (PE4)
  SYSCTL->RCGCGPIO |= (1UL << 4);

  // Do a dummy read to insert a few cycles after enabling the peripheral.
  if(SYSCTL->RCGCGPIO) {}

	// Sets PE4 for Digital out
	GPIOE->DIR |= (1UL << 4);
	GPIOE->DEN |= (1UL << 4);
}

/* Initializes PWM signal on PE4 for output to speaker 
 */
void PWM_init(void)
{
	PWM_GPIO_init();
	
	SYSCTL->RCGC0 |= (1UL << 20);
	//SYSCTL->RCGC2 |= (1UL << 4);	// Cannot configure this AND RCGCGPIO
	
	GPIOE->AFSEL |= (1UL << 4);		// Alternate function
	GPIOE->PCTL |= (0x4 << 16);		// Select M0PWM4

	SYSCTL->RCC &= ~(1 << 20);		// Do not use PWM divider (16 MHz clk)
	//SYSCTL->RCC |= (0x7 << 17);		// Divider set to divide by 64
	PWM0->_2_CTL = 0x0UL;			// Immediate update to parameters
	PWM0->_2_GENA = 0x8CUL;			// Drive PWM high when counter matches LOAD, drive low when matches CMPA
	set_frequency(frequency);
	set_voltage(5000);
	PWM0->_2_CTL = 0x1UL;			// enabled PWM module 0, generator 2
	PWM0->ENABLE |= (1UL << 4);		// enable PWM module 0
}
