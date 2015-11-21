#include "utility.h"

#define SYS_FREQ 16000000UL
#define TIMER_0A_PRESCALE 255
#define TIMER_0B_PRESCALE 255

uint8_t suppress_time_output;		// Flag for suppressing the time output
static Time system_time;				// Variable to store system time

void init_pushButton(void)
{
	// Enable GPIOF
  SYSCTL->RCGCGPIO |= (1UL << 5);
	// Do a dummy read to insert a few cycles after enabling the peripheral.
	if (SYSCTL->RCGCGPIO) {}
	
	/* Setting up PF4 (SW1) */
	GPIOF->DIR &= ~(1UL << 4);	// Set PF4 as input
	GPIOF->DEN |= (1UL << 4);		// Digital enable PF4
	GPIOF->IM &= ~(1UL << 4);		// Mask PF4 for now while configuring
	GPIOF->IS &= ~(1UL << 4);		// PF4 Edge sensitive
	GPIOF->IBE &= ~(1UL << 4);	// Interrupt generation controlled by IEV
	GPIOF->IEV |= (1UL << 4);		// Interrupt generated on rising edge
	GPIOF->PUR |= (1UL << 4); 		// Weak pull up resistor for PF4
	GPIOF->ICR |= (1UL << 4);		// Clear PF4 Interrupt
	GPIOF->IM |= (1UL << 4);		// Enable Interrupts for PF4
	
	pushButton_nvic_init();
}

static void pushButton_nvic_init(void)
{
	/* Init for GPIOF -> which contains PF4 (SW1)  */
	NVIC->IP[30] = (5 << 5);		//IRQ#30 for GPIOF, see ds pg104
	NVIC->ICPR[0] = (1UL << 30);	//Clear pending bit to be safe
	NVIC->ISER[0] = (1UL << 30);	//Enable interrup at NVIC
}


/* Configures TIMER0A one-shot and counts down */
void init_timer_0A(void)
{	
  SYSCTL->RCGCTIMER |= 1UL << 0;  //Enable TIMER0
	if (SYSCTL->RCGCTIMER) {}       //Dummy read to give TIMER time to respond
  TIMER0->CTL &= ~(1UL << 0);      //Disable TIMERA in TIMER0 during config
  TIMER0->CFG = 0x4UL;             //Independent 16-bit timers
  TIMER0->TAMR = 0x2UL;            //One-shot Timer mode
  TIMER0->TAPR = TIMER_0A_PRESCALE;         //Prescale value
	
	TIMER0->CTL |= (1UL << 1);				//TASTALL
	TIMER0->TAILR = SYS_FREQ/(TIMER_0A_PRESCALE+1) -1;	//Set timer reload to give interrupt every second
	TIMER0->ICR |= (1UL << 0);				// Clear TBTRIS
	TIMER0->IMR |= (1UL << 0);				// Interrupt Enabled
		
	timer_0A_nvic_init();
	TIMER0->CTL |= (1UL << 0);				// Enable TIMER0A
  return;
}

static void timer_0A_nvic_init(void)
{
	/* Init for TIMER0A */
	NVIC->IP[19] |= (2 << 5);			//IRQ#20 for TIMER0B, see ds pg104
	NVIC->ICPR[0] |= (1UL << 19);	//Clear pending bit to be safe
	NVIC->ISER[0] |= (1UL << 19);	//Enable interrup at NVIC
}

/* Configures TIMER0B periodic and counts down 
 *	Generates interrupt every 1 second
 */
void init_timer_0B(void)
{
  SYSCTL->RCGCTIMER |= 1UL << 0;  	//Enable TIMER0
  if (SYSCTL->RCGCTIMER) {}       	//Dummy read to give TIMER time to respond
  TIMER0->CTL &= ~(1UL << 8);     	//Disable TIMER0B during config
  TIMER0->CFG = 0x4UL;             	//Independent 16-bit timers
  TIMER0->TBMR = 0x2UL;            	//Periodic Timer mode
  TIMER0->TBPR = TIMER_0B_PRESCALE; //Prescale value
	
	TIMER0->CTL |= (1UL << 9);				//TBSTALL
	TIMER0->TBILR = SYS_FREQ/(TIMER_0B_PRESCALE+1) - 1;		//Set timer reload to give interrupt every second
	TIMER0->ICR |= (1UL << 8);				// Clear TBTRIS
	TIMER0->IMR |= (1UL << 8);				// Interrupt Enabled
	
	timer_0B_nvic_init();
	
  TIMER0->CTL |= (1UL << 8);   			//Enable TIMER0B
  return;
}

static void timer_0B_nvic_init(void) 
{
	/* Init for TIMER0B */
	NVIC->IP[20] |= (2 << 5);			//IRQ#20 for TIMER0B, see ds pg104
	NVIC->ICPR[0] |= (1UL << 20);	//Clear pending bit to be safe
	NVIC->ISER[0] |= (1UL << 20);	//Enable interrup at NVIC
}

/* Copies string from src to dest up to the input size
 */
void my_strncpy(char *dest, char *src, uint16_t size)
{
	uint16_t i;
	for(i = 0; i < size; i++) {
		*dest = *src;
		dest++;
		src++;
	}
}

/* Reset system time to 0
 */
void reset_time(void)
{
	system_time.hours = 0;
	system_time.minutes = 0;
	system_time.seconds = 0;
	suppress_time_output = 0;
}

/* Sets the time variable according to the input hours and minutes
 */
void set_time(char h1, char h0, char m1, char m0)
{
	system_time.hours = ((h1-'0')*10)+(h0-'0');
	system_time.minutes = ((m1-'0')*10)+(m0-'0');
	system_time.seconds = 0;
}

/* Returns the time variable
 */
Time get_time(void)
{
	return system_time;
}

/* Increments the system time
 * Returns current time
 */
Time increment_time(void)
{	
	system_time.seconds++;						// Update seconds
	if(system_time.seconds >= 60) {		// If seconds is at 60, rollover and update minutes
		system_time.seconds = 0;
		system_time.minutes++;
	}	
	if(system_time.minutes >= 60) {		// If minutes is at 60, rollover and update hours
		system_time.minutes = 0;
		system_time.hours = (system_time.hours+1)%100;
	}
	
	system_time.time_string[10] = '\r';
	system_time.time_string[9] = '\n';
	system_time.time_string[8] = system_time.seconds%10 + '0';
	system_time.time_string[7] = system_time.seconds/10 + '0';
	system_time.time_string[6] = ':';
	system_time.time_string[5] = system_time.minutes%10 + '0';
	system_time.time_string[4] = system_time.minutes/10 + '0';
	system_time.time_string[3] = ':';
	system_time.time_string[2] = system_time.hours%10 + '0';
	system_time.time_string[1] = system_time.hours/10 + '0';
	system_time.time_string[0] = '>';
	
	return system_time;
}

/* Doubles the speed at which the message generation timer times out
 */
void double_time_msg_freq(void){
	uint16_t current_reload = TIMER0->TBILR;
	
	if(current_reload >= 1) {
		TIMER0->TBILR = current_reload/2;
	}
}

/* Halves the speed at which the message generation timer times out
 */
void half_time_msg_freq(void) {
	uint16_t current_reload = TIMER0->TBILR;
	
	if(current_reload < (65534/2)) {
		TIMER0->TBILR = current_reload*2;
	}
}
