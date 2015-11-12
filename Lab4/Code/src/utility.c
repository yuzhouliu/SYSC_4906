#include "utility.h"

#define SYS_FREQ 16000000UL
#define TIMER_0B_PRESCALE 255

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
  TIMER0->TAMR = 0x1UL;            //One-shot Timer mode
  TIMER0->TAPR = 255UL;         //Prescale value
	
	TIMER0->CTL |= (1UL << 1);				//TASTALL
  return;
}

/* Delay timer. Input is a multiple of 562us (1T) 
 * Reload value is (SYS_FREQ/256)*562.5us-1 = 34 for 1 period. Don't calculate for wasting processor
 */
void delay_timer_0A(uint16_t num_periods)
{
	if (num_periods > 1872)
		num_periods = 1872;
	
	TIMER0->TAILR = num_periods * 35 -1;	//Set reload value to expire in num_periods*562us
	TIMER0->CTL |= (1UL << 0);   					//Enable TIMER0A
	
	// While timer has not expired, loop
	while(TIMER0->CTL & 1UL)
	{}
	return;
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
