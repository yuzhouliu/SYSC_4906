#include "CU_TM4C123.h"

// General defines
#define SYS_FREQ 16000000UL

// For Timer0B 
#define PRESCALE 255UL
#define MILLISEC_IN_SEC 1000

// For ADC
#define ANALOG1 (1UL << 2) // PE2 <-> AIN1
#define ANALOG2 (1UL << 3) // PE3 <-> AIN0
#define SAMPLING_RATE 100 //Sampling rate of 100ms

// LDR used for PE2 freq (darker)
#define MIN_LUX 190
#define MAX_LUX 450

// LDR used for PE3 volume, (lighter)
// This is because LDR is different. If the same then don't need this
#define MIN_LUX_2 1050
#define MAX_LUX_2 3000

// For PWM
static uint16_t pwm_load = 0;
static uint32_t frequency = 800;
static uint16_t volume = 90;

// Happy Birthday notes
static uint32_t notes[] = {261, 293, 329, 349, 392, 440, 494, 523, 0};
static uint32_t hbd_notes[] = {1,1,2,1,4,3,9,1,1,2,1,5,4,9,1,1,8,6,4,3,2,9,8,8,6,4,5,4};

static uint8_t numberOfElements = sizeof(hbd_notes)/sizeof(hbd_notes[0]);
static uint8_t playing_hbd = 0;
static uint16_t pitch_adjust = 0;

// Miscellaneous functions
void turn_on_green_LED(void);
void turn_off_green_LED(void);
void turn_on_red_LED(void);
void turn_off_red_LED(void);
void init_gpio(void);
void init_nvic(void);
void init_timer_0A(void);
//void delay_timer(uint32_t delay_time_ms);
void init_timer_0B(void);
void interrupt_timer_0B(uint32_t delay_time_ms);
void TIMER0B_Handler(void);

// ADC functions
void init_ADC(void);
void ADC0SS3_Handler(void);
void ADC1SS3_Handler(void);
																	
//PWM functions
void set_frequency(uint16_t frequency);
void set_volume(uint8_t volume);
void PWM_init(void);
																	
//Innovation play Happy Birthday Song
void play_hbd(void);
void init_timer_1B(void);
void interrupt_timer_1B(uint32_t delay_time_ms);
void TIMER1B_Handler(void);
void init_pushButton(void);
void GPIOF_Handler(void);

int main(void)
{
	init_gpio();

	/* Setup Timers */
	init_timer_0A();
	init_timer_0B();
	
	// Default to theremin on startup
	turn_on_red_LED();

	PWM_init();
	init_ADC();
	init_nvic();

	init_pushButton();
	
	while(1)
	{}
}

void turn_on_green_LED(void)
{
	GPIOF->DATA |= (1UL << 3);
}

void turn_off_green_LED(void)
{
	GPIOF->DATA &= ~(1UL << 3);
}

void turn_on_red_LED(void)
{
	GPIOF->DATA |= (1UL << 1);
}

void turn_off_red_LED(void)
{
	GPIOF->DATA &= ~(1UL << 1);
}

void init_gpio(void)
{
  uint32_t dummy;

  // Enable GPIOE (Speaker) and GPIOF (LED for debug)
  SYSCTL->RCGCGPIO |= ((1UL << 4) | (1UL << 5));

  // Do a dummy read to insert a few cycles after enabling the peripheral.
  dummy = SYSCTL->RCGCGPIO;

	// Sets PE4 for Digital out
	GPIOE->DIR |= (1UL << 4);
	GPIOE->DEN |= (1UL << 4);
	
	// Sets GPIOF's direction for LED
	GPIOF->DIR |= ((1UL << 3) | (1UL << 1));	// Output for PF3 LED
	GPIOF->DEN |= ((1UL << 3) | (1UL << 1));	//Enabled PF3(LED)
	
	//Enable Analog functions (disabled digital)
	//GPIOE->AFSEL |= (ANALOG1 | ANALOG2);
	GPIOE->DIR &= ~(ANALOG1|ANALOG2);	//Input
	GPIOE->DEN &= ~(ANALOG1|ANALOG2);	//Digital disable
	//GPIOE->AMSEL |= (ANALOG1|ANALOG2);	// Select Analog In
	GPIOE->PUR |= (ANALOG1|ANALOG2);	// Pull up resistor
}

/** * Initialize the ADC **/
void init_ADC(void)
{
	uint32_t dummy;
	/* Configure ADC0, SS03 */
	SYSCTL->RCGCADC |= (1UL<<0); //Enable clock to ADC0
	
	//Wait until ADC0 ready
	dummy = SYSCTL->RCGCADC;
	dummy = SYSCTL->RCGCADC;
	
	GPIOE->AFSEL |= (ANALOG1 | ANALOG2);
	GPIOE->AMSEL |= (ANALOG1 | ANALOG2);
	
	ADC0->ACTSS &= ~(1UL<<3);	//Disable SS03 while programming
	
	ADC0->EMUX |= (0x5<<12);	//SS03 on timer
	ADC0->SSMUX3 = (0x1<<0); 	//SS03 sample AIN1	
	ADC0->SSCTL3 = ((1UL<<2) | (1UL << 1));  //SS03 trigger interrupt
	
	ADC0->RIS &= ~(1UL<<3); 	//Clear interrupt		
	ADC0->IM |= (1UL<<3);		//Arm interrupt at SS03		
	ADC0->ISC &= ~(1UL<<3);		//Clear Level-senstive interrupt
		
	ADC0->ACTSS |= (1UL<<3);	//Enable SS03

	/* Configure ADC1, SS03 */
	SYSCTL->RCGCADC |= (1UL << 1); 	//Enable clock to ADC1
	dummy = SYSCTL->RCGCADC;
	dummy = SYSCTL->RCGCADC;
	
	ADC1->ACTSS &= ~(1UL << 3);	//Disable SS03 while programming
	ADC1->EMUX |= (0x5<<12);	//SS03 on timer
	ADC1->SSMUX3 = 0x0; 		//SS03 sample AIN0
	ADC1->SSCTL3 = ((1UL<<2) | (1UL << 1));  //SS03 trigger interrupt
	
	ADC1->RIS &= ~(1UL<<3); 	//Clear interrupt		
	ADC1->IM |= (1UL<<3);		//Arm interrupt at SS03		
	ADC1->ISC &= ~(1UL<<3);		//Clear Level-senstive interrupt
	ADC1->ACTSS |= (1UL<<3);	//Enable SS03
	
}

/** Interrupt handler for SS03
* 	Mapped to PE2
**/
void ADC0SS3_Handler(void)
{
	uint16_t lux1;	
	lux1 = ADC0->SSFIFO3;	//read lux value of SS03
	
	if (playing_hbd)
	{
		if(lux1 < MIN_LUX)
			pitch_adjust = 1;
		else if (190 <= lux1 && lux1 < 300)
			pitch_adjust = 2;
		else
			pitch_adjust = 3;
	}
	else 
	{
		if(lux1 < MIN_LUX)
			frequency = (MIN_LUX*MIN_LUX)/100;
		else if (lux1 > MAX_LUX)
			frequency = (MAX_LUX*MAX_LUX)/100;
		else 
			frequency = (lux1*lux1)/100;
	}
	
	set_frequency(frequency);
	set_volume(volume);
	
	ADC0->ISC |= (1UL << 3);
	NVIC->ICPR[0] = (1UL << 17);	//Clear pending bit in NVIC for IRQ#17 ADC0
}

// Mapped to PE3
void ADC1SS3_Handler(void)
{
	uint16_t lux2 = ADC1->SSFIFO3;
	
	if(lux2 < MIN_LUX_2)
		volume = 1;
	else if ( MIN_LUX_2 <= lux2 && lux2 < 1200)
		volume = 5;
	else if ( 1200 <= lux2 && lux2 < 1500)
		volume = 10;
	else if ( 1500 <= lux2 && lux2 < 2500)
		volume = 25;
	else
		volume = 50;
	
	set_volume(volume);
	ADC1->ISC |= (1UL << 3);
	NVIC->ICPR[1] = (1UL << 19);	//Clear pending bit in NVIC for IRQ#51 ADC1
}

/** Inits NVIC for TIMER0B (IRQ#20) and ADC (IRQ#17) **/
void init_nvic(void)
{
	// init for ADC0 SS03 
	NVIC->IP[17] = (2<<5); // SS03 has Priority 2
	NVIC->ICPR[0] = (1UL<<17);// Clear pending bit to be safe
	NVIC->ISER[0] = (1UL<<17);// Enable interrupt at NVIC
	
	// init for ADC1 SS03
	NVIC->IP[51] = (2<<5); // SS03 has Priority 2
	NVIC->ICPR[1] = (1UL<<19);// Clear pending bit to be safe
	NVIC->ISER[1] = (1UL<<19);// Enable interrupt at NVIC
	
	/* Init for TIMER0B */
	NVIC->IP[20] = (1 << 5);	//IRQ#20 for TIMER0B priority 1, see ds pg104
	NVIC->ICPR[0] = (1UL << 20);	//Clear pending bit to be safe
	NVIC->ISER[0] = (1UL << 20);	//Enable interrup at NVIC
	
	/* Init for TIMER1B */
	NVIC->IP[22] = (1 << 5);	//IRQ#22 for TIMER1B priority 1, see ds pg104
	NVIC->ICPR[0] = (1UL << 22);	//Clear pending bit to be safe
	NVIC->ISER[0] = (1UL << 22);	//Enable interrup at NVIC
	
	/* Init for PF4 (SW1) */
	NVIC->IP[30] = (5 << 5);	//IRQ#30 for GPIOF, see ds pg104
	NVIC->ICPR[0] = (1UL << 30);	//Clear pending bit to be safe
	NVIC->ISER[0] = (1UL << 30);	//Enable interrup at NVIC
}

/** Configures TIMER0A periodic and counts down 
*		Used for triggering ADC
**/
void init_timer_0A(void)
{
    uint32_t dummy;

    SYSCTL->RCGCTIMER |= 1UL << 0;  //Enable TIMER0
    dummy = SYSCTL->RCGCTIMER;      //Dummy read to give TIMER time to respond
    TIMER0->CTL &= ~(1UL << 0);      //Disable TIMERA in TIMER0 during config
    TIMER0->CFG = 0x4UL;             //Independent 16-bit timers
    TIMER0->TAMR = 0x2UL;            //Periodic Timer mode
    TIMER0->TAPR = PRESCALE;         //Prescale value
	
	TIMER0->CTL |= (1UL << 1);				//TASTALL
	TIMER0->CTL |= (1UL << 5);		// Allow timer to trigger ADC
	
	  TIMER0->CTL &= ~(1UL << 0);	//Disable timer while configuring
	  TIMER0->TAILR = ((SYS_FREQ/PRESCALE+1)*SAMPLING_RATE)/MILLISEC_IN_SEC;		//Set timer reload to be 
    TIMER0->CTL |= (1UL << 0);   //Enable TIMER0A
    return;
}

/** Sets TIMERA as delay timer that delays for the amount in ms specified in input
*   Returns only when timer has expired
**/
/*
void delay_timer(uint32_t delay_time_ms)
{
	uint32_t timer_reload_val;
	uint32_t prescale_output;
	
    //The reload value for the timer is the delay period we would like divided by
    //prescaled clock period
	prescale_output = SYS_FREQ/(PRESCALE+1);
	// VERY IMPORTANT: delay_time_ms/MILLISEC_IN_SEC will give 0, THE ORDER MATTERS HERE.
  timer_reload_val = (delay_time_ms*prescale_output)/MILLISEC_IN_SEC;

    TIMER0->TAILR = timer_reload_val;
    TIMER0->CTL |= (1UL << 0);   //Enable TIMER0A

    //While timer has not expired yet, Loop forever
    while( TIMER0->CTL & 1UL )
    {
    }    
    return;
}
*/

/** Configures TIMER0B as periodic count down **/
void init_timer_0B(void)
{
  uint32_t dummy;

  SYSCTL->RCGCTIMER |= 1UL << 0;  //Enable TIMER0
  dummy = SYSCTL->RCGCTIMER;      //Dummy read to give TIMER time to respond
  TIMER0->CTL &= ~(8UL << 0);      //Disable TIMERB in TIMER0 during config
	TIMER0->CFG = 0x4UL;             //Independent 16-bit timers
  TIMER0->TBMR = 0x2UL;            //Periodic Timer mode
  TIMER0->TBPR = PRESCALE;         //Prescale value
	TIMER0->ICR = (1UL << 8);		// Clear TBTRIS
	TIMER0->IMR = (1UL << 8);		// Interrupt enabled for TIMER0B
	
	TIMER0->CTL |= (1UL << 9);		//TBSTALL
  return;
}

/** Sets TIMER0B as periodic timer. Interrupt on every expiration **/
void interrupt_timer_0B(uint32_t delay_time_ms)
{
	uint32_t timer_reload_val;
	uint32_t prescale_output;
	
	init_timer_0B();
	
  //The reload value for the timer is the delay period we would like divided by
  //prescaled clock period
	prescale_output = SYS_FREQ/(PRESCALE+1);
	// VERY IMPORTANT: delay_time_ms/MILLISEC_IN_SEC will give 0, THE ORDER MATTERS HERE.
  timer_reload_val = (delay_time_ms*prescale_output)/MILLISEC_IN_SEC -1;

	TIMER0->CTL &= ~(1UL << 8);		//Disable TIMER0B
  TIMER0->TBILR = timer_reload_val;
  TIMER0->CTL |= (1UL << 8);   //Enable TIMER0B
    
  return;
}

/** Handler for TIMER0B interrupt **/
// Currently not used for anything
void TIMER0B_Handler(void)
{
	return;
}

/** Sets the frequency output of PWM signal **/
void set_frequency(uint16_t frequency)
{
	pwm_load = (SYS_FREQ/frequency)/64 - 1;
	PWM0->_2_LOAD = pwm_load;
}

/** Sets the PWM CMPA in order to get the appropriate duty cycle **/
void set_volume(uint8_t volume)
{
	uint16_t ticks_cmpa;	
	ticks_cmpa = (volume*pwm_load)/100;
	
	if (ticks_cmpa == pwm_load)
		ticks_cmpa--;
	
	PWM0->_2_CMPA = ticks_cmpa;
}

/** Initializes PWM signal on PE4 for output to speaker **/
void PWM_init(void)
{
	SYSCTL->RCGC0 |= (1UL << 20);
	//SYSCTL->RCGC2 |= (1UL << 4);	// Cannot configure this AND RCGCGPIO
	
	GPIOE->AFSEL |= (1UL << 4);		// Alternate function
	GPIOE->PCTL |= (0x4 << 16);		// Select M0PWM4

	SYSCTL->RCC |= (1 << 20);		// Use PWM divider
	SYSCTL->RCC |= (0x7 << 17);		// Divider set to divide by 64
	PWM0->_2_CTL = 0x0UL;			// Immediate update to parameters
	PWM0->_2_GENA = 0x8CUL;			// Drive PWM high when counter matches LOAD, drive low when matches CMPA
	set_frequency(frequency);
	set_volume(volume);
	PWM0->_2_CTL = 0x1UL;			// enabled PWM module 0, generator 2
	PWM0->ENABLE |= (1UL << 4);		// enable PWM module 0

}

/** Plays happy birthday song **/
void play_hbd(void)
{	
	playing_hbd = 1;
	interrupt_timer_1B(500);

	return;
}

/** Configures TIMER1B as periodic count down **/
void init_timer_1B(void)
{
  uint32_t dummy;

  SYSCTL->RCGCTIMER |= 1UL << 1;  //Enable TIMER1
  dummy = SYSCTL->RCGCTIMER;      //Dummy read to give TIMER time to respond
  TIMER1->CTL &= ~(8UL << 0);      //Disable TIMERB in TIMER1 during config
	TIMER1->CFG = 0x4UL;             //Independent 16-bit timers
  TIMER1->TBMR = 0x2UL;            //Periodic Timer mode
  TIMER1->TBPR = PRESCALE;         //Prescale value
	TIMER1->ICR = (1UL << 8);		// Clear TBTRIS
	TIMER1->IMR = (1UL << 8);		// Interrupt enabled for TIMER1B
	
	TIMER1->CTL |= (1UL << 9);		//TBSTALL
  return;
}

/** Sets TIMER0B as periodic timer. Interrupt on every expiration **/
void interrupt_timer_1B(uint32_t delay_time_ms)
{
	uint32_t timer_reload_val;
	uint32_t prescale_output;
	
	init_timer_1B();
	
  //The reload value for the timer is the delay period we would like divided by
  //prescaled clock period
	prescale_output = SYS_FREQ/(PRESCALE+1);
	// VERY IMPORTANT: delay_time_ms/MILLISEC_IN_SEC will give 0, THE ORDER MATTERS HERE.
  timer_reload_val = (delay_time_ms*prescale_output)/MILLISEC_IN_SEC -1;

	TIMER1->CTL &= ~(1UL << 8);		//Disable TIMER1B
	TIMER1->TBILR = timer_reload_val;
  TIMER1->CTL |= (1UL << 8);   //Enable TIMER1B
    
  return;
}

/** Handler for TIMER1B interrupt 
*	Used for playing hbd song
**/
void TIMER1B_Handler(void)
{
	uint32_t note;
	static uint32_t current_note = 0;
	
	// If we stopped wanting to play hbd, then disable TIMER1B and reset current note to 0
	if (!playing_hbd)
	{
		TIMER1->CTL &= ~(1UL << 8);     //Disable TIMER1B if not playing hbd
		TIMER1->ICR = (1UL << 8);		// Clear interrupt at GPTM to de-assert IRQ#22 signal
		NVIC->ICPR[0] = (1UL << 22);	//Clear pending bit in NVIC for IRQ#22 TIMER1B
		current_note = 0;
		return;
	}
	
	// If happy birthday has not finished playing, advance to next note
	if (current_note < numberOfElements)
	{
		note = hbd_notes[current_note++]-1;
		frequency = notes[note]*pitch_adjust;
	}
	else
	{
		// turn off for one, then restart the song
		frequency = 0;
		current_note = 0;
	}
	
	set_frequency(frequency);
	set_volume(volume);
	TIMER1->ICR = (1UL << 8);		// Clear interrupt at GPTM to de-assert IRQ#22 signal
	NVIC->ICPR[0] = (1UL << 22);	//Clear pending bit in NVIC for IRQ#22 TIMER1B
}

void init_pushButton(void)
{
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
}

void GPIOF_Handler(void)
{
	// If Interrupt caused by PF4
	if(GPIOF->RIS & (1UL << 4))
	{
		//playing_hbd = !playing_hbd;
		if(playing_hbd)
		{
			playing_hbd = 0;
			turn_off_green_LED();
			turn_on_red_LED();
		}
		else
		{
			playing_hbd = 1;			
			play_hbd();
			turn_on_green_LED();
			turn_off_red_LED();
		}
		
		GPIOF->ICR |= (1UL << 4);	// Clear Interrupt
	}
}
