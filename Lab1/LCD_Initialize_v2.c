//*****************************************************************************
//
// LCD_Initialize.c - Initialization Procedure for the LCD used in SYSC 4906
//
// Yuzhou Liu 100853392
// Minh Mai 100845949
//*****************************************************************************

#include "CU_TM4C123.h"

#define CONTROL_PORT GPIOA  //GPIOA used for control pins RS, R/W and E
#define DB_PORT GPIOB       //GPIOB used for data pins DB0 - DB7

#define TIMER TIMER0        //Use TIMER0 as the timer
#define MILLISEC_IN_SEC 1000
#define SYS_FREQ 16000000UL   //System frequency of uController
#define PRESCALE 255UL          //Prescale by factor of 256. Good for max timer of 1 second for 16-bit

#define READ_DELAY 1			//Delay in ms
#define WRITE_DELAY 1			//Delay in ms

#define MAX_SCROLL_DELAY 600		// Maximum delay between scrolls to give min scroll speed
#define MIN_SCROLL_DELAY 100		// Minimum delay between scrolls to give max scroll speed

#define PWM_LOAD 399
#define MAX_DUTY_CYCLE 100
#define MIN_DUTY_CYCLE 0

static uint16_t scroll_delay = 300;
static uint8_t duty_cycle = 100;

/** Function declarations START**/
void init_gpio(void);
void init_pushButton(void);
void init_timer_A(void);
void init_timer_B(void);
void nvic_init(void);

void delay_timer(uint32_t delay_time_ms);
void interrupt_timer(uint32_t delay_time_ms);
void TIMER0B_Handler(void);
void GPIOF_Handler(void);

uint8_t LCD_get_busy_flag(void);
void LCD_wait_until_ready(void);
void send_command(uint8_t command);
void send_data(uint8_t data);
void LCD_init(void);

void increment_scroll_delay(void);
void decrement_scroll_delay(void);
void increment_brightness(void);
void decrement_brightness(void);
void set_PWM_duty_cycle(uint8_t percentage);
void PWM_init(void);
/** Function declarations END **/

int main(void)
{
	char display_string1[] = "*Hello Amigos. Rolling Banner!*";
	char display_string2[] = "+-+-+-+-+";
	uint8_t i;
	
	init_gpio();			// Initialize GPIO	
  init_timer_A();		// Initialize TIMER0->TIMERA as 16-bit, one-shot and counts down
	init_timer_B();		// Init TIMER0B as 16-bit, periodic and counts down
	nvic_init();			// Init NVIC for TIMER0B
	init_pushButton();	
		
	// Initialize LCD display
	LCD_init();
	PWM_init();	

	for(i=0; display_string1[i] != '\0'; i++)
	{
		send_data(display_string1[i]);
		delay_timer(100);
		
		// Since screen has enough to display 16 chars per line, if going off line, start shifting screen.
		if ( i >= 14 )
			send_command(0x18);
	}
	send_command(0x2);		// Return cursor home
	send_command(0xC0);		// Go to line 2

	for(i=0; display_string2[i] != '\0'; i++)
	{
		send_data(display_string2[i]);
		delay_timer(100);
		
		// Since screen has enough to display 16 chars per line, if going off line, start shifting screen.
		if ( i >= 14 )
			send_command(0x18);
	}
	
	interrupt_timer(scroll_delay);		// Interrupt timer handler scrolls the screen
	//send_command(0x18);		// Shift display to the right by 1
	//send_command(0x8);		// Turn display off
	//send_command(0xF);		// Turn display on, cursor ON, cursor position ON
	//GPIOF->DATA |= (1UL << 3);    //LED ON
	//GPIOF->DATA &= ~(1UL << 3);		//LED OFF
	while(1)
	{}
}

/** Configures GPIO
*  Mapping PB0-7 as DB0-7
*  Mapping PA2: E, PA3: R/W, PA4: RS
*  Default direction is output for all pins
**/
void init_gpio(void)
{
    uint32_t dummy;

    // Enable GPIOA port (used for the control signals) and GPIOB port (used for Data Bits) and GPIOF (LED for debug and Push buttons)
    SYSCTL->RCGCGPIO |= ((1UL << 0) | (1UL << 1) | (1UL << 5) | (1UL << 4));

    // Do a dummy read to insert a few cycles after enabling the peripheral.
    dummy = SYSCTL->RCGCGPIO;

    // Enable all the GPIO pins that connect to DB0 - DB7
    DB_PORT->DIR |= 0xFF;       // Set the default direction as output for all pins in DB_PORT
    DB_PORT->DEN |= 0xFF;       // Enable all the pins in DB_PORT for digital function

    // Enable GPIO pins for the control bits RS, R/W, and E
    CONTROL_PORT->DIR |= 0x1C;  // Set the direction as output for all control pins
    CONTROL_PORT->DEN |= 0x1C;  // Enabled all the control pins
	
	// Sets GPIOF's direction for LED
	GPIOF->DIR |= (1UL << 3);	// Output for PF3 LED
	GPIOF->DEN |= (1UL << 3);	//Enabled PF3(LED)
	
	// Sets GPIOE for Digital out
	GPIOE->DIR |= (1UL << 4);
	GPIOE->DEN |= (1UL << 4);
}

/** Initializes SW1 (PF4) and SW2 (PF0) as push buttons. 
*	These are interrupt enabled
**/
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

	/* Setting up PF0 (SW2). Requires more code due to lock */
	GPIOF->IM &= ~(1UL << 0);		// Mask PF0 for now while configuring
	GPIOF->LOCK = 0x4C4F434B;		// Enables write access to GPIOCR
	*((uint32_t *) &GPIOF->CR) = 0x1F;
	GPIOF->DIR &= ~(1UL << 0);	// Set PF0 as input
	GPIOF->DEN |= (1UL << 0);		// Digital enable PF0

	GPIOF->AMSEL &= ~(1UL << 0);	
	GPIOF->PCTL &= ~(1UL << 0);
	GPIOF->AFSEL &= ~(1UL << 0);
	
	GPIOF->IS &= ~(1UL << 0);
	GPIOF->IBE &= ~(1UL << 0);
	GPIOF->IEV |= (1UL << 0);	
	GPIOF->PUR |= (1UL << 0);
	GPIOF->ICR |= (1UL << 0);
	GPIOF->IM |= (1UL << 0);
}

/** Configures TIMER0->TIMERA
*   Configures TIMERA as two 16-bit independent timer config (A & B)
*   One-shot and counts down
**/
void init_timer_A(void)
{
    uint32_t dummy;

    SYSCTL->RCGCTIMER |= 1UL << 0;  //Enable TIMER0
    dummy = SYSCTL->RCGCTIMER;      //Dummy read to give TIMER time to respond
    TIMER->CTL &= ~(1UL << 0);      //Disable TIMERA in TIMER0 during config
    TIMER->CFG = 0x4UL;             //Independent 16-bit timers
    TIMER->TAMR = 0x1UL;            //One-Shot Timer mode
    TIMER->TAPR = PRESCALE;         //Prescale value
	
	TIMER->CTL |= (1UL << 1);				//TASTALL
    return;
}

/** Configures TIMER0->TIMERB
*   Configures TIMERB as two 16-bit independent timer config (A & B)
*   Periodic and counts down
**/
void init_timer_B(void)
{
    uint32_t dummy;

    SYSCTL->RCGCTIMER |= 1UL << 0;  //Enable TIMER0
    dummy = SYSCTL->RCGCTIMER;      //Dummy read to give TIMER time to respond
    TIMER->CTL &= ~(8UL << 0);      //Disable TIMERB in TIMER0 during config
    TIMER->TBMR = 0x2UL;            //Periodic Timer mode
    TIMER->TBPR = PRESCALE;         //Prescale value
	TIMER->ICR = (1UL << 8);		// Clear TBTRIS
	TIMER->IMR = (1UL << 8);		// Interrupt enabled for TIMER0B
	
	TIMER->CTL |= (1UL << 9);		//TBSTALL
    return;
}

/** Inits NVIC for TIMER0B (IRQ#20)
*  Priority 4, enable interrupt for TIMER0B
**/
void nvic_init(void)
{
	/* Init for TIMER0B */
	NVIC->IP[20] = (4 << 5);	//IRQ#20 for TIMER0B, see ds pg104
	NVIC->ICPR[0] = (1UL << 20);	//Clear pending bit to be safe
	NVIC->ISER[0] = (1UL << 20);	//Enable interrup at NVIC
	
	/* Init for PF4 (SW1) and PF0 (SW2) */
	NVIC->IP[30] = (5 << 5);	//IRQ#30 for GPIOF, see ds pg104
	NVIC->ICPR[0] = (1UL << 30);	//Clear pending bit to be safe
	NVIC->ISER[0] = (1UL << 30);	//Enable interrup at NVIC
}

/** Sets TIMERA as delay timer that delays for the amount in ms specified in input
*   Returns only when timer has expired
**/
void delay_timer(uint32_t delay_time_ms)
{
	uint32_t timer_reload_val;
	uint32_t prescale_output;
	
    //The reload value for the timer is the delay period we would like divided by
    //prescaled clock period
	prescale_output = SYS_FREQ/(PRESCALE+1);
	// VERY IMPORTANT: delay_time_ms/MILLISEC_IN_SEC will give 0, THE ORDER MATTERS HERE.
  timer_reload_val = (delay_time_ms*prescale_output)/MILLISEC_IN_SEC;

    TIMER->TAILR = timer_reload_val;
    TIMER->CTL |= (1UL << 0);   //Enable TIMER0A

    //While timer has not expired yet, Loop forever
    while( TIMER->CTL & 1UL )
    {
    }    
    return;
}

/** Sets TIMERB as timer that causes interrupt when input time has expired
*
**/
void interrupt_timer(uint32_t delay_time_ms)
{
	uint32_t timer_reload_val;
	uint32_t prescale_output;
		
  //The reload value for the timer is the delay period we would like divided by
  //prescaled clock period
	prescale_output = SYS_FREQ/(PRESCALE+1);
	// VERY IMPORTANT: delay_time_ms/MILLISEC_IN_SEC will give 0, THE ORDER MATTERS HERE.
  timer_reload_val = (delay_time_ms*prescale_output)/MILLISEC_IN_SEC -1;

	TIMER->CTL &= ~(1UL << 8);		//Disabled TIMER0B
  TIMER->TBILR = timer_reload_val;
  TIMER->CTL |= (1UL << 8);   //Enable TIMER0B
    
  return;
}

/** Handler for TIMER0B interrupt
**/
void TIMER0B_Handler(void)
{
	send_command(0x18);
	TIMER->ICR = (1UL << 8);		// Clear interrupt at GPTM to de-assert IRQ#20 signal
	NVIC->ICPR[0] = (1UL << 20);	//Clear pending bit in NVIC for IRQ#20 TIMER0B
	return;
}

/** Handler for GPIOF pins (push buttons)
* 	Increment/decrement scroll speed
*		Increment/decrement backlight brightness
**/
void GPIOF_Handler(void)
{
	// If Interrupt caused by PF4
	if(GPIOF->RIS & (1UL << 4))
	{
		GPIOF->DATA ^= (1UL << 3);	// Toggle LED for debugging
		increment_scroll_delay();
		decrement_brightness();		
		GPIOF->ICR |= (1UL << 4);	// Clear Interrupt
	}
	// Else if Interrupt caused by PF0
	else if (GPIOF->RIS & (1UL << 0))
	{
		GPIOF->DATA ^= (1UL << 3);	// Toggle LED for debugging
		decrement_scroll_delay();
		increment_brightness();
		GPIOF->ICR |= (1UL << 0);	// Clear Interrupt
	}
	interrupt_timer(scroll_delay);
	set_PWM_duty_cycle(duty_cycle);
}

/** Check to see if Busy Flag (BF) is HIGH
/   To read BF, set RS=Low, and R/W=High. Read through DB7
**/
uint8_t LCD_get_busy_flag(void)
{
    uint8_t bf_read;

    DB_PORT->DIR &= ~(1UL << 7);        // Set the direction as input (0) for DB7
    CONTROL_PORT->DATA &= ~(1UL << 4);  // Set RS (PA4) low
    CONTROL_PORT->DATA |= (1UL << 3);   // Set R/W (PA3) High
    CONTROL_PORT->DATA |= (1UL << 2);   // Start Read

    delay_timer(READ_DELAY);
    bf_read = (DB_PORT->DATA & (1UL << 7)) >> 7;  // Read Busy Flag through DB7

    return bf_read;
}

/** Waits for LCD_busy_flag if it is set
**/
void LCD_wait_until_ready(void)
{
    //Empty loop to wait for LCD Busy Flag to go low
    while(LCD_get_busy_flag())
    {
    }
}

/** Send Commands to the MPU of LCD
/   Input: hex value of bits for DB0-7
**/
void send_command(uint8_t command)
{
	LCD_wait_until_ready();				// Wait until LCD busy flag is off
    DB_PORT->DIR |= 0xFF;               //Set direction as output
    DB_PORT->DATA = command;	        // Put Instructions on output port
    CONTROL_PORT->DATA &= ~(1UL << 4);  // Set RS (PA4) low
    CONTROL_PORT->DATA &= ~(1UL << 3);  // Set RW (PA3) low
    CONTROL_PORT->DATA |= (1UL << 2);   // Set E (PA2) high
    delay_timer(WRITE_DELAY);

    CONTROL_PORT->DATA &= ~(1UL << 2);  // Set E low
    return;
}

/** Write Data to the MPU of LCD
/	Input: hex value of bits for data
**/
void send_data(uint8_t data)
{
	LCD_wait_until_ready();	// Wait until LCD busy flag is off
	DB_PORT->DIR |= 0xFF;	//Set direction as output
	DB_PORT->DATA = data;	//Put data on output port
	CONTROL_PORT->DATA |= (1UL << 4);	// Set RS (PA4) high
	CONTROL_PORT->DATA &= ~(1UL << 3);	// Set RW (PA3) low
	CONTROL_PORT->DATA |= (1UL << 2);	// Set E (PA2) high
	delay_timer(WRITE_DELAY);

	CONTROL_PORT->DATA &= ~(1UL << 2);  // Set E low
    return;
}

/** Initialize the LCD
**/
void LCD_init(void)
{
	CONTROL_PORT->DATA &= ~(1UL << 2);   // E = 0
	delay_timer(15);			// Wait > 15ms after power is applied
	send_command(0x30);			// Wake up		
	delay_timer(4);				// Wait >4.1ms
	send_command(0x30);			// Wake up
	delay_timer(1);				// Wait 1ms
	send_command(0x30);			// Wake up	
	delay_timer(1);				// Wait 1ms
	
	send_command(0x38);			//Interface data 8 bits, 2 lines
	send_command(0x10);			//Do not set cursor moving or shift display
	send_command(0x06);			//Cursor move right, no shift
	send_command(0x1);			// Clear the display
	send_command(0xF);			// Display on, cursor ON, cursor position ON
}

void increment_scroll_delay(void)
{
	scroll_delay += 100;
	if( scroll_delay > MAX_SCROLL_DELAY )
	{
		scroll_delay = MAX_SCROLL_DELAY;
	}
}

void decrement_scroll_delay(void)
{
	scroll_delay -= 100;
	if( scroll_delay < MIN_SCROLL_DELAY )
	{
		scroll_delay = MIN_SCROLL_DELAY;
	}		
}

void decrement_brightness(void)
{
	if (duty_cycle >= (MIN_DUTY_CYCLE+20))
	{
		duty_cycle -= 20;
	}
}

void increment_brightness(void)
{	
	if (duty_cycle <= (MAX_DUTY_CYCLE-20)) 
	{
		duty_cycle += 20;
	}
}

/** Sets the PWM CMPA in order to get the appropriate duty cycle
**/
void set_PWM_duty_cycle(uint8_t percentage)
{
	uint16_t ticks_cmpa;
	
	ticks_cmpa = ((100 - percentage)*PWM_LOAD)/100;
	if (ticks_cmpa == PWM_LOAD)
	{
		ticks_cmpa--;
	}
	PWM0->_2_CMPA = ticks_cmpa;
}

/** Initializes PWM signal on PE4 in order to control backlight of LCD
**/
void PWM_init(void)
{
	SYSCTL->RCGC0 |= (1UL << 20);
	//SYSCTL->RCGC2 |= (1UL << 4);
	
	GPIOE->AFSEL |= (1UL << 4);			// Alternate function
	GPIOE->PCTL |= (0x4 << 16);			// Select M0PWM4
	SYSCTL->RCC |= (0x0 << 17);			// Divide system clock by 2 for PWM (8MHz)
	
	// Configure PWM Module 0, Generator 2, Generator A: PWM0->_2_...
	PWM0->_2_CTL = 0x0;					// No global sync
	PWM0->_2_GENA = 0x8C;				// Drive PWM high when counter matches LOAD, drive PWM low when counter matches CMPA
	PWM0->_2_LOAD = PWM_LOAD;		//Set the load to be 399 (400)
	set_PWM_duty_cycle(100);		// Set 100% duty cycle
	PWM0->_2_CTL = 0x1;					// Enable PWM Module 0, Generator 2
	PWM0->ENABLE |= (1UL << 4);		// Enabled			
}
