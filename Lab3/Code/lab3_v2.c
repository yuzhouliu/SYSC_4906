/** Cloning an IR emitter and receiver
*	PE4 is TX, PB4 is RX
*	
*	The protocol used is NEC extended IR
*	http://techdocs.altium.com/display/FPGA/NEC+Infrared+Transmission+Protocol
*	https://developer.mbed.org/users/shintamainjp/notebook/remote_ir_en/
*
*	T = 562us
* Leader: 16T(high)+8T(low) with data. 16T+4T without data
*	16 bit address. 8 bit data, 8 bit logical inverse data.
* Data bit: 1T+1T (bit 0). 1T+3T (bit 1)
*	1T as stop bit
* 
**/
#include "CU_TM4C123.h"

// General defines
#define SYS_FREQ 16000000UL
#define PRESCALE 255UL

// Globals, flags set using interrupts
volatile static uint8_t mod_period_flag = 0;
volatile static uint8_t rx_interrupt_flag = 0;

// Address is constant for all commands
uint8_t IR_address[16] = {0, 0, 0, 0, 0, 0, 0, 0,
														1, 1, 1, 1, 0, 1, 1, 1};
uint8_t IR_data_ON[8] = {1, 1, 0, 0, 0, 0, 0, 0};
uint8_t IR_data_OFF[8] = {0, 1, 0, 0, 0, 0, 0, 0};
uint8_t IR_data_RED[8] = {0, 0, 1, 0, 0, 0, 0, 0};
uint8_t IR_data_GREEN[8] = {1, 0, 1, 0, 0, 0, 0, 0};
uint8_t IR_data_BLUE[8] = {0, 1, 1, 0, 0, 0, 0, 0};

/** Function defines **/
// Misc functions
void toggle_green_LED(void);
void turn_on_green_LED(void);
void turn_off_green_LED(void);
void toggle_blue_LED(void);
void turn_on_blue_LED(void);
void turn_off_blue_LED(void);
void toggle_red_LED(void);
void turn_on_red_LED(void);
void turn_off_red_LED(void);
// General purpose functions
void GPIO_init(void);
void nvic_init(void);
// Timer functions
void init_timer_0A(void);
void delay_timer_0A(uint16_t num_periods);
void init_timer_0B(void);
void TIMER0B_Handler(void);
void init_timer_1A(void);
// IR transmitter functions
void turn_PWM_on(void);
void turn_PWM_off(void);
void tx_1_bit(uint16_t *pulse_count);
void tx_0_bit(uint16_t *pulse_count);
void stop_bit(uint8_t *current_state, uint16_t *pulse_count);
void tx_state_machine(uint8_t *current_state, uint16_t *pulse_count, uint8_t *IR_address, uint8_t *IR_data);
void PWM_init(void);
// IR receiver functions
void init_IR_rx(void);
void GPIOB_Handler(void);
void rx_handler(void);
void postProcess(uint32_t *edge_timeStamp);
void compareData(uint8_t *data);

int main(void)
{	
	uint8_t current_state = 0;				// Used to store the current state of TX state machine
	uint16_t pulse_count = 0;					// Used to store the current number of pulses
	
	GPIO_init();	
	init_timer_0A();
	init_timer_0B();
	nvic_init();	
	PWM_init();
	
	// IR Rx GPIO port (PB4) for asynch receive
	init_IR_rx();
	
	// Delay for microcontroller clock to stabilize
	delay_timer_0A(500);
	current_state = 1;  // Activate the tx_state_machine
	
	while(1)
	{		
		// If the 562us timer expired then it is time to update state machine
		/*
		if (mod_period_flag) {
			mod_period_flag = 0;
			tx_state_machine(&current_state, &pulse_count, IR_address,  IR_data_GREEN);	  //<--- TX CHANGE HERE!!				
		}
		*/
		
		// If RX port senses an edge, store the timer count
		if (rx_interrupt_flag) {
			rx_interrupt_flag = 0;
			rx_handler();			
		}		
	}
}

void toggle_green_LED(void)
{ GPIOF->DATA ^= (1UL << 3); }

void turn_on_green_LED(void)
{	GPIOF->DATA |= (1UL << 3); }

void turn_off_green_LED(void)
{	GPIOF->DATA &= ~(1UL << 3); }

void toggle_blue_LED(void)
{ GPIOF->DATA ^= (1UL << 2); }

void turn_on_blue_LED(void)
{	GPIOF->DATA |= (1UL << 2); }

void turn_off_blue_LED(void)
{	GPIOF->DATA &= ~(1UL << 2); }

void toggle_red_LED(void)
{ GPIOF->DATA ^= (1UL << 1); }

void turn_on_red_LED(void)
{	GPIOF->DATA |= (1UL << 1); }

void turn_off_red_LED(void)
{	GPIOF->DATA &= ~(1UL << 1); }

/** Init GPIO ports **/
void GPIO_init(void)
{
	uint32_t dummy;
	
	// Enable GPIOB (IR Rx), GPIOE (IR Tx) and GPIOF (LED for debug)
	SYSCTL->RCGCGPIO |= ((1UL << 1) | (1UL << 4) | (1UL << 5));
	// Do a dummy read to insert a few cycles after enabling the peripheral.
  dummy = SYSCTL->RCGCGPIO;
	
	// Sets GPIOF's direction for LED
	GPIOF->DIR |= ((1UL << 3) | (1UL << 2) | (1UL << 1));	//PF3/PF2/PF1 Green/Blue/Red LED Output
	GPIOF->DEN |= ((1UL << 3) | (1UL << 2) | (1UL << 1));	//Enabled PF3/PF2/PF1 Green/Blue/Red LED
		
	// Sets GPIOE for Digital out
	GPIOE->DIR |= (1UL << 4);
	GPIOE->DEN |= (1UL << 4);
}

/** Inits NVIC for 
*  TIMER0A (IRQ#19) Priority 2  (taken out since TIMER0A now delay timer)
*  TIMER0B (IRQ#20) Priority 2
*  GPIOB Priority 2
**/
void nvic_init(void)
{
	
	/* Init for TIMER0A */
	/*
	NVIC->IP[19] = (2 << 5);	//IRQ#19 for TIMER0B, see ds pg104 and slide3 p68
	NVIC->ICPR[0] = (1UL << 19);	//Clear pending bit to be safe
	NVIC->ISER[0] = (1UL << 19);	//Enable interrup at NVIC
	*/
	
	/* Init for TIMER0B */
	NVIC->IP[20] |= (2 << 5);	//IRQ#20 for TIMER0B, see ds pg104
	NVIC->ICPR[0] |= (1UL << 20);	//Clear pending bit to be safe
	NVIC->ISER[0] |= (1UL << 20);	//Enable interrup at NVIC

	/* Init for GPIOB (IR Rx) */
	NVIC->IP[1] |= (2 << 5);		//IRQ#1 for GPIOB, see ds pg104
	NVIC->ICPR[0] |= (1UL << 1);	//Clear pending bit to be safe
	NVIC->ISER[0] |= (1UL << 1);	//Enable interrupt at NVIC	
}

/** Configures TIMER0A one-shot and counts down **/
void init_timer_0A(void)
{	
  uint32_t dummy;

  SYSCTL->RCGCTIMER |= 1UL << 0;  //Enable TIMER0
  dummy = SYSCTL->RCGCTIMER;      //Dummy read to give TIMER time to respond
  TIMER0->CTL &= ~(1UL << 0);      //Disable TIMERA in TIMER0 during config
  TIMER0->CFG = 0x4UL;             //Independent 16-bit timers
  TIMER0->TAMR = 0x1UL;            //One-shot Timer mode
  TIMER0->TAPR = 255UL;         //Prescale value
	
	TIMER0->CTL |= (1UL << 1);				//TASTALL
  return;
}

/** Delay timer. Input is a multiple of 562us (1T) **/
// Reload value is (SYS_FREQ/256)*562.5us-1 = 34 for 1 period. Don't calculate for wasting processor
void delay_timer_0A(uint16_t num_periods)
{
	if (num_periods > 1872)
		num_periods = 1872;
	
	TIMER0->TAILR = num_periods * 35 -1;	//Set reload value to expire in num_periods*562us
	TIMER0->CTL |= (1UL << 0);   //Enable TIMER0A
	
	// While timer has not expired, loop
	while(TIMER0->CTL & 1UL)
	{}
	return;
}

/** Configures TIMER0B periodic and counts down 
*		Used for generating modulation every 562us
**/
void init_timer_0B(void)
{	
  uint32_t dummy;
	
	// Commented out due to done in init_timer_0A
  SYSCTL->RCGCTIMER |= 1UL << 0;  //Enable TIMER0
  dummy = SYSCTL->RCGCTIMER;      //Dummy read to give TIMER time to respond
  TIMER0->CTL &= ~(1UL << 8);      //Disable TIMER0B during config
  TIMER0->CFG = 0x4UL;             //Independent 16-bit timers
  TIMER0->TBMR = 0x2UL;            //Periodic Timer mode
  TIMER0->TBPR = PRESCALE;         //Prescale value
	
	TIMER0->CTL |= (1UL << 9);				//TBSTALL
	TIMER0->TBILR = ((SYS_FREQ/(PRESCALE+1))*562)/1000000 - 1;		//Set timer reload to be 34, to obtain interrupt every 560us
	dummy = ((SYS_FREQ/(PRESCALE+1))*562)/1000000 - 1;	
	TIMER0->ICR |= (1UL << 8);		// Clear TBTRIS
	TIMER0->IMR |= (1UL << 8);			// Interrupt Enabled
	
  TIMER0->CTL |= (1UL << 8);   //Enable TIMER0B
  return;
}

/** Handler for TIMER0B interrupt. Used for modulation **/
void TIMER0B_Handler(void)
{
	mod_period_flag = 1;
	
	TIMER0->ICR |= (1UL << 8);		// Clear interrupt to de-assert IRQ#20 (TIMER0B) signal
	NVIC->ICPR[0] |= (1UL << 20);	//Clear pending bit in NVIC for IRQ#20 TIMER0B
	return;
}

/* Init timer 1A used for receiver */
void init_timer_1A(void)
{	
  uint32_t dummy;

  SYSCTL->RCGCTIMER |= 1UL << 1;  //Enable TIMER1
  dummy = SYSCTL->RCGCTIMER;      //Dummy read to give TIMER time to respond
  TIMER1->CTL &= ~(1UL << 0);      //Disable TIMERA in TIMER1 during config
  TIMER1->CFG = 0x0UL;             //32-bit timer
  TIMER1->TAMR = 0x1UL;            //One-shot Timer mode
  TIMER1->TAMR |= (1UL << 4);			// Timer counts up
  TIMER1->TAPR = 0;         //Prescale value (no prescale, count up prescale is extension)
	
	TIMER1->CTL |= (1UL << 1);				//TASTALL
	TIMER1->CTL |= (1UL << 0);   //Enable TIMER1A
  return;
}

/***********************************  BEGIN IR TX FUNCTIONS ************************************************/

// State machine to generate the TX signals
void tx_state_machine(uint8_t *current_state, uint16_t *pulse_count, uint8_t *IR_address, uint8_t *IR_data)
{
	//  STATE Definitions:
	// 	0 for inactive state
	// 	1 for header 9ms HIGH (16T)
	//	2 for header 4.5ms LOW (8T)
	//	3 for 16-bit address 
	//	4 for 8-bit command
	//	5 for 8-bit logical inverse of command
	//	6 for final 562us burst to signal end
	
	// If the current mode is active (send command)
		switch(*current_state)
		{
			// In the header 9ms HIGH
			case 1:
				if (*pulse_count == 0)
					turn_PWM_on();
				else if (*pulse_count == 15)
				{ (*current_state)++; *pulse_count = 0; break; }
				(*pulse_count)++;
				break;
			// In the header 4.5ms LOW
			case 2:
				if (*pulse_count == 0)
					turn_PWM_off();
				else if (*pulse_count == 7)
				{	(*current_state)++; *pulse_count = 0; break; }
				(*pulse_count)++;
				break;
			// In the 16-bit address
			case 3:				
				if (IR_address[*pulse_count])
					tx_1_bit(pulse_count);
				else
					tx_0_bit(pulse_count);
				if (*pulse_count == 16)
				{ (*current_state)++; *pulse_count = 0; }
				break;
			// In the 8-bit command
			case 4:
				if (IR_data[*pulse_count])
					tx_1_bit(pulse_count);
				else
					tx_0_bit(pulse_count);
				if (*pulse_count == 8)
				{	(*current_state)++; *pulse_count = 0; }
				break;
			// In the logical inverse of 8-bit command
			case 5:
				if (IR_data[*pulse_count])
					tx_0_bit(pulse_count);
				else
					tx_1_bit(pulse_count);
				if (*pulse_count == 8)
				{	(*current_state)++; *pulse_count = 0; }
				break;
			// The final stop bit
			case 6:
				stop_bit(current_state, pulse_count);
			  break;
			default:
				turn_PWM_off();
		}
}

void turn_PWM_on(void)
{	PWM0->ENABLE |= (1UL << 4);	}

void turn_PWM_off(void)
{	PWM0->ENABLE &= ~(1UL << 4); }

// 1T high + 3T low
void tx_1_bit(uint16_t *pulse_count)
{
	static uint8_t bit_count = 0;
	
	if (bit_count == 0)
		turn_PWM_on();
	else if (bit_count == 1)
		turn_PWM_off();
	else if (bit_count == 3) {
		(*pulse_count)++;
		bit_count = 0;
		return;
	}
	bit_count++;
}

// 1T high + 1T low
void tx_0_bit(uint16_t *pulse_count)
{	
	static uint8_t bit_count = 0;

	if (!bit_count) {
		turn_PWM_on();
		bit_count++;
	}
	else {
		turn_PWM_off();
		(*pulse_count)++;
		bit_count = 0;
	}
	return;
}

// 1T high
void stop_bit(uint8_t *current_state, uint16_t *pulse_count)
{	
	static uint8_t bit_count = 0;
	
	if (bit_count == 0)
		turn_PWM_on();
	else {
		turn_PWM_off();
		(*current_state) = 0;
		(*pulse_count) = 0;
		bit_count = 0;
		return;
	}
	bit_count++;
}

/** Initializes PWM signal on PE4 in order to generate 38kHz carrier
**/
void PWM_init(void)
{
	SYSCTL->RCGC0 |= (1UL << 20);
	//SYSCTL->RCGC2 |= (1UL << 4);
	
	GPIOE->AFSEL |= (1UL << 4);			// Alternate function
	GPIOE->PCTL |= (0x4 << 16);			// Select M0PWM4
	SYSCTL->RCC &= ~(0xF << 17);
	SYSCTL->RCC |= (1 << 20);		// Use PWM divider
	SYSCTL->RCC |= (0x0 << 17);			// Divide system clock by 2 for PWM (8MHz)
	
	// Configure PWM Module 0, Generator 2, Generator A: PWM0->_2_...
	PWM0->_2_CTL = 0x0;					// Immediate update to parameters
	PWM0->_2_GENA = 0x8C;				// Drive PWM high when counter matches LOAD, drive PWM low when counter matches CMPA
	PWM0->_2_LOAD = 210;		//Set the load to be 208 (209) to obtain 38.277 kHz (38.222 optimal)
	PWM0->_2_CMPA = 107;		// Set 50% duty cycle
	PWM0->_2_CTL = 0x1;					// Enable PWM Module 0, Generator 2
	PWM0->ENABLE &= ~(1UL << 4);	//Disabled		
}

/******************************** END IR TX FUNCTIONS *********************************************/

/******************************** BEGIN IR RX FUNCTIONS *******************************************/

/** Init RX port (PB4) **/
void init_IR_rx(void)
{
	GPIOB->DIR &= ~(1UL << 4);	// Set PB4 as output
	GPIOB->DEN |= (1UL << 4);	// Digital enabled PB4
	GPIOB->IM &= ~(1UL << 4);	// Mask interrupts from PB4 for now while configuring
	GPIOB->IS &= ~(1UL << 4);	// PB4 edge sensitive
	GPIOB->IBE |= (1UL << 4);	// Interrupt gen on both edges
	//GPIOB->IEV &= ~(1UL << 4);	// Interrupt gen on falling edge
	//GPIOB->PUR |= (1UL << 4);	// weak pull up resistor, not needed?
	GPIOB->ICR |= (1UL << 4);	// Clear PB4 interrupts
	GPIOB->IM |= (1UL << 4);	// Enable interrupts for PB4
}

/** Handler for GPIOB pins (IR Rx)
 *  This raises flag to set rx_state_machine in motion
**/
void GPIOB_Handler(void)
{
	// If Interrupt caused by PB4
	if(GPIOB->RIS & (1UL << 4))
	{
		rx_interrupt_flag = 1;
		GPIOB->ICR |= (1UL << 4);	// Clear Interrupt PB4
		NVIC->ICPR[0] |= (1UL << 1);	//Clear pending bit in NVIC for IRQ#1 GPIOB
	}
}

/** Handles the receiver by storing received bits and post processing them **/
void rx_handler()
{	
	static uint32_t edge_timeStamp[67] = {0};
	static int rx_current_state = -1;
	
	// If this is the first interrupt indicating signal start. Init the timer
	// for subsequent interrupts
	if (rx_current_state == -1) {
		init_timer_1A();
		rx_current_state++;
		return;
	}
	
	// Read the amount of time taken between edges. Then set timer back to 0
	edge_timeStamp[rx_current_state] = TIMER1->TAR;
	TIMER1->TAV = 0;
	
	// Determine if valid header or not
	if (rx_current_state == 0) {
		edge_timeStamp[0] = (625 * edge_timeStamp[0])/10000;
		
		// If the 1st 16T long header is not between 8.8ms  9.2ms, then invalid
		if(!(8800 < edge_timeStamp[0] && edge_timeStamp[0] < 9200))	{	
			rx_current_state = -1;
			return;
		}
	}
	else if (rx_current_state == 1) {
		edge_timeStamp[1] = (625 * edge_timeStamp[1])/10000;
		
		// If the 2nd 8T long header is not between 4.3ms and 4.7ms, then invalid or empty
		if(!(4300 < edge_timeStamp[1] && edge_timeStamp[1] < 4700)) {
			rx_current_state = -1;
			return;
		}
	}
	
	rx_current_state++;
	
	// If we've reached the stop bit, then do post processing. Then reset state back to inactive
	if (rx_current_state == 68)
	{
		postProcess(edge_timeStamp);
		
	
		//New
		delay_timer_0A(300);
		rx_interrupt_flag = 0;
		rx_current_state = -1;
		return;
	}	
}

/*  Performs post processing of the collected timeStamp values 
 *  This function converts the timer count into time in micro-seconds
 *  Then compares to see if 1 or 0 bit
 */
void postProcess(uint32_t *edge_timeStamp)
{
	uint8_t address[16] = {0};
	uint8_t data[8] = {0};
	uint16_t i, temp_i;
	
	for (i = 2; i <= 66; i++) {
		// convert to time in microseconds
		edge_timeStamp[i] = (625 * edge_timeStamp[i])/10000;
	}
		
	// Get the address bit (0 or 1?)
	for (i = 3, temp_i = 0; i <= 33; i+=2, temp_i++) {
		// If time spent on second period is 1T, then it is a 0 bit. If 3T, then it is 1 bit
		if ( 450 < edge_timeStamp[i] && edge_timeStamp[i] < 650 )
			address[temp_i] = 0;
		else if ( 1500 < edge_timeStamp[i] && edge_timeStamp[i] < 1700 )
			address[temp_i] = 1;
		else
			address[temp_i] = 2;
	}
	
	// Get the data bit (0 or 1?)
	for (i = 35, temp_i = 0; i <= 65; i+=2, temp_i++) {
		// If time spent on second period is 1T, then it is a 0 bit. If 3T, then it is 1 bit
		if ( 450 < edge_timeStamp[i] && edge_timeStamp[i] < 650 )
			data[temp_i] = 0;
		else if ( 1500 < edge_timeStamp[i] && edge_timeStamp[i] < 1700 )
			data[temp_i] = 1;
		else
			data[temp_i] = 2;
	}
	
	compareData(data);	
}

/** Compares the data to match the command **/
void compareData(uint8_t *data)
{
	uint8_t data_to_val = 0;
	uint8_t on_to_val = 0, off_to_val = 0, red_to_val = 0, blue_to_val = 0, green_to_val = 0;
	uint8_t i;
	
	// Convert the array of bits into a single value to compare
	for(i = 0; i <= 7; i++) {
		data_to_val |= (data[i] << i);
		on_to_val |= (IR_data_ON[i] << i);
		off_to_val |= (IR_data_OFF[i] << i);
		red_to_val |= (IR_data_RED[i] << i);
		blue_to_val |= (IR_data_BLUE[i] << i);
		green_to_val |= (IR_data_GREEN[i] << i);
	}
	
	// Compare the data against the expected commands. Performs required actions
	if (data_to_val == on_to_val) {
		turn_on_green_LED();
		turn_on_blue_LED();
		turn_on_red_LED();
	}
	else if (data_to_val == off_to_val) {
		turn_off_green_LED();
		turn_off_blue_LED();
		turn_off_red_LED();
	}
	else if (data_to_val == red_to_val) {
		turn_off_green_LED();
		turn_off_blue_LED();
		turn_on_red_LED();
	}
	else if (data_to_val == blue_to_val) {
		turn_off_green_LED();
		turn_on_blue_LED();
		turn_off_red_LED();
	}
	else if (data_to_val == green_to_val) {
		turn_on_green_LED();
		turn_off_blue_LED();
		turn_off_red_LED();
	}
}
