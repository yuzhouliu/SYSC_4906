/** Cloning an IR emitter and receiver
*	PA2 is Tx
*	
*	The protocol used is NEC IR
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

/** Function defines **/
// Misc functions
void toggle_green_LED(void);
void turn_on_green_LED(void);
void turn_off_green_LED(void);
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
// IR functions
void turn_PWM_on(void);
void turn_PWM_off(void);
void tx_1_bit(void);
void tx_0_bit(void);
void stop_bit(void);
//void tx_state_machine(uint8_t* IR_address, uint8_t* IR_data, uint8_t* IR_data_inv);
void tx_IR(uint8_t* IR_address, uint8_t* IR_data);
void PWM_init(void);

int main(void)
{
	// Address is constant for all commands
	uint8_t IR_address[16] = {1, 1, 1, 1, 1, 1, 1, 1,
														0, 0, 0, 0, 1, 0, 0, 0};	
	uint8_t IR_data_ON[8] = {0, 0, 1, 1, 1, 1, 1, 1};
	uint8_t IR_data_RED[8] = {1, 1, 0, 1, 1, 1, 1, 1};
		
	GPIO_init();	
	init_timer_0A();
	nvic_init();	
	PWM_init();
	
	// Turn LED ON
	tx_IR(IR_address, IR_data_ON);
	delay_timer_0A(1872);			// Delay for ~1s
	
	// Red LED
	tx_IR(IR_address, IR_data_RED);
	delay_timer_0A(1872);			// Delay for ~1s
	while(1)
	{}
}

void toggle_green_LED(void)
{ GPIOF->DATA ^= (1UL << 3); }

void turn_on_green_LED(void)
{	GPIOF->DATA |= (1UL << 3); }

void turn_off_green_LED(void)
{	GPIOF->DATA &= ~(1UL << 3); }

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
	
	// Enable GPIOE (IR Tx) and GPIOF (LED for debug)
	SYSCTL->RCGCGPIO |= ((1UL << 4) | (1UL << 5));
	// Do a dummy read to insert a few cycles after enabling the peripheral.
  dummy = SYSCTL->RCGCGPIO;
	
	// Sets GPIOF's direction for LED
	GPIOF->DIR |= ((1UL << 3) | (1UL << 1));	//PF3/PF1 Green/Red LED Output
	GPIOF->DEN |= ((1UL << 3) | (1UL << 1));	//Enabled PF3/PF1 Green/Red LED
		
	// Sets GPIOE for Digital out
	GPIOE->DIR |= (1UL << 4);
	GPIOE->DEN |= (1UL << 4);
}

/** Inits NVIC for 
*  TIMER0A (IRQ#19) Priority 2
*	 TIMER0B (IRQ#20) Priority 2
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
	NVIC->IP[20] = (2 << 5);	//IRQ#20 for TIMER0B, see ds pg104
	NVIC->ICPR[0] = (1UL << 20);	//Clear pending bit to be safe
	NVIC->ISER[0] = (1UL << 20);	//Enable interrup at NVIC
	
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
	TIMER0->TBILR = ((SYS_FREQ/PRESCALE)*38000)/1000000 - 1;		//Set timer reload to be 34, to obtain interrupt every 560us
	dummy = ((SYS_FREQ/PRESCALE)*38000)/1000000 - 1;	
	TIMER0->ICR |= (1UL << 8);		// Clear TBTRIS
	TIMER0->IMR |= (1UL << 8);			// Interrupt Enabled
	
  //TIMER0->CTL |= (1UL << 8);   //Enable TIMER0B
  return;
}

/** Handler for TIMER0B interrupt. Used for modulation **/
void TIMER0B_Handler(void)
{
	TIMER0->ICR = (1UL << 8);		// Clear interrupt to de-assert IRQ#19 (TIMER0A) signal
	NVIC->ICPR[0] = (1UL << 20);	//Clear pending bit in NVIC for IRQ#19 TIMER0A
	return;
}

/* Don't use this for now. If need to convert to interrupts later, then can use this 
void tx_state_machine(uint8_t* IR_address, uint8_t* IR_address_inv, uint8_t* IR_data, uint8_t* IR_data_inv)
{
	// 0 for header 9ms HIGH (16T)
	//	1 for header 4.5ms LOW (8T)
	//	2 for 8-bit address 
	//	3 for 8-bit logical inverse of address
	//	4 for 8-bit command
	//	5 for 8-bit logical inverse of command
	//	6 for final 562us burst to signal end

	uint8_t current_state = 0;
	uint16_t pulse_count = 0;
	
	while(current_state <=6)
	{
		switch(current_state)
		{
			// In the header 9ms HIGH
			case 0:
				turn_PWM_on();
				delay_timer_0A(16);
				current_state++;
				break;
			// In the header 4.5ms LOW
			case 1:
				turn_PWM_off();
				delay_timer_0A(8);
				current_state++;
				break;
			// In the 8-bit address
			case 2:
				if (IR_address[pulse_count])
					tx_1_bit();
				else
					tx_0_bit();
				if (pulse_count >= 7)
				{ current_state++; pulse_count = 0; }
				else
				{	pulse_count++; }
				break;
			default:
				turn_PWM_off();
		}
	}
}
*/

void tx_IR(uint8_t* IR_address, uint8_t* IR_data)
{
	uint16_t i;
	uint8_t IR_data_inv[8];
	
	// Construct the inverse IR_data
	for(i = 0; i < 8; i++)
	{ IR_data_inv[i] = !(IR_data[i]); }
	
	// In the header 9ms HIGH
	turn_PWM_on();
	delay_timer_0A(16);
	
	// In the header 4.5ms LOW
	turn_PWM_off();
	delay_timer_0A(8);
	
	// In the 16-bit address
	for(i = 0; i < 16; i++) {
		if (IR_address[i])
			tx_1_bit();
		else
			tx_0_bit();
	}
	
	// In the 8-bit data command
	for(i = 0; i < 8; i++) {
		if (IR_data[i])
			tx_1_bit();
		else
			tx_0_bit();
	}
	
	// In the 8-bit data inverse command
	for(i = 0; i < 8; i++) {
		if (IR_data_inv[i])
			tx_1_bit();
		else
			tx_0_bit();
	}
	
	stop_bit();	
}

void turn_PWM_on(void)
{	PWM0->ENABLE |= (1UL << 4);	}

void turn_PWM_off(void)
{	PWM0->ENABLE &= ~(1UL << 4); }

void tx_1_bit(void)
{
	turn_PWM_on(); 
	delay_timer_0A(1); 
	turn_PWM_off(); 
	delay_timer_0A(1);
}

void tx_0_bit(void)
{
	turn_PWM_on();
	delay_timer_0A(1);
	turn_PWM_off();
	delay_timer_0A(3);
}

void stop_bit(void)
{
	turn_PWM_on();
	delay_timer_0A(1);
	turn_PWM_off();
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
	PWM0->_2_LOAD = 209;		//Set the load to be 208 (209) to obtain 38.277 kHz (38.222 optimal)
	PWM0->_2_CMPA = 104;		// Set 50% duty cycle
	PWM0->_2_CTL = 0x1;					// Enable PWM Module 0, Generator 2
	PWM0->ENABLE &= ~(1UL << 4);	//Disabled		
}
