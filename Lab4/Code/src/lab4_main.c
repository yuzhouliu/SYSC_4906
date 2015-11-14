#include "CU_TM4C123.h"

#include "led.h"
#include "utility.h"
#include "uart.h"

static uint8_t running = 1;

#define GET_SIZE(x) (sizeof(x)/sizeof((x)[0]))
static char MSG_button_press[] = "Button Pressed\n\r";

int main(void)
{	
	LED_init();							// Initialize LEDs
	init_pushButton();			// Initialize SW1 push button
	init_timer_0B();				// Init periodic timer that interrupts every second
	
	UART_init();						// Initialize UART0 for TX/RX (uses PA0-1)	
	Buffer_Pool_TX_init();	// Initialize the TX Buffer pool (circular buffer)
	
	while(running)
	{	}	
}

/* Handler for GPIOF interrupts 
 * - Handle Push Button SW1 interrupt (PF4)
 */
void GPIOF_Handler(void)
{
	Buffer * this_buf;
	
	// If Interrupt caused by SW1 (PF4)
	if(GPIOF->RIS & (1UL << 4))
	{		
		// Entering critical section, disable interrupts
		__set_PRIMASK(1);		
		this_buf = get_buffer_TX();			// Get an available buffer
		__set_PRIMASK(0);
		// Left critical section, interrupts are enabled again
				
		if( this_buf == (void *) 0) {
			turn_on_red_LED();
			running = 0;
			return;
		}
		my_strncpy(this_buf->message, MSG_button_press, GET_SIZE(MSG_button_press));
		this_buf->count = GET_SIZE(MSG_button_press);
		// Send Message to UART
		Tx_message(this_buf);
				
		GPIOF->ICR |= (1UL << 4);	// Clear Interrupt
		NVIC->ICPR[0] = (1UL << 30); 	// Clear pending bit from NVIC
	}
}

/* Handler for TIMER0B interrupt 
 * - Acts as a clock that times out every 1 second
 */
void TIMER0B_Handler(void)
{
	Time curr_time;
	Buffer *this_buf;
			
	curr_time = increment_time();
	
	// Entering critical section, disable interrupts
	__set_PRIMASK(1);		
	this_buf = get_buffer_TX();			// Get an available buffer
	__set_PRIMASK(0);
	// Left critical section, interrupts are enabled again
	if( this_buf == (void *) 0) {
		turn_on_red_LED();
		running = 0;
		return;
	}
	my_strncpy(this_buf->message, curr_time.time_string, GET_SIZE(curr_time.time_string));
	this_buf->count = GET_SIZE(curr_time.time_string);
	// Send Message to UART
	Tx_message(this_buf);
	
	TIMER0->ICR |= (1UL << 8);		// Clear interrupt to de-assert IRQ#20 (TIMER0B) signal
	NVIC->ICPR[0] |= (1UL << 20);	//Clear pending bit in NVIC for IRQ#20 TIMER0B
	return;
}
