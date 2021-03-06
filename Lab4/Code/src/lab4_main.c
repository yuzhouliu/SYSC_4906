#include "CU_TM4C123.h"

#include "led.h"
#include "utility.h"
#include "uart.h"

static uint8_t running = 1;

#define GET_SIZE(x) (sizeof(x)/sizeof((x)[0]))
static char MSG_button_press[] = "Button Pressed\n\r";

int main(void)
{	
	//char character;
	LED_init();							// Initialize LEDs
	init_pushButton();			// Initialize SW1 push button
	init_timer_0A();				// Init periodic timer for counting the clock
	init_timer_0B();				// Init periodic timer for generating clock msgs thru UART
	
	UART_init();						// Initialize UART0 for TX/RX (uses PA0-1)	
	Buffer_Pool_init(BUF_TX);	// Initialize the TX Buffer pool (circular buffer)
	
	while(running)
	{
		if(UART_rx_available())
		{
			Rx_message();			
		}
	}	
}

/* Handler for GPIOF interrupts 
 * - Handle Push Button SW1 interrupt (PF4)
 */
void GPIOF_Handler(void)
{
	int res;
	
	// If Interrupt caused by SW1 (PF4)
	if(GPIOF->RIS & (1UL << 4))
	{
		// Send Message to UART
		res = Tx_message(MSG_button_press, GET_SIZE(MSG_button_press));
		if (res == -1) {
			running = 0;
			turn_on_red_LED();
			return;
		}
				
		GPIOF->ICR |= (1UL << 4);	// Clear Interrupt
		NVIC->ICPR[0] = (1UL << 30); 	// Clear pending bit from NVIC
	}
}

/* Handler for TIMER0A interrupt
 * - Increments the clock count every second
 */
void TIMER0A_Handler(void)
{
	increment_time();
	
	TIMER0->ICR |= (1UL << 0);		// Clear interrupt to de-assert IRQ#19 (TIMER0A) signal
	NVIC->ICPR[0] |= (1UL << 19);	//Clear pending bit in NVIC for IRQ#19 TIMER0A
}

/* Handler for TIMER0B interrupt 
 * - Sends the clock message every set interval
 */
void TIMER0B_Handler(void)
{
	int res;
	Time curr_time;			
	curr_time = get_time();	
	
	if (!suppress_time_output)
	{	
		// Send Message to UART
		res = Tx_message(curr_time.time_string, GET_SIZE(curr_time.time_string));
		if (res == -1) {
			running = 0;
			turn_on_red_LED();
			return;
		}
	}
	
	TIMER0->ICR |= (1UL << 8);		// Clear interrupt to de-assert IRQ#20 (TIMER0B) signal
	NVIC->ICPR[0] |= (1UL << 20);	//Clear pending bit in NVIC for IRQ#20 TIMER0B
	return;
}
