#include "CU_TM4C123.h"

#include "led.h"
#include "utility.h"
#include "uart.h"

int main(void)
{
	// Test
	Buffer *this_buf, *good_buf;
	char char_msg = '0';
	
	LED_init();
	init_pushButton();
	
	UART_init();
	
	init_timer_0B();		// Init periodic timer that interrupts every second
	
	Buffer_Pool_init();
	
	while(1)
	{
		// Test
		this_buf = get_buffer();
		if( this_buf == (void *) 0) {
			turn_on_red_LED();
			break;
		}
		else
			good_buf = this_buf;
		this_buf->free = 0;
		this_buf->message[0] = char_msg;
		
		char_msg++;
	}
	
	// Test
	release_buffer(good_buf);
	this_buf = get_buffer();
}

/* Handler for GPIOF interrupts 
 * - Handle Push Button SW1 interrupt (PF4)
 */
void GPIOF_Handler(void)
{
	// If Interrupt caused by SW1 (PF4)
	if(GPIOF->RIS & (1UL << 4))
	{
		// toggle green LED for debugging
		toggle_green_LED();
		// Send Message to UART
		UART_OutChar('B'); 
		UART_OutChar('u');
		UART_OutChar('t');
		UART_OutChar('t');
		UART_OutChar('o');
		UART_OutChar('n');
		
		GPIOF->ICR |= (1UL << 4);	// Clear Interrupt
		NVIC->ICPR[0] = (1UL << 30); 	// Clear pending bit from NVIC
	}
}

/* Handler for TIMER0B interrupt 
 * - Acts as a clock that times out every 1 second
 */
void TIMER0B_Handler(void)
{
	toggle_blue_LED();
	
	TIMER0->ICR |= (1UL << 8);		// Clear interrupt to de-assert IRQ#20 (TIMER0B) signal
	NVIC->ICPR[0] |= (1UL << 20);	//Clear pending bit in NVIC for IRQ#20 TIMER0B
	return;
}
