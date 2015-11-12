#include "CU_TM4C123.h"

#include "led.h"
#include "utility.h"
#include "uart.h"

int main(void)
{
	LED_init();
	init_pushButton();
	
	UART_init();
	
	while(1)
	{}	
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
