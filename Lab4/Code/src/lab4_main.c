#include "CU_TM4C123.h"

#include "led.h"
#include "utility.h"

int main(void)
{
	LED_init();
	turn_on_green_LED();
	
	init_pushButton();
	//pushButton_nvic_init();
	
	return 0;
}

/* Handler for GPIOF interrupts 
 * - Handle SW1 interrupt (PF4)
 */
void GPIOF_Handler(void)
{
	// If Interrupt caused by SW1 (PF4)
	if(GPIOF->RIS & (1UL << 4))
	{
		GPIOF->ICR |= (1UL << 4);	// Clear Interrupt
		NVIC->ICPR[0] = (1UL << 30); 	// Clear pending bit from NVIC
	}
}
