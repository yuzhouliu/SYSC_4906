#include "CU_TM4C123.h"

#include "led.h"
#include "utility.h"

int main(void)
{
	LED_init();
	turn_on_green_LED();
	
	init_pushButton();
	
	return 0;
}
