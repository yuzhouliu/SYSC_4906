#include "led.h"

/** Init LED ports **/
void LED_init(void)
{
	uint32_t dummy;
	
	// Enable GPIOB (IR Rx), GPIOE (IR Tx) and GPIOF (LED for debug)
	SYSCTL->RCGCGPIO |= (1UL << 5);
	// Do a dummy read to insert a few cycles after enabling the peripheral.
  dummy = SYSCTL->RCGCGPIO;
	
	// Sets GPIOF's direction for LED
	GPIOF->DIR |= ((1UL << 3) | (1UL << 2) | (1UL << 1));	//PF3/PF2/PF1 Green/Blue/Red LED Output
	GPIOF->DEN |= ((1UL << 3) | (1UL << 2) | (1UL << 1));	//Enabled PF3/PF2/PF1 Green/Blue/Red LED
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
