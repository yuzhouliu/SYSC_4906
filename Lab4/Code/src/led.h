#ifndef _LED_H
#define _LED_H

/** LED functionality
**/

#include "CU_TM4C123.h"

/* Initialize the LED GPIO ports for digital output */
void LED_init(void);

void toggle_green_LED(void);
void turn_on_green_LED(void);
void turn_off_green_LED(void);
void toggle_blue_LED(void);
void turn_on_blue_LED(void);
void turn_off_blue_LED(void);
void toggle_red_LED(void);
void turn_on_red_LED(void);
void turn_off_red_LED(void);

#endif /* _LED_H */
