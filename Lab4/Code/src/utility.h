#include "CU_TM4C123.h"

/*********************************************************************************
 Functions for Push Button functionality start
 *********************************************************************************/
/* Initializes SW1 (PF4) as interrupt enabled push button. */
void init_pushButton(void);

/* Sets the NVIC for push button interrupts */
static void pushButton_nvic_init(void);

/* Note the Handler for GPIOF pins (push buttons) has been relocated into
 * lab4_main.c. This is so that the main function can control interrupt response
 */

/*********************************************************************************
 Functions for Push Button functionality end
 *********************************************************************************/
