#include "CU_TM4C123.h"

/* Initializes SW1 (PF4) as interrupt enabled push button. */
void init_pushButton(void);

/* Sets the NVIC for push button interrupts */
static void pushButton_nvic_init(void);

/* Handler for GPIOF pins (push buttons) */
void GPIOF_Handler(void);
