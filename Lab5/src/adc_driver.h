#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include "stdint.h"
#include "CU_TM4C123.h"

/* Initialize ADC GPIO ports
 */
static void init_ADC_GPIO(void);

/* Initialize ADC */
void init_ADC(void);

/* Configures NVIC for ADC interrupts 
 */
static void init_adc_nvic(void);

/* Interrupt handler for ADC0 SS03
 * 	Mapped to PE2
 */
void ADC0SS3_Handler(void);

/* Interrupt handler for ADC1 SS03
 * Mapped to PE3
 */
void ADC1SS3_Handler(void);

#endif /* ADC_DRIVER_H */
