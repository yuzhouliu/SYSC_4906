
#include "adc_driver.h"
#include "led.h"
#include "main.h"
#include "utility.h"

#define ANALOG1 (1UL << 2) // PE2 <-> AIN1
#define ANALOG2 (1UL << 3) // PE3 <-> AIN0

int weird_i = 0;

void init_ADC_GPIO(void)
{
	SYSCTL->RCGCGPIO |= (1UL << 4);		// Enable clock for Port E
	if(SYSCTL->RCGCGPIO) {}					// Dummy read
		
	GPIOE->DIR &= ~(ANALOG1|ANALOG2);	//Input
	GPIOE->DEN &= ~(ANALOG1|ANALOG2);	//Digital disable
	GPIOE->PUR |= (ANALOG1|ANALOG2);	// Pull up resistor
	
}

/* Initialize ADC */
void init_ADC(void)
{
	//uint32_t dummy;
	init_ADC_GPIO();
	
	/* Configure ADC0, SS03 */
	SYSCTL->RCGCADC |= (1UL<<0); //Enable clock to ADC0
	
	//Wait until ADC0 ready
	if(SYSCTL->RCGCADC) {}				// Dummy read
	if(SYSCTL->RCGCADC) {}				// Dummy read
	
	GPIOE->AFSEL |= (ANALOG1 | ANALOG2);
	GPIOE->AMSEL |= (ANALOG1 | ANALOG2);
	
	ADC0->ACTSS &= ~(1UL<<3);	//Disable SS03 while programming
	
	ADC0->EMUX |= (0x5<<12);	//SS03 on timer
	ADC0->SSMUX3 = (0x1<<0); 	//SS03 sample AIN1	
	ADC0->SSCTL3 = ((1UL<<2) | (1UL << 1));  //SS03 trigger interrupt
	
	ADC0->RIS &= ~(1UL<<3); 	//Clear interrupt		
	ADC0->IM |= (1UL<<3);		//Arm interrupt at SS03		
	ADC0->ISC &= ~(1UL<<3);		//Clear Level-senstive interrupt
		
	ADC0->ACTSS |= (1UL<<3);	//Enable SS03

	/* Configure ADC1, SS03 */
	SYSCTL->RCGCADC |= (1UL << 1); 	//Enable clock to ADC1
	if(SYSCTL->RCGCADC) {}				// Dummy read
	if(SYSCTL->RCGCADC) {}				// Dummy read
	
	ADC1->ACTSS &= ~(1UL << 3);	//Disable SS03 while programming
	ADC1->EMUX |= (0x5<<12);	//SS03 on timer
	ADC1->SSMUX3 = 0x0; 		//SS03 sample AIN0
	ADC1->SSCTL3 = ((1UL<<2) | (1UL << 1));  //SS03 trigger interrupt
	
	ADC1->RIS &= ~(1UL<<3); 	//Clear interrupt		
	ADC1->IM |= (1UL<<3);		//Arm interrupt at SS03		
	ADC1->ISC &= ~(1UL<<3);		//Clear Level-senstive interrupt
	ADC1->ACTSS |= (1UL<<3);	//Enable SS03

	init_adc_nvic();
}

/* Configures NVIC for ADC interrupts 
 */
static void init_adc_nvic(void)
{	
	// init for ADC0 SS03 
	NVIC->IP[17] = (2<<5); // SS03 has Priority 2
	NVIC->ICPR[0] = (1UL<<17);// Clear pending bit to be safe
	NVIC->ISER[0] = (1UL<<17);// Enable interrupt at NVIC
	
	// init for ADC1 SS03
	NVIC->IP[51] = (2<<5); // SS03 has Priority 2
	NVIC->ICPR[1] = (1UL<<19);// Clear pending bit to be safe
	NVIC->ISER[1] = (1UL<<19);// Enable interrupt at NVIC	
}

/* Interrupt handler for ADC0 SS03
 * Mapped to PE2
 * >> Used to read in Noise Reference for this application
 */
void ADC0SS3_Handler(void)
{		
	/*
	uint16_t noise_ref = ADC0->SSFIFO3;
	//toggle_blue_LED();
	
	input_ref.buffer[input_ref.tail] = noise_ref;     // get input sample
	input_ref.tail += (input_ref.tail+1)%FILTER_LENGTH;		// Update index for most recent entry
	input_ref.head += (input_ref.head+1)%FILTER_LENGTH;		// Update index for the last entry
		
	state = NOISE_REF_READY;
	*/

	set_voltage(ADC0->SSFIFO3);
	//uint16_t noise_ref = ADC0->SSFIFO0;
	
	//PWM0->_2_CMPA = noise_ref>>4;
	
	//PWM0->_2_CMPA = (ADC0->SSFIFO3-160)>>4;
	
	ADC0->ISC |= (1UL << 3);
	NVIC->ICPR[0] = (1UL << 17);	//Clear pending bit in NVIC for IRQ#17 ADC0
}

/* Interrupt handler for ADC1 SS03
 * Mapped to PE3
 * >> Used to read in Error for this application
 */
void ADC1SS3_Handler(void)
{
	//uint16_t error_here;
	//error_here = ADC1->SSFIFO3;
	error = ADC1->SSFIFO3;
	
	//set_voltage(error_here);			// Debug - Make a copy of this voltage onto PE4 (PWM output)
	
	//toggle_red_LED();
	
	state = ERROR_READY;
	
	ADC1->ISC |= (1UL << 3);
	NVIC->ICPR[1] = (1UL << 19);	//Clear pending bit in NVIC for IRQ#51 ADC1
}
