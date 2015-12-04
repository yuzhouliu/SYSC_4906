#include "CU_TM4C123.h"
#include "stdint.h"

#define WAITING_FOR_ADC 0
#define NOISE_REF_READY 1
#define ERROR_READY 2
extern volatile uint8_t state;

#define FILTER_LENGTH 8
extern volatile uint16_t error;

typedef struct {
	uint8_t head;
	uint8_t tail;
	uint16_t buffer[FILTER_LENGTH];
} circular_buffer;

extern signed long int weight_coeff[FILTER_LENGTH];
extern circular_buffer input_ref;

/* Initialize all variables in circular buffer to 0
 */
void set_circ_buffer_0(circular_buffer *buff);

/* Calculates and outputs the anti-noise
 */
void output_anti_noise(void);

/* Updates the weights used for the LMS adaptation filter
 */
void update_LMS_weights(void);
