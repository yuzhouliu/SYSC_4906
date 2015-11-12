#include "CU_TM4C123.h"

/* Initialize the UART for 115,200 baud rate (assuming 16 MHz UART clock),
 * 8-bit word length, no parity bit, one stop bit, FIFOs enabled
 */
void UART_init(void);

/* Output 8-bit data to serial port through TX FIFO
 */
void UART_OutChar(unsigned char data);
