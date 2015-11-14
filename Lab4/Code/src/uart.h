#ifndef _UART_H
#define _UART_H

#include <stdint.h>
#include "CU_TM4C123.h"
#include "utility.h"

#define MSG_SIZE 20
#define BUF_POOL_SIZE 5

#define BUF_TX 0
#define BUF_RX 1

// Buffer stores the messages
typedef struct {
	uint8_t curr_i;					// Index for current character
	uint8_t count;					// Num of characters in message
	char message[MSG_SIZE];	// message
}Buffer;

// Buffer pool stores an array of buffers
typedef struct {
	uint8_t head;
	uint8_t tail;
	uint8_t count;
	Buffer buffer[BUF_POOL_SIZE];		// Array of buffers
}Buffer_pool;


/* Returns the appropriate buffer pool (TX or RX)
 */
Buffer_pool * get_buf_pool(uint8_t buf_pool_type);

/* Initialize the Buffer pool
 */
void Buffer_Pool_init(uint8_t buf_pool_type);

/* Returns a pointer to buffer allocated from the TX buffer pool
 */
Buffer * get_buffer(uint8_t buf_pool_type);

/* Releases the input buffer back to TX buffer pool
 */
void release_buffer(Buffer * a_buffer, uint8_t buf_pool_type);

/* Initialize UART0 for 115,200 baud rate (assuming 16 MHz UART clock),
 * 8-bit word length, no parity bit, one stop bit, FIFOs enabled
 */
void UART_init(void);

/* Configure the NVIC for UART0 interrupts
 */
static void UART0_nvic_init(void);

/* Output 8-bit data to serial port through TX FIFO
 */
void UART_OutChar(unsigned char data);

/* Will transmit the input message over UART 
 */
int Tx_message(char *msg, uint8_t size);

#endif /* _UART_H */
