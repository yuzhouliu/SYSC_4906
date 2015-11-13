#include "stdint.h"
#include "CU_TM4C123.h"

#define MSG_SIZE 20
#define BUF_POOL_SIZE 5

// Buffer stores the messages
typedef struct {
	uint8_t free;						// Indicates if buffer is in use
	uint8_t head;						// Points to head of buffer
	uint8_t tail;						// Points to tail of buffer
	uint8_t count;					// Num of characters in message
	char message[MSG_SIZE];	// message
}Buffer;

// Buffer pool stores an array of buffers
typedef struct {
	Buffer buffer[BUF_POOL_SIZE];		// Array of buffers
}Buffer_pool;

/* Initialize the Buffer pool
 */
void Buffer_Pool_init(void);

/* Returns a pointer to buffer allocated from the pool
 */
Buffer * get_buffer(void);

/* Releases the input buffer back to pool
 */
void release_buffer(Buffer * a_buffer);

/* Initialize UART0 for 115,200 baud rate (assuming 16 MHz UART clock),
 * 8-bit word length, no parity bit, one stop bit, FIFOs enabled
 */
void UART_init(void);

/* Output 8-bit data to serial port through TX FIFO
 */
void UART_OutChar(unsigned char data);
