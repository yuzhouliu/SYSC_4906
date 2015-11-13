#include "uart.h"

static Buffer_pool buffer_pool;

/* Initialize the buffer pool by setting individual buffers to their
 * default values
 */
void Buffer_Pool_init(void)
{
	uint8_t i;
	Buffer *curr_buf;
	for(i = 0; i < BUF_POOL_SIZE; i++){
		curr_buf = &(buffer_pool.buffer[i]);
		
		curr_buf->free = 1;					// buffer is free
		curr_buf->count = 0;				// no chars stored
		curr_buf->head = 0;
		curr_buf->tail = 0;
		curr_buf->message[0] = '-';	// empty message
	}
	curr_buf = curr_buf;			// Get rid of compiler warning "set but never used"
}

/* Returns a pointer to buffer allocated from the pool
 */
Buffer * get_buffer(void)
{
	uint8_t i;
	Buffer *curr_buf;
	
	// Search for a free buffer by iterating through the list
	for(i = 0; i < BUF_POOL_SIZE; i++) {
		curr_buf = &(buffer_pool.buffer[i]);
		
		// Free buffer found, return it
		if(curr_buf->free)
			return curr_buf;
	}
	
	// Couldn't find free buffer, return NULL pointer
	return (void *) 0;
}

/* Releases the input buffer back to pool
 */
void release_buffer(Buffer * a_buffer)
{
	// If the input was NULL, then return
	if( a_buffer == (void *) 0)
		return;
	
	a_buffer->free = 1;			//Mark the buffer is free
}

/* Initialize UART0 for 115,200 baud rate (assuming 16 MHz UART clock),
 * 8-bit word length, no parity bit, one stop bit, FIFOs enabled
 */
void UART_init(void) 
{
	SYSCTL->RCGCUART |= (1UL << 0);		// activate UART0
	SYSCTL->RCGCGPIO |= (1UL << 0); 	// activate port A
	UART0->CTL &= ~UART_CTL_UARTEN;		// Disable UART
	UART0->IBRD = 8;									// 16-bit integer baud rate divisor: 16,000,000/(16 * 115,200) = 8.680555 -> 8
	UART0->FBRD = 44;									// 8-bit fractional baud rate divisor: 0.680555 * 64 + 0.5 = 44
	UART0->LCRH = (UART_LCRH_WLEN_8|UART_LCRH_FEN);		// 8 bit word length (no parity bits, one stop bit, enable FIFOs)
	
	GPIOA->AMSEL &= ~((1UL << 0)|(1UL << 1)); // disable analog functionality on PA0-1
  GPIOA->AFSEL |= (1UL << 0)|(1UL << 1);    // enable alt funct on PA1-0
  GPIOA->DEN |= (1UL << 0)|(1UL << 1);      // enable digital I/O on PA1-0                                           
  GPIOA->PCTL = (GPIOA -> PCTL & 0xFFFFFF00)+0x00000011;  // configure PA1-0 as UART   (just in case)
	UART0->CTL |= UART_CTL_UARTEN;          // enable UART
}

/* Output 8-bit data to serial port through TX FIFO
 */
void UART_OutChar(unsigned char data)
{
  // wait for space in Tx FIFO
  while((UART0->FR & UART_FR_TXFF) != 0);
  // write char to Tx FIFO
  UART0->DR = data;
}
