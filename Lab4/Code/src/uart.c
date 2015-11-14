#include "uart.h"

static Buffer_pool buffer_pool_TX;

/* Initialize the buffer pool by setting individual buffers to their
 * default values
 */
void Buffer_Pool_TX_init(void)
{
	uint8_t i;
	Buffer *curr_buf;
	for(i = 0; i < BUF_POOL_SIZE; i++){
		curr_buf = &(buffer_pool_TX.buffer[i]);		
		curr_buf->count = 0;			// no chars stored
		curr_buf->curr_i = 0;			// current index at 0
	}
	buffer_pool_TX.head = 0;
	buffer_pool_TX.tail = 0;
	buffer_pool_TX.count = 0;
	curr_buf = curr_buf;				// Get rid of compiler warning "set but never used"
}

/* Returns a pointer to buffer allocated from the pool
 */
Buffer * get_buffer_TX(void)
{
	Buffer *curr_buf;
	
	// If there are no free buffers in the buffer pool, return NULL pointer
	if (buffer_pool_TX.count == BUF_POOL_SIZE){
		return (void *) 0;
	}
	
	// Update the buffer pool so that a buffer element has been taken
	curr_buf = &(buffer_pool_TX.buffer[buffer_pool_TX.tail]);
	buffer_pool_TX.tail = (buffer_pool_TX.tail + 1) % BUF_POOL_SIZE;
	buffer_pool_TX.count++;
	
	return curr_buf;
}

/* Releases the input buffer back to pool
 */
void release_buffer_TX(Buffer * a_buffer)
{
	// If the input was NULL, then return
	if( a_buffer == (void *) 0)
		return;
	
	// Reset the elements inside the buffer
	a_buffer->count = 0;
	a_buffer->curr_i = 0;
	
	// Update head to point to next buffer element. Update count to reflect # buffers occupied
	buffer_pool_TX.head = (buffer_pool_TX.head+1) % BUF_POOL_SIZE;
	buffer_pool_TX.count--;
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
	
	UART0->ICR |= (1UL << 5);								// TX interrupt clear
	UART0->IM &= ~(1UL << 5);								// Mask TX Interrupts
	UART0->CTL |= (1UL << 4);								// Interrupt on End Of Transmission
	UART0->CTL |= UART_CTL_UARTEN;          // enable UART
	
	UART0_nvic_init();
	UART_OutChar('>');
	UART_OutChar('\n');
	UART_OutChar('\r');
}

static void UART0_nvic_init(void)
{
	NVIC->IP[5] |= (2 << 5);			//IRQ#5 for UART0, see ds pg104
	NVIC->ICPR[0] |= (1UL << 5);	//Clear pending bit to be safe
	NVIC->ISER[0] |= (1UL << 5);	//Enable interrupt at NVIC
}

void UART0_Handler(void)
{
	Buffer * curr_buf;
	
	// If interrupt caused by empty TX FIFO 
	if ((UART0->FR & UART_FR_TXFF) == 0) {
		
		// If there's no more TX messages to send, mask UART0 TX interrupt
		if(buffer_pool_TX.count == 0) {
			UART0->IM &= ~(1UL << 5);
			return;
		}
		
		curr_buf = &(buffer_pool_TX.buffer[buffer_pool_TX.head]);
		
		// If the end of message has been reached, release the current buffer
		if(curr_buf->curr_i == curr_buf->count) {
			release_buffer_TX(curr_buf);
			return;
		}
		
		// Otherwise, write the current char to TX FIFO and update curr_i
		UART0->DR = curr_buf->message[curr_buf->curr_i];
		curr_buf->curr_i++;
	}
	
	UART0->ICR |= (1UL << 5);			// Clear interrupt from UART0
	NVIC->ICPR[0] |= (1UL << 5);	//Clear UART0 interrupt pending bit
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

/* Will transmit the input message over UART 
 */
void Tx_message(Buffer *msg_buffer)
{
	// Unmask interrupts, sets UART0 to start TX
	UART0->IM |= (1UL << 5);		
}
