#include "uart.h"

static Buffer_pool buffer_pool_TX;
static Buffer_pool buffer_pool_RX;

/* Returns the appropriate buffer pool (TX or RX)
 */
Buffer_pool * get_buf_pool(uint8_t buf_pool_type)
{
	switch(buf_pool_type) {
		case BUF_TX:
			return &buffer_pool_TX;
		case BUF_RX:
			return &buffer_pool_RX;
		default:
			return (void *)0;
	}
}

/* Initialize the buffer pool by setting individual buffers to their
 * default values
 */
void Buffer_Pool_init(uint8_t buf_pool_type)
{
	uint8_t i;
	Buffer_pool *buffer_pool;
	Buffer *curr_buf;
	
	buffer_pool = get_buf_pool(buf_pool_type);
	if (buffer_pool == (void *)0)
		return;
	
	for(i = 0; i < BUF_POOL_SIZE; i++){
		curr_buf = &(buffer_pool->buffer[i]);		
		curr_buf->count = 0;			// no chars stored
		curr_buf->curr_i = 0;			// current index at 0
	}
	buffer_pool->head = 0;
	buffer_pool->tail = 0;
	buffer_pool->count = 0;
	curr_buf = curr_buf;				// Get rid of compiler warning "set but never used"
}

/* Returns a pointer to buffer allocated from the pool
 */
Buffer * get_buffer(uint8_t buf_pool_type)
{
	Buffer_pool *buffer_pool;
	Buffer *curr_buf;
	
	buffer_pool = get_buf_pool(buf_pool_type);
	if (buffer_pool == (void *)0)
		return (void *)0;
	
	// If there are no free buffers in the buffer pool, return NULL pointer
	if (buffer_pool->count == BUF_POOL_SIZE){
		return (void *)0;
	}
	
	// Update the buffer pool so that a buffer element has been taken
	curr_buf = &(buffer_pool->buffer[buffer_pool->tail]);
	buffer_pool->tail = (buffer_pool->tail + 1) % BUF_POOL_SIZE;
	buffer_pool->count++;
	
	return curr_buf;
}

/* Releases the input buffer back to pool
 */
void release_buffer(Buffer * a_buffer, uint8_t buf_pool_type)
{
	Buffer_pool *buffer_pool;
	
	// If the input was NULL, then return
	if( a_buffer == (void *)0)
		return;
	
	buffer_pool = get_buf_pool(buf_pool_type);
	if (buffer_pool == (void *)0)
		return;
	
	// Reset the elements inside the buffer
	a_buffer->count = 0;
	a_buffer->curr_i = 0;
	
	// Update head to point to next buffer element. Update count to reflect # buffers occupied
	buffer_pool->head = (buffer_pool->head+1) % BUF_POOL_SIZE;
	buffer_pool->count--;
}

/* Initialize UART0 for 115,200 baud rate (assuming 16 MHz UART clock),
 * 8-bit word length, no parity bit, one stop bit, FIFOs enabled
 */
void UART_init(void) 
{
	SYSCTL->RCGCUART |= SYSCTL_RCGCUART_R0;		// activate UART0
	SYSCTL->RCGCGPIO |= SYSCTL_RCGCGPIO_R0; 	// activate port A
	UART0->CTL &= ~UART_CTL_UARTEN;		// Disable UART
	UART0->IBRD = 8;									// 16-bit integer baud rate divisor: 16,000,000/(16 * 115,200) = 8.680555 -> 8
	UART0->FBRD = 44;									// 8-bit fractional baud rate divisor: 0.680555 * 64 + 0.5 = 44
	UART0->LCRH = (UART_LCRH_WLEN_8|UART_LCRH_FEN);		// 8 bit word length (no parity bits, one stop bit, enable FIFOs)
	
	GPIOA->AMSEL &= ~((1UL << 0)|(1UL << 1)); // disable analog functionality on PA0-1
  GPIOA->AFSEL |= (1UL << 0)|(1UL << 1);    // enable alt funct on PA1-0
  GPIOA->DEN |= (1UL << 0)|(1UL << 1);      // enable digital I/O on PA1-0                                           
  GPIOA->PCTL = (GPIOA -> PCTL & 0xFFFFFF00)+0x00000011;  // configure PA1-0 as UART   (just in case)
	
	UART0->ICR |= (1UL << 5);						// TX interrupt clear
	UART0->CTL |= (1UL << 4);							// Interrupt on End Of Transmission	
	
	//UART0->ICR = 1UL;											// Clear time-out interupt
	//UART0->ICR |= (1UL << 4);						// RX interrupt clear
	//UART0->IFLS &= ~UART_IFLS_RX_M;			// Trigger when >= 1/8 FIFO full
	
	//UART0->IM |= (UART_IM_TXIM | UART_IM_RXIM);
	UART0->IM |= UART_IM_TXIM;						// TX interrupt enable
	UART0->CTL |= UART_CTL_UARTEN;        // enable UART
	
	UART0_nvic_init();
	
	// Send out test string
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
	//char rx_msg;
	
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
			release_buffer(curr_buf, BUF_TX);
			return;
		}
		
		// Otherwise, write the current char to TX FIFO and update curr_i
		UART0->DR = curr_buf->message[curr_buf->curr_i];
		curr_buf->curr_i++;
		
		UART0->ICR |= (1UL << 5);			// Clear interrupt from UART0
	}
	/*
	// If interrupt caused by RX
	else if ((UART0->RIS & UART_RIS_RXRIS) != 0) {
		toggle_blue_LED();
		//rx_msg = (char) (UART0->DR & 0xFF);
		//Tx_message(&rx_msg, 1);
		UART0->ICR |= (1UL << 4);								// RX interrupt clear
	}
	*/
	
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
int Tx_message(char *msg, uint8_t size)
{
	Buffer * this_buf;
	
	// Entering critical section, disable interrupts
	__set_PRIMASK(1);		
	this_buf = get_buffer(BUF_TX);			// Get an available TX buffer
	__set_PRIMASK(0);
	// Left critical section, interrupts are enabled again
	
	if( this_buf == (void *) 0) {
			return -1;
	}
	
	my_strncpy(this_buf->message, msg, size);
	this_buf->count = size;
	
	// Unmask interrupts, sets UART0 to start TX
	UART0->IM |= (1UL << 5);
	return 0;
}

/* Checks to see if a RX message is available
 * Note this method is polling method. In the future may come back and revise this
 */
uint8_t UART_rx_available(void) 
{
	return !(UART0->FR & UART_FR_RXFE);
}

/* Handles the received character from UART
 * Determines the appropriate action for the input command
 */
void Rx_message(void) 
{
	static char char_input[6];
	static int count = 0;
	char curr_char;
	
	curr_char = UART0->DR & 0xFF;
	Tx_message(&curr_char, 1);
	
	if(curr_char == '\r') {
		if(count == 1) {
			// If it is single letter command
			switch(char_input[0]) {
				case 'p':
					// Pause the generation of time messages
					suppress_time_output = 1;
					break;
				case 'c':
					// Continue generation of time messages
					suppress_time_output = 0;
					break;
				case 'f':
					// Double the frequency of time message output
					double_time_msg_freq();
					break;
				case 's':
					// Half the frequency of time message output
					half_time_msg_freq();
					break;
				default:
					//Unknown command
					toggle_blue_LED();
			}
		}
		else if((count == 6) && (char_input[0] == 'T')) {
			// If it is set time command
			set_time(char_input[1], char_input[2], char_input[4], char_input[5]);
		}
		else {
			// Unknown command
			toggle_blue_LED();
		}
		count = 0;
		curr_char = '\n';
		Tx_message(&curr_char, 1);
	}
	else {
		char_input[count++] = curr_char;
	}
}
