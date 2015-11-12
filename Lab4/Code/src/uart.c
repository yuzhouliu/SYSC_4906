#include "uart.h"


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

void UART_OutChar(unsigned char data)
{
  // wait for space in Tx FIFO
  while((UART0->FR & UART_FR_TXFF) != 0);
  // write char to Tx FIFO
  UART0->DR = data;
}
