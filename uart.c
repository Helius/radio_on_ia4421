/* USART-Init beim ATmegaXX */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#include "uart.h"

#define UART_BAUD_RATE 19200
#define UART_RXBUFSIZE 50
#define UART_TXBUFSIZE 50


#define UART_BAUD_CALC(UART_BAUD_RATE,F_OSC) ((F_OSC)/((UART_BAUD_RATE)*16L)-1)


void uart_init() {
   UBRR0L = 25;
   UBRR0H = 0x00;
   // установка формата посылки, 8 бит данных, 1 стоп-бит
   UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
   // разрешение работы приемника и передатчика
   UCSR0B = ((1 << RXEN0) | (1 << TXEN0));
}

void uart_putc(char ch) {
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = ch;
	return;
}


void uart_putstr(char *str) {
	while(*str) {
		uart_putc(*str++);
	}
}

void uart_putstr_P(PGM_P str) {
	char tmp;
	while((tmp = pgm_read_byte(str))) {
		uart_putc(tmp);
		str++;
	}
}

void uart_hexdump(char *buf, int len)
{
	unsigned char x=0;
	char sbuf[3];

	while(len--){
		itoa(*buf++, sbuf, 16);
		if (sbuf[1] == 0) uart_putc(' ');
		uart_putstr(sbuf);
		uart_putc(' ');
		if(++x == 16) {
			uart_putstr_P(PSTR("\r\n"));
			x = 0;
		}
	}
}

char uart_getc()
{
	while (!(UCSR0A & (1<<RXC0)));	// warten bis Zeichen verfuegbar
	return UDR0;			// Zeichen aus UDR zurueckgeben
}

// returns 1 on success
char uart_getc_nb(char *c)
{
	if (UCSR0A & (1<<RXC0)) {		// Zeichen verfuegbar
		*c = UDR0;
		return 1;
	}

	return 0;
}
