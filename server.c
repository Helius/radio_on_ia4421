#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "rfm12.h"
#include "board.h"
#include "delay.h"
#include "uart.h"

#define true 1
#define false 0

#define LED_GRN_ON PORTD |=1<<LED_GRN;
#define LED_RED_ON PORTD |=1<<LED_RED;
#define LED_GRN_OFF PORTD &=~(1<<LED_GRN);
#define LED_RED_OFF PORTD &=~(1<<LED_RED);
#define TST_BIT(reg,bit) (reg&(1<<bit))

//char tx_data[] = "12345";

void init_gpio (void)
{
	DDRD |= (1<<LED_GRN) | (1<<LED_RED);
	PORTD = 0;
	DDRC ^= ~(1<<BTN_TMP);
	PORTC |= (1<<BTN_TMP);
}

int main (void)
{
	uart_init();
	init_gpio ();
	rfm12_init ();
	//rfm12_set_wakeup_timer(0xEA00 | 40);

	sei();
	char msg [32];
// dont'work with it!!!!
	LED_RED_ON;
	__delay_cycles (65000);
	LED_RED_OFF;
				
	uart_putstr ("server start\n\r");

	while (1) {

		// if we receive something	
		if (rfm12_rx_status() == STATUS_COMPLETE) {
			uint8_t * buf = rfm12_rx_buffer();
			int i;
			for (i = 0; i < rfm12_rx_len(); i++) {
				sprintf (msg, "%d ", buf[i]);
				uart_putstr (msg);
			}
			uart_putstr ("\n\r");
			int div;
			if (buf[0] == 0) {
				if (buf[1]&1)
					div = 5;
				else
					div = 0;
				sprintf (msg, "#%d.%d\n\r", buf[1]/2,div);
			} else {
				int tmp = (buf[1]^0xFF)+1;
				if (tmp&1)
					div = 5;
				else
					div = 0;
				sprintf (msg, "#-%d.%d\n\r",tmp/2,div);
			}
			uart_putstr(msg);
			rfm12_rx_clear (); // realise rx buffer
			LED_RED_ON;
			__delay_cycles (65000);
			LED_RED_OFF;
			rfm12_tx(3,0,"ok!");
			rfm12_tick();
			LED_GRN_ON;
			__delay_cycles (65000);
			LED_GRN_OFF;
			//__delay_cycles (65000);
		}
		rfm12_tick();
	}
	return 0;
}

