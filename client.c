#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "rfm12.h"
#include "board.h"
#include "delay.h"

#define true 1
#define false 0

#define LED_GRN_ON PORTD |=1<<LED_GRN;
#define LED_RED_ON PORTD |=1<<LED_RED;
#define LED_GRN_OFF PORTD &=~(1<<LED_GRN);
#define LED_RED_OFF PORTD &=~(1<<LED_RED);
#define TST_BIT(reg,bit) (reg&(1<<bit))

char tx_data[] = "12345";


void go_sleep (void)
{
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	sleep_cpu();
}


void init_gpio (void)
{
	DDRD |= (1<<LED_GRN) | (1<<LED_RED);
	PORTD = 0;
	DDRC ^= ~(1<<BTN_TMP);
	PORTC |= (1<<BTN_TMP);
}

int main (void)
{
	int btn_press = false;

	init_gpio ();
	rfm12_init ();
	sei();

// dont'work with it!!!!
	LED_GRN_ON;
	__delay_cycles (65000);
	LED_GRN_OFF;

	while (1) {
		// on btn press send data
	//	if (!TST_BIT(PINC,BTN_TMP) && (btn_press == false)) {
			btn_press = true;
			if (rfm12_tx (5, 0, tx_data) == RFM12_TX_ENQUEUED) {
				LED_GRN_ON;
			}
			else {
				LED_RED_ON;
			}
			__delay_cycles (65000);
			LED_RED_OFF;
			LED_GRN_OFF;

	//	}
		if (TST_BIT(PINC,BTN_TMP)) {
			btn_press = false;
		}

		// if we receive something	
		if (rfm12_rx_status() == STATUS_COMPLETE) {
			LED_RED_ON;
			__delay_cycles (65000);
			__delay_cycles (65000);
			rfm12_rx_clear (); // realise rx buffer
			LED_RED_OFF;
		}
		rfm12_set_wakeup_timer(1124);	
		rfm12_tick();
			__delay_cycles (65000);
		go_sleep();

// rfm12_rx_clear
	}
	return 0;
}

