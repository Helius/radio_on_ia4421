#include <avr/io.h>
#include <avr/interrupt.h>
#include "board.h"

#define LED_GRN_ON PORTD|=1<<LED_GRN;
#define LED_RED_ON PORTD|=1<<LED_RED;
#define LED_GRN_OFF PORTD ^=~(1<<LED_GRN);
#define LED_RED_OFF PORTD^=~(1<<LED_RED);
#define TST_BIT(reg,bit) (reg&(1<<bit))

void init_gpio (void)
{
	DDRD |= (1<<LED_GRN) | (1<<LED_RED);
	DDRC ^= ~(1<<BTN_TMP);
	PORTC |= (1<<BTN_TMP);
}

int main (void)
{
	init_gpio ();
	while (1) {
		if (TST_BIT(PINC,BTN_TMP)) {
			LED_GRN_ON;
			LED_RED_OFF;
		}	else {
			LED_RED_ON;
			LED_GRN_OFF;
		}
	}
	return 0;
}
