#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "rfm12.h"
#include "board.h"
#include "delay.h"

#define true 1
#define false 0

#define LED_GRN_ON PORTD |=1<<LED_GRN;
#define LED_GRN_OFF PORTD &=~(1<<LED_GRN);
#define LED_RED_ON PORTD |=1<<LED_RED;
#define LED_RED_OFF PORTD &=~(1<<LED_RED);
#define TST_BIT(reg,bit) (reg&(1<<bit))

char tx_data[] = "12345";



void go_sleep (void)
{
	RFM12_WAKEUP_INT_SETUP();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	sei();
	sleep_cpu();
	sleep_disable();
	RFM12_INT_SETUP();
}

void init_gpio (void)
{
	DDRD |= (1<<LED_GRN) | (1<<LED_RED);
	PORTD = 1<<PD2;
	DDRC |= (1<<ADC_EN);
	PORTC |= (1<<ADC_EN);
}
void clear_gpio (void)
{
}


int adc_get () {
	ADMUX = (1<<REFS1) | (1<<REFS0) | (1<<MUX0) | (1<<MUX1) | (1<<ADLAR);
	//ADMUX = (1<<REFS1) | (1<<REFS0) | (1<<MUX3) |(1<<MUX2) | (1<<MUX1) | (1<<ADLAR);
	__delay_cycles (5000);
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS0);
	ADCSRA |= 1<<ADSC;
	while (ADCSRA & (1<<ADIF));
	return ADCH;
}

void adc_off () {
	ADCSRA = 0;
}

int wake_up_me;

#define STATE_TX 3
uint8_t payload [8];
uint8_t Ubatt;
uint8_t adc_idle = 0;
int main (void)
{
	init_gpio ();
	sei();
	rfm12_init ();
	rfm12_set_wakeup_timer(0xEA00 | 5);
	//uart_init();
	
// dont'work with it!!!!
	LED_GRN_ON;
	__delay_cycles (65000);
	__delay_cycles (65000);
	__delay_cycles (65000);
	LED_GRN_OFF;
	
	while (1) {
			
		LED_GRN_ON;

		// get battery voltage
		if (!adc_idle--) {
			adc_idle = 10;
			//get temperature
			DDRC |= (1<<2);
			PORTC |= (1<<2);
			__delay_cycles (5000);
			adc_get();
			unsigned int tmp = adc_get();
			Ubatt = (tmp*17)/100;
			adc_off();
			DDRC &=~(1<<2); 
			PORTC &=~(1<<2); 
		}

		// get temperature data
		payload[0] = 1;
		payload[1] = 2;
		payload[2] = Ubatt;

		/*    TX     */
#include "rfm12/src/include/rfm12_hw.h"
#include "rfm12/src/include/rfm12_core.h"
		rfm12_data(RFM12_CMD_CFG | RFM12_CFG_EL | RFM12_CFG_EF | RFM12_BASEBAND | RFM12_XTAL_12PF);
		rfm12_tx (3, 0, payload);
		rfm12_tick();
		//wait while TX complite, otherwise Tx interrupt will wakeup us!
		while(ctrl.rfm12_state == STATE_TX);
		LED_GRN_OFF;


		/*    RX     */
		LED_RED_ON;
		while (rfm12_rx_status() != STATUS_COMPLETE);
		rfm12_rx_clear ();	





		/*    SLEEP     */
		rfm12_data(RFM12_CMD_CFG | RFM12_BASEBAND | RFM12_XTAL_12PF);
		rfm12_data(CLEAR_FIFO);
		rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT);
		LED_RED_OFF;
		go_sleep();

	}
	return 0;
}

