#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "rfm12.h"
#include "board.h"
#include "delay.h"
#include "uart.h"

#define true 1
#define false 0

#define LED_GRN_ON PORTD |=1<<LED_GRN;
#define LED_GRN_OFF PORTD &=~(1<<LED_GRN);
#define LED_RED_ON PORTD |=1<<LED_RED;
#define LED_RED_OFF PORTD &=~(1<<LED_RED);
#define TST_BIT(reg,bit) (reg&(1<<bit))


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

void init_led (void)
{
	DDRD |= (1<<LED_GRN) | (1<<LED_RED);       //set led pin OUTPUT
	PORTD &= ~((1<<LED_GRN) | (1<<LED_RED));   //set led pin LOW
	//PORTD = 1<<PD2;
}
void clear_led (void)
{
	DDRD &= ~((1<<LED_GRN) | (1<<LED_RED));    //set led pin INPUT
	PORTD &= ~((1<<LED_GRN) | (1<<LED_RED));   //disable pullups
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

int button_pressed (void) {
	PORTC |= 1<<BTN_TMP; // set pullups
	__delay_cycles (500);
	int res = !(PINC & (1<<BTN_TMP));
	PORTC &= ~(1<<BTN_TMP);
	return res;
}

int wait_receive (void) {
	int cnt = 30;
	while ((rfm12_rx_status() != STATUS_COMPLETE) && (cnt)) {
		__delay_cycles (50000);
		cnt--;
	}
	return cnt;
}

#define STATE_TX 3
int wait_transmit (void) {
	int cnt = 30;
	while ((ctrl.rfm12_state == STATE_TX) && (cnt)) {
		__delay_cycles (50000);
		cnt--;
	}
	return cnt;
}

int wake_up_me;

uint8_t payload [8];
uint8_t Ubatt;
uint8_t adc_idle = 0;
int main (void)
{
	sei();
	rfm12_init ();
	rfm12_set_wakeup_timer(0xEA00 | 60);
	uart_init();
	
// dont'work with it!!!!
	init_led();
	LED_GRN_ON;
	__delay_cycles (50000);
	__delay_cycles (50000);
	__delay_cycles (50000);
	__delay_cycles (50000);
	LED_GRN_OFF;
	clear_led();
	
	while (1) {
			

		// get battery voltage
		if (!adc_idle--) {
			adc_idle = 10;
			//get temperature
			DDRC |= (1<<2);
			PORTC |= (1<<2);
			__delay_cycles (5000);
			adc_get();
			unsigned int tmp = adc_get();
			Ubatt = (tmp*17)/100; //value 31 = 3.1V
			adc_off();
			DDRC &=~(1<<2); 
			PORTC &=~(1<<2); 
		}

		ReadTempr(payload);
		payload[2] = Ubatt;

		/*    TX     */
#include "rfm12/src/include/rfm12_hw.h"
#include "rfm12/src/include/rfm12_core.h"
		rfm12_data(RFM12_CMD_CFG | RFM12_CFG_EL | RFM12_CFG_EF | RFM12_BASEBAND | RFM12_XTAL_12PF);


		while (button_pressed()) { // ping mode
			init_led();
			LED_RED_ON;
			rfm12_tx (4, 1, "ping");
			rfm12_tick();
			wait_transmit();
			LED_RED_OFF;

			if (wait_receive()) {
				LED_GRN_ON;
				__delay_cycles (50000);
				__delay_cycles (50000);
				__delay_cycles (50000);
				__delay_cycles (50000);
				__delay_cycles (50000);
				__delay_cycles (50000);
				__delay_cycles (50000);
				__delay_cycles (50000);
				__delay_cycles (50000);
				__delay_cycles (50000);
				__delay_cycles (50000);
				__delay_cycles (50000);
				rfm12_rx_clear ();
				LED_GRN_OFF;
			}
		}
		clear_led();

		rfm12_tx (3, 0, payload);
		rfm12_tick();
		//wait while TX complite, otherwise Tx interrupt will wakeup us!
		wait_transmit ();


		/*    RX     */
		
/*	Rx disable	
		wait_receive();
		rfm12_rx_clear ();	*/


		/*    SLEEP     */
		rfm12_data(RFM12_CMD_CFG | RFM12_BASEBAND | RFM12_XTAL_12PF);
		rfm12_data(CLEAR_FIFO);
		rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT);
		//LED_RED_OFF;
		go_sleep();

	}
	return 0;
}

