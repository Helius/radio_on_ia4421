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


unsigned int SPI_MasterTransmit(char cData_high, char cData_low)
{
char t1;
char t2;
unsigned int temp = 0x0000;    
  PORTB &= ~(1 << PORTB2);                        // Вывод SS переводим в низкий уровень
  SPDR = cData_high;                              // Запуск передачи байта
  while(!(SPSR & (1<<SPIF)));                     // Ожидание завершения передачи данных 
  t1 = SPDR;
  SPDR = cData_low;                               // Запуск передачи байта
  while(!(SPSR & (1<<SPIF)));                     // Ожидание завершения передачи данных 
  t2 = SPDR;
  PORTB |= (1 << PORTB2);                         // Вывод SS переводим в высокий уровень
  temp = t1;
  temp <<= 8;                                     // сдвигаем влево на 1 и присваиваем
  temp |= t2;    
  return (temp);     
}


/* helius disable wakeup and clock sourse, set timeout, enable timer and disable clock source*/
void rfm12_setup_wakeup_timer (uint16_t sec)
{
//		rfm12_data (0x8201);											// Отключить таймер
//		rfm12_data (0xEA00 | sec);								// T = 1.03 * 2^10 * period + 0.5 мс
//		rfm12_data (0x820B);											// Включить таймер
}

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
	DDRC |= (1<<PD2);
	PORTC = 1<<PD2;
}
void clear_gpio (void)
{
}


int adc_get () {
	ADCSRA = 0;
	ADMUX = (1<<REFS1) | (1<<REFS0) | (1<<MUX0) | (1<<MUX1) | (1<<ADLAR);
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS0);
	ADCSRA |= 1<<ADSC;
	//DIDR0 = (1<<ADC3D);
	while (ADCSRA & (1<<ADIF));
	return ADCH;
}

void adc_off () {
	ADCSRA = 0;
	DDRC &=~(1<<2); 
	PORTC &=~(1<<2); 
}

int wake_up_me;

#define STATE_TX 3
uint8_t payload [8];
int main (void)
{
	init_gpio ();
	sei();
	rfm12_init ();
	rfm12_set_wakeup_timer(0xEA00 | 5);
	uart_init();
	
// dont'work with it!!!!
	LED_GRN_ON;
	__delay_cycles (65000);
	__delay_cycles (65000);
	__delay_cycles (65000);
	LED_GRN_OFF;
	
	while (1) {
			
		LED_GRN_ON;
		//get temperature
		__delay_cycles (25000);
		payload[0] = 0xa0;
		payload[1] = 0xa1;
		payload[2] = adc_get();
		adc_off();

		rfm12_tx (3, 0, payload);
		rfm12_tick();
		//wait while TX complite, otherwise Tx interrupt will wakeup us!
		while(ctrl.rfm12_state == STATE_TX);
		LED_GRN_OFF;

		LED_RED_ON;
		while (rfm12_rx_status() != STATUS_COMPLETE);
		rfm12_rx_clear ();	
#include "rfm12/src/include/rfm12_hw.h"
#include "rfm12/src/include/rfm12_core.h"
	/*		__delay_cycles (65000);
			__delay_cycles (65000);
			__delay_cycles (65000);*/
		rfm12_data(RFM12_CMD_CFG | RFM12_BASEBAND | RFM12_XTAL_12PF);
		rfm12_data(CLEAR_FIFO);
		//__delay_cycles (65000);
		rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT);
		LED_RED_OFF;
			go_sleep();
		rfm12_data(RFM12_CMD_CFG | RFM12_CFG_EL | RFM12_CFG_EF | RFM12_BASEBAND | RFM12_XTAL_12PF);
	}
	return 0;
}

