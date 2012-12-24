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
/*
********************************************************************************
*                       Инициализация RF-приемопередатчика
********************************************************************************
*//*
void init_RFM12B(void)
{
	
  SPI_MasterTransmit(0x80, 0xD7);   // el, ef, 433band, 12.0 pF
  SPI_MasterTransmit(0x82, 0x08);   // !er, !ebb, !et, !es, ex, !eb, !ew, !dc
////////////////////////////////////////////////////////////////////////////////
//                               Частота 
////////////////////////////////////////////////////////////////////////////////
  if ((CHANNEL == 0) || (CHANNEL == 10))
  {SPI_MasterTransmit(0xA5, 0x0C);} // 433.23MHz  №0
  else if ((CHANNEL == 1) || (CHANNEL == 11))
  {SPI_MasterTransmit(0xA5, 0x44);} // 433.37MHz  №1
  else if ((CHANNEL == 2) || (CHANNEL == 12))
  {SPI_MasterTransmit(0xA5, 0x7C);} // 433.51MHz  №2
  else if ((CHANNEL == 3) || (CHANNEL == 13))
  {SPI_MasterTransmit(0xA5, 0xB4);} // 433.65MHz  №3
  else if ((CHANNEL == 4) || (CHANNEL == 14))
  {SPI_MasterTransmit(0xA6, 0xEC);} // 433.79MHz  №4
  else if (CHANNEL == 16)
  {SPI_MasterTransmit(0xA6, 0x24);} // 433.93MHz  ///////// служебный //////////
  else if ((CHANNEL == 5) || (CHANNEL == 15))
  {SPI_MasterTransmit(0xA6, 0x5C);} // 434.07MHz  №5
  else if (CHANNEL == 6)
  {SPI_MasterTransmit(0xA6, 0x94);} // 434.21MHz  №6
  else if (CHANNEL == 7)
  {SPI_MasterTransmit(0xA6, 0xCC);} // 434.35MHz  №7
  else if (CHANNEL == 8)
  {SPI_MasterTransmit(0xA7, 0x04);} // 434.49MHz  №8
  else if (CHANNEL == 9)
  {SPI_MasterTransmit(0xA7, 0x3C);} // 434.63MHz  №9 

////////////////////////////////////////////////////////////////////////////////
//                               Скорость
////////////////////////////////////////////////////////////////////////////////  
  //SPI_MasterTransmit(0xC6, 0xC6); // 0.6   kbps
  //SPI_MasterTransmit(0xC6, 0xA2); // 1.2   kbps
  //SPI_MasterTransmit(0xC6, 0x8E); // 2.4   kbps
  SPI_MasterTransmit(0xC6, 0x47);   // 4.8   kbps
  //SPI_MasterTransmit(0xC6, 0x22); // 9.6   kbps
  //SPI_MasterTransmit(0xC6, 0x10); // 19.2  kbps
  //SPI_MasterTransmit(0xC6, 0x07); // 38.4  kbps
  //SPI_MasterTransmit(0xC6, 0x03); // 76.8  kbps
  //SPI_MasterTransmit(0xC6, 0x01); // 115.2 kbps
  //SPI_MasterTransmit(0xC6, R   ); // BR    kbps
////////////////////////////////////////////////////////////////////////////////
//                       Полоса приема, чувствительность
////////////////////////////////////////////////////////////////////////////////   
  //SPI_MasterTransmit(0x90, 0xC0); // p16(VDI), d1 d0 (FAST), BW = 67 kHz, !g1 !g0 (0dBm), !r2 !r1 !r0(-103dBm),
  SPI_MasterTransmit(0x90, 0xA0);   // p16(VDI), d1 d0 (FAST), BW = 134kHz, !g1 !g0 (0dBm), !r2 !r1 !r0(-103dBm),
  //SPI_MasterTransmit(0x90, 0x80); // p16(VDI), d1 d0 (FAST), BW = 200kHz, !g1 !g0 (0dBm), !r2 !r1 !r0(-103dBm),
  //SPI_MasterTransmit(0x90, 0x60); // p16(VDI), d1 d0 (FAST), BW = 270kHz, !g1 !g0 (0dBm), !r2 !r1 !r0(-103dBm),
////////////////////////////////////////////////////////////////////////////////
//                              Fifo, power
////////////////////////////////////////////////////////////////////////////////  
  SPI_MasterTransmit(0xC2, 0xAC);   // AL, !ml, DIG, DQD4
  SPI_MasterTransmit(0xCA, 0x81);   // FIFO8, SYNC, !ff, dr
  SPI_MasterTransmit(0xCE, 0xD4);   // SYNC = 2DD4;
  SPI_MasterTransmit(0xC4, 0x83);   // @PWR, NO RSTRIC, !st, !fi, OE, EN
////////////////////////////////////////////////////////////////////////////////
//                               Девиация
//////////////////////////////////////////////////////////////////////////////// 
  //SPI_MasterTransmit(0x98, 0x10); // !mp, 30kHz, 0dBm
  //SPI_MasterTransmit(0x98, 0x20); // !mp, 45kHz, 0dBm
  SPI_MasterTransmit(0x98, 0x30);   // !mp, 60kHz, 0dBm
  //SPI_MasterTransmit(0x98, 0x50); // !mp, 90kHz, 0dBm
  //SPI_MasterTransmit(0x98, 0x70); // !mp, 120kHz, 0dBm
  //SPI_MasterTransmit(0x98, 0xB0); // !mp, 180kHz, 0dBm
  //SPI_MasterTransmit(0x98, pm  ); // !mp, dev_kHz, power_dBm
////////////////////////////////////////////////////////////////////////////////
//                             
//////////////////////////////////////////////////////////////////////////////// 
  //SPI_MasterTransmit(0xCC, 0x67);   // OB1, OBO, !lpx, !ddy, DDIT, BWo
  SPI_MasterTransmit(0xCC, 0x77);   // low_power RFM12
  SPI_MasterTransmit(0xE0, 0x00);   // NOT USE
  SPI_MasterTransmit(0xC8, 0x00);   // NOT USE
////////////////////////////////////////////////////////////////////////////////
//                       Пороговый уровень  напряжения
////////////////////////////////////////////////////////////////////////////////
  //SPI_MasterTransmit(0xC0, 0x04);   // 2.65 В
  SPI_MasterTransmit(0xC0, 0x03);   // 2.55 В   
}
*/



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

#define STATE_TX 3
uint8_t payload [8];
int main (void)
{
	init_gpio ();
	sei();
	rfm12_init ();
	rfm12_set_wakeup_timer(0xEA00 | 5);
//	uart_init();
	
// dont'work with it!!!!
	LED_GRN_ON;
	__delay_cycles (65000);
	__delay_cycles (65000);
	__delay_cycles (65000);
	LED_GRN_OFF;
	
	while (1) {
		
		LED_RED_ON;
		//get temperature
		//__delay_cycles (65000);
		__delay_cycles (25000);
		payload[0] = 0xF0;
		payload[1] = 0xF1;
		payload[2] = adc_get();
		//__delay_cycles (65000);
		adc_off();

		rfm12_tx (3, 0, payload);
		rfm12_tick();
		__delay_cycles (65000);
		__delay_cycles (20000);
		while(ctrl.rfm12_state == STATE_TX);
		LED_RED_OFF;
		go_sleep();
	}
	return 0;
}

