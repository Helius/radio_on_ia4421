#ifndef MEMORY_H_INCLUDED
#define MEMORY_H_INCLUDED

#define DDR_TM    DDRD
#define PORT_TM   PORTD
#define PIN_TM    PIND
#define TM        0
#define clkMhz    8

BYTE OneWire_ResetDevice(void);                 // делает сброс на шине и ожидает присутствия устройства
void OneWire_WriteByte(BYTE);                   // пишет байт в шину
void OneWire_ReadData(BYTE*, BYTE);             // читает len байт в буфер, возвращает
BYTE OneWire_CRC_calc(BYTE *, BYTE);            // подсчитывает CRC для буфера
void ReadTempr(unsigned char *);

#endif
