#ifndef MEMORY_H_INCLUDED
#define MEMORY_H_INCLUDED

#define DDR_TM    DDRD
#define PORT_TM   PORTD
#define PIN_TM    PIND
#define TM        0
#define clkMhz    8

BYTE OneWire_ResetDevice(void);                 // ������ ����� �� ���� � ������� ����������� ����������
void OneWire_WriteByte(BYTE);                   // ����� ���� � ����
void OneWire_ReadData(BYTE*, BYTE);             // ������ len ���� � �����, ����������
BYTE OneWire_CRC_calc(BYTE *, BYTE);            // ������������ CRC ��� ������
void ReadTempr(unsigned char *);

#endif
