#include <avr/io.h>
#include "type.h"
#include "1wire.h"
#include "uart.h"
#include "delay.h"


//******************************************************************************
//  ������� �������� �������� �� 4 �� 65000 ������
//******************************************************************************
/*void __delay_cycles(WORD __count)
{
        __count >>= 2;
       __asm__ volatile (
		"1: sbiw %0,1" "\n\t"
		"brne 1b"
		: "=w" (__count)
		: "0" (__count)
        );
}*/


//******************************************************************************
//  ������� �������� CRC ��� �������� {10011001} (�������� iButton)
//******************************************************************************
BYTE OneWire_CRC_calc(BYTE *data, BYTE len)
{//uint8_t _crc_ibutton_update(uint8_t crc, uint8_t data)
BYTE j, i, crc = 0;

    for(j = 0; j < len; j ++)
    {
        crc = crc ^ data[j];
        for (i = 0; i < 8; i++)
        {
            if (crc & 0x01)
            crc = (crc >> 1) ^ 0x8C;
            else
            crc >>= 1;
        }
    }
return crc;
}

//*****************************************************************************
// ��������� reset �� ����, ��������� ����������� ����������
// ����������
// 0 - ���������� ����������������,
// 1 - ��� �������� preset
// 2 - ���� ��������� � �� ������������ � HIGH
BYTE OneWire_ResetDevice(void)
{
  BYTE tcnt;
  if(!(PIN_TM & (1 << TM))) return 2; // ���� ���������

  // ������� ������
  SETBIT(DDR_TM,  TM);                    // �����
  CLRBIT(PORT_TM, TM);                    // ������ �������
  __delay_cycles(clkMhz * 500);           // 500 ��� ������� ������
  CLRBIT(DDR_TM,  TM);                    // ����
  SETBIT(PORT_TM, TM);                    // c ���������

  // �������������� �������� �����������
  tcnt = 0;
  __delay_cycles(clkMhz * 10);
  while(PIN_TM & (1 << TM))
  {
  __delay_cycles(clkMhz * 10);
  tcnt ++;
  // ���� ��� - �������
  if(tcnt > 10) return 1;
  }

  // �������������� ���������� (����������� ����� � HIGH)
  tcnt = 0;
  __delay_cycles(clkMhz * 10);
  while(!(PIN_TM & (1 << TM)))
  {
  __delay_cycles(clkMhz * 20);
  tcnt ++;
  // ���� ����� ��� ��� � ������ ������ - ����� ��
  if(tcnt > 15) return 2;
  }
  return 0;
}



//*******************************************************************************
// ����� ���� � ���� (�������)
void OneWire_WriteByte(BYTE Data)
{
    BYTE tcnt;
  __delay_cycles(clkMhz * 20);

  for(tcnt = 0; tcnt < 8; tcnt ++)
  {
    if(!(Data & (1 << tcnt)))
    {
    // ���� 0 ������������ ������ ����� + 60 ��� + �������� �����
      SETBIT(DDR_TM,  TM);                    // �����
      CLRBIT(PORT_TM, TM);                    // ������ �������
      __delay_cycles(clkMhz * 45);
      SETBIT(PORT_TM, TM);                    // ������� �������
    }
    else
    {
    // ���� 1 ������������ ������ ����� + 15 ��� + �������� ����� + 45 ���
      SETBIT(DDR_TM,  TM);                    // �����
      CLRBIT(PORT_TM, TM);                    // ������ �������
      __delay_cycles(clkMhz * 10);
      SETBIT(PORT_TM, TM);                    // ������� �������
      __delay_cycles(clkMhz * 45);
    }
  __delay_cycles(clkMhz * 20);
  }
}



//****************************************************************************
// ������ len ���� � �����, ����������
void OneWire_ReadData(BYTE * Data, BYTE len)
{
    BYTE i, temp, tcnt;
  // ������ ������
  for(i = 0; i < len; i++) // �������� ����
  {
    temp = 0;

    for(tcnt = 0; tcnt < 8; tcnt ++)          // ������� ����
    {
    // ���� ����� 5 ���
      SETBIT(DDR_TM,  TM);                    // �����
      CLRBIT(PORT_TM, TM);                    // ������ �������
      __delay_cycles(clkMhz * 5);
    // ������������� �� ���� � ���������
      CLRBIT(DDR_TM,  TM);                    // ����
      SETBIT(PORT_TM, TM);                    // ��������
    // ���� 15 ���
      __delay_cycles(clkMhz * 15);
    // ������ ������
      temp >>=1;                              // �������� ���.������
      if(PIN_TM & (1 << TM)) temp |= (1 << 7);// 1
    // ���������� �����������
    __delay_cycles(clkMhz * 45);
    }
    Data[i] = temp;                           // ��������� ���������� ���������
  }
}



//*******************************************************************
// ������ ����������� � DS1820
void ReadTempr(unsigned char * tmpr)
{
 BYTE DS_Buff[9];
 BYTE res = 1;

/*
    * �������� ������� ������ � ��������� ����� ����������.
    * �������� ������� Skip ROM [CCh].
    * �������� ������� Convert T [44h].
    * ��������� �������� ������� 750��.
    * �������� ������� ������ � ��������� ����� ����������.
    * �������� ������� Skip ROM [CCh].
    * �������� ������� Read Scratchpad [BEh].
    * ������ ������ �� �������������� ��� (8 ����) � CRC.
    * ��������� CRC, � ���� ������ ������� �����, ��������� �����������.
*/
    OneWire_ResetDevice();
    OneWire_WriteByte(0xcc);
    OneWire_WriteByte(0x44);
		// wait 850 ms
		int i;
		for (i = 0; i < 140; i++) {
			__delay_cycles (50000);
		}
    OneWire_ResetDevice();
    OneWire_WriteByte(0xcc);
    OneWire_WriteByte(0xbe);

    OneWire_ReadData(DS_Buff, 9);
		tmpr[0] = DS_Buff[1];
		tmpr[1] = DS_Buff[0];
    if (OneWire_CRC_calc(DS_Buff, 9) != 0) {
			res = 0;
    }
		CLRBIT(DDR_TM,  TM);
		//SETBIT(PORT_TM, TM);
	return res;
}

//
//
//
//
//
//
//
////******************************************************************************
////          ������� ������ ����� iButton
//// input:
//// *code - ��������� �� ����� ��� ����� 8 ����
//// TM - ��� �������� ����� ������ ������ ������������ ������
//// output:
//// 0 - ��;
//// 1 - ��� �������� �����������;
//// 2 - ������������ ��������� ����� (��?)
//// 3 - ������ CRC
////******************************************************************************
//BYTE ReadKey(BYTE *code, BYTE TM)
//{
//BYTE tcnt, Data, i;
//
//  // ������� ������
//  SETBIT(DDR_TM,  TM);                    // �����
//  CLRBIT(PORT_TM, TM);                    // ������ �������
//  __delay_cycles(clkMhz * 500);           // 500 ��� ������� ������
//  CLRBIT(DDR_TM,  TM);                    // ����
//  SETBIT(PORT_TM, TM);                    // c ���������
//
//  // �������������� �������� �����������
//  tcnt = 0;
//  __delay_cycles(clkMhz * 10);
//  while(PIN_TM & (1 << TM))
//  {
//  __delay_cycles(clkMhz * 10);
//  tcnt ++;
//  // ���� ��� - �������
//  if(tcnt > 10) return 1;
//  }
//
//  // �������������� ���������� �����
//  tcnt = 0;
//  __delay_cycles(clkMhz * 10);
//  while(!(PIN_TM & (1 << TM)))
//  {
//  __delay_cycles(clkMhz * 20);
//  tcnt ++;
//  // ���� ����� ��� ��� � ������ ������ - ����� ��
//  if(tcnt > 15) return 2;
//  }
//
//  __delay_cycles(clkMhz * 20);
//  Data = 0x33;
//  // ���� �� - �������� �������
//  for(tcnt = 0; tcnt < 8; tcnt ++)
//  {
//    if(!(Data & (1 << tcnt)))
//    {
//    // ���� 0 ������������ ������ ����� + 60 ��� + �������� �����
//      SETBIT(DDR_TM,  TM);                    // �����
//      CLRBIT(PORT_TM, TM);                    // ������ �������
//      __delay_cycles(clkMhz * 45);
//      SETBIT(PORT_TM, TM);                    // ������� �������
//    }
//    else
//    {
//    // ���� 1 ������������ ������ ����� + 15 ��� + �������� ����� + 45 ���
//      SETBIT(DDR_TM,  TM);                    // �����
//      CLRBIT(PORT_TM, TM);                    // ������ �������
//      __delay_cycles(clkMhz * 5);
//      SETBIT(PORT_TM, TM);                    // ������� �������
//      __delay_cycles(clkMhz * 45);
//    }
//  __delay_cycles(clkMhz * 10);
//  }
//
//
//
//  // �������� CRC
//  if (CRC_calc(code) == 0)
//  {
//    return 0;
//  }
//
//return 3;
//  // �������
//}
