# author Eugene Samoylov ghelius@gmail.com

MCU=atmega48

CC=avr-gcc
OBJCOPY=avr-objcopy
# optimize for size:
CFLAGS=-g -mmcu=$(MCU) -Wall -Wstrict-prototypes -Os -mcall-prologues


.PHONY: all client server
all: client.hex server.hex
	@echo "done"

client: client.hex
	@echo "done"

server: server.hex
	@echo "done"


client.hex : client.elf
	$(OBJCOPY) -R .eeprom -O ihex client.elf client.hex
	avr-size client.elf

client.elf : client.o rfm12.o delay.o uart.o
	$(CC) $(CFLAGS) -o client.elf client.o rfm12.o delay.o uart.o

server.hex : server.elf
	$(OBJCOPY) -R .eeprom -O ihex server.elf server.hex
	avr-size server.elf

server.elf : server.o rfm12.o delay.o uart.o
	$(CC) $(CFLAGS) -o server.elf server.o rfm12.o delay.o uart.o



# Обычное (generic) правило для компилирования файлов на языке C:
.c.o:
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o *.map *.elf *.hex



