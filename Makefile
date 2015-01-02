MCU = atmega1284p

CFLAGS = -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wextra -Wall -Werror -std=gnu99 -DF_CPU=16000000UL

PROGRAMMER = arduino

PORT = /dev/ttyUSB0

SRCS = mega6502.c
OBJS = $(SRCS:.c=.o)

TARGET = avr

all: flash

# External crystal/resonator high-freq, startup time 258CK + 64ms
# Preserve EEPROM through chip erase cycle
fuse:
#	sudo avrdude -c $(PROGRAMMER) -p $(MCU) -U lfuse:w:0xde:m -U hfuse:w:0x99:m -U efuse:w:0xff:m
	sudo avrdude -c $(PROGRAMMER) -p $(MCU) -U lfuse:w:0xde:m -U hfuse:w:0x91:m -U efuse:w:0xff:m

build: clean
	avr-gcc $(CFLAGS) -g -mmcu=$(MCU) -c $(SRCS)
	avr-gcc $(CFLAGS) -mmcu=$(MCU) -o $(TARGET).elf $(OBJS)
	avr-objcopy -j .text -j .data -O ihex $(TARGET).elf $(TARGET).hex

flash: build
	# avrdude -C/home/dfc/bin/arduino-1.5.8/hardware/tools/avr/etc/avrdude.conf -v -p $(MCU) -c $(PROGRAMMER) -P/dev/ttyUSB1 -b115200 -D -U flash:w:$(TARGET).hex:i 
	/usr/share/arduino/hardware/tools/avrdude -C/usr/share/arduino/hardware/tools/avrdude.conf -v -v -v -v -patmega1284p -c$(PROGRAMMER) -P$(PORT) -b115200 -D -Uflash:w:$(TARGET).hex:i

clean:
	rm *.o
	rm *.elf
	rm *.hex
