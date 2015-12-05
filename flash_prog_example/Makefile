all:
	avr-gcc -g -Wall -Os -mmcu=atmega328p -DF_CPU=8000000L   '-DMAX_TIME_COUNT=F_CPU>>4' '-DNUM_LED_FLASHES=1' '-DWATCHDOG_MODS' -DBAUD_RATE=57600 -DDOUBLE_SPEED   -c -o main.o main.c
	avr-gcc -g -Wall -Os -mmcu=atmega328p -DF_CPU=8000000L   '-DMAX_TIME_COUNT=F_CPU>>4' '-DNUM_LED_FLASHES=1' '-DWATCHDOG_MODS' -DBAUD_RATE=57600 -DDOUBLE_SPEED -Wl,--section-start=.text=0x0000 -o main.elf main.o 
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	avr-size main.elf
	rm main.elf main.o


AVRDUDE_DIR=/Users/joeschaack/Documents/git/panstamp_stuffs/optiboot/optiboot/bootloaders/optiboot/avrdude-6.2/
prog: all
	$(AVRDUDE_DIR)/avrdude -C $(AVRDUDE_DIR)/avrdude.conf -c arduino -p atmega328p -P /dev/tty.usbserial-A900fKM4 -b 115200 -v -Uflash:w:main.hex:i

