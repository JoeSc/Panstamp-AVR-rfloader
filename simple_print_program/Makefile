
TARGET = atmega328
MCU_TARGET = atmega328p
AVR_FREQ = 8000000L 
LDSECTION  = --section-start=.text=0x0000

CC         = avr-gcc
CXX        = avr-gcc
OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump
SIZE           = avr-size

OPTIMIZE   = -Os
CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) -DF_CPU=$(AVR_FREQ) $(DEFS)
CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>4' '-DNUM_LED_FLASHES=1' '-DWATCHDOG_MODS' -DBAUD_RATE=57600 -DDOUBLE_SPEED
CFLAGS += -fno-common -ffunction-sections -fdata-sections
CPPFLAGS = $(CFLAGS)
LDFLAGS       = -Wl,$(LDSECTION) -Wl,--gc-sections

SRCS ?= main.c
PROG := $(firstword $(SRCS:.c=))
OBJ := $(SRCS:.c=.o)


all: $(PROG).elf $(PROG).hex $(PROG).lst

%.o: %.cpp
	$(CXX) $(CPPFLAGS) -c $<

%.elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)
	for x in *.o; do $(SIZE) $$x | tail -1; done

	$(SIZE) $@

clean:
	rm -rf *.o *.elf *.lst *.map *.sym *.lss *.eep *.srec *.bin *.hex

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

AVRDUDE_DIR=/Users/joeschaack/Documents/git/panstamp_stuffs/optiboot/optiboot/bootloaders/optiboot/avrdude-6.2/
prog: all
	$(AVRDUDE_DIR)/avrdude -C $(AVRDUDE_DIR)/avrdude.conf -c arduino -p atmega328p -P /dev/tty.usbserial-A900fKM4 -b 115200 -v -Uflash:w:$(PROG).hex:i

