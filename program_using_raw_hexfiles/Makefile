
TARGET = atmega328
MCU_TARGET = atmega328p
AVR_FREQ = 8000000L 
LDSECTION = -Wl,--section-start=.text=0x4000 -Wl,--section-start=.jump_to=0x0000 
#LDSECTION  = --section-start=.text=0x2000 --section-start=.version=0x1000

COMP_PATH = /Applications/Arduino.app/Contents//Resources/Java/hardware/tools/avr/bin/

CC         = avr-gcc
CXX        = avr-gcc
OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump
SIZE           = avr-size

OPTIMIZE   = -Os
CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) -DF_CPU=$(AVR_FREQ) $(DEFS)
CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>4' '-DNUM_LED_FLASHES=1' '-DWATCHDOG_MODS' -DBAUD_RATE=57600 -DDOUBLE_SPEED
CFLAGS += -fno-common -ffunction-sections -fdata-sections
#CFLAGS += -nostartfiles
CPPFLAGS = $(CFLAGS)
LDFLAGS       = $(LDSECTION) -Wl,--undefined=keep_me -Wl,--gc-sections

SRCS ?= main.cpp
PROG := $(firstword $(SRCS:.cpp=))
OBJ := $(SRCS:.cpp=.o)


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
	$(OBJCOPY) -j .text -j .data -j .jump_to --set-section-flags .jump_to=alloc,load -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -j .jump_to --set-section-flags .jump_to=alloc,load -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -j .jump_to --set-section-flags .jump_to=alloc,load -O binary $< $@

AVRDUDE_DIR=/Users/joeschaack/Documents/git/panstamp_stuffs/optiboot/optiboot/bootloaders/optiboot/avrdude-6.2/
prog: all
	$(AVRDUDE_DIR)/avrdude -C $(AVRDUDE_DIR)/avrdude.conf -c arduino -p atmega328p -P /dev/tty.usbserial-A900fKM4 -b 115200 -v -Uflash:w:$(PROG).hex:i

