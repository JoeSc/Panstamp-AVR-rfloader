
#include "optiboot.h"
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

#define LED_DDR  DDRB
#define LED_PORT PORTB
#define LED_PIN  PINB
#define LED      PINB5

void putch(char);
void flash_led(uint8_t);

static int uart_putchar(char c, FILE *stream);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL,
        _FDEV_SETUP_WRITE);
static int
uart_putchar(char c, FILE *stream)
{
    if (c == '\n')
        uart_putchar('\r', stream);
    putch(c);
    return 0;
}








const char flash_buffer[SPM_PAGESIZE] __attribute__ (( aligned(SPM_PAGESIZE) )) PROGMEM= {
  "0123456789ABCDEFGHIJKLMNOPQRSTUVW"
};

void setup() {
  int i;
  uint8_t c;
  uint16_t w;
  uint8_t ram_buffer[SPM_PAGESIZE];

  // Print current flash buffer content
  printf("Current flash contents:\n");
  for (i=0;i<SPM_PAGESIZE;i++) {
    c = pgm_read_byte(&flash_buffer[i]);
    printf("%02x ", c);
  }
  printf("\n");

  // Erasing FLASH page
  printf("Erasing\n");
  printf("flash @ 0x%04x = 0x%02x\n",0x80, pgm_read_byte(0x80));
  optiboot_page_erase((uint16_t)(void*) &flash_buffer[0]);
  printf("flash @ 0x%04x = 0x%02x\n",0x80, pgm_read_byte(0x80));
  ram_buffer[0] = 0xde;
  ram_buffer[1] = 0xad;
  ram_buffer[2] = 0xbe;
  ram_buffer[3] = 0xef;

  // Copy ram buffer to temporary flash buffer
  printf("Writing to temporary flash buffer\n");
  for (i=0;i<SPM_PAGESIZE;i++) {
    if (i % 2 == 0) { // we must write WORDs
      w = ram_buffer[i];
    } else {
      w += (ram_buffer[i] << 8);
      //optiboot_page_fill((uint16_t)(void*) &flash_buffer[i],w);
    }
  }
  
  printf("flash @ 0x%04x = 0x%02x\n",0x80, pgm_read_byte(0x80));
  // Writing temporary buffer to FLASH
  printf("Writing buffer to flash\n");
  //optiboot_page_write((uint16_t)(void*) &flash_buffer[0]);

  printf("flash @ 0x%04x = 0x%02x\n",0x80, pgm_read_byte(0x80));
  printf("Write done, thank you!\n");
  printf("Now you can reset or power cycle board and check for new contents!\n");

  printf("flash @ 0x%04x = 0x%02x\n",0x80, pgm_read_byte(0x80));
}


int main(void)
{
//--	uint8_t ch;
//--	ch = MCUSR;
	MCUSR = 0;

	WDTCSR |= _BV(WDCE) | _BV(WDE);
	WDTCSR = 0;

//--	// Check if the WDT was used to reset, in which case we dont bootload and skip straight to the code. woot.
//--	if (! (ch &  _BV(EXTRF))) // if its a not an external reset...
//--		app_start();  // skip bootloader

	UCSR0A = (1<<U2X0); //Double speed mode USART0
	UBRR0L = (uint8_t)(F_CPU/(BAUD_RATE*8L)-1);
	UBRR0H = (F_CPU/(BAUD_RATE*8L)-1) >> 8;

	UCSR0B = (1<<RXEN0) | (1<<TXEN0);
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);

	/* Enable internal pull-up resistor on pin D0 (RX), in order
	to supress line noise that prevents the bootloader from
	timing out (DAM: 20070509) */
	DDRD &= ~_BV(PIND0);
	PORTD |= _BV(PIND0);


	/* set LED pin as output */
	LED_DDR |= _BV(LED);


 	//flash_led(2);

	int i = '0';
    stdout = &mystdout;
    printf("Hello!!");
    setup();
	while(i<'9') {
		putch('T');
		putch('a');
		putch('z');
		putch('T');
		putch(':');
		putch(i++);
		putch('\r');
		putch('\n');
		if (i > '~')
			i = '0';
		_delay_ms(100);
	}
    return 0;
}

void flash_led(uint8_t count)
{
	while (count--) {
		LED_PORT |= _BV(LED);
		_delay_ms(100);
		LED_PORT &= ~_BV(LED);
		_delay_ms(100);
	}
}

void putch(char ch)
{
	while (!(UCSR0A & _BV(UDRE0)));
	UDR0 = ch;
}

