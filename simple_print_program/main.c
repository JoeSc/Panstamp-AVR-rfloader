
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
	while(1) {
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
