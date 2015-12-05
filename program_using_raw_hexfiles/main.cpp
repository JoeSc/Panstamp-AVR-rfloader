
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "optiboot.h"

#define LED_DDR  DDRB
#define LED_PORT PORTB
#define LED_PIN  PINB
#define LED      PINB5

uint16_t getTargetAddress(uint8_t *line);
void putch(char);
uint8_t get_pkt(void);
char getch(void);
void flash_led(uint8_t);

#ifdef __cplusplus
extern "C"{
 FILE * uart_str;
 }
#endif

//static int uart_putchar(char c, FILE *stream);
//static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL,
//        _FDEV_SETUP_WRITE);


static int
uart_putchar(char c, FILE *stream)
{
    if (c == '\n')
        uart_putchar('\r', stream);
    putch(c);
    return 0;
}


void jump_to_wbl(void) __attribute__ ((naked)) __attribute__ ((section (".jump_to")));
int main(void) __attribute__ ((OS_main)) __attribute__ ((section (".init9")));

//const char user_pgm_reset_vec_jump[SPM_PAGESIZE] __attribute__ (( aligned(SPM_PAGESIZE) )) PROGMEM= { 0x0};
const char user_pgm_reset_vec_jump[SPM_PAGESIZE] __attribute__((section(".progmem.data"))) __attribute__ (( aligned(SPM_PAGESIZE) ))  = {0x0} ;//= { 0x0};

void jump_to_wbl()
{
    asm volatile (
      ".global keep_me\n"
      "keep_me:\n"
      "   jmp    __vectors\n"
    );
}

void jumpToUserCode(void)
{
//  // Exit upgrade mode
//  uint8_t state = (uint8_t)SYSTATE_RESTART;
//  TRANSMIT_SWAP_STATUS_STATE(state);
    asm volatile (
      /* "   rjmp    1f\n"*/
      "   jmp    .progmem.data\n"
      "1:\n"
    );
}

uint8_t pkt[21];

const uint16_t USER_PGM_START   = 0x0000;
const uint16_t USER_PGM_END     = 0x1000;


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

uart_str = fdevopen(uart_putchar, NULL);
stdout = stdin = uart_str;
    //stdout = &mystdout;
    printf("Flash Test\n");

	putch('F');
	putch('l');
	putch('a');
	putch('s');
	putch('h');
	putch('_');
	putch('_');
	putch('_');
	putch('T');
	putch('E');
	putch('S');
	putch('T');
	putch('\n');

	uint8_t ch,i;
	uint16_t address = USER_PGM_START;
	uint16_t last_address = 0;
	uint8_t firstLine = 1;


    uint16_t pgm_address = 0x0;
    uint8_t first_page_data[SPM_PAGESIZE];
    uint8_t first_page = 1;

    printf("tttttt = 0x%04x\n",RAMEND);

    if (*((uint8_t *)(RAMEND-52)) == 0x32)
        printf("Got 32 at RAMEND-52");

	while(1) {
        putch('r');
		ch = getch();
        if (ch == 'J') {
            jumpToUserCode();
        }

		if (ch == ':') {
			if(get_pkt())
				putch('E');
			else 
			{
				putch('S');
        		uint16_t addrFromHexFile = getTargetAddress(pkt);
				if (pkt[3] == 0)
				{
                    if (addrFromHexFile != pgm_address)
                    {
                        putch('X');
                        continue;
                    }
                    if (first_page)
                    {
                        putch('C');
					    for(i=0; i < 16; i++)
                            first_page_data[pgm_address++] = pkt[4+i];

                        /* If we finished getting the first page data */
                        if( pgm_address == SPM_PAGESIZE ) 
                        {
                            putch('P');
                            // First put new reset vec into the user_pgm_reset_vec_jump area
							optiboot_page_erase((uint16_t)(void*) &user_pgm_reset_vec_jump[0]);
                            for(i=0; i<4; i+=2)
						        optiboot_page_fill((uint16_t)(void*) &user_pgm_reset_vec_jump[i], (first_page_data[i+1] <<8 ) + first_page_data[i]);
							optiboot_page_write((uint16_t)(void*) &user_pgm_reset_vec_jump[0]);

                            for(i=0; i<4; i++)
                            {
                                printf("Read 0x%02x\n",pgm_read_byte(i));
                                first_page_data[i] = pgm_read_byte(i);
                            }

							optiboot_page_erase(0x0);
                            for(i=0; i<SPM_PAGESIZE; i+=2)
						        optiboot_page_fill(i, (first_page_data[i+1] <<8 ) + first_page_data[i]);
							optiboot_page_write(0x0);

                            first_page = 0;
                        }
                    }
                    else
                    {
                        /* If this is the first hexline from a page then erase it */
					    if((addrFromHexFile % SPM_PAGESIZE) == 0) {
                            putch('E');
                            optiboot_page_erase(addrFromHexFile);
                        }

					    for(i=0; i < 16; i+=2) {
                            pgm_address +=2;
					    	optiboot_page_fill(addrFromHexFile, (pkt[5 + i] << 8) + pkt[4 + i]);
					    	addrFromHexFile += 2;
					    }
	
					    last_address = addrFromHexFile;
					    if((addrFromHexFile % SPM_PAGESIZE) == 0) {
					    	putch('W');
					    	optiboot_page_write(addrFromHexFile - SPM_PAGESIZE);
					    }
                    }
				}
				else // Assume we are done here
				{
					// Round address down to last page if we didn't finish on a boundary
					if(last_address % SPM_PAGESIZE) {
						last_address = last_address - (last_address % SPM_PAGESIZE);
                        printf("WRiting page 0x%04x\n",last_address);
						optiboot_page_write(last_address);
					}
				}
			}
		}
	}
}

/* utility function to convert hex character representation to their nibble (4 bit) values */
uint8_t nibbleFromChar(char c)
{
	if(c >= '0' && c <= '9') return c - '0';
	if(c >= 'a' && c <= 'f') return c - 'a' + 10;
	if(c >= 'A' && c <= 'F') return c - 'A' + 10;
	return 255;
}


uint8_t checkCRC(uint8_t *data, uint8_t len)
{
  uint8_t crc = 0;
  uint8_t i, dataLen = len - 1;
  
  for(i=0 ; i<dataLen ; i++)
  {
      printf("%02x ",data[i]);
    crc += data[i];  
  }
  printf("\n");

  crc = ~crc + 1;

  printf("0x%02x =?= 0x%02x\n",crc,data[dataLen]);

  if (crc == data[dataLen])
    return 0;
  
  putch('!');
  return 1;
}

//pkt
uint8_t get_pkt()
{
	uint8_t i;
	pkt[0] = (nibbleFromChar(getch()) << 4) | nibbleFromChar(getch());
	for(i = 1; i < pkt[0] + 5; i++)
		pkt[i] = (nibbleFromChar(getch()) << 4) | nibbleFromChar(getch());
    printf("Got packet of size %d\n",pkt[0]);

	return checkCRC(pkt, pkt[0] + 5);
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

char getch(void)
{
	uint32_t count = 0;
	while(!(UCSR0A & _BV(RXC0))){
		/* 20060803 DojoCorp:: Addon coming from the previous Bootloader*/               
		/* HACKME:: here is a good place to count times*/
		count++;
		if (count > MAX_TIME_COUNT)
		{
			putch('.');
			return 0xff;
		}
	}
	return UDR0;
}


uint16_t getTargetAddress(uint8_t *line)
{
  uint16_t address = line[1];
  address <<= 8;
  address |= line[2];

  return address;
}
