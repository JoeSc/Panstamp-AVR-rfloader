

#define use_prints

#ifdef use_prints
#include <stdio.h>
#else
#define printf(fmt, ...) do {} while (0)
#endif
#include "main.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "optiboot.h"
#include "swap.h"


#define LED_DDR  DDRB
#define LED_PORT PORTB
#define LED_PIN  PINB
#define LED      PINB5

void flash_led(uint8_t);
void putch(char);
char getch(void);
void jumpToUserCode(void);

#ifdef use_prints
#ifdef __cplusplus
extern "C"{
 FILE * uart_str;
 }
#endif

static int
uart_putchar(char c, FILE *stream)
{
    if (c == '\n')
        uart_putchar('\r', stream);
    putch(c);
    return 0;
}
#endif

void jump_to_wbl(void) __attribute__ ((naked)) __attribute__ ((section (".jump_to")));
int main(void) __attribute__ ((OS_main)) __attribute__ ((section (".init9")));



const char user_pgm_reset_vec_jump[SPM_PAGESIZE] __attribute__((section(".progmem.data"))) __attribute__ (( aligned(SPM_PAGESIZE) ))  = {0x0} ;//= { 0x0};

void jump_to_wbl()
{
    asm volatile (
      /* "   rjmp    1f\n"*/
            ".global keep_me\n"
      "keep_me:\n"
      "   jmp    __vectors\n"
      "1:\n"
    );
}

void jumpToUserCode(void)
{
    asm volatile ("jmp .progmem.data");
}

const uint16_t RAM_MAGIC_LOC    = 0x0824;
const uint16_t USER_PGM_END     = 0x5000;


// Responses from server have to be received before 10000 ms after sending
// the query
#define RESPONSE_TIMEOUT  10000
// Maximum number of queries sent to the server for a given line of firmware
#define MAX_REPEAT_QUERY  10

// User code address
uint16_t userCodeAddr;
// Global packet
CCPACKET packet;
// New firmware line received
volatile bool lineReceived = false;
// First hex line flag
bool firstLine = true;
// Length of last line received
uint8_t lineLength = 0;
// Pointer to line buffer
uint8_t *ptrLine;
//// ISR vector table
//uint8_t isrTable[8][16];


/**
 * readHexLine
 *
 * Read wireless packet and extract HEX line
 * 
 * @param lineNb Line number being queried
 * 
 * @return Correct line received (true) or not (false)
 */
bool readHexLine(uint16_t lineNumber)
{
  // Any packet waiting to be read?
  if (swap.radio.receiveData(&packet) > 0)
  {
    // Is CRC OK?
    if (packet.crc_ok)
    {
      #ifdef SWAP_EXTENDED_ADDRESS
      if ((packet.SWAP_ADDRTYPE) == SWAPADDR_EXTENDED)
      #else
      if ((packet.SWAP_ADDRTYPE) == SWAPADDR_SIMPLE)
      #endif
      {
        // Function
        if ((packet.SWAP_FUNCTION) == SWAPFUNCT_STA)
        {                
          // Data payload
          ptrLine = packet.data + SWAP_DATA_HEAD_LEN + 1;
          lineLength = packet.length - SWAP_DATA_HEAD_LEN - 1;

          // // Firmware page received?
          if(packet.SWAP_REGID == REGI_FIRMWARE)
          {
            // Correct data length?
            if (lineLength <= BYTES_PER_LINE)
            {
              // Correct line number?
              if (getLineNumber(ptrLine) == lineNumber)
              {
                ptrLine += 2;
                lineLength -= 2;
                lineReceived = true;
                return true;
              }                       
            }
          }
        }
      }
    }
  }
  return false;
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

#ifdef use_prints
uart_str = fdevopen(uart_putchar, NULL);
stdout = stdin = uart_str;
#endif
    printf("swap test\n");


	// Start of copy from rfloader.cpp
  uint8_t state, status, bytes, i, count=0;
  // Current firmware line being queried from hex file
  uint16_t lineNumber = 0;
  uint16_t last_address = 0;

	// This flag will tell us whether wireless bootloading needs to start or not
	bool * ptr1;
	ptr1 = (bool*)RAM_MAGIC_LOC;  // Memory address at the end of the stack
	bool runUserCode = *ptr1;       // Read value

    uint16_t userCodeAddr = pgm_read_byte( (uint16_t)(void*) &user_pgm_reset_vec_jump[0]);

	cli();

	// Valid starting address of user code?
	if (userCodeAddr)
	{ 
		printf("Valid Program");
		// Jump to user code if the wireless bootloader was not called from there
//		if (runUserCode) {
//			putch('U');
//			jumpToUserCode();
//		}
	}

	swap.init();

//int p;
//
//    for (p=0; p<10; p++)
//    {
//        puts("PCODE\n");
//    TRANSMIT_SWAP_STATUS_PCODE();
//  getch();
//  getch();
//  getch();
//  getch();
//  getch();
//        puts("RESTART\n");
//  TRANSMIT_SWAP_STATUS_STATE((uint8_t)SYSTATE_RESTART);
//  getch();
//  getch();
//  getch();
//  getch();
//  getch();
//    }


  // Transmit default product code only if no user application is still flashed
  if (!userCodeAddr)
    TRANSMIT_SWAP_STATUS_PCODE();

  // Enter upgrade mode
  state = (uint8_t)SYSTATE_UPGRADE;
  TRANSMIT_SWAP_STATUS_STATE(state);
 
  // Pointer at the begining of user flash
  //asdasdasduint16_t address = USER_PGM_START;

    uint16_t pgm_address = 0x0;
    uint8_t first_page_data[SPM_PAGESIZE];
    uint8_t first_page = 1;

  while(1)
  { 
    // Wait for new line from server
    while(!lineReceived)
    {
      // Query firmware line
      TRANSMIT_SWAP_QUERY_LINE(lineNumber);
      
      // Repeat query a limited amount of times if necessary
      if (count++ == MAX_REPEAT_QUERY)
      {
          printf("timeout\n");
        // If the flash was not erased then start user code
        if (first_page)
        {
          printf("Jumping to User Code\n");
          jumpToUserCode();
        }
        // Otherwise continue forever asking
      }
      
	  int timeout = 0;
      while (timeout++ < RESPONSE_TIMEOUT)
      {
        status = swap.radio.readReg(CC1101_PKTSTATUS, CC1101_STATUS_REGISTER);//ReadSingleReg(PKTSTATUS);
        bytes = swap.radio.readReg(CC1101_RXBYTES, CC1101_STATUS_REGISTER);//ReadSingleReg(RXBYTES);
        
        // Poll PKSTATUS and number of bytes in the Rx FIFO
        if ((status & 0x01) && bytes)
        {
          while (swap.radio.readReg(CC1101_PKTSTATUS, CC1101_STATUS_REGISTER) & 0x01);
		LED_PORT |= _BV(LED);
          // Packet received. Read packet and extract HEX line
          if (readHexLine(lineNumber))
          {
            count = 0;  // Reset counter
            break;
          }
		LED_PORT &= ~_BV(LED);
        }
      }
    }

    lineReceived = false;

    // Is the line received OK?
    if (checkCRC(ptrLine, lineLength))
    {  
        printf("CRC OK");
      if (TYPE_OF_RECORD(ptrLine) == RECTYPE_DATA)
      {
        // Get target address
        uint16_t addrFromHexFile = getTargetAddress(ptrLine);

                    if (first_page)
                    {
                        printf("f page addr = 0x%04x",addrFromHexFile);
					    for(i=0; i < 16; i++)
                            first_page_data[pgm_address++] = ptrLine[3+i];

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
					    	optiboot_page_fill(addrFromHexFile, (ptrLine[4 + i] << 8) + ptrLine[3 + i]);
					    	addrFromHexFile += 2;
					    }
	
					    last_address = addrFromHexFile;
					    if((addrFromHexFile % SPM_PAGESIZE) == 0) {
					    	putch('W');
					    	optiboot_page_write(addrFromHexFile - SPM_PAGESIZE);
					    }
                    }
        lineNumber++;
      }
      else  // Probably end of file
				{
					// Round address down to last page if we didn't finish on a boundary
					if(last_address % SPM_PAGESIZE) {
						last_address = last_address - (last_address % SPM_PAGESIZE);
                        printf("WRiting page 0x%04x\n",last_address);
						optiboot_page_write(last_address);
					}
        // Jump to user code
        jumpToUserCode();
				}
    }
  }

  // Enter restart mode
  state = (uint8_t)SYSTATE_RESTART;
  TRANSMIT_SWAP_STATUS_STATE(state);


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


/**
 * getLineNumber
 *
 * Get number of line from payload
 *
 * @param data payload from packet received
 *
 * @return line number
 */
uint16_t getLineNumber(uint8_t *data)
{
  uint16_t lineNb = data[0];
  lineNb <<= 8;
  lineNb |= data[1];

  return lineNb;
}

/**
 * checkCRC
 *
 * Check CRC from hex line. The CRC byte (last byte) is a 2's complement
 * of the whole data buffer
 *
 * @param data payload from packet received
 * @param len packet length
 *
 * @return true if CRC is correct
 */
bool checkCRC(uint8_t *data, uint8_t len)
{
  uint8_t crc = len - 4;
  uint8_t i, dataLen = len - 1;
  
  for(i=0 ; i<dataLen ; i++)
    crc += data[i];  

  crc = ~crc + 1;

  if (crc == data[dataLen])
    return true;
  
  return false;
}

/**
 * getTargetAddress
 *
 * Get address from HEX line where the write has to be done
 *
 * @param line line from HEX file
 *
 * @return target address
 */
uint16_t getTargetAddress(uint8_t *line)
{
  uint16_t address = line[0];
  address <<= 8;
  address |= line[1];

  return address;
}
