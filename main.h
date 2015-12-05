/**
 * Copyright (c) 2014 panStamp <contact@panstamp.com>
 * 
 * This file is part of the panStamp project.
 * 
 * panStamp  is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * any later version.
 * 
 * panStamp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with panStamp; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 
 * USA
 * 
 * Author: Daniel Berenguer
 * Creation date: 04/01/2014
 */

#ifndef _RFLOADER_H
#define _RFLOADER_H

#include "swap.h"

/**
 * Uncomment only in case of using GDB bootloader
 */
//#define GDB_SERIAL_BOOT
 
/**
 * Default product code
 */
uint8_t DEFAULT_PRODUCT_CODE[] = {0,0,0,1,0xFF,0,0,0};
/**
 * SWAP status packets
 */
// Transmit SWAP product code
#define TRANSMIT_SWAP_STATUS_PCODE(state)     swap.sendPacket((uint8_t)SWAPFUNCT_STA, (uint8_t)REGI_PRODUCTCODE, DEFAULT_PRODUCT_CODE, sizeof(DEFAULT_PRODUCT_CODE))
// Transmit SWAP state
#define TRANSMIT_SWAP_STATUS_STATE(state)     swap.sendPacketVal((uint8_t)SWAPFUNCT_STA, (uint8_t)REGI_SYSSTATE, state)
// Query product code from node with address 1
#define TRANSMIT_SWAP_QUERY_PCODE()           swap.sendPacket((uint8_t)SWAPFUNCT_QRY, (uint8_t)REGI_PRODUCTCODE, 0, 0)
// Query firmware line from node with address 1
#define TRANSMIT_SWAP_QUERY_LINE(line)        swap.sendPacketVal((uint8_t)SWAPFUNCT_QRY, (uint8_t)REGI_FIRMWARE, line)
// Type of record
#define TYPE_OF_RECORD(line)                  line[2]

/**
 * Type of HEX file record
 */
#define RECTYPE_DATA  0x00
#define RECTYPE_EOF   0x01

/**
 * Firmware server
 */
// Product code
#define FIRMSERVER_PRODUCT_CODE  {0,0,0,1,0,0,0,16}
// Maximum number of bytes per line
#define BYTES_PER_LINE  48

/**
 * SWAP object
 */
SWAP swap;

/**
 * getLineNumber
 *
 * Get number of line from payload
 *
 * @param data payload from packet received
 *
 * @return line number
 */
uint16_t getLineNumber(uint8_t *data);

/**
 * getTargetAddress
 *
 * Get address from HEX line where the write has to be done
 *
 * @param line line from HEX file
 *
 * @return target address
 */
uint16_t getTargetAddress(uint8_t *line);

/**
 * checkCRC
 *
 * Check CRC from hex line
 *
 * @param data payload from packet received
 * @param len packet length
 *
 * @return true if CRC is correct
 */
bool checkCRC(uint8_t *data, uint8_t len);

/**
 * sleep
 *
 * Enter low-power mode
 */
void sleep(void);

/**
 * jumpToUserCode
 *
 * Jump to user code after exiting SWAP upgrade mode
 */
void jumpToUserCode(void);



/**
 * delayClockCycles
 *
 * Clock cycle delay
 *
 * @param n clock cycles to wait
 */
void delayClockCycles(register uint32_t n);

#endif

