/***************************************************************
 *
 * OpenBeacon.org 2.4GHz proximity tag - main entry, CRC, behaviour
 *
 * Copyright (C) 2012 Milosch Meriac <meriac@openbeacon.de>
 * - Ported to PIC16LF1825
 *
 * Copyright (C) 2011 Milosch Meriac <meriac@openbeacon.de>
 * - Unified Proximity/Tracking-Mode into one Firmware
 * - Increased Tracking/Proximity rate to 12 packets per second
 * - Implemented Tag-ID assignment/management via RF interface to
 *   simplify production process
 *
 * Copyright (C) 2008 Istituto per l'Interscambio Scientifico I.S.I.
 * - extended by Ciro Cattuto <ciro.cattuto@gmail.com> by support for
 *   SocioPatterns.org platform
 *
 * Copyright (C) 2006 Milosch Meriac <meriac@openbeacon.de>
 * - Optimized en-/decryption routines, CRC's, EEPROM handling
 *
/***************************************************************

/*
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; version 2.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

#include <htc.h>
#include <stdlib.h>
#include <stdint.h>
#include "config.h"
#include "openbeacon.h"
#include "timer.h"
#include "nRF_CMD.h"
#include "nRF_HW.h"

/*    configuration word 1
      32 1098 7654 3210
      00 1001 1100 1100 = 0x0DCC */
__CONFIG (0x09CC);

/*    configuration word 2
      32 1098 7654 3210
      01 0110 0001 0000 = 0x1610 */
__CONFIG (0x1610);

uint8_t pkt[16];

/*

#define ntohs htons
static uint16_t
htons (uint16_t src)
{
	uint16_t res;

	((unsigned char *) &res)[0] = ((unsigned char *) &src)[1];
	((unsigned char *) &res)[1] = ((unsigned char *) &src)[0];

	return res;
}

#define ntohl htonl
static uint32_t
htonl (uint32_t src)
{
	uint32_t res;

	((uint8_t *) & res)[0] = ((uint8_t *) & src)[3];
	((uint8_t *) & res)[1] = ((uint8_t *) & src)[2];
	((uint8_t *) & res)[2] = ((uint8_t *) & src)[1];
	((uint8_t *) & res)[3] = ((uint8_t *) & src)[0];

	return res;
}

static uint16_t
crc16 (const unsigned char *buffer, unsigned char size)
{
	unsigned short crc = 0xFFFF;
	if (buffer)
	{
		while (size--)
		{
			crc = (crc >> 8) | (crc << 8);
			crc ^= *buffer++;
			crc ^= ((unsigned char) crc) >> 4;
			crc ^= crc << 12;
			crc ^= (crc & 0xFF) << 5;
		}
	}
	return crc;
}
*/

void
main (void)
{
	uint8_t j;

	/* configure CPU peripherals */
	OSCCON = CONFIG_CPU_OSCCON_SLOW;
	OPTION_REG = CONFIG_CPU_OPTION;
	PORTA = CONFIG_CPU_PORTA;
	PORTC = CONFIG_CPU_PORTC;
	TRISA = CONFIG_CPU_TRISA;
	TRISC = CONFIG_CPU_TRISC;
	WPUA = CONFIG_CPU_WPUA;
	WPUC = CONFIG_CPU_WPUC;
	ANSELA = CONFIG_CPU_ANSELA;
	ANSELC = CONFIG_CPU_ANSELC;

	INTE = 0;
	CONFIG_PIN_SENSOR = 0;
	CONFIG_PIN_TX_POWER = 0;

	/* initalize hardware */
	timer_init ();

	/* verify RF chip */
	nRFCMD_Init ();
	while(1)
	{
		nRFCMD_Channel (23);
		if(nRFCMD_RegGet (NRF_REG_RF_CH)==23)
		{
			nRFCMD_Channel (42);
			if(nRFCMD_RegGet (NRF_REG_RF_CH)==42)
				break;
		}

		CONFIG_PIN_LED = 1;
		sleep_jiffies (JIFFIES_PER_MS (25));
		CONFIG_PIN_LED = 0;
		sleep_jiffies (JIFFIES_PER_MS (25));
	}

	/* blink to show readyiness */
	for (j = 0; j <= 10; j++)
	{
		CONFIG_PIN_LED = j & 1;
		sleep_jiffies (JIFFIES_PER_MS (25));
	}

	// switch to listening channel
	nRFCMD_Channel (CONFIG_TRACKER_CHANNEL);

	while (1)
	{
		nRFCMD_Listen (JIFFIES_PER_MS (500));

		if (!CONFIG_PIN_IRQ)
		{
			while ((nRFCMD_RegGet (NRF_REG_FIFO_STATUS) &
					NRF_FIFO_RX_EMPTY) == 0)
			{
				// receive raw data
				nRFCMD_RegRead (RD_RX_PLOAD, pkt, sizeof (pkt));
				nRFCMD_RegPut (NRF_REG_STATUS | WRITE_REG,
							   NRF_CONFIG_MASK_RX_DR);

				CONFIG_PIN_LED = 1;
				sleep_jiffies (JIFFIES_PER_MS (100));
			}
		}

		CONFIG_PIN_LED = 1;

		// blink for 1ms
		sleep_jiffies (JIFFIES_PER_MS (1));

		// disable LED
		CONFIG_PIN_LED = 0;
	}
}
