/***************************************************************
 *
 * OpenBeacon.org - main file for OpenBeacon USB II Bluetooth
 *
 * Copyright 2010 Milosch Meriac <meriac@openbeacon.de>
 *
 ***************************************************************

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
#include <openbeacon.h>
#include "3d_acceleration.h"
#include "usbserial.h"
#include "bluetooth.h"
#include "iap.h"
#include "spi.h"
#include "nRF_API.h"
#include "nRF_CMD.h"
#include "xxtea.h"
#include "openbeacon-proto.h"

#define PROXIMITY_SLOTS 32
#define FIFO_DEPTH 10

typedef struct
{
	int x, y, z;
} TFifoEntry;

typedef struct
{
	uint16_t oid;
	uint32_t seq;
	uint8_t strength[FWDTAG_STRENGTH_COUNT];
	uint8_t flags;
} TProximitySlot;

/* proximity aggregation buffer */
static TProximitySlot prox[PROXIMITY_SLOTS];
/* device UUID */
static uint16_t tag_id;
static TDeviceUID device_uuid;
/* random seed */
static uint32_t random_seed;
/* logfile position */
static uint32_t g_storage_items;
static uint32_t g_sequence;
static volatile uint8_t g_do_tx;

#define TX_STRENGTH_OFFSET 2

#define MAINCLKSEL_IRC 0
#define MAINCLKSEL_SYSPLL_IN 1
#define MAINCLKSEL_WDT 2
#define MAINCLKSEL_SYSPLL_OUT 3

#ifdef  CUSTOM_ENCRYPTION_KEY
#include "custom-encryption-key.h"
#else /*CUSTOM_ENCRYPTION_KEY */
/* Default TEA encryption key of the tag - MUST CHANGE ! */
static const uint32_t xxtea_key[XXTEA_BLOCK_COUNT] = {
	0x00112233,
	0x44556677,
	0x8899AABB,
	0xCCDDEEFF
};
#endif /*CUSTOM_ENCRYPTION_KEY */

/* set nRF24L01 broadcast mac */
static const unsigned char broadcast_mac[NRF_MAX_MAC_SIZE] = {
	1, 2, 3, 2, 1
};

/* OpenBeacon packet */
static uint8_t g_Beacon[16];

static void
nRF_tx (uint8_t power)
{
	/* set TX power */
	nRFAPI_SetTxPower (power & 0x3);

	/* upload data to nRF24L01 */
	nRFAPI_TX (g_Beacon, sizeof (g_Beacon));

	/* transmit data */
	nRFCMD_CE (1);

	/* wait for packet to be transmitted */
	pmu_wait_ms (2);

	/* transmit data */
	nRFCMD_CE (0);
}

#if 0
static uint32_t
rnd (uint32_t range)
{
	static uint32_t v1 = 0x52f7d319;
	static uint32_t v2 = 0x6e28014a;

	/* reseed random with timer */
	random_seed += LPC_TMR32B0->TC ^ g_sequence;

	/* MWC generator, period length 1014595583 */
	return ((((v1 = 36969 * (v1 & 0xffff) + (v1 >> 16)) << 16) ^
			 (v2 =
			  30963 * (v2 & 0xffff) + (v2 >> 16))) ^ random_seed) % range;
}
#endif

static inline void
pin_init (void)
{
	LPC_IOCON->PIO2_0 = 0;
	GPIOSetDir (2, 0, 1);														//OUT
	GPIOSetValue (2, 0, 0);

	LPC_IOCON->RESET_PIO0_0 = 0;
	GPIOSetDir (0, 0, 0);														//IN

	LPC_IOCON->PIO0_1 = 0;
	GPIOSetDir (0, 1, 0);														//IN

	LPC_IOCON->PIO1_8 = 0;
	GPIOSetDir (1, 8, 1);														//OUT
	GPIOSetValue (1, 8, 1);

	LPC_IOCON->PIO0_2 = 0;
	GPIOSetDir (0, 2, 1);														//OUT
	GPIOSetValue (0, 2, 0);

	LPC_IOCON->PIO0_3 = 0;
	GPIOSetDir (0, 3, 0);														//IN

	LPC_IOCON->PIO0_4 = 1 << 8;
	GPIOSetDir (0, 4, 1);														//OUT
	GPIOSetValue (0, 4, 1);

	/* switch PMU to high power mode */
	LPC_IOCON->PIO0_5 = 1 << 8;
	GPIOSetDir (0, 5, 1);														//OUT
	GPIOSetValue (0, 5, 0);

	LPC_IOCON->PIO1_9 = 0;														//FIXME
	GPIOSetDir (1, 9, 1);														//OUT
	GPIOSetValue (1, 9, 0);

	LPC_IOCON->PIO0_6 = 0;
	GPIOSetDir (0, 6, 1);														//OUT
	GPIOSetValue (0, 6, 1);

	LPC_IOCON->PIO0_7 = 0;
	GPIOSetDir (0, 7, 1);														//OUT
	GPIOSetValue (0, 7, 0);

	LPC_IOCON->PIO1_7 = 0;
	GPIOSetDir (1, 7, 1);														//OUT
	GPIOSetValue (1, 7, 0);

	LPC_IOCON->PIO1_6 = 0;
	GPIOSetDir (1, 6, 1);														//OUT
	GPIOSetValue (1, 6, 0);

	LPC_IOCON->PIO1_5 = 0;
	GPIOSetDir (1, 5, 1);														//OUT
	GPIOSetValue (1, 5, 0);

	LPC_IOCON->PIO3_2 = 0;														// FIXME
	GPIOSetDir (3, 2, 1);														//OUT
	GPIOSetValue (3, 2, 1);

	LPC_IOCON->PIO1_11 = 0x80;													//FIXME
	GPIOSetDir (1, 11, 1);														// OUT
	GPIOSetValue (1, 11, 0);

	LPC_IOCON->PIO1_4 = 0x80;
	GPIOSetDir (1, 4, 0);														// IN

	LPC_IOCON->ARM_SWDIO_PIO1_3 = 0x81;
	GPIOSetDir (1, 3, 1);														// OUT
	GPIOSetValue (1, 3, 0);

	LPC_IOCON->JTAG_nTRST_PIO1_2 = 0x81;
	GPIOSetDir (1, 2, 1);														// OUT
	GPIOSetValue (1, 2, 0);

	LPC_IOCON->JTAG_TDO_PIO1_1 = 0x81;
	GPIOSetDir (1, 1, 1);														// OUT
	GPIOSetValue (1, 1, 0);

	LPC_IOCON->JTAG_TMS_PIO1_0 = 0x81;
	GPIOSetDir (1, 0, 1);														// OUT
	GPIOSetValue (1, 0, 0);

	LPC_IOCON->JTAG_TDI_PIO0_11 = 0x81;
	GPIOSetDir (0, 11, 1);														// OUT
	GPIOSetValue (0, 11, 0);

	LPC_IOCON->PIO1_10 = 0x80;
	GPIOSetDir (1, 10, 1);														// OUT
	GPIOSetValue (1, 10, 1);

	LPC_IOCON->JTAG_TCK_PIO0_10 = 0x81;
	GPIOSetDir (0, 10, 1);														// OUT
	GPIOSetValue (0, 10, 0);

	LPC_IOCON->PIO0_9 = 0;
	GPIOSetDir (0, 9, 1);														// OUT
	GPIOSetValue (0, 9, 0);

	/* select MISO function for PIO0_8 */
	LPC_IOCON->PIO0_8 = 1;
}

static inline void
show_version (void)
{
	debug_printf (" * Device UID: %08X:%08X:%08X:%08X\n",
				  device_uuid[0], device_uuid[1],
				  device_uuid[2], device_uuid[3]);
	debug_printf (" * OpenBeacon MAC: %02X:%02X:%02X:%02X:%02X\n",
				  broadcast_mac[0], broadcast_mac[1], broadcast_mac[2],
				  broadcast_mac[3], broadcast_mac[4]);
	debug_printf (" *         Tag ID: %04X\n", tag_id);
	debug_printf (" * Stored Logfile Items: %i\n", g_storage_items);
}

static void
blink (uint8_t times)
{
	GPIOSetValue (1, 1, 0);
	pmu_wait_ms (500);
	while (times)
	{
		times--;

		GPIOSetValue (1, 1, 1);
		pmu_wait_ms (100);
		GPIOSetValue (1, 1, 0);
		pmu_wait_ms (200);
	}
	pmu_wait_ms (500);
}

static inline void
tag_aggregate (uint16_t oid, uint8_t strength, uint8_t flags, uint32_t seq)
{
	int i;
	TProximitySlot *p;

	if (!oid || (strength > FWDTAG_STRENGTH_MASK))
		return;

#ifdef  VERBOSE
	debug_printf ("\"rx\":{\"id\":\"%04X\", \"strength\":%u, "
				  "\"seq\":%u, \"flags\":%u},\n", oid, strength, seq, flags);
#endif /*VERBOSE*/
		p = prox;
	for (i = 0; i < PROXIMITY_SLOTS; i++)
	{
		/* search for free entry */
		if (p->oid)
		{
			/* skip to next */
			if (p->oid != oid)
			{
				p++;
				continue;
			}
		}
		else
			/* ...else allocate new entry */
			p->oid = oid;

		/* aggregate packet */
		if (p->strength[strength] < 0xFF)
			p->strength[strength]++;
		else
			p->flags |= RFBFLAGS_OVERFLOW;

		if (seq > p->seq)
			p->seq = seq;
		p->flags |= flags;

		break;
	}
}

static inline void
tag_aggregate_tx (void)
{
	/* fire up LED to indicate rx */
	GPIOSetValue (1, 1, 1);
	/* light LED for 10ms */
	pmu_wait_ms (10);
	/* turn LED off */
	GPIOSetValue (1, 1, 0);

	/* prepare packet */
	bzero (&g_Beacon, sizeof (g_Beacon));

	/* transmit packet */
	nRF_tx (3);
}

void
CDC_GetCommand (unsigned char *command)
{
	(void)command;
//	debug_printf ("Unknown command: '%s'\n", command);
	debug_printf ("RX: '%s'\n", command);
	g_do_tx = 1;
}

int
main (void)
{
	int x;
#ifdef  ENABLE_BLUETOOTH
	uint8_t bt_enabled;
#endif /*ENABLE_BLUETOOTH */
	volatile int t;

	/* wait on boot - debounce */
	for (t = 0; t < 2000000; t++);

	/* Initialize GPIO (sets up clock) */
	GPIOInit ();

	/* initialize pins */
	pin_init ();

	/* fire up LED 1 */
	GPIOSetValue (1, 1, 1);

	/* Power Management Unit Initialization */
	pmu_init ();

	/* prepare 32B0 system timer */
	LPC_SYSCON->SYSAHBCLKCTRL |= EN_CT32B0;
	LPC_TMR32B0->TCR = 2;
	LPC_TMR32B0->PR = SYSTEM_CRYSTAL_CLOCK / LPC_SYSCON->SYSAHBCLKDIV;
	LPC_TMR32B0->EMR = 0;
	/* start 32B0 timer */
	LPC_TMR32B0->TCR = 1;

	/* read device UUID */
	bzero (&device_uuid, sizeof (device_uuid));
	iap_read_uid (&device_uuid);

	/* make sure tag-id is always >0x8000 to avoid collisions with other tags */
	tag_id = crc16 ((uint8_t *) & device_uuid, sizeof (device_uuid)) | 0x8000;
	random_seed =
		device_uuid[0] ^ device_uuid[1] ^ device_uuid[2] ^ device_uuid[3];

#ifdef  ENABLE_BLUETOOTH
	/* Init Bluetooth */
	bt_init (TRUE, tag_id);
	bt_enabled = FALSE;
#else
	UARTInit (115200, 0);
#endif /*ENABLE_BLUETOOTH */

	/* CDC USB Initialization */
	init_usbserial ();

	/* initialize SPI */
	spi_init ();

	/* peripherals initialized */
	blink (1);

	/* Init 3D acceleration sensor */
	acc_init (1);

	/* Initialize OpenBeacon nRF24L01 interface */
	while (!nRFAPI_Init (CONFIG_PROX_CHANNEL,
						 broadcast_mac, sizeof (broadcast_mac), 0))
		blink (3);

	/* set tx power power to high */
	nRFCMD_Power (1);

	/* reset proximity buffer */
	bzero (&prox, sizeof (prox));

	/* enable TX mode */
	nRFAPI_SetRxMode (0);
	nRFCMD_CE (0);
	nRFAPI_SetChannel (CONFIG_TRACKER_CHANNEL);

	/* blink two times to show readyness */
	blink (2);
	g_sequence = 0;
	g_do_tx = 0;

	while (1)
	{
		if(g_do_tx)
		{
			tag_aggregate_tx ();
			g_do_tx = 0;
		}

#ifdef  ENABLE_BLUETOOTH
		/* check for incoming bluetooth connection */
		if (UARTCount != 0)
		{
			if(!bt_enabled)
				for(x=0;x<(int)UARTCount;x++)
					if(UARTBuffer[x]=='\n')
					{
//						bt_enabled=TRUE;
//						EnableBluetoothConsole ( TRUE );
						break;
					}
			/* prepare outgoing packet */
			tag_aggregate_tx ();
			UARTCount = 0;
		}
#endif /*ENABLE_BLUETOOTH */
	}
	return 0;
}
