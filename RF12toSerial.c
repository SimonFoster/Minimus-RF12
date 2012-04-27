/*
             LUFA Library
     Copyright (C) Dean Camera, 2012.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2012  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the USBtoSerial project. This file contains the main tasks of
 *  the project and is responsible for the initial application hardware configuration.
 */

#include "RF12toSerial.h"
#include "spi.h"

/** Circular buffer to hold data from the serial port before it is sent to the host. */
static RingBuffer_t RF12toUSB_Buffer;

/** Underlying data buffer for \ref USARTtoUSB_Buffer, where the stored bytes are located. */
static uint8_t      RF12toUSB_Buffer_Data[128];

static uint8_t      RxLength;

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber         = 0,
				.DataINEndpoint                 =
					{
						.Address                = CDC_TX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.DataOUTEndpoint                =
					{
						.Address                = CDC_RX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.NotificationEndpoint           =
					{
						.Address                = CDC_NOTIFICATION_EPADDR,
						.Size                   = CDC_NOTIFICATION_EPSIZE,
						.Banks                  = 1,
					},
			},
	};

void RingBuffer_InsertString( char* p )
{
    char* lp;
    for ( lp = p; *lp; lp++ ) RingBuffer_Insert( &RF12toUSB_Buffer, *lp );
}

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();

	RingBuffer_InitBuffer(&RF12toUSB_Buffer, RF12toUSB_Buffer_Data, sizeof(RF12toUSB_Buffer_Data));

	sei();

	for (;;)
	{
        /* Not interested in any data from the host so just read and discard */
        (void) CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

        uint16_t status = RFM12B_SPI_Transfer(0);
        
        if ( status & 0x0100 )
        {
            PORTD &= ~0x40;
        }
        else
        {
            PORTD |= 0x40;
        }
        
        if ( status & 0x8000 )
        {
            uint8_t ch = RFM12B_SPI_Transfer( 0xB000 ) & 0xFF;

            /* A character has been received, check the value of RxLength
               if it's zero then the byte just received must be the length
               byte, so grab it */
               
            if ( RxLength == 0 )
            {
                RxLength = ch;
            }
            else
            {
                /* If RxLength is not zero then we are receiving payload
                   bytes so send to the host via USB */
                
                RingBuffer_Insert( &RF12toUSB_Buffer, ch );            
                if ( --RxLength == 0 )
                {
                    /* Decrement the RxLength for each byte of payload received.
                       When it gets to zero toggle the 'ff' flag to re-enable
                       synchronisation detection for next packet */
                    
                    RFM12B_SPI_Transfer( 0xCA81 );
                    RFM12B_SPI_Transfer( 0xCA83 );
                }
            }
        }
        
		/* Check if the UART receive buffer flush timer has expired or the buffer is nearly full */
		uint16_t BufferCount = RingBuffer_GetCount(&RF12toUSB_Buffer);
		if ((TIFR0 & (1 << TOV0)) || (BufferCount > (uint8_t)(sizeof(RF12toUSB_Buffer_Data) * .75)))
		{
			/* Clear flush timer expiry flag */
			TIFR0 |= (1 << TOV0);

			/* Read bytes from the USART receive buffer into the USB IN endpoint */
			while (BufferCount--)
			{
				/* Try to send the next byte of data to the host, abort if there is an error without dequeuing */
				if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface,
				                        RingBuffer_Peek(&RF12toUSB_Buffer)) != ENDPOINT_READYWAIT_NoError)
				{
					break;
				}

				/* Dequeue the already sent byte from the buffer now we have confirmed that no transmission error occurred */
				RingBuffer_Remove(&RF12toUSB_Buffer);
			}
		}

		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

    RFM12B_SPI_Init();

    /* Commands are not accepted while the RF12 is in reset so
       keep reading the status until the POR bit is clear    */
    
    uint16_t status = RFM12B_SPI_Transfer( 0 );
    while ( status & 0x4000 )
        status = RFM12B_SPI_Transfer( 0 );
    
    /* For commands see also http://tools.jeelabs.org/rfm12b */
    
    RFM12B_SPI_Transfer(0x80E7);  /* 868MHz, 12pF, el=1, ef=1 */ 
    RFM12B_SPI_Transfer(0xA640);  /* centre Frequency = 868MHz */ 
    RFM12B_SPI_Transfer(0xC606);  /* Data Rate approx 49.2 Kbps, (10000/29/(1+6)) Kbps */
    RFM12B_SPI_Transfer(0x94A2);  /* VDI,FAST,134kHz,0dBm,-91dBm */ 
    RFM12B_SPI_Transfer(0xC2AC);  /* AL,!ml,DIG,DQD4 */ 
    RFM12B_SPI_Transfer(0xCA83);  /* FIFO8,2-SYNC,!ff,DR */ 
    RFM12B_SPI_Transfer(0xCE55);  /* Group? = 0x55 */
    RFM12B_SPI_Transfer(0xC483);  /* @PWR,NO RSTRIC,!st,!fi,OE,EN */ 
    RFM12B_SPI_Transfer(0x9850);  /* !mp,90kHz,MAX OUT */ 
    RFM12B_SPI_Transfer(0xCC77);  /* OB1,OB0, LPX,!ddy,DDIT,BW0  */ 
    RFM12B_SPI_Transfer(0xE000);  /* NOT USE */ 
    RFM12B_SPI_Transfer(0xC800);  /* NOT USE */ 
    RFM12B_SPI_Transfer(0xC049);  /* 1.66MHz, 3.1V */ 

    RxLength = 0;
    RFM12B_SPI_Transfer(0x82DD);  /* Turn Receiver On */

	/* Hardware Initialization */
	LEDs_Init();
	USB_Init();

	/* Start the flush timer so that overflows occur rapidly to push received bytes to the USB interface */
	TCCR0B = (1 << CS02);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}
