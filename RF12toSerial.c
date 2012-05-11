#include "RF12toSerial.h"
#include "spi.h"

#define END_OF_PACKET           '|'

#define RFM12B_TXREG_WRITE          0xB800
#define RFM12B_RX_FIFO_READ         0xB000

#define RFM12B_CMD_RX_ON            0x82DD
#define RFM12B_CMD_TX_ON            0x823D

/* Bits Returned by the Status Read Command */

#define RFM12B_STATUS_RGIT          0x8000              /* TX register empty */
#define RFM12B_STATUS_FFIT          0x8000              /* RX FIFO reached limit */
#define RFM12B_STATUS_POR           0x4000              /* Power-on reset */
#define RFM12B_STATUS_RGUR          0x2000              /* TX register under run */
#define RFM12B_STATUS_FFOV          0x2000              /* RX FIFO overflow */
#define RFM12B_STATUS_WKUP          0x1000              /* Wake-up timer overflow */
#define RFM12B_STATUS_EXT           0x0800              /* Interrupt pin (pin 16) low */
#define RFM12B_STATUS_LBD           0x0400              /* Low battery detect */
#define RFM12B_STATUS_FFEM          0x0200              /* RX FIFO Empty */
#define RFM12B_STATUS_ATS           0x0100              /* Antenna circuit detected strong RF */
#define RFM12B_STATUS_RSSI          0x0100              /* received RSSI above limit */
#define RFM12B_STATUS_DQD           0x0080              /* Data quality detector output */
#define RFM12B_STATUS_CRL           0x0040              /* Clock recovery locked */
#define RFM12B_STATUS_ATGL          0x0020              /* Toggling in each AFC cycle */

/* Adjust the buffer sizes to suit the application.  Note that the 
   Transmit Buffer must be longer than the USBtoRF12 buffer to allow for the
   extra bytes added during packetization.  There's probably a way of
   doing this that uses less RAM! */

static uint8_t      RF12toUSB_Buffer_Data[80];
static uint8_t      USBtoRF12_Buffer_Data[80];
static uint8_t      Transmit_Buffer_Data[90];

static RingBuffer_t RF12toUSB_Buffer;
static RingBuffer_t USBtoRF12_Buffer;
static RingBuffer_t Transmit_Buffer;

static uint8_t      RxTimeout;
static uint8_t      RxLength;

static bool         RFM12B_Transmit_Active;

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
{
    .Config =
    {
        .ControlInterfaceNumber     = 0,
        .DataINEndpoint             =
        {
            .Address                = CDC_TX_EPADDR,
            .Size                   = CDC_TXRX_EPSIZE,
            .Banks                  = 1,
        },
        .DataOUTEndpoint            =
        {
            .Address                = CDC_RX_EPADDR,
            .Size                   = CDC_TXRX_EPSIZE,
            .Banks                  = 1,
        },
        .NotificationEndpoint       =
        {
            .Address                = CDC_NOTIFICATION_EPADDR,
            .Size                   = CDC_NOTIFICATION_EPSIZE,
            .Banks                  = 1,
        },
    },
};

void RingBuffer_InsertString( RingBuffer_t* buff, char* s )
{
    char* lp;
    for ( lp = s; *lp; lp++ ) RingBuffer_Insert( buff, *lp );
}

void RFM12B_Transmit( void )
{
    if ( RingBuffer_GetCount( &Transmit_Buffer ))
    {
        RFM12B_SPI_Transfer( RFM12B_TXREG_WRITE + 
            RingBuffer_Remove( &Transmit_Buffer ));
    }
    else
    {
        RFM12B_SPI_Transfer( RFM12B_CMD_RX_ON );
        RFM12B_Transmit_Active = false;
        PORTD |= 0x20;
    } 
}

/** Put the end of packet indicator into the buffer and
 *  toggle the 'ff' flag to re-enable synchronisation 
 *  detection for next packet
 */

void RFM12B_EndOfPacket( void )
{
    RingBuffer_Insert( &RF12toUSB_Buffer, END_OF_PACKET );
    RFM12B_SPI_Transfer( 0xCA81 );
    RFM12B_SPI_Transfer( 0xCA83 );
}

/** A character has been received, check the value of RxLength
 *  if it's zero then the byte just received must be the length
 *  byte, so grab it
 */
   
void RFM12B_Receive( void )
{
    uint8_t ch = RFM12B_SPI_Transfer( RFM12B_RX_FIFO_READ ) & 0xFF;

    if ( RxLength == 0 )
    {
        if ( ch == 0 )
        {
            RFM12B_EndOfPacket();
        }
        else
        {
            RxLength = ch;
            RxTimeout = 0;
        }
    }
    else
    {
        RingBuffer_Insert( &RF12toUSB_Buffer, ch );

        /* Decrement the RxLength for each byte of payload received.
           When it gets to zero that's the end of the packet */
           
        if ( --RxLength == 0 ) RFM12B_EndOfPacket();
    }
}


void RFM12B_Start_Transmit( void )
{
   uint16_t BufferCount = RingBuffer_GetCount( &USBtoRF12_Buffer );
   
   RingBuffer_InsertString( &Transmit_Buffer, "\xAA\xAA\x2D\x55" );
   RingBuffer_Insert( &Transmit_Buffer, BufferCount & 0xFF );
   
   while ( BufferCount-- )
   {
       RingBuffer_Insert( &Transmit_Buffer, 
           RingBuffer_Remove( &USBtoRF12_Buffer ));
   }

   RingBuffer_InsertString( &Transmit_Buffer, "\xAA\xAA" );
   RFM12B_Transmit_Active = true;
   PORTD &= ~0x20;
   RFM12B_SPI_Transfer( RFM12B_CMD_TX_ON );
}

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */

int main(void)
{
    SetupHardware();

    RingBuffer_InitBuffer(&RF12toUSB_Buffer, RF12toUSB_Buffer_Data, sizeof(RF12toUSB_Buffer_Data));
    RingBuffer_InitBuffer(&USBtoRF12_Buffer, USBtoRF12_Buffer_Data, sizeof(USBtoRF12_Buffer_Data));
    RingBuffer_InitBuffer(&Transmit_Buffer,  Transmit_Buffer_Data,  sizeof(Transmit_Buffer_Data));

    sei();

    for (;;)
    {
        uint16_t RFM12B_status = RFM12B_SPI_Transfer(0);
        
        if (( RFM12B_status & RFM12B_STATUS_RSSI )
           && !RFM12B_Transmit_Active )
        {
            PORTD &= ~0x40;
        }
        else
        {
            PORTD |= 0x40;
        }
        
        /* Only try to read in bytes from the CDC interface if the transmit buffer is not full */
        if (  !RingBuffer_IsFull( &USBtoRF12_Buffer )
           && !RFM12B_Transmit_Active )
        {
            int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

            /* Read bytes from the USB OUT endpoint into the USART transmit buffer */
            if (!(ReceivedByte < 0)) 
            {  
               if ( ReceivedByte != END_OF_PACKET )
               {
                   RingBuffer_Insert( &USBtoRF12_Buffer, ReceivedByte );
               }
               else
               {
                   /* TODO : Need to implement LBT */
                   RFM12B_Start_Transmit();
               }
            }
        }

        /* Check to see if there's an RGIT or an FFIT bit set in the status
           register indicating that we need to send or receive a byte */
        if ( RFM12B_status & RFM12B_STATUS_RGIT )
        {
            if ( RFM12B_Transmit_Active )
            {
                RFM12B_Transmit();
            }
            else
            {
                RFM12B_Receive();
            }
        }
        
        /* Check the flush timer.  This is used to timeout incoming packets
           that don't arrive in time or to flush the data to the USB host.
           First, check to see if we are in the middle of receiving a packet */
        if ( RxLength )
        {
            /* A packet is being received. Check the flush timer
               and timeout the packet if it takes too long */
            if ( TIFR0 & _BV(TOV0))
            {
                /* Clear flush timer expiry flag */
                TIFR0 |= _BV(TOV0);

                /* Flush timer overflows every 4ms.  256 bytes at ~50k
                   should take approximately 42ms so we allow 12 counts
                   for overflow.  This should probably be calculated
                   based on the bit rate and max message length */
                if ( ++RxTimeout > 12 )
                {
                    RingBuffer_Insert( &RF12toUSB_Buffer, END_OF_PACKET );
                    RFM12B_SPI_Transfer( 0xCA81 );
                    RFM12B_SPI_Transfer( 0xCA83 );
                    RxLength = 0;
                }
            }
        }
        else
        {
            /* No packet is being received.  Check if the UART receive 
               buffer flush timer has expired or the buffer is nearly full */
            uint16_t BufferCount = RingBuffer_GetCount(&RF12toUSB_Buffer);
            if ((TIFR0 & (1 << TOV0)) || (BufferCount > (uint8_t)(sizeof(RF12toUSB_Buffer_Data) * .75)))
            {
                /* Clear flush timer expiry flag */
                TIFR0 |= _BV(TOV0);
    
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
    RFM12B_Transmit_Active = false;


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