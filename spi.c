#include <avr/io.h>

#define RFM12B_SS                   0x01                /* PB.0 */
#define RFM12B_SCLK                 0x02                /* PB.1 */
#define RFM12B_MOSI                 0x04                /* PB.2 */
#define RFM12B_MISO                 0x08                /* PB.3 */

void RFM12B_SPI_Init( void )
{
    /* Set SPI Interface to Master mode at f(CPU)/8 = 2MHz */
    SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0);
    SPSR = _BV(SPI2X);
        
    PORTB  |= RFM12B_SS;                /* Chip Select High         */
    DDRB   |= RFM12B_SS | RFM12B_SCLK | RFM12B_MOSI;
}    

uint16_t RFM12B_SPI_Transfer( uint16_t command )
{
    /* Assuming 2MHz SPI bus rate each 16-bit
       transfer takes 8us */
    
    uint16_t status;
    
    PORTB &= ~RFM12B_SS;                /* Chip Select Low          */
    SPDR = command >> 8;                /* Send command MSB         */
    while (!(SPSR & _BV(SPIF)))
        ;                               /* Wait for Xfer complete   */
    status = SPDR << 8;                 /* Read status MSB          */
    SPDR = ( command & 0x00FF );        /* Send command LSB         */
    while (!(SPSR & _BV(SPIF)))
        ;                               /* Wait for Xfer complete   */
    status |= SPDR;                     /* Read status LSB          */
    PORTB  |= RFM12B_SS;                /* Chip Select High         */
    return status;
}

uint16_t RFM12B_SPI_Transfer8( uint16_t command )
{
    /* Assuming 2MHz SPI bus rate each 16-bit
       transfer takes 8us */
    
    uint16_t status;
    
    PORTB &= ~RFM12B_SS;                /* Chip Select Low          */
    SPDR = command >> 8;                /* Send command MSB         */
    while (!(SPSR & _BV(SPIF)))
        ;                               /* Wait for Xfer complete   */
    status = SPDR << 8;                 /* Read status MSB          */
    PORTB  |= RFM12B_SS;                /* Chip Select High         */
    return status;
}
