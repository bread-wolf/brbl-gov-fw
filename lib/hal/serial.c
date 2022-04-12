/*
 * Author: Augustin.B
 * Date  : 09.04.2022
 */

#include "serial.h"

#include "avr/interrupt.h"

// Defines to decode serial_format enum.
// | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 |        Bit 0        |
// |       Number of data bits     |  Parity Type  | Number of Stop bits |
#define SERIAL_NUM_DATA_BITS_MASK        0x78
#define SERIAL_NUM_DATA_BITS_SHIFT       3
#define SERIAL_PARITY_TYPE_MASK          0x06
#define SERIAL_PARITY_TYPE_SHIFT         2
#define SERIAL_NUM_STOP_BITS_MASK        0x01
#define SERIAL_NUM_STOP_BITS_SHIFT       0

// Serial parity options
#define SERIAL_PARITY_NONE               0
#define SERIAL_PARITY_RESERVED           1
#define SERIAL_PARITY_EVEN               2
#define SERIAL_PARITY_ODD                3

// Constants of the hardware.
// MAX_NUM_DATA_BITS should be left at 8.
#define SERIAL_MAX_NUM_DATA_BITS    	 8
#define SERIAL_MIN_NUM_DATA_BITS         5
#define SERIAL_MAX_BAUDRATE              ((F_CPU) / 64) // Assume we only use 16x oversampling

bool serial_init(USART_t* channel, uint32_t baudrate, serial_format format)
{
    // Verify input arguments
    if (channel == NULL)
        return false;

    if (baudrate > SERIAL_MAX_BAUDRATE)
        return false;

    uint8_t numDataBits = (format & SERIAL_NUM_DATA_BITS_MASK) >> SERIAL_NUM_DATA_BITS_SHIFT;
    uint8_t parityType  = (format & SERIAL_PARITY_TYPE_MASK) >> SERIAL_PARITY_TYPE_SHIFT;
    uint8_t numStopBits = (format & SERIAL_NUM_STOP_BITS_MASK) >> SERIAL_NUM_STOP_BITS_SHIFT;

    if ((numDataBits > SERIAL_MAX_NUM_DATA_BITS) || (numDataBits < SERIAL_MIN_NUM_DATA_BITS))
        return false;

    if (parityType == SERIAL_PARITY_RESERVED)
        return false;

    // Disable interrupts and configure UART module.
    cli();

    // Write baudrate
    // Assume we only use 16x oversampling.
    uint16_t baudReg = (4 * (uint32_t)F_CPU) / baudrate;
    channel->BAUDH = (baudReg & 0xF0) >> 8;
    channel->BAUDL = (baudReg & 0x0F) >> 0;

    // Set frame format
    channel->CTRLC = 0;
    channel->CTRLC |= ((numDataBits - 5) << USART_CHSIZE_gp) & USART_CHSIZE_gm;
    channel->CTRLC |= (numStopBits << USART_SBMODE_bp) & USART_SBMODE_bm;
    channel->CTRLC |= (parityType << USART_PMODE_gp) & USART_PMODE_gm;

    // Enable transmitter and receiver and enforce UART normal mode.
    channel->CTRLB = (USART_RXEN_bm | USART_TXEN_bm);

    // Enable needed interrupts.
    channel->CTRLA = (USART_RXCIE_bm | USART_TXCIE_bm);

    // Enable interrupts and return true.
    sei();
    return true;
}
