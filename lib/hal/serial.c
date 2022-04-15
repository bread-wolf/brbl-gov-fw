/*
 * Author: Augustin.B
 * Date  : 09.04.2022
 */

#include "serial.h"

#include "helpers/ringBuff.h"

#include "avr/interrupt.h"
#include <avr/io.h>

// Configure which UART channels will be used.
#define SERIAL_USE_UART0
//#define SERIAL_USE_UART1
//#define SERIAL_USE_UART2
//#define SERIAL_USE_UART3

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

// UART Software buffer size, must be a power of 2
#define SERIAL_RING_BUFF_LENGTH 16

// Macro to define serial_channeln structs.
// Contain the pointer to the USART registers, as well as a ring buffer as defined in helpers/ringBuff.h.
// These are instantiated lower along with IRQ code.
#define SERIAL_INSTANTIATE_CHANNEL(n)                                     \
    static uint8_t serial_channel ## n ## _data[SERIAL_RING_BUFF_LENGTH]; \
    static ringBuff serial_channel ## n ## _rb = {                        \
        .data = serial_channel ## n ## _data,                             \
        .head = 0,                                                        \
        .tail = 0,                                                        \
        .length = SERIAL_RING_BUFF_LENGTH,                                \
    };                                                                    \
    serial_channel serial_channel ## n = {                                \
        .serial_reg = &USART ## n,                                        \
        .serial_ringBuff = &serial_channel ## n ## _rb,                   \
};

// Local IRQ handler functions, called from inside each UARTx IRQ.



bool serial_init(serial_channel channel, uint32_t baudrate, serial_format format)
{
    // Verify input arguments
    if (channel.serial_reg == NULL)
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

    // Write baudrate, fomula assumes we only use 16x oversampling.
    uint16_t baudReg = (4 * (uint32_t)F_CPU) / baudrate;
    channel.serial_reg->BAUDH = (baudReg & 0xF0) >> 8;
    channel.serial_reg->BAUDL = (baudReg & 0x0F) >> 0;

    // Set frame format
    channel.serial_reg->CTRLC = 0;
    channel.serial_reg->CTRLC |= ((numDataBits - 5) << USART_CHSIZE_gp) & USART_CHSIZE_gm;
    channel.serial_reg->CTRLC |= (numStopBits << USART_SBMODE_bp) & USART_SBMODE_bm;
    channel.serial_reg->CTRLC |= (parityType << USART_PMODE_gp) & USART_PMODE_gm;

    // Enable transmitter and receiver and enforce UART normal mode.
    channel.serial_reg->CTRLB = (USART_RXEN_bm | USART_TXEN_bm);

    // Enable Receive Complete and Transmit Complete interrupts.
    channel.serial_reg->CTRLA = (USART_RXCIE_bm | USART_TXCIE_bm);

    // Enable interrupts and return true.
    sei();
    return true;
}


#ifdef SERIAL_USE_UART0
SERIAL_INSTANTIATE_CHANNEL(0);
#endif /* SERIAL_USE_UART0 */

#ifdef SERIAL_USE_UART1
SERIAL_INSTANTIATE_CHANNEL(1);
#endif /* SERIAL_USE_UART1 */

#ifdef SERIAL_USE_UART2
SERIAL_INSTANTIATE_CHANNEL(2);
#endif /* SERIAL_USE_UART2 */

#ifdef SERIAL_USE_UART3
SERIAL_INSTANTIATE_CHANNEL(3);
#endif /* SERIAL_USE_UART3 */