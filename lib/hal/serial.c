/*
 * Author: Augustin.B
 * Date  : 09.04.2022
 */

#include "serial.h"

#include "ringBuff.h"

#include "avr/interrupt.h"
#include <avr/io.h>

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

// Ring buffer length, must be a power of 2.
#define SERIAL_RING_BUFF_LENGTH          16

// Instantiate serial_channel structs here.
// Structs contain the pointer to the USART registers, as well as a ring buffer as defined in helpers/ringBuff.h
// If we don't use the USART, then the struct is instantiated with NULL pointers which would make the uart init fail.
#ifndef SERIAL_USE_UART0
serial_channel serial_channel0 = {
    .serial_reg = NULL,
    .serial_ringBuff = NULL,
};
#else
static uint8_t serial_channel0_data[SERIAL_RING_BUFF_LENGTH];
static ringBuff serial_channel0_rb = {
    .data = serial_channel0_data,
    .head = 0,
    .tail = 0,
    .length = SERIAL_RING_BUFF_LENGTH,
};
serial_channel serial_channel0 = {
    .serial_reg = USART0,
    .serial_ringBuff = &serial_channel0_rb,
};
#endif /* SERIAL_USE_UART0 */

#ifndef SERIAL_USE_UART1
serial_channel serial_channel1 = {
    .serial_reg = NULL,
    .serial_ringBuff = NULL,
};
#else
static uint8_t serial_channel1_data[SERIAL_RING_BUFF_LENGTH];
static ringBuff serial_channel1_rb = {
    .data = serial_channel1_data,
    .head = 0,
    .tail = 0,
    .length = SERIAL_RING_BUFF_LENGTH,
};
serial_channel serial_channel1 = {
    .serial_reg = USART1,
    .serial_ringBuff = &serial_channel1_rb,
};
#endif  /* SERIAL_USE_UART1 */

#ifndef SERIAL_USE_UART2
serial_channel serial_channel2 = {
    .serial_reg = NULL,
    .serial_ringBuff = NULL,
};
#else
static uint8_t serial_channel2_data[SERIAL_RING_BUFF_LENGTH];
static ringBuff serial_channel2_rb = {
    .data = serial_channel2_data,
    .head = 0,
    .tail = 0,
    .length = SERIAL_RING_BUFF_LENGTH,
};
serial_channel serial_channel2 = {
    .serial_reg = USART2,
    .serial_ringBuff = &serial_channel2_rb,
};
#endif  /* SERIAL_USE_UART2 */

#ifndef SERIAL_USE_UART3
serial_channel serial_channel3 = {
    .serial_reg = NULL,
    .serial_ringBuff = NULL,
};
#else
static uint8_t serial_channel3_data[SERIAL_RING_BUFF_LENGTH];
static ringBuff serial_channel3_rb = {
    .data = serial_channel3_data,
    .head = 0,
    .tail = 0,
    .length = SERIAL_RING_BUFF_LENGTH,
};
serial_channel serial_channel3 = {
    .serial_reg = USART3,
    .serial_ringBuff = &serial_channel3_rb,
};
#endif  /* SERIAL_USE_UART3 */



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
