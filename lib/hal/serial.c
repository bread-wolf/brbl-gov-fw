/*
 * Author: Augustin.B
 * Date  : 09.04.2022
 */

#include "serial.h"

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

// UART Software buffer size, should be a power of 2 to optimize modulo operations.
#define SERIAL_RING_BUFF_LENGTH          16

// Busy wait for tx buffer to empty in case we try to write to a full buffer.
// If false, we just drop the data instead of corrupting the oldest data in the buffer.
#define SERIAL_TX_BLOCK_WHEN_FULL       true

// Macro to define serial_channeln structs.
// Contain the pointer to the USART registers, as well as a ring buffer as defined in header.
// These are instantiated lower along with IRQ code.
#define SERIAL_INSTANTIATE_CHANNEL(n)                                          \
    static uint8_t serial_channel ## n ## _rx_array[SERIAL_RING_BUFF_LENGTH];  \
    static uint8_t serial_channel ## n ## _tx_array[SERIAL_RING_BUFF_LENGTH];  \
    static serial_buffer serial_channel ## n ## _rx_readbuffer = {             \
        .data = serial_channel ## n ## _rx_array,                              \
        .head = 0,                                                             \
        .tail = 0,                                                             \
        .length = SERIAL_RING_BUFF_LENGTH,                                     \
    };                                                                         \
    static serial_buffer serial_channel ## n ## _tx_readbuffer = {             \
        .data = serial_channel ## n ## _tx_array,                              \
        .head = 0,                                                             \
        .tail = 0,                                                             \
        .length = SERIAL_RING_BUFF_LENGTH,                                     \
    };                                                                         \
    serial_channel serial_channel ## n = {                                     \
        .serial_reg = &USART ## n,                                             \
        .serial_rxBuff = &serial_channel ## n ## _rx_readbuffer,               \
        .serial_txBuff = &serial_channel ## n ## _tx_readbuffer,               \
};

// Local IRQ handler functions, called from inside each UARTx IRQ.
static void handle_receiveComplete(serial_channel channel);
static void handle_transmitEmpty(serial_channel channel);

// Helper to calculate the number of elements in a ring buffer.
static size_t serial_count(serial_buffer* buffer);



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

    // Calculate baudrate register, fomula assumes we only use 16x oversampling.
    uint16_t baudReg = ((40 * (uint32_t)F_CPU) / baudrate) / 10;

    // Make sure interrupts are disabled during initialization.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        // Write baudrate
    channel.serial_reg->BAUD = baudReg;

        // Set frame format (Top two bits are left to 0 for synchronous mode).
    channel.serial_reg->CTRLC = 0;
    channel.serial_reg->CTRLC |= ((numDataBits - 5) << USART_CHSIZE_gp) & USART_CHSIZE_gm;
    channel.serial_reg->CTRLC |= (numStopBits << USART_SBMODE_bp) & USART_SBMODE_bm;
    channel.serial_reg->CTRLC |= (parityType << USART_PMODE_gp) & USART_PMODE_gm;

        // Enable normal mode by clearing RXMODE field, and enable TX and RX.
        // It is still possible to enable SFDEN or ODME by configuring before calling this serial_init function.
        channel.serial_reg->CTRLB &= ~(USART_RXMODE1_bm | USART_RXMODE0_bm);
        channel.serial_reg->CTRLB |= (USART_RXEN_bm | USART_TXEN_bm);

        // Enable Receive Complete interrupts.
        // Data register empty (UDRE) interrupt is configured elsewhere.
        channel.serial_reg->CTRLA = (USART_RXCIE_bm);
    }

    return true;
}

bool serial_read(serial_channel channel, uint8_t* rData)
{
    // If head == tail, buffer is empty, return false.
    if (channel.serial_rxBuff->head == channel.serial_rxBuff->tail)
        return false;

    // Calculate next tail, and read data into rData at current tail.
    size_t next = (channel.serial_rxBuff->tail + 1) & (channel.serial_rxBuff->length - 1);
    *rData = channel.serial_rxBuff->data[channel.serial_rxBuff->tail];

    // Atomically update tail and return true.
    cli();
    channel.serial_rxBuff->tail = next;
    sei();

    return true;
}

bool serial_write(serial_channel channel, const uint8_t* wData)
{
    // Calculate next head, if it collides with tail, buffer is full, return false.
    size_t next = (channel.serial_txBuff->head + 1) & (channel.serial_txBuff->length - 1);
    if (next == channel.serial_txBuff->tail)
        return false;

    // Write wData into data array at current head.
    channel.serial_txBuff->data[channel.serial_txBuff->head] = *wData;

    // Atomically update head, and return.
    cli();
    channel.serial_txBuff->head = next;
    sei();

    return true;
}

int serial_available(serial_channel channel)
{
    return serial_count(channel.serial_rxBuff);
}

int serial_availableForWrite(serial_channel channel)
{
    return channel.serial_txBuff->length - serial_count(channel.serial_txBuff);
}

void serial_flush(serial_channel channel)
{
    while (serial_count(channel.serial_txBuff) > 0);
}

void serial_clear(serial_channel channel)
{
    // Empty hardware buffer.


    // Empty software buffer.
    cli();
    channel.serial_rxBuff->head = 0;
    channel.serial_rxBuff->tail = 0;
    sei();
}
static void handle_receiveComplete(serial_channel channel)
{

}

static void handle_transmitEmpty(serial_channel channel)
{

}

#ifdef SERIAL_USE_UART0
SERIAL_INSTANTIATE_CHANNEL(0);
ISR(USART0_RXC_vect)
{
    handle_receiveComplete(serial_channel0);
}
ISR(USART0_DRE_vect)
{
    handle_transmitEmpty(serial_channel0);
}
#endif /* SERIAL_USE_UART0 */

#ifdef SERIAL_USE_UART1
SERIAL_INSTANTIATE_CHANNEL(1);
ISR(USART1_RXC_vect)
{
    handle_receiveComplete(serial_channel1);
}
ISR(USART1_DRE_vect)
{
    handle_transmitEmpty(serial_channel1);
}
#endif /* SERIAL_USE_UART1 */

#ifdef SERIAL_USE_UART2
SERIAL_INSTANTIATE_CHANNEL(2);
ISR(USART2_RXC_vect)
{
    handle_receiveComplete(serial_channel2);
}
ISR(USART2_DRE_vect)
{
    handle_transmitEmpty(serial_channel2);
}
#endif /* SERIAL_USE_UART2 */

#ifdef SERIAL_USE_UART3
SERIAL_INSTANTIATE_CHANNEL(3);
ISR(USART3_RXC_vect)
{
    handle_receiveComplete(serial_channel3);
}
ISR(USART3_DRE_vect)
{
    handle_transmitEmpty(serial_channel3);
}
#endif /* SERIAL_USE_UART3 */