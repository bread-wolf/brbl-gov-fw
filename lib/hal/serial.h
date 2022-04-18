/*
 * Author: Augustin.B
 * Date  : 09.04.2022
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <avr/io.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Serial ring buffer struct, used internally.
typedef struct {
    uint8_t* const data;
    volatile size_t head, tail;
    const size_t length; // Length must be a power of 2.
} serial_buffer;

// Define SERIAL_USE_UARTx in serial.c to instantiate needed channels.
typedef struct
{
    USART_t* const serial_reg;
    serial_buffer* const serial_rxBuff;
    serial_buffer* const serial_txBuff;
} serial_channel;
extern serial_channel serial_channel0;
extern serial_channel serial_channel1;
extern serial_channel serial_channel2;
extern serial_channel serial_channel3;

typedef enum
{
    SERIAL_FRAME_7N1 = 0x38,
    SERIAL_FRAME_7N2 = 0x39,
    SERIAL_FRAME_7E1 = 0x3C,
    SERIAL_FRAME_7E2 = 0x3D,
    SERIAL_FRAME_7O1 = 0x3E,
    SERIAL_FRAME_7O2 = 0x3F,
    SERIAL_FRAME_8N1 = 0x40,
    SERIAL_FRAME_8N2 = 0x41,
    SERIAL_FRAME_8E1 = 0x44,
    SERIAL_FRAME_8E2 = 0x45,
    SERIAL_FRAME_8O1 = 0x46,
    SERIAL_FRAME_8O2 = 0x47,
} serial_format;

// Used to init or reinit chosen UART peripheral.
// Does not handle pin mux or direction (RX and TX need to be set as input and output).
bool serial_init(serial_channel channel, uint32_t baudrate, serial_format format);

// Read and write single byte from/to uart ring buffers.
bool serial_read(serial_channel channel, uint8_t* rData);
bool serial_write(serial_channel channel, const uint8_t* wData);

// Returns bytes available for reading, and remaining free space in write buffer.
int serial_available(serial_channel channel);
int serial_availableForWrite(serial_channel channel);

// Wait for any data in transmit buffer to actually transmit.
void serial_flush(serial_channel channel);

// Discard received data that has not been read.
void serial_clear(serial_channel channel);


#endif /* SERIAL_H_ */