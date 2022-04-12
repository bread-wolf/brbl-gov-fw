/*
 * Author: Augustin.B
 * Date  : 09.04.2022
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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
bool serial_init(USART_t* channel, uint32_t baudrate, serial_format format);

// Read and write uint8_t arrays from/to uart ring buffers.
bool serial_read(USART_t* channel, uint8_t* rData, size_t numBytes);
bool serial_write(USART_t* channel, const uint8_t* wData, size_t numBytes);

// Returns bytes available for read, and remaining free space in write buffer.
int serial_available(USART_t* channel);
int serial_availableForWrite(USART_t* channel);

// Wait for any data in transmit buffer to actually transmit.
void serial_flush(USART_t* channel);

// Discard received data that has not been read.
void serial_clear(USART_t* channel);


#endif /* SERIAL_H_ */