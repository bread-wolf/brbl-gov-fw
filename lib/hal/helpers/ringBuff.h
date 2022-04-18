/*
 * Author: Augustin.B
 * Date  : 13.04.2022
 */

#ifndef RINGBUFF_H_
#define RINGBUFF_H_

#include <avr/interrupt.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t* const data;
    size_t head, tail;
    const size_t length; // Length must be a power of 2.
} ringBuff;

inline __attribute__((always_inline)) bool ringBuff_push(ringBuff* buffer, const uint8_t* wByte)
{
    size_t next = (buffer->head + 1) & (buffer->length - 1);
    if (next == buffer->tail)
        return false;

    buffer->data[buffer->head] = *wByte;

    cli();
    buffer->head = next;
    sei();

    return true;
}

inline __attribute__((always_inline)) bool ringBuff_pop(ringBuff* buffer, uint8_t* rByte)
{
    if (buffer->head == buffer->tail)
        return false;

    size_t next = (buffer->tail + 1) & (buffer->length - 1);
    *rByte = buffer->data[buffer->tail];

    cli();
    buffer->tail = next;
    sei();

    return true;
}

inline __attribute__((always_inline)) size_t ringBuff_count(ringBuff* buffer)
{
    return (buffer->head >= buffer->tail) ? (buffer->head - buffer->tail) : (buffer->length - (buffer->tail - buffer->head));
}

inline __attribute__((always_inline)) void ringBuff_clear(ringBuff* buffer)
{
    cli();
    buffer->head = 0;
    buffer->tail = 0;
    sei();
}

#endif /* RINGBUFF_H_ */