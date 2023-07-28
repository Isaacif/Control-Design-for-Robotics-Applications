#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

typedef struct sensors_circular_buffer 
{
    uint8_t* begin;
    uint8_t* end;
    uint8_t* head;
    uint8_t* tail;
    uint32_t size;
} sensors_circular_buffer_t;

sensors_circular_buffer_t* createCicularBuffer(uint32_t size);
void destroyCircularBuffer(sensors_circular_buffer_t** buffer);
uint32_t addToRingBuffer(sensors_circular_buffer_t* buffer, uint8_t value);
uint8_t peekFromRingBuffer(sensors_circular_buffer_t* buffer);
void nPeekFromRingBuffer(sensors_circular_buffer_t* buffer, uint8_t* data, uint32_t n);
uint8_t removeFromRingBuffer(sensors_circular_buffer_t* buffer);
uint32_t nRemoveFromRingBuffer(sensors_circular_buffer_t* buffer, uint8_t* data, uint32_t n);

#endif