#ifndef UTILS_H
#define UTILS_H

#define BUFFER_SIZE     100

#include <stdint.h>

typedef struct sensors_circular_buffer 
{
    float buffer[BUFFER_SIZE];
    uint8_t head;
    uint8_t tail;
    bool full;
} sensors_circular_buffer_t;

void cicularBufferInit(sensors_circular_buffer_t *circular_buffer);
bool circularBufferEmpty(sensors_circular_buffer_t *circular_buffer);
bool circularBufferFull(sensors_circular_buffer_t *circular_buffer);
void circularBufferPush(sensors_circular_buffer_t *circular_buffer, float data);
void circularBufferPop(sensors_circular_buffer_t *circular_buffer);


#endif