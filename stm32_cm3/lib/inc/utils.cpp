#include "utils.h"

void cicularBufferInit(sensors_circular_buffer_t *circular_buffer)
{
    circular_buffer->head = 0;
    circular_buffer->tail = 0;
    circular_buffer->full = false;
}

bool circularBufferEmpty(sensors_circular_buffer_t *circular_buffer)
{
    return (circular_buffer->head == circular_buffer->tail) && !circular_buffer->full;
}

bool circularBufferFull(sensors_circular_buffer_t *circular_buffer)
{
    return circular_buffer->full;
}

void circularBufferPush(sensors_circular_buffer_t *circular_buffer, float data)
{
    circular_buffer->buffer[circular_buffer->head] = data;
    circular_buffer->head = (circular_buffer->head + 1) % BUFFER_SIZE;

    if (circular_buffer->head == circular_buffer->tail)
    {
        circular_buffer->full = true;
    }
    else
    {
        circular_buffer->full = false;
    }
}

void circularBufferPop(sensors_circular_buffer_t *circular_buffer)
{
    if (!circularBufferEmpty(circular_buffer))
    {
        circular_buffer->tail = (circular_buffer->tail + 1) % BUFFER_SIZE;
        circular_buffer->full = false;
    }
}

float circularBufferGetRelativeElement(sensors_circular_buffer_t *circular_buffer, int index)
{
    if (circularBufferEmpty(circular_buffer))
    {
        return 0.0; 
    }
    int relative_index = circular_buffer->head + index;
    while (relative_index < 0)
    {
        relative_index += BUFFER_SIZE; 
    }

    uint8_t final_index = relative_index % BUFFER_SIZE;
    return circular_buffer->buffer[final_index];
}

void circularBufferClean(sensors_circular_buffer_t *circular_buffer)
{
    uint16_t circular_buffer_index;
    for(circular_buffer_index = 0; circular_buffer_index < BUFFER_SIZE; circular_buffer_index++)
    {
        circularBufferPush(circular_buffer, 0);
    }
}

void * operator new(size_t size)
{
    return pvPortMalloc(size);
}

void * operator new[](size_t size)
{
    return pvPortMalloc(size);
}

void operator delete(void * ptr)
{
    vPortFree (ptr);
}

void operator delete[](void * ptr)
{
    vPortFree (ptr);
}