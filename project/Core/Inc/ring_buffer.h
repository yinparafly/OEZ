#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#include "stm32f10x.h"
#include <stdbool.h>

#define RING_BUFFER_SIZE    3584
#define RING_BUFFER_MASK    (RING_BUFFER_SIZE - 1)

typedef struct {
    uint32_t data[RING_BUFFER_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t count;
} RingBuffer_t;

extern RingBuffer_t g_buffer;

void Buffer_Init(void);
bool Buffer_Write(uint32_t value);
uint32_t Buffer_Read(uint32_t offset);
uint32_t Buffer_GetCount(void);
uint32_t Buffer_GetHead(void);

#endif
