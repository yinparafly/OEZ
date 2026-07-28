#include "ring_buffer.h"

RingBuffer_t g_buffer;

void Buffer_Init(void) {
    g_buffer.head = 0;
    g_buffer.tail = 0;
    g_buffer.count = 0;
}

bool Buffer_Write(uint32_t value) {
    uint32_t next = (g_buffer.head + 1) & RING_BUFFER_MASK;
    if (next == g_buffer.tail) {
        g_buffer.tail = (g_buffer.tail + 1) & RING_BUFFER_MASK;
        if (g_buffer.count > 0) g_buffer.count--;
    }
    g_buffer.data[g_buffer.head] = value;
    g_buffer.head = next;
    g_buffer.count++;
    return true;
}

uint32_t Buffer_Read(uint32_t offset) {
    uint32_t pos = (g_buffer.tail + offset) & RING_BUFFER_MASK;
    return g_buffer.data[pos];
}

uint32_t Buffer_GetCount(void) { return g_buffer.count; }
uint32_t Buffer_GetHead(void) { return g_buffer.head; }
