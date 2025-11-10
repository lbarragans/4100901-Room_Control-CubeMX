#include "ring_buffer.h"

void ring_buffer_init(ring_buffer_t* rb, uint8_t* buffer, uint16_t size) {
    rb->buffer = buffer;
    rb->size = size;
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

bool ring_buffer_write(ring_buffer_t* rb, uint8_t data) {
    if (rb->count >= rb->size)
        return false; // buffer lleno

    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % rb->size;
    rb->count++;
    return true;
}

bool ring_buffer_read(ring_buffer_t* rb, uint8_t* data) {
    if (rb->count == 0)
        return false; // buffer vacío

    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % rb->size;
    rb->count--;
    return true;
}

uint16_t ring_buffer_count(ring_buffer_t* rb) {
    return rb->count;
}
