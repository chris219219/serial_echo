#ifndef RINGBUF_H
#define RINGBUF_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Structure to store ring buffer information.
 * 
 * The ring buffer structure is dynamically allocated with ringbuf_new(). It
 * stores a byte buffer, the size of the byte buffer, and the start and end
 * indexes of the ring buffer. The current amount of bytes stored in the ring
 * buffer can be retrieved with ringbuf_len().
 */
typedef struct ringbuf_t ringbuf_t;
struct ringbuf_t
{
    size_t maxlen; /** Length of the byte buffer. */
    size_t start; /** Start index of the ring buffer. */
    size_t end; /** End index of the ring buffer. */
    uint8_t buf[]; /** The byte buffer. */
};

ringbuf_t *ringbuf_new(size_t size);
bool ringbuf_push(ringbuf_t *ringbuf, uint8_t data);
bool ringbuf_pop(ringbuf_t *ringbuf, uint8_t *data);
size_t ringbuf_push_buf(ringbuf_t *ringbuf, uint8_t *buf, size_t n);
size_t ringbuf_pop_buf(ringbuf_t *ringbuf, uint8_t *buf, size_t n);
size_t ringbuf_len(ringbuf_t *ringbuf);
void ringbuf_print_all(ringbuf_t *ringbuf);

#endif