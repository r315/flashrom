
#include "fifo.h"

// ---------------------------------------------------
// Fifo handling
// ---------------------------------------------------

void fifo_init(fifo_t *fifo, uint8_t *buffer, uint32_t size)
{
	fifo->head = 0;
	fifo->tail = 0;
    fifo->size = 0;
    fifo->buf = buffer;
	fifo->capacity = size;
}

/**
 * Adds a character to the fifo
 *
 * @param fifo:    destination fifo
 * @param c:       character to put
 *
 * returns         1 success, 0 fifo full
 * */
uint32_t fifo_put(fifo_t *fifo, uint8_t c)
{
	// check if FIFO has room
	if (fifo->size == fifo->capacity) {
		// full
		return 0;
	}

	fifo->buf[fifo->head] = c;
    fifo->size++;
	fifo->head = (fifo->head + 1) % fifo->capacity;

	return 1;
}

/**
 *  retrieves one character from the given fifo if fifo
 *  is not empty
 *
 * @param fifo:		pointer to target fifo
 * @param pc:		placeholder for retrieved char
 * returns			1 if one char was placed on placeholder, 0 no data on fifo
 * */
uint32_t fifo_get(fifo_t *fifo, uint8_t *pc)
{
	// check if FIFO has data
	if (fifo->size == 0) {
        // Empty
		return 0;
	}

	*pc = fifo->buf[fifo->tail];
    fifo->size--;
	fifo->tail = (fifo->tail + 1) % fifo->capacity;

	return 1;
}

/**
 * get the number of bytes on fifo
 * 
 * @param fifo:		pointer to target fifo
 * returns			number of bytes on fifo, 0 if empty
 * */
uint32_t fifo_avail(fifo_t *fifo)
{
	return fifo->size;
}

/**
 * get the number of available bytes on fifo
 * 
 * @param fifo:		pointer to target fifo
 * returns			number of bytes available on fifo, 0 if full
 * */
uint32_t fifo_free(fifo_t *fifo)
{
	return fifo->capacity - fifo->size;
}

/**
 * Discard all data on fifo
 *
 * @param fifo     fifo to be discarted
 *
 * */

void fifo_flush(fifo_t *fifo)
{
	fifo->head = 0;
	fifo->tail = 0;
}

/**
 * Observe data on fifo head
 * 
 * @param fifo:		pointer to target fifo
 * return : 		value pointed by fifo head
 * */
uint32_t fifo_peek(fifo_t *fifo){
	return fifo->buf[fifo->tail];
}
