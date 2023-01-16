#include "serial_server.h"
#include "serial.h"
#include "shared_ringbuffer.h"
#include <string.h>
#include <stdlib.h>
/* Ring handle components -
Need to have access to the same ring buffer mechanisms as the driver, so that we can enqueue
buffers to be serviced by the driver.*/

uintptr_t rx_avail;
uintptr_t rx_used;
uintptr_t tx_avail;
uintptr_t tx_used;

uintptr_t shared_dma;

struct serial_server global_serial_server = {};

/*
Return -1 on failure.
*/
int serial_server_printf(char *string) {
    struct serial_server *local_server = &global_serial_server;
    // Get a buffer from the tx ring

    // Address that we will pass to dequeue to store the buffer address
    uintptr_t buffer_addr;
    // Integer to store the length of the buffer
    unsigned int buffer_len; 

    // Dequeue a buffer from the available ring from the tx buffer
    int ret = dequeue_avail(&local_server->tx_ring, &buffer_addr, &buffer_len, NULL);

    if(ret != 0) {
        sel4cp_dbg_puts(sel4cp_name);
        sel4cp_dbg_puts(": serial server printf, unable to dequeue from tx ring, tx ring empty\n");
        return -1;
    }

    // Need to copy over the string into the buffer, if it is less than the buffer length
    int print_len = strlen(string);

    if(print_len > buffer_len) {
        sel4cp_dbg_puts(sel4cp_name);
        sel4cp_dbg_puts(": print string too long for buffer\n");
        return -1;
    }

    // Copy over the string to be printed to the buffer
    memcpy((void *)buffer_addr, string, print_len);

    // We then need to add this buffer to the transmit used ring structure

    bool is_empty = ring_empty(local_server->tx_ring.used_ring);

    ret = enqueue_avail(&local_server->tx_ring, buffer_addr, buffer_len, NULL);

    if(ret != 0) {
        sel4cp_dbg_puts(sel4cp_name);
        sel4cp_dbg_puts(": serial server printf, unable to enqueue to tx used ring\n");
        return -1;
    }

    /*
    First we will check if the transmit used ring is empty. If not empty, then the driver was processing
    the used ring, however it was not finished, potentially running out of budget and being pre-empted. 
    Therefore, we can just add the buffer to the used ring, and wait for the driver to resume. However if 
    empty, then we can notify the driver to start processing the used ring.
    */

    if(is_empty) {
        // Notify the driver through the printf channel
        sel4cp_notify(SERVER_PRINT_CHANNEL);
    }

    return 0;
}

// Return char on success, -1 on failure
int getchar() {
    // Notify the driver that we want to get a character. In Patrick's design, this increments 
    // the chars_for_clients value.
    sel4cp_notify(SERVER_GETCHAR_CHANNEL);

    struct serial_server *local_server = &global_serial_server;

    /* Now that we have notified the driver, we can attempt to dequeue from the used ring.
    When the driver has processed an interrupt, it will add the inputted character to the used ring.*/
    
    // Address that we will pass to dequeue to store the buffer address
    uintptr_t buffer_addr;
    // Integer to store the length of the buffer
    unsigned int buffer_len; 


    while (dequeue_used(&local_server->rx_ring, &buffer_addr, &buffer_len, NULL) != 0) {
        /* The ring is currently empty, so there is no character to get. 
        We will spin here until we have gotten a character. As the driver is a higher priority than us, 
        it should be able to pre-empt this loop
        */
    }

    // We are only getting one character at a time, so we just need to cast the buffer to an int

    char got_char = *((char *) buffer_addr);

    // Clear the buffer
    buffer_addr = 0;

    /* Now that we are finished with the used buffer, we can add it back to the available ring*/

    int ret = enqueue_avail(&local_server->rx_ring, buffer_addr, buffer_len, NULL);

    if (ret != 0) {
         sel4cp_dbg_puts(sel4cp_name);
        sel4cp_dbg_puts(": getchar - unable to enqueue used buffer back into available ring\n");
    }

    return (int) got_char;
}

// Init function required by sel4cp, initialise serial datastructres for server here
void init(void) {
    sel4cp_dbg_puts(sel4cp_name);
    sel4cp_dbg_puts(": elf PD init function running\n");

    // Here we need to init ring buffers and other data structures

    struct serial_server *local_server = &global_serial_server;
    
    // Init the shared ring buffers
    ring_init(&local_server->rx_ring, (ring_buffer_t *)rx_avail, (ring_buffer_t *)rx_used, NULL, 0);
    // We will also need to populate these rings with memory from the shared dma region

    // Add buffers to the rx ring
    for (int i = 0; i < NUM_BUFFERS; i++) {
        int ret = enqueue_avail(&local_server->rx_ring, shared_dma + (i * BUFFER_SIZE), BUFFER_SIZE, NULL);

        if (ret != 0) {
            sel4cp_dbg_puts(sel4cp_name);
            sel4cp_dbg_puts(": rx buffer population, unable to enqueue buffer\n");
        }
    }

    ring_init(&local_server->rx_ring, (ring_buffer_t *)tx_avail, (ring_buffer_t *)tx_used, NULL, 0);

    // Add buffers to the tx ring
    for (int i = 0; i < NUM_BUFFERS; i++) {
        // Have to start at the memory region left of by the rx ring
        int ret = enqueue_avail(&local_server->rx_ring, shared_dma + (i * BUFFER_SIZE) + (NUM_BUFFERS * BUFFER_SIZE), BUFFER_SIZE, NULL);

        if (ret != 0) {
            sel4cp_dbg_puts(sel4cp_name);
            sel4cp_dbg_puts(": tx buffer population, unable to enqueue buffer\n");
        }
    }

    /* Some basic tests for the serial driver */

    serial_server_printf("Attempting to use the server printf!\n");

    serial_server_printf("Enter char to test getchar");
    char test = getchar();
    serial_server_printf("We got the following char: ");
    serial_server_printf(&test);
    serial_server_printf("\n ---END OF TEST---");
    
}


void notified(sel4cp_channel ch) {
    return;
}