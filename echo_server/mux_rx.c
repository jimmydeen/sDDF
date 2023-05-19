/* We need to determine direction of chars to get*/

#include <stdbool.h>
#include <stdint.h>
#include <sel4cp.h>
#include <sel4/sel4.h>
#include "serial.h"
#include "shared_ringbuffer.h"

#define CLI_CH 1
#define DRV_CH 11

#define NUM_CLIENTS 1

/* Memory regions as defined in the system file */

// Transmit rings with the driver
uintptr_t rx_avail_drv;
uintptr_t rx_used_drv;

// Transmit rings with the client
uintptr_t rx_avail_cli;
uintptr_t rx_used_cli;

uintptr_t shared_dma_rx_drv;
uintptr_t shared_dma_rx_cli;

// Have an array of client rings. 
ring_handle_t rx_ring;
ring_handle_t drv_rx_ring;

/* We need to do some processing of the input stream to determine when we need 
to change direction. */

/* To switch input direction, type the "@" symbol followed immediately by a number.
Otherwise, can put "\" before "@" to escape this.*/

int escape_character;
int client;

int give_char(int curr_client, char got_char) {

}

/* We will check for escape characters in here, as well as dealing with switching direction*/
int handle_rx(int curr_client) {
    // We want to request a character here, then busy wait until we get anything back

    // Notify driver that we want to get a character
    sel4cp_notify(DRV_CH);

    // Address that we will pass to dequeue to store the buffer address
    uintptr_t buffer = 0;
    // Integer to store the length of the buffer
    unsigned int buffer_len = 0; 

    void *cookie = 0;

    sel4cp_dbg_puts("Busy waiting until we are able to dequeue something from the rx ring buffer\n");
    while (dequeue_used(&rx_ring, &buffer, &buffer_len, &cookie) != 0) {
        /* The ring is currently empty, as there is no character to get. 
        We will spin here until we have gotten a character. As the driver is a higher priority than us, 
        it should be able to pre-empt this loop
        */
        sel4cp_dbg_puts(""); /* From Patrick, this is apparently needed to stop the compiler from optimising out the 
        as it is currently empty. When compiled in a release version the puts statement will be compiled
        into a nop command.
        */
    }

    sel4cp_dbg_puts("Finished looping, dequeue used buffer successfully\n");

    // We are only getting one character at a time, so we just need to cast the buffer to an int

    char got_char = *((char *) buffer);

    /* Now that we are finished with the used buffer, we can add it back to the available ring*/

    int ret = enqueue_avail(&rx_ring, buffer, buffer_len, NULL);

    if (ret != 0) {
        sel4cp_dbg_puts(sel4cp_name);
        sel4cp_dbg_puts(": getchar - unable to enqueue used buffer back into available ring\n");
    }

    // We have now gotten a character, deal with the input direction switch

    if (escape_character == 0) {
        // No escape character has been set
        if (got_char == "\\") {
            escape_character == 1;
            // We want to somehow grab another character here
        } else {
            give_char(curr_client, got_char);
        }
    } else if (escape_character == 1) {
        // We have previously recieved the escape character
    } else if (escape_character == 2) {

    } 



}

void init (void) {
    // We want to init the client rings here. Currently this only inits one client
    ring_init(&rx_ring, (ring_buffer_t *)rx_avail_cli, (ring_buffer_t *)rx_used_cli, NULL, 0);
    ring_init(&drv_rx_ring, (ring_buffer_t *)rx_avail_drv, (ring_buffer_t *)rx_used_drv, NULL, 0);

    for (int i = 0; i < NUM_BUFFERS - 1; i++) {
        int ret = enqueue_avail(&local_server->rx_ring, shared_dma_rx_drv + (i * BUFFER_SIZE), BUFFER_SIZE, NULL);

        if (ret != 0) {
            sel4cp_dbg_puts(sel4cp_name);
            sel4cp_dbg_puts(": rx buffer population, unable to enqueue buffer\n");
        }
    }

    // We initialise the current client to 1
    client = 1;

    sel4cp_dbg_puts("mux rx init finished\n");  
}

void notified(sel4cp_channel ch) {
    sel4cp_dbg_puts("In the mux tx notified channel: ");
    // puthex64(ch);
    sel4cp_dbg_puts("\n");
    // We should only ever recieve notifications from the client
    // Sanity check the client
    if (ch < 1 || ch > NUM_CLIENTS) {
        sel4cp_dbg_puts("Received a bad client channel\n");
        return;
    }

    handle_rx(ch);
}
