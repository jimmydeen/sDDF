/* We need to determine direction of chars to get*/

#include <stdbool.h>
#include <stdint.h>
#include <sel4cp.h>
#include <sel4/sel4.h>
#include "serial.h"
#include "shared_ringbuffer.h"

#define CLI_CH 10
#define DRV_CH 9

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

int escape_character;


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

    sel4cp_dbg_puts("mux tx init finished\n");  

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

    handle_tx(ch);
}
