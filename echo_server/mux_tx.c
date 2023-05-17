/* The policy of the mux tx is that all clients can always request to transmit */

#include <stdbool.h>
#include <stdint.h>
#include <sel4cp.h>
#include <sel4/sel4.h>
#include "serial.h"
#include "shared_ringbuffer.h"
#include "util.h"

/* TODO: ADD IN DIFFERENT COLOURS TO DIFFERENTIATE DIFFERENT CLIENT STREAMS */

#define CLI_CH 10
#define DRV_CH 11

#define NUM_CLIENTS 1

/* Memory regions as defined in the system file */

// Transmit rings with the driver
uintptr_t tx_avail_drv;
uintptr_t tx_used_drv;

// Transmit rings with the client
uintptr_t tx_avail_cli;
uintptr_t tx_used_cli;

// Have an array of client rings. 
ring_handle_t cli_tx_ring[NUM_CLIENTS];

ring_handle_t drv_tx_ring;

int handle_tx(int curr_client) {
    sel4cp_dbg_puts("In the mux tx handle tx function\n");
    // Copy data from the client ring to the driver rings.
    uintptr_t buffer = 0;
    unsigned int len = 0;
    void *cookie = 0;

    ring_handle_t tx_ring = cli_tx_ring[curr_client];

    while(!driver_dequeue(tx_ring.used_ring, &buffer, &len, &cookie)) {
        // We want to enqueue into the drivers used ring
        uintptr_t drv_buffer = 0;
        unsigned int drv_len = 0;
        void *drv_cookie = 0;

        int ret = driver_dequeue(drv_tx_ring.avail_ring, &drv_buffer, &drv_len, &drv_cookie);
        if (ret != 0) {
            sel4cp_dbg_puts("Failed to dequeue buffer from drv tx avail ring\n");
            return;
        }
        memcpy(drv_buffer, buffer, len);
        drv_len = len;
        drv_cookie = cookie;

        ret = enqueue_used(drv_tx_ring, &drv_buffer, &drv_len, &drv_cookie);
        if (ret != 0) {
            sel4cp_dbg_puts("Failed to enqueue buffer to drv tx used ring\n");
            // Don't know if I should return here, because we need to enqueue a
            // serpeate buffer
        }

        // enqueue back to the client avail ring
        enqueue_avail(tx_ring.used_ring, &buffer, &len, &cookie);
    }
}

void init (void) {
    // We want to init the client rings here. Currently this only inits one client
    ring_init(&cli_tx_ring[0], (ring_buffer_t *)tx_avail_cli, (ring_buffer_t *)tx_used_cli, NULL, 0);
    ring_init(&drv_tx_ring, (ring_buffer_t *)tx_avail_drv, (ring_buffer_t *)tx_used_drv, NULL, 0);

}

void notified(sel4cp_channel ch) {
    // We should only ever recieve notifications from the client
    // Sanity check the client
    if (curr_client < 1 || curr_client > NUM_CLIENTS) {
        sel4cp_dbg_puts("Received a bad client channel\n");
        return;
    }

    handle_tx(ch);
}
