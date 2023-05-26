/*
 * Copyright 2022, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */


#include <stdbool.h>
#include <stdint.h>
#include <sel4cp.h>
#include <sel4/sel4.h>
#include "serial.h"
#include "shared_ringbuffer.h"
#include "util.h"
#include <string.h>

#define CLI_CH 10
#define DRV_CH 9

#define NUM_CLIENTS 2

static char cml_memory[2048*1024*2];

// /* exported in cake.S */
extern void cml_main(void);

extern void *cml_heap;
extern void *cml_stack;
extern void *cml_stackend;

// Transmit rings with the driver
uintptr_t tx_avail_drv;
uintptr_t tx_used_drv;

// Transmit rings with the client
uintptr_t tx_avail_cli;
uintptr_t tx_used_cli;
uintptr_t tx_avail_cli2;
uintptr_t tx_used_cli2;

uintptr_t shared_dma_tx_drv;
uintptr_t shared_dma_tx_cli;
uintptr_t shared_dma_tx_cli2;

// Have an array of client rings. 
ring_handle_t tx_ring[NUM_CLIENTS];
ring_handle_t drv_tx_ring;

/*------------ Helper functions ------------ */

/* Helper FFI functions copied from the cakeml standard basis_ffi template */
void int_to_byte4(int i, unsigned char *b){
    /* i is encoded on 8 bytes */
    /* i is cast to long long to ensure having 64 bits */
    /* assumes CHAR_BIT = 8. use static assertion checks? */
    b[0] = ((uint32_t) i >> 24) & 0xFF;
    b[1] = ((uint32_t) i >> 16) & 0xFF;
    b[2] = ((uint32_t) i >> 8) & 0xFF;
    b[3] =  (uint32_t) i & 0xFF;
}

int byte4_to_int(unsigned char *b){
    return ((b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3]);
}

void int_to_byte2(int i, unsigned char *b){
    /* i is encoded on 2 bytes */
    b[0] = (i >> 8) & 0xFF;
    b[1] = i & 0xFF;
}

int byte2_to_int(unsigned char *b){
    return ((b[0] << 8) | b[1]);
}

void int_to_byte8(int i, unsigned char *b){
    /* i is encoded on 8 bytes */
    /* i is cast to long long to ensure having 64 bits */
    /* assumes CHAR_BIT = 8. use static assertion checks? */
    b[0] = ((long long) i >> 56) & 0xFF;
    b[1] = ((long long) i >> 48) & 0xFF;
    b[2] = ((long long) i >> 40) & 0xFF;
    b[3] = ((long long) i >> 32) & 0xFF;
    b[4] = ((long long) i >> 24) & 0xFF;
    b[5] = ((long long) i >> 16) & 0xFF;
    b[6] = ((long long) i >> 8) & 0xFF;
    b[7] =  (long long) i & 0xFF;
}

int byte8_to_int(unsigned char *b){
    return (((long long) b[0] << 56) | ((long long) b[1] << 48) |
             ((long long) b[2] << 40) | ((long long) b[3] << 32) |
             (b[4] << 24) | (b[5] << 16) | (b[6] << 8) | b[7]);
}

void uintptr_to_byte8(uintptr_t i, unsigned char *b){
    /* i is encoded on 8 bytes */
    /* i is cast to long long to ensure having 64 bits */
    /* assumes CHAR_BIT = 8. use static assertion checks? */
    b[0] = (char) ( i >> 56) & 0xFF;
    b[1] = (char) ( i >> 48) & 0xFF;
    b[2] = (char) ( i >> 40) & 0xFF;
    b[3] = (char) ( i >> 32) & 0xFF;
    b[4] = (char) ( i >> 24) & 0xFF;
    b[5] = (char) ( i >> 16) & 0xFF;
    b[6] = (char) ( i >> 8) & 0xFF;
    b[7] = (char) i & 0xFF;
}

uintptr_t byte8_to_uintptr(unsigned char *b){
    return (((uintptr_t) b[0] << 56) | ((uintptr_t) b[1] << 48) |
             ((uintptr_t) b[2] << 40) | ((uintptr_t) b[3] << 32) |
             ((uintptr_t) b[4] << 24) | (uintptr_t) (b[5] << 16) | (uintptr_t) (b[6] << 8) | b[7]);
}

/*------------ MUX FFI functions ------------ */

/* Get the hash define of NUM_CLIENTS. Can potentially in the future just run the C preprocessor over the Pancake file.
This method allows us to only have to change the hash define once. */
void ffinum_clients(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("In the mux tx num clients function\n");
    if (alen != 1) {
        return;
    }

    a[0] = NUM_CLIENTS;
}


/* Wrapper around the ring empty function. Specifically for the drv ring */
void ffidrv_ring_empty(unsigned char *c, long clen, unsigned char *a, long alen) {

    if (clen != 1 || alen != 1) {
        // Insufficient args
        sel4cp_dbg_puts("Insufficient args or ret space\n");
        return;
    }
    int ring = c[0];
    // Arbitrarily initialising ret to 5, don't want to init to 0 or 1
    char ret = 5;

    if (ring == 0) {
        // 0 case for used ring
        sel4cp_dbg_puts("In the drv ring empty used case\n");
        ret = (char) ring_empty(drv_tx_ring.used_ring);
    } else if (ring == 1) {
        // 1 case for avail ring
        ret = (char) ring_empty(drv_tx_ring.avail_ring);
    }

    if (ret) {
        sel4cp_dbg_puts("The ring was empty\n");
    }

    a[0] = ret;
}

/* Wrapper around the dequeue used function. Takes in client number as an argument */
void ffidequeue_used(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("In the dequeue used funtion\n");
    if (clen != 1 || alen != 24) {
        sel4cp_dbg_puts("Insufficient args or ret space\n");
        return;
    }

    int client = c[0];

    // Check that we have recieved a valid client number
    if (client < 0 || client >= NUM_CLIENTS) {
        sel4cp_dbg_puts("Incorrect client number\n");
        c[0] = 1;
        return;
    }

    uintptr_t buffer;
    unsigned int len = 0;
    void *cookie = 0;

    int ret = dequeue_used(&tx_ring[client], &buffer, &len, &cookie);

    if (ret) {
        sel4cp_dbg_puts("WE SHOULD BE BREAKING FROM THE LOOP\n");
        c[0] = 1;
        return;
    } else {
        c[0] = 0;
    }
    


    // Place the ret value in the C array, place all the other arguments in the A array
    // c[0] = ret;
    uintptr_to_byte8(buffer, a);
    uintptr_to_byte8(len, &a[8]);
    uintptr_to_byte8(cookie, &a[16]);

    return;
}

void ffibatch_driver_dequeue_enqueue(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("In the batch driver dequeue enqueue function\n");
    if (clen != 24 || alen != 1) {
        sel4cp_dbg_puts("Insufficient args or ret space\n");
        return;
    }

    uintptr_t buffer = byte8_to_uintptr(c);
    unsigned int len = byte8_to_uintptr(&c[8]);
    void *cookie = byte8_to_uintptr(&c[16]);

    uintptr_t drv_buffer = 0;
    unsigned int drv_len = 0;
    void *drv_cookie = 0;


    int ret = driver_dequeue(drv_tx_ring.avail_ring, &drv_buffer, &drv_len, &drv_cookie);
    if (ret != 0) {
        sel4cp_dbg_puts("Failed to dequeue buffer from drv tx avail ring\n");
        a[0] = 1;
        return;
    }

    char *string = (char *) buffer;

    memcpy((char *) drv_buffer, string, len);
    drv_len = len;
    drv_cookie = cookie;

    ret = enqueue_used(&drv_tx_ring, drv_buffer, drv_len, drv_cookie);
    if (ret != 0) {
        sel4cp_dbg_puts("Failed to enqueue buffer to drv tx used ring\n");
        // Don't know if I should return here, because we need to enqueue a
        // serpeate buffer
    }

    a[0] = 0;
}

/* Wrapper around the enqueue avail function for the client rings */
void fficli_enqueue_avail(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("In the cli enqueue avail function\n");
    if (clen != 24 || alen != 1) {
        sel4cp_dbg_puts("Insufficient args\n");
        return;
    }

    int client = a[0];


    // Check that we have recieved a valid client number
    if (client < 0 || client >= NUM_CLIENTS) {
        sel4cp_dbg_puts("Incorrect client number\n");
        return;
    }

    uintptr_t buffer = byte8_to_uintptr(c);
    unsigned int len = byte8_to_uintptr(&c[8]);
    void *cookie = byte8_to_uintptr(&c[16]);

    enqueue_avail(&tx_ring[client], buffer, len, cookie);

}

void ffinotify_driver() {
    sel4cp_dbg_puts("WE ARE NOTIFYING THE DRIVER THAT WE HAVE SOMETHING TO PRINT\n");
    sel4cp_notify(DRV_CH);
}

/*---------- Functions needed by cakeml ----------*/
void cml_exit(int arg) {
    // We should never get to this function
    sel4cp_dbg_puts("In the cml_exit function, we should not be here\n");    
}

void cml_clear() {
    sel4cp_dbg_puts("Trying to clear cache, we should not be here\n");
}

void init_pancake_mem() {
    unsigned long sz = 2048*1024; // 1 MB unit\n",
    unsigned long cml_heap_sz = sz;    // Default: 1 MB heap\n", (* TODO: parameterise *)
    unsigned long cml_stack_sz = sz;   // Default: 1 MB stack\n", (* TODO: parameterise *)
    cml_heap = &cml_memory[0];
    cml_stack = cml_heap + cml_heap_sz;
    cml_stackend = cml_stack + cml_stack_sz;
}

/*---------- sel4cp Entry Points ----------*/

void init(void)
{
    init_pancake_mem();

    // We want to init the client rings here. Currently this only inits one client
    ring_init(&tx_ring[0], (ring_buffer_t *)tx_avail_cli, (ring_buffer_t *)tx_used_cli, NULL, 0);
    ring_init(&tx_ring[1], (ring_buffer_t *)tx_avail_cli2, (ring_buffer_t *)tx_used_cli2, NULL, 0);
    ring_init(&drv_tx_ring, (ring_buffer_t *)tx_avail_drv, (ring_buffer_t *)tx_used_drv, NULL, 0);

    // Add buffers to the drv tx ring from our shared dma region
    for (int i = 0; i < NUM_BUFFERS - 1; i++) {
        // Have to start at the memory region left of by the rx ring
        int ret = enqueue_avail(&drv_tx_ring, shared_dma_tx_drv + ((i + NUM_BUFFERS) * BUFFER_SIZE), BUFFER_SIZE, NULL);

        if (ret != 0) {
            sel4cp_dbg_puts(sel4cp_name);
            sel4cp_dbg_puts(": tx buffer population, unable to enqueue buffer\n");
        }
    }

    sel4cp_dbg_puts("mux tx init finished\n"); 
}

void notified(sel4cp_channel ch)
{
    sel4cp_dbg_puts("In the mux tx ffi notified function\n");
    if (ch < 1 || ch > NUM_CLIENTS) {
        sel4cp_dbg_puts("Received a bad client channel\n");
        return;
    }

    cml_main();
}