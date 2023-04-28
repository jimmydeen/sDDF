/*
 * Copyright 2022, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <sel4cp.h>
#include <sel4/sel4.h>
#include "eth.h"
#include "shared_ringbuffer.h"
#include "util.h"

static char cml_memory[2048*1024*2];
// Attempt to save the address that notified needs to return to
void *notified_return;
unsigned int argc;
char **argv;

// /* exported in cake.S */
extern void cml_main(void);

extern void *cml_heap;
extern void *cml_stack;
extern void *cml_stackend;

#define MDC_FREQ    20000000UL

/* Memory regions. These all have to be here to keep compiler happy */
uintptr_t tx_avail_drv;
uintptr_t tx_used_drv;

uintptr_t tx_avail_cli;
uintptr_t tx_used_cli;

uintptr_t shared_dma_vaddr;
uintptr_t uart_base;

#define CLIENT_CH 0
#define NUM_CLIENTS 1
#define DRIVER_TX_CH 1

// Keep this in C for the moment, can maybe keep pointers to these ring handles in pancake later on
typedef struct state {
    /* Pointers to shared buffers */
    ring_handle_t tx_ring_drv;
    ring_handle_t tx_ring_clients[NUM_CLIENTS];
} state_t;

state_t state;

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

/*------------ Debugging FFI functions ------------ */

void ffiin_loop(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("Looping\n");
}

void ffibreaking(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("Attempting to break from loop");
}

/*------------ MUX FFI functions ------------ */

/* Wrapper around the ring empty function. Specifically for the drv ring */
void ffidrv_ring_empty(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In the drv ring empty function\n");
    if (clen != 1 || alen != 1) {
        // Insufficient args
        sel4cp_dbg_puts("Insufficient args or ret space\n");
        return;
    }
    int ring = c[0];

    char ret = 5;

    if (ring == 0) {
        // 0 case for used ring
        ret = (char) ring_empty(state.tx_ring_drv.used_ring);
    } else if (ring == 1) {
        // 1 case for avail ring
        ret = (char) ring_empty(state.tx_ring_drv.avail_ring);
    }

    a[0] = ret;
}

/* Wrapper around the ring empty function. Specifically for the client ring */
void fficlient_ring_empty(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In the client ring empty function\n");
    if (clen != 2 || alen != 1) {
        // Need 1 arg for client num, and one for ring distinction
        sel4cp_dbg_puts("Insufficient args or ret space\n");
        return;
    }

    int client = c[0];
    int ring = c[1];

    char ret = 5;

    if (ring == 0) {
        // 0 case for used ring
        ret = (char) ring_empty(state.tx_ring_clients[client].used_ring);
    } else if (ring == 1) {
        // 1 case for avail ring
        ret = (char) ring_empty(state.tx_ring_clients[client].avail_ring);
    }

    a[0] = ret;
}

/* Batch the dequeue and enqueue from the used rings.*/
void ffiprocess_dequeue_enqueue(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In the process dequeue enqueue func\n");
    if (alen != 1) {
        sel4cp_dbg_puts("Insufficent ret array size\n");
        return;
    }

    uintptr_t addr;
    unsigned int len;
    void *cookie;

    int err = dequeue_used(&state.tx_ring_clients[0], &addr, &len, &cookie);
    assert(!err);
    err = enqueue_used(&state.tx_ring_drv, addr, len, cookie);
    assert(!err);

    a[0] = 0;
}

/* Check paramaters, and notify the driver's transmit channel*/
void ffiprocess_set_signal(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In the process set signal func\n");
    // First 8 bytes for size of ring buffer
    // Next 4 bytes for the number of enqueues
    if (clen != 12) {
        sel4cp_dbg_puts("Insufficient args\n");
        return;
    }
    
    uint64_t size = byte8_to_uintptr(c);
    int enqueued = byte4_to_int(&c[8]);

    //sel4cp_dbg_puts("This is the value of size: ");
    // puthex64(size);
    //sel4cp_dbg_puts("\n");

    //sel4cp_dbg_puts("This is the value of enqueued: ");
    // puthex64(enqueued);
    //sel4cp_dbg_puts("\n");

    if ((size == 0 || size + enqueued != ring_size(state.tx_ring_drv.used_ring)) && enqueued != 0) {
        //sel4cp_dbg_puts("We are calling notify delayed\n");
        sel4cp_notify_delayed(DRIVER_TX_CH);
    }

    return;
}

/* Batch dequeue and enqueue from the avail rings */
void fficomplete_dequeue_enqueue(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In the complete dequeue enqueue func\n");
    uintptr_t addr;
    unsigned int len;
    void *cookie;
    dequeue_avail(&state.tx_ring_drv, &addr, &len, &cookie);
    enqueue_avail(&state.tx_ring_clients[0], addr, len, cookie);
}

/* Wrapper around the ring size function. Specifically for the drv ring */
void ffidrv_ring_size(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In the drv ring size func\n");
    if (clen != 1 || alen != 8) {
        sel4cp_dbg_puts("Insufficient size for arg or return arrays\n");
        return;
    }

    int ring = c[0];
    uint64_t size = 0;
    if (ring == 0) {
        size = ring_size(state.tx_ring_drv.used_ring);
    } else {
        size = ring_size(state.tx_ring_drv.avail_ring);
    }

    uintptr_to_byte8(size, a);

    return;
}

/**
 * Get the size of either used or avail ring of tx drv ring
*/
void ffidrv_ring_full(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (clen != 1 || alen != 1) {
        sel4cp_dbg_puts("Insufficent size for arf or return arrays\n");
        return;
    }

    int ring = c[0];

    if (ring == 0) {
        // used ring
        a[0] = ring_full(state.tx_ring_drv.used_ring);
    } else {
        // avail ring
        a[0] = ring_full(state.tx_ring_drv.avail_ring);
    }

    return;
}

/* Wrapper around the notify client sel4cp call */
void ffinotify_client(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_notify(CLIENT_CH);
}

/*---------- Functions needed by cakeml ----------*/
void cml_exit(int arg) {
    // We should never get to this function
    //sel4cp_dbg_puts("In the cml_exit function, we should not be here\n");    
}

/* Need to come up with a replacement for this clear cache function. Might be worth testing just flushing the entire l1 cache, but might cause issues with returning to this file*/
void cml_clear() {
//   __builtin___clear_cache(&cake_codebuffer_begin, &cake_codebuffer_end);
    ////sel4cp_dbg_puts("Trying to clear cache\n");
}

void init_pancake_mem() {
    ////sel4cp_dbg_puts("In the init pancake mem function\n");
    unsigned long sz = 2048*1024; // 1 MB unit\n",
    unsigned long cml_heap_sz = sz;    // Default: 1 MB heap\n", (* TODO: parameterise *)
    unsigned long cml_stack_sz = sz;   // Default: 1 MB stack\n", (* TODO: parameterise *)
    cml_heap = &cml_memory[0];
    //sel4cp_dbg_puts("Pancake heap start: ");
    // puthex64(cml_heap);
    //sel4cp_dbg_puts("\n");
    cml_stack = cml_heap + cml_heap_sz;
    //sel4cp_dbg_puts("Pancake stack start: ");
    // puthex64(cml_stack);
    //sel4cp_dbg_puts("\n");
    cml_stackend = cml_stack + cml_stack_sz;
    //sel4cp_dbg_puts("Pancake stack end: ");
    // puthex64(cml_stackend);
    //sel4cp_dbg_puts("\n");
}

/*---------- sel4cp Entry Points ----------*/

void init(void)
{
    init_pancake_mem();
    print("Pancake: mem initialised\n");
    ring_init(&state.tx_ring_drv, (ring_buffer_t *)tx_avail_drv, (ring_buffer_t *)tx_used_drv, NULL, 1);
    ring_init(&state.tx_ring_clients[0], (ring_buffer_t *)tx_avail_cli, (ring_buffer_t *)tx_used_cli, NULL, 0);
    print("MUX: initialised\n");
}

void notified(sel4cp_channel ch)
{
    sel4cp_dbg_puts("---------- In the mux tx notified func ----------\n");

    if (ch == CLIENT_CH || ch == DRIVER_TX_CH) {
        //sel4cp_dbg_puts("Entering cml_main\n");
        cml_main();
        //sel4cp_dbg_puts("Returning from pancake\n");
        // return;
    } else {
        print("MUX TX|ERROR: unexpected notification from channel: ");
        puthex64(ch);
        print("\n");
        assert(0);
            sel4cp_dbg_puts("Finished the mux tx notified func\n");

        return;
    }

        sel4cp_dbg_puts("Finished the mux tx notified func\n");

}