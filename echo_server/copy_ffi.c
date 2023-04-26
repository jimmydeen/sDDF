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
#include "netif/ethernet.h"

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

// cml arg 1 for just process_free, cml arg 2 for process complete and process f
int cml_arg = 0;

#define MDC_FREQ    20000000UL

/* Memory regions. These all have to be here to keep compiler happy */

uintptr_t rx_avail_mux;
uintptr_t rx_used_mux;

uintptr_t rx_avail_cli;
uintptr_t rx_used_cli;

uintptr_t shared_dma_vaddr_mux;
uintptr_t shared_dma_vaddr_cli;
uintptr_t uart_base;

#define MUX_RX_CH 0
#define CLIENT_CH 1

#define BUF_SIZE 2048
#define NUM_BUFFERS 512
#define SHARED_DMA_SIZE (BUF_SIZE * NUM_BUFFERS)

ring_handle_t rx_ring_mux;
ring_handle_t rx_ring_cli;
int initialised = 0;


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
    //sel4cp_dbg_puts("Looping\n");
}

void ffiin_inner_loop(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In inner loop\n");
}

void ffifinished_loop(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("Finished loop\n");
}

void ffibreaking(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("Attempting to break from loop");
}

void ffifirst_func(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In the first function\n");
}

void ffifinished_func(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("Finished function\n");
}

/*------------ COPY FFI functions ------------ */

/* Wrapper around the ring empty function for the FFI */
void ffiring_empty(unsigned char *c, long clen, unsigned char *a, long alen) {
    // sel4cp_dbg_puts("In the ring empty func\n");
    if (clen != 2 || alen != 1) {
        sel4cp_dbg_puts("Arg/ret arrays of incorrect size\n");
        return;
    }

    int ring_type = c[0];
    int used_avail = c[1];

    ring_handle_t ring;

    if (ring_type == 0) {
        ring = rx_ring_cli;
    } else {
        ring = rx_ring_mux;
    }

    if (used_avail == 0) {
        a[0] = ring_empty(ring.used_ring);
    } else {
        a[0] = ring_empty(ring.avail_ring);
    }
    

    return;
}

/* Wrapper around the ring full function for FFI */
void ffiring_full(unsigned char *c, long clen, unsigned char *a, long alen) {
    // sel4cp_dbg_puts("In the ring full func\n");
    if (clen != 2 || alen != 1) {
        sel4cp_dbg_puts("Arg/ret arrays of incorrect size\n");
        return;
    }

    int ring_type = c[0];
    int used_avail = c[1];

    ring_handle_t ring;

    if (ring_type == 0) {
        ring = rx_ring_cli;
    } else {
        ring = rx_ring_mux;
    }

    if (used_avail == 0) {
        a[0] = ring_full(ring.used_ring);
    } else {
        a[0] = ring_full(ring.avail_ring);
    }
    
    return;
}

/* Wrapper around the ring size function for the FFI */
void ffiring_size(unsigned char *c, long clen, unsigned char *a, long alen) {
    // sel4cp_dbg_puts("In the ring size func\n");
    //sel4cp_dbg_puts("In the drv ring size func\n");
    if (clen != 1 || alen != 8) {
        //sel4cp_dbg_puts("Insufficient size for arg or return arrays\n");
        return;
    }

    // int ring = c[0];
    int ring = 1;
    uint64_t size = 0;
    if (ring == 0) {
        size = ring_size(rx_ring_mux.used_ring);
    } else {
        size = ring_size(rx_ring_mux.avail_ring);
    }

    uintptr_to_byte8(size, a);

    return;
}

/* Not sure how much point there is to put this in Pancake. There's not much logic here, 
and alot of shared memory interactions. */
void ffibatch_dequeue_memcpy_enqueue(unsigned char *c, long clen, unsigned char *a, long alen) {
    // sel4cp_dbg_puts("In the batch dequeue memcpy and enqueue func\n");
    uintptr_t m_addr, c_addr = 0;
    unsigned int m_len, c_len = 0;
    void *cookie = NULL;
    void *cookie2 = NULL;
    int err;

    err = dequeue_used(&rx_ring_mux, &m_addr, &m_len, &cookie);
    assert(!err);
    // get an available one from clients queue.
    err = dequeue_avail(&rx_ring_cli, &c_addr, &c_len, &cookie2);
    assert(!err);
    if (!c_addr ||
            c_addr < shared_dma_vaddr_cli ||
            c_addr >= shared_dma_vaddr_cli + SHARED_DMA_SIZE)
    {
        print("COPY|ERROR: Received an insane address: ");
        puthex64(c_addr);
        print(". Address should be between ");
        puthex64(shared_dma_vaddr_cli);
        print(" and ");
        puthex64(shared_dma_vaddr_cli + SHARED_DMA_SIZE);
        print("\n");
    }

    if (c_len < m_len) {
        print("COPY|ERROR: client buffer length is less than mux buffer length.\n");
        print("client length: ");
        puthex64(c_len);
        print(" mux length: ");
        puthex64(m_len);
        print("\n");
    }
    // copy the data over to the clients address space.
    memcpy((void *)c_addr, (void *)m_addr, m_len);

    /* Now that we've copied the data, enqueue the buffer to the client's used ring. */
    err = enqueue_used(&rx_ring_cli, c_addr, m_len, cookie2);
    assert(!err);
    /* enqueue the old buffer back to dev_rx_ring.avail so the driver can use it again. */
    err = enqueue_avail(&rx_ring_mux, m_addr, BUF_SIZE, cookie);
    assert(!err);
}

void ffinotify_client(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_notify_delayed(CLIENT_CH);
}

void ffinotify_delayed(unsigned char *c, long clen, unsigned char *a, long alen) {
    // sel4cp_dbg_puts("In the notify delayed func\n");
    // First 8 bytes for size of ring buffer
    // Next 4 bytes for the number of enqueues
    // Next byte for was full val
    if (clen != 13) {
        sel4cp_dbg_puts("Insufficient args\n");
        return;
    }
    
    uint64_t mux_avail_original_size = byte8_to_uintptr(c);
    int enqueued = byte4_to_int(&c[8]);
    int mux_was_full = c[12];

    if ((mux_avail_original_size == 0 || mux_was_full || 
            mux_avail_original_size + enqueued != ring_size(rx_ring_mux.avail_ring)) 
            && enqueued) {
        if (have_signal) {
            // We need to notify the client, but this should
            // happen first. 
            sel4cp_notify(CLIENT_CH);
        }
        sel4cp_notify_delayed(MUX_RX_CH);
    }

    return;
}

void ffimain_loop_invariant(unsigned char *c, long clen, unsigned char *a, long alen) {
    // sel4cp_dbg_puts("In the main loop invariant func\n");
    if (!ring_empty(rx_ring_mux.used_ring) &&
            !ring_empty(rx_ring_cli.avail_ring) &&
            !ring_full(rx_ring_mux.avail_ring) &&
            !ring_full(rx_ring_cli.used_ring)) {
        a[0] = 0;
    } else {
        a[0] = 1;
    }
}


/*---------- Functions needed by cakeml ----------*/
void cml_exit(int arg) {
    // We should never get to this function
    sel4cp_dbg_puts("In the cml_exit function, we should not be here!\n");    
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
    //puthex64(cml_heap);
    //sel4cp_dbg_puts("\n");
    cml_stack = cml_heap + cml_heap_sz;
    //sel4cp_dbg_puts("Pancake stack start: ");
    //puthex64(cml_stack);
    //sel4cp_dbg_puts("\n");
    cml_stackend = cml_stack + cml_stack_sz;
    //sel4cp_dbg_puts("Pancake stack end: ");
    //puthex64(cml_stackend);
    //sel4cp_dbg_puts("\n");
}

/*---------- sel4cp Entry Points ----------*/


void init(void)
{
    sel4cp_dbg_puts("In the copy init func\n");
    init_pancake_mem();

    /* Set up shared memory regions */
    ring_init(&rx_ring_mux, (ring_buffer_t *)rx_avail_mux, (ring_buffer_t *)rx_used_mux, NULL, 1);
    ring_init(&rx_ring_cli, (ring_buffer_t *)rx_avail_cli, (ring_buffer_t *)rx_used_cli, NULL, 0);

    /* Enqueue available buffers for the mux to access */
    for (int i = 0; i < NUM_BUFFERS - 1; i++) {
        uintptr_t addr = shared_dma_vaddr_mux + (BUF_SIZE * i);
        int err = enqueue_avail(&rx_ring_mux, addr, BUF_SIZE, NULL);
        assert(!err);
    }

    return;
}

void notified(sel4cp_channel ch)
{
    sel4cp_dbg_puts("---------- In the copy notified function ----------\n");
    if (!initialised) {
        // sel4cp_dbg_puts("In copy notified init case\n");
        /*
         * Propogate this down the line to ensure everyone is
         * initliased in correct order.
         */
        sel4cp_notify(MUX_RX_CH);
        initialised = 1;
        sel4cp_dbg_puts("Returning from the copy notified init case\n");
        return;
    }

    if (ch == CLIENT_CH || ch == MUX_RX_CH) {
        // sel4cp_dbg_puts("We are in copy, and are entering process rx complete\n");
        /* We have one job. */
        cml_main();
    } else {
        print("COPY|ERROR: unexpected notification from channel: ");
        puthex64(ch);
        print("\n");
        assert(0);
    }
        sel4cp_dbg_puts("Finished the copy notified func\n");

}