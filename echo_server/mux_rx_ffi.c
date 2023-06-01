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

#define CLI_CH 1
#define DRV_CH 11

#define NUM_CLIENTS 2

static char cml_memory[1024*1024*2];

// /* exported in cake.S */
extern void cml_main(void);

extern void *cml_heap;
extern void *cml_stack;
extern void *cml_stackend;

/* Memory regions as defined in the system file */

// Transmit rings with the driver
uintptr_t rx_avail_drv;
uintptr_t rx_used_drv;

// Transmit rings with the client
uintptr_t rx_avail_cli;
uintptr_t rx_used_cli;
uintptr_t rx_avail_cli2;
uintptr_t rx_used_cli2;

uintptr_t shared_dma_rx_drv;
uintptr_t shared_dma_rx_cli;
uintptr_t shared_dma_rx_cli2;

// Have an array of client rings. 
ring_handle_t rx_ring[NUM_CLIENTS];
ring_handle_t drv_rx_ring;

/* We need to do some processing of the input stream to determine when we need 
to change direction. */

/* To switch input direction, type the "@" symbol followed immediately by a number.
Otherwise, can put "\" before "@" to escape this.*/

int escape_character;
int client;
// We want to keep track of each clients requests, so that they can be serviced once we have changed 
// input direction
int num_to_get_chars[NUM_CLIENTS];

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

void ffiget_num_clients(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("In the mux tx num clients function\n");
    if (alen != 1) {
        return;
    }

    a[0] = NUM_CLIENTS;
}

/* Getters and Setters for the escape character and current client global variables*/

void ffiget_escape_character(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (alen != 1) {
        sel4cp_dbg_puts("Insufficient args or ret space -- get escape char\n");
        return;
    }

    a[0] = escape_character;
}

void ffiset_escape_character(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (clen != 1) {
        sel4cp_dbg_puts("Insufficient args or ret space -- set escape char\n");
        return;
    }

    escape_character = c[0];
}

void ffiget_client(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (alen != 1) {
        sel4cp_dbg_puts("Insufficient args or ret space -- get client\n");
        return;
    }

    a[0] = client;
}

void ffiset_client(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("IN THE SET CLIENT FUNCTION\n");
    if (clen != 1) {
        sel4cp_dbg_puts("Insufficient args or ret space -- set client\n");
        return;
    }

    client = c[0];
}

/* Check that there is a non zero value for the current client's num_to_get_char*/
void fficheck_num_to_get_chars(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (alen != 1) {
        sel4cp_dbg_puts("Insufficient args or ret space -- num to get chars\n");
        return;
    }

    if (num_to_get_chars[client - 1] <= 0) {
        a[0] = 0;
    } else {
        a[0] = 1;
    }
}

/* Wrapper around the dequeue used function. Takes in client number as an argument */
void ffidrv_dequeue_used(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("In the MUX RX dequeue used funtion\n");
    if (clen != 1 || alen != 1) {
        sel4cp_dbg_puts("Insufficient args or ret space -- drv dequeue used\n");
        return;
    }

    uintptr_t buffer;
    unsigned int buffer_len = 0;
    void *cookie = 0;

   // We can only be here if we have been notified by the driver
    int ret = dequeue_used(&drv_rx_ring, &buffer, &buffer_len, &cookie) != 0;
    if (ret != 0) {
        sel4cp_dbg_puts(sel4cp_name);
        sel4cp_dbg_puts(": getchar - unable to dequeue used buffer\n");
        c[0] = 1;
    }

    // We are only getting one character at a time, so we just need to cast the buffer to an int

    char got_char = *((char *) buffer);

    /* Now that we are finished with the used buffer, we can add it back to the available ring*/

    ret = enqueue_avail(&drv_rx_ring, buffer, buffer_len, cookie);

    if (ret != 0) {
        sel4cp_dbg_puts(sel4cp_name);
        sel4cp_dbg_puts(": getchar - unable to enqueue used buffer back into available ring\n");
        c[0] = 1;
    }


    // We will return the character that we got to the pancake program
    a[0] = got_char;
    c[0] = 0;

    return;
}

void ffibatch_cli_dequeue_enqueue(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("In the batch driver dequeue enqueue function\n");
    if (clen != 2 || alen != 1) {
        sel4cp_dbg_puts("Insufficient args or ret space -- batch cli dequeue enqueue\n");
        return;
    }

    int curr_client = c[0];
    char got_char = c[1];

    // Address that we will pass to dequeue to store the buffer address
    uintptr_t buffer = 0;
    // Integer to store the length of the buffer
    unsigned int buffer_len = 0; 

    void *cookie = 0;

    int ret = dequeue_avail(&rx_ring[curr_client - 1], &buffer, &buffer_len, &cookie);

    if (ret != 0) {
        // sel4cp_dbg_puts(sel4cp_name);
        sel4cp_dbg_puts(": unable to dequeue from the rx available ring\n");
        return;
    }

    ((char *) buffer)[0] = (char) got_char;

    // Now place in the rx used ring
    ret = enqueue_used(&rx_ring[curr_client - 1], buffer, 1, &cookie);

    if (ret != 0) {
        // sel4cp_dbg_puts(sel4cp_name);
        sel4cp_dbg_puts(": unable to enqueue to the tx available ring\n");
        return 1;
    }

    num_to_get_chars[curr_client - 1] -= 1;
}

/*---------- Debugging Functions ----------*/
void ffiescape_0(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("WE ARE IN THE ESCAPE CHARACTER 0 CASE\n");
}

void ffiescape_1(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("WE ARE IN THE ESCAPE CHARACTER 1 CASE\n");
}

void ffiescape_2(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("WE ARE IN THE ESCAPE CHARACTER 2 CASE\n");
}

void ffiat_case(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("WE ARE IN THE @ SUBCASE\n");
}

void ffiescape_case(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("WE ARE IN THE \\ SUBCASE\n");
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
    unsigned long sz = 1024*1024; // 1 MB unit\n",
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
    ring_init(&rx_ring[0], (ring_buffer_t *)rx_avail_cli, (ring_buffer_t *)rx_used_cli, NULL, 0);
    ring_init(&rx_ring[1], (ring_buffer_t *)rx_avail_cli2, (ring_buffer_t *)rx_used_cli2, NULL, 0);

    ring_init(&drv_rx_ring, (ring_buffer_t *)rx_avail_drv, (ring_buffer_t *)rx_used_drv, NULL, 0);

    for (int i = 0; i < NUM_BUFFERS - 1; i++) {
        int ret = enqueue_avail(&drv_rx_ring, shared_dma_rx_drv + (i * BUFFER_SIZE), BUFFER_SIZE, NULL);

        if (ret != 0) {
            sel4cp_dbg_puts(sel4cp_name);
            sel4cp_dbg_puts(": mux rx buffer population, unable to enqueue buffer\n");
            return;
        }
    }

    // We initialise the current client to 1
    client = 1;
    // Set the current escape character to 0, we can't have recieved an escape character yet
    escape_character = 0;
    // No chars have been requested yet
    num_to_get_chars[0] = 0;
    num_to_get_chars[1] = 0;
    sel4cp_dbg_puts("mux rx init finished\n");  
}

void notified(sel4cp_channel ch)
{
    if (ch == DRV_CH) {
        cml_main();
    } else if (ch < 1 || ch > NUM_CLIENTS) {
        sel4cp_dbg_puts("Received a bad client channel\n");
        return;
    }  else {
        // This was recieved on a client channel. Index the number of characters to get
        num_to_get_chars[ch - 1] += 1;
    }

}