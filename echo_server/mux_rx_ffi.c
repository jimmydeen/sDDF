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
uintptr_t rx_avail_drv;
uintptr_t rx_used_drv;

uintptr_t rx_avail_cli;
uintptr_t rx_used_cli;

uintptr_t shared_dma_vaddr;
uintptr_t uart_base;

#define NUM_CLIENTS 1

#define COPY_CH 0
#define DRIVER_CH 1

#define ETHER_MTU 1500

typedef struct state {
    /* Pointers to shared buffers */
    ring_handle_t rx_ring_drv;
    ring_handle_t rx_ring_clients[NUM_CLIENTS];
    uint8_t mac_addrs[NUM_CLIENTS][6];
} state_t;

state_t state;
int initialised = 0;
uint8_t broadcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint64_t dropped = 0;
bool rx_avail_was_empty = false;

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

void ffiget_args(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (alen != 1) {
        //sel4cp_dbg_puts("Return array of incorrect size\n");
        return;
    }

    a[0] = cml_arg;
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

void ffiprint_client(unsigned char *c, long clen, unsigned char *a, long alen) {
    int index = c[0];
    int val_at_index = c[1];

    //sel4cp_dbg_puts("This is the current client: ");
    //puthex64(index);
    //sel4cp_dbg_puts("\nThis is the value at client in notify array: ");
    //puthex64(val_at_index);
    //sel4cp_dbg_puts("\n");
}

void ffistoring_notify_clients(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("---------We are storing in the notify clients array---------\n");
}

/*------------ MUX FFI functions ------------ */

int compare_mac(uint8_t *mac1, uint8_t *mac2)
{
    for (int i = 0; i < 6; i++) {
        if (mac1[i] != mac2[i]) {
            return -1;
        }
    }
    return 0;
}

static void
dump_mac(uint8_t *mac)
{
    for (unsigned i = 0; i < 6; i++) {
        sel4cp_dbg_putc(hexchar((mac[i] >> 4) & 0xf));
        sel4cp_dbg_putc(hexchar(mac[i] & 0xf));
        if (i < 5) {
            sel4cp_dbg_putc(':');
        }
    }
    sel4cp_dbg_putc('\n');
}

/* Return the client ID if the Mac address is a match. */
int get_client(uintptr_t dma_vaddr) {
    uint8_t dest_addr[6];
    struct eth_hdr *ethhdr = (struct eth_hdr *)dma_vaddr;
    dest_addr[0] = ethhdr->dest.addr[0];
    dest_addr[1] = ethhdr->dest.addr[1];
    dest_addr[2] = ethhdr->dest.addr[2];
    dest_addr[3] = ethhdr->dest.addr[3];
    dest_addr[4] = ethhdr->dest.addr[4];
    dest_addr[5] = ethhdr->dest.addr[5];

    for (int client = 0; client < NUM_CLIENTS; client++) {
        if (compare_mac(dest_addr, state.mac_addrs[client]) == 0) {
            return client;
        }
        if (compare_mac(dest_addr, broadcast) == 0) {
            // broadcast packet, send the packet to the first client to handle.
            // This is temporary, eventually we will have a different
            // component to deal with this.
            return 0;
        }
    }
    return -1;
}

/* Call the ring empty function on the drv ring. Used/available is determined by supplied
args. */
void ffidrv_ring_empty(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In the drv ring empty func\n");
    if (clen != 1 || alen != 1) {
        //sel4cp_dbg_puts("Arg/ret arrays of incorrect size\n");
        return;
    }

    int ring = c[0];
    if (ring == 0) {
        a[0] = ring_empty(state.rx_ring_drv.used_ring);
    } else {
        a[0] = ring_empty(state.rx_ring_drv.avail_ring);
    }

    //sel4cp_dbg_puts("This is the value of a[0]: ");
    //puthex64(a[0]);
    //sel4cp_dbg_puts("\n");

    return;
}

/* Wrapper around the ring_empty function for the client */
void fficli_ring_empty(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In the cli ring empty func\n");
    if (clen != 2 || alen != 1) {
        //sel4cp_dbg_puts("Arg/ret arrays of incorrect size\n");
        return;
    }

    int client = c[0];
    int ring = c[1];

    if (ring == 0) {
        a[0] = ring_empty(state.rx_ring_clients[client].used_ring);
    } else {
        a[0] = ring_empty(state.rx_ring_clients[client].avail_ring);
    }

    //sel4cp_dbg_puts("This is the value of cli ring empty: ");
    //puthex64(a[0]);
    //sel4cp_dbg_puts("\n");

    return;
}

/* Wrapper around the ring_full function for the client*/
void fficli_ring_full(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In the cli ring full func\n");
    if (clen != 2 || alen != 1) {
        //sel4cp_dbg_puts("Arg/ret arrays of incorrect size\n");
        return;
    }

    int client = c[0];
    int ring = c[1];

    if (ring == 0) {
        a[0] = ring_full(state.rx_ring_clients[client].used_ring);
    } else {
        a[0] = ring_full(state.rx_ring_clients[client].avail_ring);
    }

    return;
}

/* Wrapper around the ring size function */
void ffidrv_ring_size(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In the drv ring size func\n");
    if (clen != 1 || alen != 8) {
        //sel4cp_dbg_puts("Insufficient size for arg or return arrays\n");
        return;
    }

    // int ring = c[0];
    int ring = 1;
    uint64_t size = 0;
    if (ring == 0) {
        size = ring_size(state.rx_ring_drv.used_ring);
    } else {
        size = ring_size(state.rx_ring_drv.avail_ring);
    }

    uintptr_to_byte8(size, a);

    return;
}

/* Dequeue from the used ring of the drv ring*/
void ffidrv_dequeue_used(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In the drv dequeue used func\n");
    // We need to return the addr, len and cookie to pancake. Assuming 8 bytes for each
    if (clen != 1 || alen != 24) {
        //sel4cp_dbg_puts("Return array of incorrect size\n");
        return;
    }

    uintptr_t addr = 0;
    unsigned int len = 0;
    void *cookie = NULL;

    int err = dequeue_used(&state.rx_ring_drv, &addr, &len, &cookie);
    assert(!err);
    err = seL4_ARM_VSpace_Invalidate_Data(3, addr, addr + ETHER_MTU);
    if (err) {
        print("MUX RX|ERROR: ARM Vspace invalidate failed\n");
        //puthex64(err);
        print("\n");
        c[0] = 1;
    }

    uintptr_to_byte8(addr, a);
    uintptr_to_byte8(len, &a[8]);
    uintptr_to_byte8(cookie, &a[16]);

    return;
}

/* Enqueue into the client used ring */
void fficli_enqueue_used(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In the cli enqueue used func\n");
    if (clen != 24 || alen != 1) {
        //sel4cp_dbg_puts("Argument/Return array of incorrect size\n");
        return;
    }

    uintptr_t addr = byte8_to_uintptr(c);
    unsigned int len = byte8_to_uintptr(&c[8]);
    void *cookie = byte8_to_uintptr(&c[16]);
    int client = a[0];

    int err = enqueue_used(&state.rx_ring_clients[client], addr, len, cookie);
    assert(!err);
    if (err) {
        print("MUX RX|ERROR: failed to enqueue onto used ring\n");
        a[0] = 1;
    }

    a[0] = 0;
    return;
}

/* Enqueue into the drv avail ring. On return of 1, break from loop*/
void ffidrv_enqueue_avail(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In teh drv enqueue avail func\n");
    if (clen != 24 || alen != 1) {
        //sel4cp_dbg_puts("Argument/Return array of incorrect size\n");
        return;
    }

    uintptr_t addr = byte8_to_uintptr(c);
    unsigned int len = byte8_to_uintptr(&c[8]);
    void *cookie = byte8_to_uintptr(&c[16]);
    if (addr == 0) {
        print("MUX RX|ERROR: Attempting to add NULL buffer to driver RX ring\n");
        a[0] = 1;
        return;
    }
    int err = enqueue_avail(&state.rx_ring_drv, addr, len, cookie);
    if (err) {
        print("MUX RX|ERROR: Failed to enqueue available to driver RX ring\n");
    }

    a[0] = 1;

    return;

}

void ffibatch_dequeue_enqueue(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In the batch dequeue enqueue func\n");
    if (clen != 1) {
        //sel4cp_dbg_puts("Argument array of incorrect size\n");
        return;
    }

    int client = c[0];

    uintptr_t addr;
    unsigned int len;
    void *buffer;
    int err = dequeue_avail(&state.rx_ring_clients[client], &addr, &len, &buffer);
    assert(!err);
    err = enqueue_avail(&state.rx_ring_drv, addr, len, buffer);
    assert(!err);
}


/* Wrapper for the ffi around the get client function */
void ffiget_client(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In the get client func\n");
    if (clen != 8 || alen != 1) {
        //sel4cp_dbg_puts("Argument/Return array of incorrect size\n");
        return;
    }

    uintptr_t addr = byte8_to_uintptr(c);

    int ret = get_client(addr);
    if (ret < 0) {
        c[0] = 1;
        return;
    } else {
        c[0] = 0;
    }
    //sel4cp_dbg_puts("This is the value of get_client: ");
    //puthex64(ret);
    //sel4cp_dbg_puts("\n");

    // We are storing this int int a char for simplicity, change this later to allow for more clients
    a[0] = (unsigned char) ret;

    return;
}

/* Wrapper around the sel4cp_notify function */
void ffinotify_client(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In the notify client func\n");
    if (clen != 1) {
        //sel4cp_dbg_puts("Argument array of incorrect size\n");
        return;
    }

    int client = c[0];

    sel4cp_notify(client);

    return;
}

void ffiprocess_set_signal(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In the process set signal func\n");
    // First 8 bytes for size of ring buffer
    // Next 4 bytes for the number of enqueues
    if (clen != 12) {
        //sel4cp_dbg_puts("Insufficient args\n");
        return;
    }
    
    uint64_t size = byte8_to_uintptr(c);
    int enqueued = byte4_to_int(&c[8]);

    //sel4cp_dbg_puts("This is the value of size: ");
    //puthex64(size);
    //sel4cp_dbg_puts("\n");

    //sel4cp_dbg_puts("This is the value of enqueued: ");
    //puthex64(enqueued);
    //sel4cp_dbg_puts("\n");

    if ((size == 0 || size + enqueued != ring_size(state.rx_ring_drv.avail_ring)) && enqueued != 0) {
        //sel4cp_dbg_puts("We are calling notify delayed\n");
        sel4cp_notify_delayed(DRIVER_CH);
    }

    return;
}

/* Manipulate the global dropped variable*/
void ffiset_dropped(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In the set dropped func\n");
    dropped = 0;
}
void ffiincrement_dropped(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In the increment dropped func\n");
    dropped = dropped + 1;
}

/* Set the value of the rx avail ring. Need to keep the global variables in here */
void ffiset_rx_avail_was_empty(unsigned char *c, long clen, unsigned char *a, long alen) {
    //sel4cp_dbg_puts("In the set rx avail was empty func\n");
    rx_avail_was_empty = ring_empty(state.rx_ring_drv.avail_ring);
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

seL4_MessageInfo_t
protected(sel4cp_channel ch, sel4cp_msginfo msginfo)
{
    if (ch >= NUM_CLIENTS) {
        //sel4cp_dbg_puts("Received ppc on unexpected channel ");
        //puthex64(ch);
        return sel4cp_msginfo_new(0, 0);
    }
    // return the MAC address.
    uint32_t lower = (state.mac_addrs[ch][0] << 24) |
                     (state.mac_addrs[ch][1] << 16) |
                     (state.mac_addrs[ch][2] << 8) |
                     (state.mac_addrs[ch][3]);
    uint32_t upper = (state.mac_addrs[ch][4] << 24) | (state.mac_addrs[ch][5] << 16);
    //sel4cp_dbg_puts("Mux rx is sending mac: ");
    dump_mac(state.mac_addrs[ch]);
    sel4cp_mr_set(0, lower);
    sel4cp_mr_set(1, upper);
    return sel4cp_msginfo_new(0, 2);
}

void init(void)
{
    init_pancake_mem();
   // set up client macs
    // use a dummy one for our one client. 
    state.mac_addrs[0][0] = 0x52;
    state.mac_addrs[0][1] = 0x54;
    state.mac_addrs[0][2] = 0x1;
    state.mac_addrs[0][3] = 0;
    state.mac_addrs[0][4] = 0;
    state.mac_addrs[0][5] = 0;

    // This is the legitimate hw address
    // (can be useful when debugging). 
    /*state.mac_addrs[0][0] = 0;
    state.mac_addrs[0][1] = 0x4;
    state.mac_addrs[0][2] = 0x9f;
    state.mac_addrs[0][3] = 0x5;
    state.mac_addrs[0][4] = 0xf8;
    state.mac_addrs[0][5] = 0xcc;*/

    /* Set up shared memory regions */
    ring_init(&state.rx_ring_drv, (ring_buffer_t *)rx_avail_drv, (ring_buffer_t *)rx_used_drv, NULL, 1);

    // FIX ME: Use the notify function pointer to put the notification in?
    ring_init(&state.rx_ring_clients[0], (ring_buffer_t *)rx_avail_cli, (ring_buffer_t *)rx_used_cli, NULL, 0);
}

void notified(sel4cp_channel ch)
{
    //sel4cp_dbg_puts("---------- In the mux rx notified func ----------\n");
    if (!initialised) {
        //sel4cp_dbg_puts("In the not init case\n");
        cml_arg = 1;

        cml_main();

        //sel4cp_dbg_puts("Returned back from pancake\n");

        sel4cp_notify(DRIVER_CH);
        initialised = 1;
        return;
    }

    if (ch == COPY_CH || ch == DRIVER_CH) {
        //sel4cp_dbg_puts("In the complete + free case\n");
        cml_arg = 2;
        cml_main();
        //sel4cp_dbg_puts("Returned back from pancake\n");
    } else {
        print("MUX RX|ERROR: unexpected notification from channel: ");
        puthex64(ch);
        print("\n");
        assert(0);
    }

    //sel4cp_dbg_puts("Finished the mux rx notified func\n");
}