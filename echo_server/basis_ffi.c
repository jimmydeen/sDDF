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
#include "basis_ffi.h"

static char cml_memory[2048*2048*2];
// Attempt to save the address that notified needs to return to
void *notified_return;
unsigned int argc;
char **argv;

/* exported in cake.S */
extern void cml_main(void);
extern void *cml_heap;
extern void *cml_stack;
extern void *cml_stackend;

extern char cake_text_begin;
extern char cake_codebuffer_begin;
extern char cake_codebuffer_end;

#define IRQ_CH 1
#define TX_CH  2
#define RX_CH  2
#define INIT   4

#define MDC_FREQ    20000000UL

/* Memory regions. These all have to be here to keep compiler happy */
uintptr_t hw_ring_buffer_vaddr;
uintptr_t hw_ring_buffer_paddr;
uintptr_t shared_dma_vaddr;
uintptr_t shared_dma_paddr;
uintptr_t rx_cookies;
uintptr_t tx_cookies;
uintptr_t rx_avail;
uintptr_t rx_used;
uintptr_t tx_avail;
uintptr_t tx_used;
uintptr_t uart_base;

/* Make the minimum frame buffer 2k. This is a bit of a waste of memory, but ensures alignment */
#define PACKET_BUFFER_SIZE  2048
#define MAX_PACKET_SIZE     1536

#define RX_COUNT 256
#define TX_COUNT 256

/* Pointers to shared_ringbuffers */
ring_handle_t rx_ring;
ring_handle_t tx_ring;

char current_channel;

_Static_assert((512 * 2) * PACKET_BUFFER_SIZE <= 0x200000, "Expect rx+tx buffers to fit in single 2MB page");
_Static_assert(sizeof(ring_buffer_t) <= 0x200000, "Expect ring buffer ring to fit in single 2MB page");

static uint8_t mac[6];

volatile struct enet_regs *eth = (void *)(uintptr_t)0x2000000;

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

/* Functions needed by cakeml*/
void cml_exit(int arg) {
    sel4cp_dbg_puts("CALLING CML_EXIT\n");

    if (current_channel == IRQ_CH) {
        have_signal = true;
        signal_msg = seL4_MessageInfo_new(IRQAckIRQ, 0, 0, 0);
        signal = (BASE_IRQ_CAP + IRQ_CH);
    }

    current_channel = 0;
    if (notified_return == 0) {
        sel4cp_dbg_puts("We have not saved notified return properly\n");    
    }
    void (*foo)(void) = (void (*)())notified_return;
    foo();

}

/* Need to come up with a replacement for this clear cache function. Might be worth testing just flushing the entire l1 cache, but might cause issues with returning to this file*/
void cml_clear() {
//   __builtin___clear_cache(&cake_codebuffer_begin, &cake_codebuffer_end);
    sel4cp_dbg_puts("Trying to clear cache\n");
}

/* FFI call to get the current channel that has notified us. Need to find a better
way to pass an argument to cml_main() */
void ffiget_channel(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (alen != 1) {
        return;
    }

    a[0] = current_channel;
}

/* Eth driver functions */

static void get_mac_addr(volatile struct enet_regs *reg, uint8_t *mac)
{
    uint32_t l, h;
    l = reg->palr;
    h = reg->paur;

    mac[0] = l >> 24;
    mac[1] = l >> 16 & 0xff;
    mac[2] = l >> 8 & 0xff;
    mac[3] = l & 0xff;
    mac[4] = h >> 24;
    mac[5] = h >> 16 & 0xff;
}

static void set_mac(volatile struct enet_regs *reg, uint8_t *mac)
{
    reg->palr = (mac[0] << 24) | (mac[1] << 16) | (mac[2] << 8) | (mac[3]);
    reg->paur = (mac[4] << 24) | (mac[5] << 16);
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
}

uintptr_t getPhysAddr(uintptr_t virtual)
{
    uint64_t offset = virtual - shared_dma_vaddr;
    uintptr_t phys;

    if (offset < 0) {
        print("getPhysAddr: offset < 0");
        return 0;
    }

    phys = shared_dma_paddr + offset;
    return phys;
}



static inline void
enable_irqs(volatile struct enet_regs *eth, uint32_t mask)
{
    eth->eimr = mask;
}

uintptr_t alloc_rx_buf(size_t buf_size, void **cookie)
{
    uintptr_t addr;
    unsigned int len;

    /* Try to grab a buffer from the available ring */
    if (driver_dequeue(rx_ring.avail_ring, &addr, &len, cookie)) {
        print("RX Available ring is empty\n");
        return 0;
    }

    uintptr_t phys = getPhysAddr(addr);

    return getPhysAddr(addr);
}

static void 
eth_setup(void)
{
    get_mac_addr(eth, mac);
    sel4cp_dbg_puts("MAC: ");
    dump_mac(mac);
    sel4cp_dbg_puts("\n");

    /* Perform reset */
    eth->ecr = ECR_RESET;
    while (eth->ecr & ECR_RESET);
    eth->ecr |= ECR_DBSWP;

    /* Clear and mask interrupts */
    eth->eimr = 0x00000000;
    eth->eir  = 0xffffffff;

    /* set MDIO freq */
    eth->mscr = 24 << 1;

    /* Disable */
    eth->mibc |= MIBC_DIS;
    while (!(eth->mibc & MIBC_IDLE));
    /* Clear */
    eth->mibc |= MIBC_CLEAR;
    while (!(eth->mibc & MIBC_IDLE));
    /* Restart */
    eth->mibc &= ~MIBC_CLEAR;
    eth->mibc &= ~MIBC_DIS;

    /* Descriptor group and individual hash tables - Not changed on reset */
    eth->iaur = 0;
    eth->ialr = 0;
    eth->gaur = 0;
    eth->galr = 0;

    if (eth->palr == 0) {
        // the mac address needs setting again. 
        set_mac(eth, mac);
    }

    eth->opd = PAUSE_OPCODE_FIELD;

    /* coalesce transmit IRQs to batches of 128 */
    eth->txic0 = TX_ICEN | ICFT(128) | 0xFF;
    eth->tipg = TIPG;
    /* Transmit FIFO Watermark register - store and forward */
    eth->tfwr = 0;

    /* enable store and forward. This must be done for hardware csums*/
    eth->rsfl = 0;
    /* Do not forward frames with errors + check the csum */
    eth->racc = RACC_LINEDIS | RACC_IPDIS | RACC_PRODIS;

    /* Set RDSR */
    eth->rdsr = hw_ring_buffer_paddr;
    eth->tdsr = hw_ring_buffer_paddr + (sizeof(struct descriptor) * RX_COUNT);

    /* Size of max eth packet size */
    eth->mrbr = MAX_PACKET_SIZE;

    eth->rcr = RCR_MAX_FL(1518) | RCR_RGMII_EN | RCR_MII_MODE;
    eth->tcr = TCR_FDEN;

    /* set speed */
    eth->ecr |= ECR_SPEED;

    /* Set Enable  in ECR */
    eth->ecr |= ECR_ETHEREN;

    eth->rdar = RDAR_RDAR;

    /* enable events */
    eth->eir = eth->eir;
    eth->eimr = IRQ_MASK;
}

void ffienable_rx() {
    eth->rdar = RDAR_RDAR;
}

void ffiget_irq(unsigned char *c, long clen, unsigned char *a, long alen) {
    uint32_t e = eth->eir & IRQ_MASK;
    /* write to clear events */
    eth->eir = e;

    int_to_byte4(e, a);

}

/* Wrappers around the libsharedringbuffer functions for the pancake FFI */

void ffieth_driver_dequeue_used(unsigned char *c, long clen, unsigned char *a, long alen) {

    // // We expect the calling program to place the appropriate arguments in the following order
    // uintptr_t buffer = (uintptr_t) byte8_to_int(c);
    // unsigned int buffer_len = (unsigned int) byte8_to_int(&c[8]);
    // void * cookie = (void *) byte8_to_int(&c[16]);

    uintptr_t buffer = 0;
    unsigned int buffer_len = 0;
    void *cookie = NULL;

    sel4cp_dbg_puts("In the serial driver dequeue used function\n");
    if (clen != 1) {
        sel4cp_dbg_puts("There are no arguments supplied when args are expected\n");
        return;
    }

    bool rx_tx = a[0];

    sel4cp_dbg_puts("Attempting driver dequeue\n");
    int ret = 0;
    if (rx_tx == 0) {
        ret = driver_dequeue(rx_ring.used_ring, &buffer, &buffer_len, &cookie);
    } else {
        ret = driver_dequeue(tx_ring.used_ring, &buffer, &buffer_len, &cookie);
    }

    // Place the values that we have gotten from the dequeue function into the a array
    uintptr_to_byte8(buffer, c);
    uintptr_to_byte8(buffer_len, &c[8]);
    uintptr_to_byte8(cookie, &c[16]);

    sel4cp_dbg_puts("Finished buffer dequeue\n");
}

void ffieth_driver_enqueue_used(unsigned char *c, long clen, unsigned char *a, long alen) {
    // In this case we assume that the 'c' array will contain the address of the cookie that we need
    
    buff_desc_t *desc = (buff_desc_t *) byte8_to_int(c);

    enqueue_used(&tx_ring, desc->encoded_addr, desc->len, desc->cookie);
}

void ffieth_driver_enqueue_avail(unsigned char *c, long clen, unsigned char *a, long alen) {
    // In this case we assume that the 'c' array will contain the address of the cookie that we need
    
    buff_desc_t *desc = (buff_desc_t *) byte8_to_int(c);

    enqueue_avail(&tx_ring, desc->encoded_addr, desc->len, desc->cookie);
}

void ffieth_ring_empty(unsigned char *c, long clen, unsigned char *a, long alen) {
    char ret = (char) ring_empty(rx_ring.used_ring);
    a[0] = ret;
}

void ffieth_ring_size(unsigned char *c, long clen, unsigned char *a, long alen) {
    int ret = ring_size(rx_ring.avail_ring);
    int_to_byte4(ret, a);
}

void ffisynchronise_call() {
    __sync_synchronize();
}

void ffitx_descr_active() {
    if (!(eth->tdar & TDAR_TDAR)) {
        eth->tdar = TDAR_TDAR;
    }
}

void ffiget_rx_phys(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (alen != 8) {
        sel4cp_dbg_puts("Alen not of expected length\n");
        return;
    }
    uintptr_to_byte8(a, shared_dma_paddr);
}

void ffiget_tx_phys(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (alen != 8) {
        sel4cp_dbg_puts("Alen not of expected length\n");
        return;
    }
    uintptr_to_byte8(a, shared_dma_paddr + sizeof(struct descriptor) * RX_COUNT);
}

void ffiget_rx_cookies(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (alen != 8) {
        sel4cp_dbg_puts("Alen not of expected length\n");
        return;
    }
    uintptr_to_byte8(a, (void **)rx_cookies);
}

void ffiget_tx_cookies(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (alen != 8) {
        sel4cp_dbg_puts("Alen not of expected length\n");
        return;
    }
    uintptr_to_byte8(a, (void **)tx_cookies);
}

void ffiget_rx_descr(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (alen != 8) {
        sel4cp_dbg_puts("Alen not of expected length\n");
        return;
    }
    uintptr_to_byte8(a, (volatile struct descriptor *)hw_ring_buffer_vaddr);
}

void ffiget_tx_descr(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (alen != 8) {
        sel4cp_dbg_puts("Alen not of expected length\n");
        return;
    }
    uintptr_to_byte8(a, (volatile struct descriptor *)(hw_ring_buffer_vaddr + (sizeof(struct descriptor) * RX_COUNT)));
}

// Getters and Setters for the cookies, descr and phys arrays

/* 
Index will be in c[0], address starts from c[1]
The address that we get will be stored starting from a[0]
*/

void ffiget_cookies(unsigned char *c, long clen, unsigned char *a, long alen) {
    int index = c[0];
    void **cookies = (void **) byte8_to_uintptr(&c[1]);

    void *cookie = cookies[index];

    uintptr_to_byte8(cookie, &c[1]);
}

void ffiset_cookies(unsigned char *c, long clen, unsigned char *a, long alen) {
    int index = c[0];
    void *cookie = (void **) byte8_to_uintptr(&c[1]);
    void**cookies = (void *) byte8_to_int(&c[9]);

    cookies[index] = cookie;
}

void ffiget_descr(unsigned char *c, long clen, unsigned char *a, long alen) {
    int index = c[0];
    struct descriptor **desc = (struct descriptor **) byte8_to_uintptr(&c[1]);
    struct descriptor *d = desc[index];
    
    uintptr_to_byte8(d, a);
}

void ffiset_descr(unsigned char *c, long clen, unsigned char *a, long alen) {
    // DO NOT NEED FOR NOW
}

void ffialloc_rx_buff(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (alen != 8 || clen != 8) {
        sel4cp_dbg_puts("Len was not of correct size -- alloc_rx_buf\n");
        return;
    }

    void *cookie = (void *) byte8_to_uintptr(c);
    uintptr_t addr;
    unsigned int len;

    /* Try to grab a buffer from the available ring */
    if (driver_dequeue(rx_ring.avail_ring, &addr, &len, cookie)) {
        print("RX Available ring is empty\n");
        return 0;
    }

    uintptr_t phys = getPhysAddr(addr);

    uintptr_to_byte8(phys, a);
}

void ffiupdate_descr_slot(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (clen != 26) {
        sel4cp_dbg_puts("Clen was not of correct size -- update_descr_slot\n");
        return;
    }

    // Descriptor array address in c[0]
    struct descriptor *descr = (struct descriptor *) byte8_to_uintptr(c);

    // Index in c[9]
    int index = c[8];

    // Phys from c[9]
    uintptr_t phys = byte8_to_uintptr(&c[9]);

    // Len in c[17]
    int len = c[17];

    // Stat in c[18]
    int64_t stat = (int64_t) byte8_to_uintptr(&c[18]);

    // Array of size 26

    volatile struct descriptor *d = &descr[index];
    d->addr = phys;
    d->len = len;

    /* Ensure all writes to the descriptor complete, before we set the flags
     * that makes hardware aware of this slot.
     */
    __sync_synchronize();

    d->stat = stat;
}

void ffiupdate_descr_slot_raw_tx(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (clen != 26) {
        sel4cp_dbg_puts("Clen was not of correct size -- update_descr_slot\n");
        return;
    }

    // Descriptor array address in c[0]
    struct descriptor *descr = (struct descriptor *) byte8_to_uintptr(c);

    // Index in c[8]
    int index = c[8];

    // Phys from c[9]
    uintptr_t phys = byte8_to_uintptr(&c[9]);

    // Len in c[17]
    int len = c[17];

    // Stat in c[18]
    int64_t stat = (int64_t) byte8_to_uintptr(&c[18]);

    // Array of size 26

    volatile struct descriptor *d = &descr[index];
    d->addr = phys;
    d->len = len;

    /* Ensure all writes to the descriptor complete, before we set the flags
     * that makes hardware aware of this slot.
     */
    __sync_synchronize();

    d->stat = stat;

    // Increment phys here, and store the result 
    phys++;

    // Store the new phys where the old phys was
    uintptr_to_byte8(&c[9], phys);
}

void ffitry_buffer_release(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (clen != 9) {
        sel4cp_dbg_puts("Clen not of correct len -- try_buffer_release\n");
        a[0] = 1;
        return;
    }
    // Descriptor array address in c[0]
    struct descriptor *descr = (struct descriptor *) byte8_to_uintptr(c);

    // Index in c[8]
    int index = c[8];

    volatile struct descriptor *d = &descr[index];

    if (d->stat & TXD_READY) {
        if (!(eth->tdar & TDAR_TDAR)) {
            eth->tdar = TDAR_TDAR;
        }
        
        if (d->stat & TXD_READY) {
            a[0] = 1;
            return;
        }
    }
    a[0] = 0;
}   

void fficheck_empty(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (clen != 9) {
        sel4cp_dbg_puts("Clen not of correct len -- try_buffer_release\n");
        a[0] = 1;
        return;
    }
    // Descriptor array address in c[0]
    struct descriptor *descr = (struct descriptor *) byte8_to_uintptr(c);

    // Index in c[8]
    int index = c[8];

    volatile struct descriptor *d = &descr[index];

    if (d->stat & RXD_EMPTY) {
        a[0] = 1;
    } else {
        a[0] = 0;
    }

}

void ffiget_phys(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (clen != 8 || alen != 8) {
        sel4cp_dbg_puts("Len was not of correct size -- get_phys\n");
        return;
    }

    // Buffer address will be in c[0]
    uintptr_t buffer = byte8_to_uintptr(c);

    uintptr_t phys = getPhysAddr(buffer);

    uintptr_to_byte8(a, phys);
}

void ffinotify_rx(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_notify(RX_CH);
}

void ffiinit_post(unsigned char *c, long clen, unsigned char *a, long alen)
{

    sel4cp_dbg_puts("In the init_post function\n");
    // fill_rx_bufs();
    sel4cp_dbg_puts(sel4cp_name);
    sel4cp_dbg_puts(": init complete -- waiting for interrupt\n");
    sel4cp_notify(INIT);

    /* Now take away our scheduling context. Uncomment this for a passive driver. */
    /* have_signal = true;
    msg = seL4_MessageInfo_new(0, 0, 0, 1);
    seL4_SetMR(0, 0);
    signal = (MONITOR_EP); */
}

void ffisetup_ring_init(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("In the setup ring init function\n");
    /* Set up shared memory regions */
    ring_init(&rx_ring, (ring_buffer_t *)rx_avail, (ring_buffer_t *)rx_used, NULL, 0);
    ring_init(&tx_ring, (ring_buffer_t *)tx_avail, (ring_buffer_t *)tx_used, NULL, 0);
    sel4cp_dbg_puts("Finished the setup ring function\n");
}

void init_pancake_mem() {
    unsigned long sz = 2048*2048; // 1 MB unit\n",
    unsigned long cml_heap_sz = sz;    // Default: 1 MB heap\n", (* TODO: parameterise *)
    unsigned long cml_stack_sz = sz;   // Default: 1 MB stack\n", (* TODO: parameterise *)
    cml_heap = cml_memory;
    cml_stack = cml_heap + cml_heap_sz;
    cml_stackend = cml_stack + cml_stack_sz;
}

/* sel4cp Entry Points */

void init(void)
{
    notified_return = __builtin_return_address(0);
    sel4cp_dbg_puts(sel4cp_name);
    sel4cp_dbg_puts(": elf PD init function running\n");

    eth_setup();
    init_pancake_mem();
    // For now we are going to call handle_notified with a negative integer to get pseudo-pancake 
    // to setup its structs and populate them

    handle_notified(INIT_PAN_DS);
    /* Now wait for notification from lwip that buffers are initialised */
}

seL4_MessageInfo_t
protected(sel4cp_channel ch, sel4cp_msginfo msginfo)
{
    notified_return = __builtin_return_address(0);
    sel4cp_dbg_puts("Entering protected function\n");

    switch (ch) {
        case INIT:
            // return the MAC address. 
            sel4cp_mr_set(0, eth->palr);
            sel4cp_mr_set(1, eth->paur);
            return sel4cp_msginfo_new(0, 2);
        case TX_CH:
            handle_notified(ch);
            break;
        default:
            sel4cp_dbg_puts("Received ppc on unexpected channel ");
            puthex64(ch);
            break;
    }
    return sel4cp_msginfo_new(0, 0);
}

void notified(sel4cp_channel ch)
{
    notified_return = __builtin_return_address(0);
    
    sel4cp_dbg_puts("Entering the notified function\n");
    if (ch == INIT) {
        sel4cp_dbg_puts("Notified INIT case\n");
    } else if (ch == TX_CH) {
        sel4cp_dbg_puts("Notified TX case\n");
    } else if (ch == RX_CH) {
        sel4cp_dbg_puts("Notified RX case\n");
    } else if (ch == IRQ_CH) {
        sel4cp_dbg_puts("Notified IRQ case\n");
    }
    current_channel = ch;
    // Need to also do some signal ack here or in exit
    handle_notified(ch);
}

void handle_notified(int ch) {
    sel4cp_dbg_puts("Jumping to pancake main function\n");
    cml_main();
}
