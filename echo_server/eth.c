#include <stdbool.h>
#include <stdint.h>
#include <sel4cp.h>
#include <sel4/sel4.h>
#include "eth.h"
#include "shared_ringbuffer.h"
#include "util.h"
#include "basis_ffi.h"


ring_ctx_t rx;
ring_ctx_t tx;
unsigned int tx_lengths[TX_COUNT];

static void update_ring_slot(
    ring_ctx_t *ring,
    unsigned int idx,
    uintptr_t phys,
    uint16_t len,
    uint16_t stat)
{
    volatile struct descriptor *d = &(ring->descr[idx]);
    d->addr = phys;
    d->len = len;

    /* Ensure all writes to the descriptor complete, before we set the flags
     * that makes hardware aware of this slot.
     */
    synchronise_call();

    d->stat = stat;
}

static void fill_rx_bufs()
{
    ring_ctx_t *ring = &rx;
    synchronise_call();
    while (ring->remain > 0) {
        /* request a buffer */
        void *cookie = NULL;
        uintptr_t phys = alloc_rx_buf(MAX_PACKET_SIZE, &cookie);
        if (!phys) {
            break;
        }
        uint16_t stat = RXD_EMPTY;
        int idx = ring->tail;
        int new_tail = idx + 1;
        if (new_tail == ring->cnt) {
            new_tail = 0;
            stat |= WRAP;
        }
        ring->cookies[idx] = cookie;
        update_ring_slot(ring, idx, phys, 0, stat);
        ring->tail = new_tail;
        /* There is a race condition if add/remove is not synchronized. */
        ring->remain--;
    }
    
    synchronise_call();

    if (ring->tail != ring->head) {
        /* Make sure rx is enabled */
        enable_rx();
    }
}

static void
handle_rx()
{
    ring_ctx_t *ring = &rx;
    unsigned int head = ring->head;

    int num = 1;

    unsigned char empty_c[1];
    long empty_clen = 1;
    unsigned char empty_a[1];
    long empty_alen = 1;

    eth_ring_empty(empty_c, empty_clen, empty_a, empty_alen);

    int was_empty = empty_a[0];

    unsigned char size_c[1];
    long size_clen = 1;
    unsigned char size_a[1];
    long size_alen = 1;

    eth_ring_size(size_c, size_clen, size_a, size_alen);

    int ring_size = size_a[0];

    // we don't want to dequeue packets if we have nothing to replace it with
    while (head != ring->tail && (ring_size > num)) {
        volatile struct descriptor *d = &(ring->descr[head]);

        /* If the slot is still marked as empty we are done. */
        if (d->stat & RXD_EMPTY) {
            break;
        }

        void *cookie = ring->cookies[head];
        /* Go to next buffer, handle roll-over. */
        if (++head == ring->cnt) {
            head = 0;
        }
        ring->head = head;

        /* There is a race condition here if add/remove is not synchronized. */
        ring->remain++;

        unsigned char c_arr[8];
        long clen = 8;
        unsigned char a_arr[1];
        long alen = 0;
        int_to_byte8(cookie, c_arr);

        eth_driver_enqueue_avail(c_arr, clen, a_arr, alen);

        num++;

        // This might not be correct
        eth_ring_size(size_c, size_clen, size_a, size_alen);
        ring_size = size_a[0];
    }

    /* Notify client (only if we have actually processed a packet and 
    the client hasn't already been notified!) */
    if (num > 1 && was_empty) {
        sel4cp_notify(RX_CH);
    } 
}

static void
complete_tx()
{
    unsigned int cnt_org;
    void *cookie;
    ring_ctx_t *ring = &tx;
    unsigned int head = ring->head;
    unsigned int cnt = 0;

    while (head != ring->tail) {
        if (0 == cnt) {
            cnt = tx_lengths[head];
            if ((0 == cnt) || (cnt > TX_COUNT)) {
                /* We are not supposed to read 0 here. */
                print("complete_tx with cnt=0 or max");
                return;
            }
            cnt_org = cnt;
            cookie = ring->cookies[head];
        }

        volatile struct descriptor *d = &(ring->descr[head]);

        /* If this buffer was not sent, we can't release any buffer. */
        if (d->stat & TXD_READY) {

            /* give it another chance */
            tx_descr_active();

            if (d->stat & TXD_READY) {
                return;
            }
        }

        /* Go to next buffer, handle roll-over. */
        if (++head == TX_COUNT) {
            head = 0;
        }

        if (0 == --cnt) {
            ring->head = head;
            /* race condition if add/remove is not synchronized. */
            ring->remain += cnt_org;
            /* give the buffer back */
            unsigned char c_arr[8];
            long clen = 8;
            unsigned char a_arr[1];
            long alen = 0;
            int_to_byte8(cookie, c_arr);

            eth_driver_enqueue_avail(c_arr, clen, a_arr, alen);

        }
    }

    /* The only reason to arrive here is when head equals tails. If cnt is not
     * zero, then there is some kind of overflow or data corruption. The number
     * of tx descriptors holding data can't exceed the space in the ring.
     */
    if (0 != cnt) {
        print("head reached tail, but cnt!= 0");
    }
}

static void
raw_tx(unsigned int num, uintptr_t *phys,
                  unsigned int *len, void *cookie)
{
    ring_ctx_t *ring = &tx;

    /* Ensure we have room */
    if (ring->remain < num) {
        /* not enough room, try to complete some and check again */
        complete_tx();
        unsigned int rem = ring->remain;
        if (rem < num) {
            print("TX queue lacks space");
            return;
        }
    }

    synchronise_call();

    unsigned int tail = ring->tail;
    unsigned int tail_new = tail;

    unsigned int i = num;
    while (i-- > 0) {
        uint16_t stat = TXD_READY;
        if (0 == i) {
            stat |= TXD_ADDCRC | TXD_LAST;
        }

        unsigned int idx = tail_new;
        if (++tail_new == TX_COUNT) {
            tail_new = 0;
            stat |= WRAP;
        }
        update_ring_slot(ring, idx, *phys++, *len++, stat);
    }

    ring->cookies[tail] = cookie;
    tx_lengths[tail] = num;
    ring->tail = tail_new;
    /* There is a race condition here if add/remove is not synchronized. */
    ring->remain -= num;

    synchronise_call();

    tx_descr_active();

}

static void 
handle_eth()
{
    unsigned char c_arr[1];
    long clen = 1;
    unsigned char a_arr[8];
    long alen = 0;

    get_irq(c_arr, clen, a_arr, alen);

    uint32_t e = byte4_to_int(a_arr);

    while (e & IRQ_MASK) {
        if (e & NETIRQ_TXF) {
            complete_tx();
        }
        if (e & NETIRQ_RXF) {
            handle_rx();
            fill_rx_bufs();
        }
        if (e & NETIRQ_EBERR) {
            print("Error: System bus/uDMA");
            while (1);
        }
        get_irq(c_arr, clen, a_arr, alen);

        uint32_t e = byte4_to_int(a_arr);
    }
}

static void 
handle_tx()
{
    uintptr_t buffer = 0;
    unsigned int len = 0;
    void *cookie = NULL;

    // Setting up a driver dequeue ffi call
    unsigned char c_arr[64];
    long clen = 64;
    // For now, can only accomodate for inputs of up to 2048 characters. The same size as the buffers
    unsigned char a_arr[1];
    a_arr[0] = 1;
    // a_arr[0] = 1;
    long alen = 1;

    int_to_byte8(buffer, c_arr);
    int_to_byte8(len, &c_arr[8]);
    int_to_byte8(cookie, &c_arr[16]);

    eth_driver_dequeue_used(c_arr, clen, a_arr, alen);
    int driver_dequeue_ret = a_arr[0];

    //driver_dequeue(tx_ring.used_ring, &buffer, &len, &cookie)

    // We need to put in an empty condition here. 

    while ((tx.remain > 1) && !driver_dequeue_ret) {
        uintptr_t phys = getPhysAddr(buffer);
        raw_tx(1, &phys, &len, cookie);

        eth_driver_dequeue_used(c_arr, clen, a_arr, alen);
        int driver_dequeue_ret = a_arr[0];
    }
}

void create_ds() {
    /* set up descriptor rings */

    unsigned char c_arr[8];
    long clen = 8;
    unsigned char a_arr[1];
    long alen = 0;

    rx.cnt = RX_COUNT;
    rx.remain = rx.cnt - 2;
    rx.tail = 0;
    rx.head = 0;
    get_rx_phys(c_arr, clen, a_arr, alen);
    uintptr_t shared_dma_paddr = byte8_to_int(a_arr);
    rx.phys = shared_dma_paddr;
    get_rx_cookies(c_arr, clen, a_arr, alen);
    void **rx_cookies = (void **) byte8_to_int(a_arr);
    rx.cookies = rx_cookies;
    get_rx_descr(c_arr, clen, a_arr, alen);
    // I belive this is essentially a 256 entry buffer
    volatile struct descriptor *hw_ring_buffer_vaddr = (volatile struct descriptor *) byte8_to_int(a_arr);
    rx.descr = (volatile struct descriptor *)hw_ring_buffer_vaddr;

    tx.cnt = TX_COUNT;
    tx.remain = tx.cnt - 2;
    tx.tail = 0;
    tx.head = 0;
    get_tx_phys(c_arr, clen, a_arr, alen);
    uintptr_t tx_phys = byte8_to_int(a_arr);
    tx.phys = tx_phys;
    get_tx_cookies(c_arr, clen, a_arr, alen);
    void **tx_cookies = (void **) byte8_to_int(a_arr);
    tx.cookies = tx_cookies;
    get_tx_descr(c_arr, clen, a_arr, alen);
    
    volatile struct descriptor *tx_hw_ring_buffer_vaddr = (volatile struct descriptor *) byte8_to_int(a_arr);
    tx.descr = tx_hw_ring_buffer_vaddr;

}

void handle_notified(int ch) {
    switch(ch) {

    case INIT_PAN_DS:
        create_ds();
    case IRQ_CH:
        handle_eth();
        have_signal = true;
        signal_msg = seL4_MessageInfo_new(IRQAckIRQ, 0, 0, 0);
        signal = (BASE_IRQ_CAP + IRQ_CH);
        return;
    case INIT:
        init_post();
        fill_rx_bufs();
        break;
    case TX_CH:
        handle_tx();
        break;
    default:
        sel4cp_dbg_puts("eth driver: received notification on unexpected channel\n");
        break;
    }
}