#include <stdbool.h>
#include <stdint.h>
#include <sel4cp.h>
#include <sel4/sel4.h>
#include "eth.h"
#include "shared_ringbuffer.h"
#include "util.h"
#include "basis_ffi.h"

#define IRQ_CH 1
#define TX_CH  2
#define RX_CH  2
#define INIT   4
/* Make the minimum frame buffer 2k. This is a bit of a waste of memory, but ensures alignment */
#define PACKET_BUFFER_SIZE  2048
#define MAX_PACKET_SIZE     1536

#define RX_COUNT 256
#define TX_COUNT 256
_Static_assert((512 * 2) * PACKET_BUFFER_SIZE <= 0x200000, "Expect rx+tx buffers to fit in single 2MB page");
_Static_assert(sizeof(ring_buffer_t) <= 0x200000, "Expect ring buffer ring to fit in single 2MB page");

struct descriptor {
    uint16_t len;
    uint16_t stat;
    uint32_t addr;
};

typedef struct {
    unsigned int cnt;
    unsigned int remain;
    unsigned int tail;
    unsigned int head;
    volatile struct descriptor *descr;
    uintptr_t phys;
    void **cookies;
} ring_ctx_t;

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
    int was_empty = ring_empty(rx_ring.used_ring);

    // we don't want to dequeue packets if we have nothing to replace it with
    while (head != ring->tail && (ring_size(rx_ring.avail_ring) > num)) {
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

        buff_desc_t *desc = (buff_desc_t *)cookie;

        enqueue_used(&rx_ring, desc->encoded_addr, d->len, desc->cookie);
        num++;
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
            buff_desc_t *desc = (buff_desc_t *)cookie;

            enqueue_avail(&tx_ring, desc->encoded_addr, desc->len, desc->cookie);
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

    uint32_t e = byte4_to_int(a);

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

        uint32_t e = byte4_to_int(a);
    }
}

static void 
handle_tx()
{
    uintptr_t buffer = 0;
    unsigned int len = 0;
    void *cookie = NULL;

    // Setting up a driver dequeue ffi call
    unsigned char c_arr[20];
    c_arr[0] = 1;
    long clen = 1;
    // For now, can only accomodate for inputs of up to 2048 characters. The same size as the buffers
    unsigned char a_arr[2048];
    // a_arr[0] = 1;
    long alen = 2048;

    eth_driver_dequeue_used(c_arr, clen, a_arr, alen);
    int driver_dequeue_ret = a_arr[0];

    // We need to put in an empty condition here. 
    while ((tx.remain > 1) && !driver_dequeue(tx_ring.used_ring, &buffer, &len, &cookie)) {
        uintptr_t phys = getPhysAddr(buffer);
        raw_tx(1, &phys, &len, cookie);
    }
}



void handle_notified(int ch) {
    switch(ch) {
    case IRQ_CH:
        handle_eth();
        have_signal = true;
        signal_msg = seL4_MessageInfo_new(IRQAckIRQ, 0, 0, 0);
        signal = (BASE_IRQ_CAP + IRQ_CH);
        return;
    case INIT:
        init_post();
        break;
    case TX_CH:
        handle_tx();
        break;
    default:
        sel4cp_dbg_puts("eth driver: received notification on unexpected channel\n");
        break;
    }
}