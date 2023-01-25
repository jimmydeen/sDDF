#include "shared_ringbuffer.h"
#include "util.h"
#include "netif/ethernet.h"

uintptr_t rx_avail_drv;
uintptr_t rx_used_drv;

uintptr_t rx_avail_cli;
uintptr_t rx_used_cli;

uintptr_t shared_dma_vaddr;
uintptr_t uart_base;

#define NUM_CLIENTS 1
/* Channel ID for driver, only used for initialisation. */
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
uint64_t total = 0;
uint64_t dropped = 0;

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

int compare_mac(uint8_t *mac1, uint8_t *mac2)
{
    for (int i = 0; i < 6; i++) {
        if (mac1[i] != mac2[i]) {
            return -1;
        }
    }
    return 0;
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
        } else if (compare_mac(dest_addr, broadcast) == 0) {
            // broadcast packet, send the packet to the first client to handle.
            // This is temporary, eventually we will have a different
            // component to deal with this.
            return 0;
        }
    }
    return -1;
}

/*
Loop over driver and insert all used rx buffers to appropriate client queues.
*/
void process_rx_complete(void)
{
    int notify_clients[NUM_CLIENTS];
    while(!ring_empty(state.rx_ring_drv.used_ring)) {
        total++;
        uintptr_t addr = 0;
        unsigned int len = 0;
        void *cookie = NULL;

        dequeue_used(&state.rx_ring_drv, &addr, &len, &cookie);
        int err = seL4_ARM_VSpace_Invalidate_Data(3, addr, addr + ETHER_MTU);
        if (err) {
            print("ARM Vspace invalidate failed\n");
            puthex64(err);
            print("\n");
        }

        // Get MAC address and work out which client it is.
        int client = get_client(addr);
        if (client >= 0) {
            /* enqueue it. */
            int was_empty = ring_empty(state.rx_ring_clients[client].used_ring);
            int err = enqueue_used(&state.rx_ring_clients[client], addr, len, cookie);
            if (err) {
                print("Mux_rx enqueue used failed. Ring full\n");
            }
            if (was_empty) {
                notify_clients[client] = 1;
            }
        } else {
            // no match, not for us, return the buffer to the driver.
            if (addr == 0) {
                print("MUX RX|ERROR: Attempting to add NULL buffer to driver RX ring\n");
            }
            err = enqueue_avail(&state.rx_ring_drv, addr, len, cookie);
            if (err) {
                print("MUX RX|ERROR: Failed to enqueue available to driver RX ring\n");
            }
            dropped++;
        }
    }

    /* Loop over bitmap and see who we need to notify. */
    for (int client = 0; client < NUM_CLIENTS; client++) {
        if (notify_clients[client]) {
            sel4cp_notify(client);
        }
    }
}

// Loop over all client rings and return unused rx buffers to the driver
void process_rx_free(void)
{
    /* If we have enqueued to the driver's available ring and the available
     * ring was empty, we want to notify the driver. We also only want to
     * notify it once.
     */
    bool was_empty = ring_empty(state.rx_ring_drv.avail_ring);
    bool enqueued = false;
    for (int i = 0; i < NUM_CLIENTS; i++) {
        while (!ring_empty(state.rx_ring_clients[i].avail_ring)) {
            uintptr_t addr;
            unsigned int len;
            void *buffer;
            int err = dequeue_avail(&state.rx_ring_clients[i], &addr, &len, &buffer);
            assert(!err);
            err = enqueue_avail(&state.rx_ring_drv, addr, len, buffer);
            assert(!err);
            enqueued = true;
        }
    }
    if (enqueued && was_empty) {
        sel4cp_notify(DRIVER_CH);
    }
}

seL4_MessageInfo_t
protected(sel4cp_channel ch, sel4cp_msginfo msginfo)
{
    if (ch >= NUM_CLIENTS) {
        sel4cp_dbg_puts("Received ppc on unexpected channel ");
        puthex64(ch);
        return sel4cp_msginfo_new(0, 0);
    }
    // return the MAC address.
    uint32_t lower = (state.mac_addrs[ch][0] << 24) |
                     (state.mac_addrs[ch][1] << 16) |
                     (state.mac_addrs[ch][2] << 8) |
                     (state.mac_addrs[ch][3]);
    uint32_t upper = (state.mac_addrs[ch][4] << 24) | (state.mac_addrs[ch][5] << 16);
    sel4cp_dbg_puts("Mux rx is sending mac: ");
    dump_mac(state.mac_addrs[ch]);
    sel4cp_mr_set(0, lower);
    sel4cp_mr_set(1, upper);
    return sel4cp_msginfo_new(0, 2);
}

void notified(sel4cp_channel ch)
{
    if (initialised) {
        process_rx_complete();
        process_rx_free();
    } else {
        process_rx_free();
        sel4cp_notify(DRIVER_CH);
        initialised = 1;
    }
}

void init(void)
{
    // set up client macs
    /*state.mac_addrs[0][0] = 0x52;
    state.mac_addrs[0][1] = 0x54;
    state.mac_addrs[0][2] = 0x1;
    state.mac_addrs[0][3] = 0;
    state.mac_addrs[0][4] = 0;
    state.mac_addrs[0][5] = 0;*/
    state.mac_addrs[0][0] = 0;
    state.mac_addrs[0][1] = 0x4;
    state.mac_addrs[0][2] = 0x9f;
    state.mac_addrs[0][3] = 0x5;
    state.mac_addrs[0][4] = 0xf8;
    state.mac_addrs[0][5] = 0xcc;

    /* Set up shared memory regions */
    ring_init(&state.rx_ring_drv, (ring_buffer_t *)rx_avail_drv, (ring_buffer_t *)rx_used_drv, NULL, 1);

    // FIX ME: Use the notify function pointer to put the notification in?
    ring_init(&state.rx_ring_clients[0], (ring_buffer_t *)rx_avail_cli, (ring_buffer_t *)rx_used_cli, NULL, 0);

    return;
}
