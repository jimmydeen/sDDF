#pragma once

#include "eth.h"
#include <stdint.h>

/* Helper FFI functions */
void int_to_byte4(int i, unsigned char *b);
int byte4_to_int(unsigned char *b);

/* Driver functions */
static void get_mac_addr(volatile struct enet_regs *reg, uint8_t *mac);
static void set_mac(volatile struct enet_regs *reg, uint8_t *mac);
static void dump_mac(uint8_t *mac);
static uintptr_t getPhysAddr(uintptr_t virtual);
static inline void enable_irqs(volatile struct enet_regs *eth, uint32_t mask);
static uintptr_t alloc_rx_buf(size_t buf_size, void **cookie);
static void fill_rx_bufs();
static void eth_setup(void);
void init_post();
void enable_rx();
void synchronise_call();
void tx_descr_active();
void eth_driver_dequeue_used(unsigned char *c, long clen, unsigned char *a, long alen);
void eth_driver_enqueue_used(unsigned char *c, long clen, unsigned char *a, long alen);
void eth_driver_enqueue_avail(unsigned char *c, long clen, unsigned char *a, long alen);
void eth_ring_empty(unsigned char *c, long clen, unsigned char *a, long alen);
void eth_ring_size(unsigned char *c, long clen, unsigned char *a, long alen);

static void handle_eth(volatile struct enet_regs *eth);