#pragma once

#include "eth.h"
#include <stdint.h>

/* Helper FFI functions */
void int_to_byte4(int i, unsigned char *b);
int byte4_to_int(unsigned char *b);

void int_to_byte8(int i, unsigned char *b);
int byte8_to_int(unsigned char *b);

/* Driver functions */

uintptr_t getPhysAddr(uintptr_t virtual);
uintptr_t alloc_rx_buf(size_t buf_size, void **cookie);
void init_post();
void enable_rx();
void synchronise_call();
void tx_descr_active();
void eth_driver_dequeue_used(unsigned char *c, long clen, unsigned char *a, long alen);
void eth_driver_enqueue_used(unsigned char *c, long clen, unsigned char *a, long alen);
void eth_driver_enqueue_avail(unsigned char *c, long clen, unsigned char *a, long alen);
void eth_ring_empty(unsigned char *c, long clen, unsigned char *a, long alen);
void eth_ring_size(unsigned char *c, long clen, unsigned char *a, long alen);
void get_rx_phys(unsigned char *c, long clen, unsigned char *a, long alen);
void get_tx_phys(unsigned char *c, long clen, unsigned char *a, long alen);
void get_rx_cookies(unsigned char *c, long clen, unsigned char *a, long alen);
void get_tx_cookies(unsigned char *c, long clen, unsigned char *a, long alen);
void get_rx_descr(unsigned char *c, long clen, unsigned char *a, long alen);
void get_tx_descr(unsigned char *c, long clen, unsigned char *a, long alen);

