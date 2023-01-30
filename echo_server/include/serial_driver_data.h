#pragma once

#include <stdint.h>

/*
Header file to split the handling of data structures and device registers to a seperate interface
*/

/*
The following functions deal with interacting with the memory mapped device registers
*/
static void imx_uart_set_baud(long bps);

void internal_is_tx_fifo_busy(unsigned char *c, long clen, unsigned char *a, long alen);

int serial_configure(long bps, int char_size, enum serial_parity parity, int stop_bits);

void getchar(unsigned char *c, long clen, unsigned char *a, long alen);

void putchar_regs(unsigned char *c, long clen, unsigned char *a, long alen);

void init_post(unsigned char *c, long clen, unsigned char *a, long alen);

/*
The following functions deal with accessing the shared memory between the driver and the server
*/
/*
rx_tx - 0 for rx, non-zero for tx
*/
void serial_dequeue_avail(unsigned char *c, long clen, unsigned char *a, long alen);

/*
rx_tx - 0 for rx, non-zero for tx
*/
int serial_enqueue_used(uintptr_t addr, unsigned int len, void *cookie, int rx_tx);

/*
rx_tx - 0 for rx, non-zero for tx
*/
int serial_driver_dequeue_used(uintptr_t *addr, unsigned int *len, void **cookie, int rx_tx);

/*
rx_tx - 0 for rx, non-zero for tx
*/
int serial_enqueue_avail(uintptr_t addr, unsigned int len, void *cookie, int rx_tx);