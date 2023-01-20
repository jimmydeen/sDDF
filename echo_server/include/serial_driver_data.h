#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <sel4cp.h>
#include <sel4/sel4.h>
#include "serial.h"
#include "shared_ringbuffer.h"


/*
Header file to split the handling of data structures and device registers to a seperate interface
*/

static void imx_uart_set_baud(long bps);

int internal_is_tx_fifo_busy();

int serial_configure(long bps, int char_size, enum serial_parity parity, int stop_bits);

int getchar();

int putchar_regs(int c);

void init_post();

/*
rx_tx - 0 for rx, non-zero for tx
*/
int serial_dequeue_avail(uintptr_t *addr, unsigned int *len, void **cookie, int rx_tx);

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