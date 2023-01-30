/*
* Seperated file to deal with shared data structures and device register interaction.
*/

#include <stdbool.h>
#include <stdint.h>
#include <sel4cp.h>
#include <sel4/sel4.h>
#include "serial.h"
#include "shared_ringbuffer.h"
#include "serial_driver_data.h"

/* Shared Memory regions. These all have to be here to keep compiler happy */
// Ring handle components
uintptr_t rx_avail;
uintptr_t rx_used;
uintptr_t tx_avail;
uintptr_t tx_used;
uintptr_t shared_dma_vaddr;
// Base of the uart registers
uintptr_t uart_base;

/* Pointers to shared_ringbuffers */
ring_handle_t rx_ring;
ring_handle_t tx_ring;

struct serial_driver global_serial_driver_data = {0};

/*
 * BaudRate = RefFreq / (16 * (BMR + 1)/(BIR + 1) )
 * BMR and BIR are 16 bit
 * Function taken from seL4 util_libs serial.c implementation for imx8mm
 */
static void imx_uart_set_baud(long bps)
{
    imx_uart_regs_t *regs = (imx_uart_regs_t *) uart_base;

    uint32_t bmr, bir, fcr;
    fcr = regs->fcr;
    fcr &= ~UART_FCR_RFDIV_MASK;
    fcr |= UART_FCR_RFDIV(4);
    bir = 0xf;
    bmr = UART_REF_CLK / bps - 1;
    regs->bir = bir;
    regs->bmr = bmr;
    regs->fcr = fcr;
}

void internal_is_tx_fifo_busy(unsigned char *c, long clen, unsigned char *a, long alen)
{
    imx_uart_regs_t *regs = (imx_uart_regs_t *) uart_base;

    /* check the TXFE (transmit buffer FIFO empty) flag, which is cleared
     * automatically when data is written to the TxFIFO. Even though the flag
     * is set, the actual data transmission via the UART's 32 byte FIFO buffer
     * might still be in progress.
     */

    char ret = (0 == (regs->sr2 & UART_SR2_TXFIFO_EMPTY));
    sel4cp_dbg_puts("Attempting to access a buffer\n");
    a[0] = ret;
    sel4cp_dbg_puts("Returning from tx fifo busy function\n");
}

int serial_configure(
    long bps,
    int char_size,
    enum serial_parity parity,
    int stop_bits)
{
    imx_uart_regs_t *regs = (imx_uart_regs_t *) uart_base;
    
    uint32_t cr2;
    /* Character size */
    cr2 = regs->cr2;
    if (char_size == 8) {
        cr2 |= UART_CR2_WS;
    } else if (char_size == 7) {
        cr2 &= ~UART_CR2_WS;
    } else {
        return -1;
    }
    /* Stop bits */
    if (stop_bits == 2) {
        cr2 |= UART_CR2_STPB;
    } else if (stop_bits == 1) {
        cr2 &= ~UART_CR2_STPB;
    } else {
        return -1;
    }
    /* Parity */
    if (parity == PARITY_NONE) {
        cr2 &= ~UART_CR2_PREN;
    } else if (parity == PARITY_ODD) {
        /* ODD */
        cr2 |= UART_CR2_PREN;
        cr2 |= UART_CR2_PROE;
    } else if (parity == PARITY_EVEN) {
        /* Even */
        cr2 |= UART_CR2_PREN;
        cr2 &= ~UART_CR2_PROE;
    } else {
        return -1;
    }
    /* Apply the changes */
    regs->cr2 = cr2;
    sel4cp_dbg_puts("finished configuring the line, setting the baud rate\n");
    /* Now set the board rate */
    imx_uart_set_baud(bps);

    sel4cp_dbg_puts("Configured serial, enabling uart\n");

    /* Enable the UART */
    regs->cr1 |= UART_CR1_UARTEN;                /* Enable The uart.                  */
    regs->cr2 |= UART_CR2_RXEN | UART_CR2_TXEN;  /* RX/TX enable                      */
    regs->cr2 |= UART_CR2_IRTS;                  /* Ignore RTS                        */
    regs->cr3 |= UART_CR3_RXDMUXDEL;             /* Configure the RX MUX              */
    /* Initialise the receiver interrupt.                                             */
    regs->cr1 &= ~UART_CR1_RRDYEN;               /* Disable recv interrupt.           */
    regs->fcr &= ~UART_FCR_RXTL_MASK;            /* Clear the rx trigger level value. */
    regs->fcr |= UART_FCR_RXTL(1);               /* Set the rx tigger level to 1.     */
    regs->cr1 |= UART_CR1_RRDYEN;                /* Enable recv interrupt.            */
    return 0;
}

void getchar(unsigned char *c, long clen, unsigned char *a, long alen)
{
    imx_uart_regs_t *regs = (imx_uart_regs_t *) uart_base;

    uint32_t reg = 0;
    int c_reg = -1;

    if (regs->sr2 & UART_SR2_RXFIFO_RDR) {
        reg = regs->rxd;
        if (reg & UART_URXD_READY_MASK) {
            c_reg = reg & UART_BYTE_MASK;
        }
    }

    char got_char = (char) c_reg;

    a[0]= got_char;
}

// Putchar that is using the hardware FIFO buffers --> Switch to DMA later 
void putchar_regs(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("Entered putchar in serial_driver_data\n");
    
    imx_uart_regs_t *regs = (imx_uart_regs_t *) uart_base;

    regs->txd = c[0];
}

void init_post(unsigned char *c, long clen, unsigned char *a, long alen) {
    // Setup the ring buffer mechanisms here as well as init the global serial driver data


    sel4cp_dbg_puts("Init the ring buffers\n");

    // Init the shared ring buffers
    ring_init(&rx_ring, (ring_buffer_t *)rx_avail, (ring_buffer_t *)rx_used, NULL, 0);
    ring_init(&tx_ring, (ring_buffer_t *)tx_avail, (ring_buffer_t *)tx_used, NULL, 0);

    // Setup the global serial driver data
    global_serial_driver_data.regs = (imx_uart_regs_t *) uart_base;
    global_serial_driver_data.rx_ring = rx_ring;
    global_serial_driver_data.tx_ring = tx_ring;
    global_serial_driver_data.num_to_get_chars = 0;

    sel4cp_dbg_puts("Line configuration\n");

    /* Line configuration */
    int ret = serial_configure(115200, 8, PARITY_NONE, 1);

    if (ret != 0) {
        sel4cp_dbg_puts("Error occured during line configuration\n");
    }
}

void serial_dequeue_avail(unsigned char *c, long clen, unsigned char *a, long alen) {
    // Dequeue from shared mem avail avail buffer

    if (clen != 1) {
        sel4cp_dbg_puts("There are no arguments supplied when args are expected");
        return;
    }

    bool rx_tx = c[0];

    void *cookie = 0;

    // Address that we will pass to dequeue to store the buffer address
    uintptr_t buffer = 0;
    // Integer to store the length of the buffer
    unsigned int buffer_len = 0; 

    if (rx_tx == 0) {
        a[0] = dequeue_avail(&rx_ring, &buffer, &buffer_len, cookie);
    } else {
        a[0] = dequeue_avail(&tx_ring, &buffer, &buffer_len, cookie);
    }
}

int serial_enqueue_used(uintptr_t addr, unsigned int len, void *cookie, int rx_tx) {
    if (rx_tx == 0) {
        return enqueue_used(&rx_ring, addr, len, cookie);
    } else {
        return enqueue_used(&tx_ring, addr, len, cookie);
    }
}

int serial_driver_dequeue_used(uintptr_t *addr, unsigned int *len, void **cookie, int rx_tx) {
    if (rx_tx == 0) {
        return driver_dequeue(rx_ring.used_ring, addr, len, cookie);
    } else {
        return driver_dequeue(tx_ring.used_ring, addr, len, cookie);
    }
}

int serial_enqueue_avail(uintptr_t addr, unsigned int len, void *cookie, int rx_tx) {
    if (rx_tx == 0) {
        return enqueue_avail(&rx_ring, addr, len, cookie);
    } else {
        return enqueue_avail(&tx_ring, addr, len, cookie);
    }
}