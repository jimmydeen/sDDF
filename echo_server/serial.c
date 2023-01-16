/*
* Sample serial driver for imx8mm based on the sDDF
*/

#include <stdbool.h>
#include <stdint.h>
#include <sel4cp.h>
#include <sel4/sel4.h>
#include "serial.h"
#include "shared_ringbuffer.h"

#define BIT(nr) (1UL << (nr))

// Defines to manage interrupts and notifications
#define IRQ_CH 1
#define TX_CH  8
#define RX_CH  10
#define INIT   4

/* Memory regions. These all have to be here to keep compiler happy */
// Ring handle components
uintptr_t rx_avail;
uintptr_t rx_used;
uintptr_t tx_avail;
uintptr_t tx_used;
uintptr_t shared_dma_vaddr;
uintptr_t shared_dma_paddr;
// Base of the uart registers
uintptr_t uart_base;

/* Pointers to shared_ringbuffers */
ring_handle_t rx_ring;
ring_handle_t tx_ring;

// Global serial_driver variable

struct serial_driver global_serial_driver = {};

// Function taken from eth.c
static uintptr_t 
getPhysAddr(uintptr_t virtual)
{
    uint64_t offset = virtual - shared_dma_vaddr;
    uintptr_t phys;

    if (offset < 0) {
        return 0;
    }

    phys = shared_dma_paddr + offset;
    return phys;
}

/*
 * BaudRate = RefFreq / (16 * (BMR + 1)/(BIR + 1) )
 * BMR and BIR are 16 bit
 * Function taken from seL4 util_libs serial.c implementation for imx8mm
 */
static void imx_uart_set_baud(long bps)
{
    imx_uart_regs_t *regs = &uart_base;
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

static int internal_is_tx_fifo_busy(
    imx_uart_regs_t *regs)
{
    /* check the TXFE (transmit buffer FIFO empty) flag, which is cleared
     * automatically when data is written to the TxFIFO. Even though the flag
     * is set, the actual data transmission via the UART's 32 byte FIFO buffer
     * might still be in progress.
     */
    return (0 == (regs->sr2 & UART_SR2_TXFIFO_EMPTY));
}

int getchar()
{
    imx_uart_regs_t *regs = &uart_base;
    uint32_t reg = 0;
    int c = -1;

    if (regs->sr2 & UART_SR2_RXFIFO_RDR) {
        reg = regs->rxd;
        if (reg & UART_URXD_READY_MASK) {
            c = reg & UART_BYTE_MASK;
        }
    }
    return c;
}

// Putchar that is using the hardware FIFO buffers --> Switch to DMA later 
int putchar(int c) {

    imx_uart_regs_t *regs = &uart_base;

    if (internal_is_tx_fifo_busy(regs)) {
        // A transmit is probably in progress, we will have to wait
        return -1;
    }

    if (c == '\n') {
        // For now, by default we will have Auto-send CR(Carriage Return) enabled
        /* write CR first */
        regs->txd = '\r';
        /* if we transform a '\n' (LF) into '\r\n' (CR+LF) this shall become an
         * atom, ie we don't want CR to be sent and then fail at sending LF
         * because the TX FIFO is full. Basically there are two options:
         *   - check if the FIFO can hold CR+LF and either send both or none
         *   - send CR, then block until the FIFO has space and send LF.
         * Assuming that if SERIAL_AUTO_CR is set, it's likely this is a serial
         * console for logging, so blocking seems acceptable in this special
         * case. The IMX6's TX FIFO size is 32 byte and TXFIFO_EMPTY is cleared
         * automatically as soon as data is written from regs->txd into the
         * FIFO. Thus the worst case blocking is roughly the time it takes to
         * send 1 byte to have room in the FIFO again. At 115200 baud with 8N1
         * this takes 10 bit-times, which is 10/115200 = 86,8 usec.
         */
        while (internal_is_tx_fifo_busy(regs)) {
            /* busy loop */
        }
    }

    return 0;
}

// Called from handle tx, write each character stored in the buffer to the serial port
static void
raw_tx(uintptr_t *phys, unsigned int *len, void *cookie)
{
    // This is byte by byte for now, switch to DMA use later
    for (int i = 0; i < *len; i++) {
        int ret = putchar(phys[i]);
        if (ret == -1) {
            // If the FIFO queue is full, we will just fail
            //  Maybe try requeue this?
            return;
        }
        // Zero out the curr location in the buffer
        phys[i] = 0;
    }
}

void handle_tx() {
    uintptr_t buffer = 0;
    unsigned int len = 0;
    void *cookie = 0;
    // Dequeue something from the Tx ring -> the server will have placed something in here, if its empty then nothing to do
    while (!driver_dequeue(tx_ring.used_ring, &buffer, &len, &cookie)) {
        // Buffer cointaining the bytes to write to serial
        uintptr_t phys = getPhysAddr(buffer);
        // Handle the tx
        raw_tx(&phys, &len, cookie);
        // Then enqueue this buffer back into the available queue, so that it can be collected and reused by the server
        enqueue_avail(&tx_ring, buffer, len, &cookie);
    }
}

// Called from handle rx, write each character stored in the buffer to the serial port
int
raw_rx(uintptr_t *phys, unsigned int *len, void *cookie)
{
    int c = getchar();
    if (c == -1) {
        return -1;
    }

    *phys = c;
    *len = 1;

    return 0;
    
}

// Very inefficient as each DMA buffer will hold one character, should get lots of get chars at a time, or potentially until EOF or new line
// This means that there can only be 512 characters read in at a time
// TO-DO
void handle_rx() {
    uintptr_t buffer = 0;
    unsigned int len = 0;
    void *cookie = 0;

    // Dequeue a DMA'able buffer from the avail rx ring, and pass address to write to
    int ret = driver_dequeue(rx_ring.avail_ring, &buffer, &len, &cookie);

    // There are no available rings, cannot do anything here
    if (ret == -1) {
        sel4cp_dbg_puts(sel4cp_name);
        sel4cp_dbg_puts(":Rx available ring is full!\n");
    }

    uintptr_t phys = getPhysAddr(buffer);

    // Handle the rx
    ret = raw_rx(&phys, &len, cookie);

    if (ret == -1) {
        // Nothing was read, put the buffer back into the available queue
        enqueue_avail(&rx_ring, buffer, len, &cookie);
    } else {
        enqueue_used(&rx_ring, buffer, len, &cookie);
        // As we have actually processed an rx request, notify the server
        // TO-DO - setup the channels between the driver and server
        sel4cp_notify(RX_CH);
    }
}

int serial_configure(
    long bps,
    int char_size,
    enum serial_parity parity,
    int stop_bits)
{
    imx_uart_regs_t *regs = &uart_base;
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
    /* Now set the board rate */
    imx_uart_set_baud(bps);
    return 0;
}

void handle_irq() {
    // TO-DO
    // And what each serial irq corresponds to - can't find the appropriate documentation

    /* Here we have interrupted because a character has been inputted. We first want to get the 
    character from the hardware FIFO queue.

    Then we want to dequeue from the rx available ring, and populate it, then add to the rx used queue
    ready to be processed by the client server
    
    */
    imx_uart_regs_t *regs = &uart_base;

    int input = getchar();

    if (input == -1) {
        sel4cp_dbg_puts(sel4cp_name);
        sel4cp_dbg_puts(": invalid input when attempting to getchar\n");
        return;
    }

    // Only process the character if we need to, that is the server is waiting on a getchar request
    while (global_serial_driver.num_to_get_chars > 0) {
        // Address that we will pass to dequeue to store the buffer address
        uintptr_t buffer_addr;
        // Integer to store the length of the buffer
        int buffer_len; 

        int ret = dequeue_avail(&rx_ring, &buffer_addr, &buffer_len, NULL);

        if (ret != 0) {
            sel4cp_dbg_puts(sel4cp_name);
            sel4cp_dbg_puts(": unable to dequeue from the rx available ring\n");
            return;
        }

        *buffer_addr = input;

        // Now place in the rx used ring
        ret = enqueue_used(&rx_ring, &buffer_addr, 1, NULL);

        if (ret != 0) {
            sel4cp_dbg_puts(sel4cp_name);
            sel4cp_dbg_puts(": unable to enqueue to the tx available ring\n");
            return;
        }

        // We have serviced one getchar request, we can now decrement the count
        global_serial_driver.num_to_get_chars--;
    }
}

void init_post() {
    sel4cp_dbg_puts(sel4cp_name);
    sel4cp_dbg_puts(": init_post function running\n");
    // Init the shared ring buffers
    ring_init(&rx_ring, (ring_buffer_t *)rx_avail, (ring_buffer_t *)rx_used, NULL, 0);
    ring_init(&rx_ring, (ring_buffer_t *)tx_avail, (ring_buffer_t *)tx_used, NULL, 0);
    
    sel4cp_notify(INIT);

}

// Init function required by CP for every PD
void init(void) {
    sel4cp_dbg_puts(sel4cp_name);
    sel4cp_dbg_puts(": elf PD init function running\n");


    imx_uart_regs_t *regs = &uart_base;

    /* Software reset */
    regs->cr2 &= ~UART_CR2_SRST;
    while (!(regs->cr2 & UART_CR2_SRST));

    /* Line configuration */
    serial_configure(115200, 8, PARITY_NONE, 1);

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

    global_serial_driver.regs = regs;
    global_serial_driver.rx_ring = rx_ring;
    global_serial_driver.tx_ring = tx_ring;
    global_serial_driver.num_to_get_chars = 0;
}

// Entry point that is invoked on a serial interrupt
void notified(sel4cp_channel ch) {
    sel4cp_dbg_puts(sel4cp_name);
    sel4cp_dbg_puts(": elf PD notified function running\n");

    switch(ch) {
        case IRQ_CH:
            handle_irq();
            sel4cp_irq_ack(ch);
            return;
        case INIT:
            init_post();
            break;
        case TX_CH:
            handle_tx();
            break;
        case RX_CH:
            // handle_rx();
            global_serial_driver.num_to_get_chars++;
            break;
        default:
            sel4cp_dbg_puts("eth driver: received notification on unexpected channel\n");
            break;
    }
}