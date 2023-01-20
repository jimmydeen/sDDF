/*
* Sample serial driver for imx8mm based on the sDDF
*/

#include <stdbool.h>
#include <stdint.h>
#include <sel4cp.h>
#include <sel4/sel4.h>
#include "serial.h"
#include "shared_ringbuffer.h"
#include "serial_driver_data.h"

#define BIT(nr) (1UL << (nr))

// Defines to manage interrupts and notifications
#define IRQ_CH 1
#define TX_CH  8
#define RX_CH  10
#define INIT   4

// Global serial_driver variable

struct serial_driver global_serial_driver = {0};

int putchar(int c) {

    if (internal_is_tx_fifo_busy()) {
        // A transmit is probably in progress, we will have to wait
        return -1;
    } else {
        // Try and keep as much of the logic here as possible
        if (c == '\n') {
            // For now, by default we will have Auto-send CR(Carriage Return) enabled
            /* write CR first */
            putchar_regs('\r');
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
        }

        // Wait for the tx fifo buffer to pass through the '\r' character, refer to above comment
        while (internal_is_tx_fifo_busy()) {
            /* busy loop */
        }

        putchar_regs(c);
    }

    return 0;
}

// Called from handle tx, write each character stored in the buffer to the serial port
static void
raw_tx(char *phys, unsigned int len, void *cookie)
{
    sel4cp_dbg_puts("entering raw tx function\n");
    // This is byte by byte for now, switch to DMA use later
    for (int i = 0; i < len || phys[i] != '\0'; i++) {
        // Loop until the fifo queue is ready to transmit
        while (putchar(phys[i]) != 0);
    }
}

void handle_tx() {
    sel4cp_dbg_puts("In the handle tx func\n");
    uintptr_t buffer = 0;
    unsigned int len = 0;
    void *cookie = 0;
    // Dequeue something from the Tx ring -> the server will have placed something in here, if its empty then nothing to do
    sel4cp_dbg_puts("Dequeuing and printing everything currently in the ring buffer\n");
    while (!serial_driver_dequeue_used(&buffer, &len, &cookie, 1)) {
        sel4cp_dbg_puts("in the driver dequeue loop\n");
        // Buffer cointaining the bytes to write to serial
        char *phys = (char * )buffer;
        // Handle the tx
        raw_tx(phys, len, cookie);
        // Then enqueue this buffer back into the available queue, so that it can be collected and reused by the server
        serial_enqueue_avail(buffer, len, &cookie, 1);
    }
    sel4cp_dbg_puts("Finished handle_tx\n");
}

// Increment the number of chars that the server has requested us to get.
void handle_rx() {
    global_serial_driver.num_to_get_chars++;
}


void handle_irq() {
    /* Here we have interrupted because a character has been inputted. We first want to get the 
    character from the hardware FIFO queue.

    Then we want to dequeue from the rx available ring, and populate it, then add to the rx used queue
    ready to be processed by the client server
    */

    sel4cp_dbg_puts("Entering handle irq function\n");

    int input = getchar();

    if (input == -1) {
        sel4cp_dbg_puts(sel4cp_name);
        sel4cp_dbg_puts(": invalid input when attempting to getchar\n");
        return;
    }

    // Only process the character if we need to, that is the server is waiting on a getchar request

    /*
    I'm not too sure if we need to loop here, as we only have one server and one driver, and the 
    server should be blocking on getchar calls. Additionally, currently the driver has an unlimited budget, 
    and is running at a higher priority than any of the other PD's so it shouldn't be pre-empted. 

    However, if we have multiple clients waiting on getchars we may have an issue, I need to look 
    more into the expected behaviour of getchar in these situations.
    */
    sel4cp_dbg_puts("Looping to service all current requests to getchar\n");

    while (global_serial_driver.num_to_get_chars > 0) {
        sel4cp_dbg_puts("In loop\n");
        // Address that we will pass to dequeue to store the buffer address
        uintptr_t buffer = 0;
        // Integer to store the length of the buffer
        unsigned int buffer_len = 0; 

        void *cookie = 0;

        int ret = serial_dequeue_avail(&buffer, &buffer_len, &cookie, 0);

        if (ret != 0) {
            sel4cp_dbg_puts(sel4cp_name);
            sel4cp_dbg_puts(": unable to dequeue from the rx available ring\n");
            return;
        }

        ((char *) buffer)[0] = (char) input;

        // Now place in the rx used ring
        ret = serial_enqueue_used(buffer, 1, &cookie, 0);

        if (ret != 0) {
            sel4cp_dbg_puts(sel4cp_name);
            sel4cp_dbg_puts(": unable to enqueue to the tx available ring\n");
            return;
        }

        // We have serviced one getchar request, we can now decrement the count
        global_serial_driver.num_to_get_chars--;
    }

    sel4cp_dbg_puts("Finished handling the irq\n");
}

// Moved to serial_driver_data.c
// void init_post() {
//     sel4cp_dbg_puts(sel4cp_name);
//     sel4cp_dbg_puts(": init_post function running\n");

    
//     // Redundant right now, as a channel has not been set up for init calls
//     // sel4cp_notify(INIT);
// }

// Init function required by CP for every PD
void init(void) {
    sel4cp_dbg_puts(sel4cp_name);
    sel4cp_dbg_puts(": elf PD init function running\n");


    // Call init_post here to setup the ring buffer regions. The init_post case in the notified
    // switch statement may be redundant.
    init_post();

    // imx_uart_regs_t *regs = (imx_uart_regs_t *) uart_base;

    // Software reset results in failed uart init, not too sure why
    /* Software reset */
    // regs->cr2 &= ~UART_CR2_SRST;
    // while (!(regs->cr2 & UART_CR2_SRST));

    sel4cp_dbg_puts("Line configuration\n");

    /* Line configuration */
    int ret = serial_configure(115200, 8, PARITY_NONE, 1);

    if (ret != 0) {
        sel4cp_dbg_puts("Error occured during line configuration\n");
    }
}

// Entry point that is invoked on a serial interrupt, or notifications from the server using the TX and RX channels
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
            sel4cp_dbg_puts("Notified to print something\n");
            handle_tx();
            break;
        case RX_CH:
            handle_rx();
            break;
        default:
            sel4cp_dbg_puts("eth driver: received notification on unexpected channel\n");
            break;
    }
}