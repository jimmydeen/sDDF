/*
* Sample serial driver for odroid c4 (amlogic meson gx uart) based on the sDDF
*/

#include <stdbool.h>
#include <stdint.h>
#include <sel4cp.h>
#include <sel4/sel4.h>
#include "serial.h"
#include "shared_ringbuffer.h"
#include "util.h"
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
uintptr_t shared_dma_rx_drv;
// Base of the uart registers
uintptr_t uart_base;

/* Pointers to shared_ringbuffers */
ring_handle_t rx_ring;
ring_handle_t tx_ring;

// Global serial_driver variable

struct serial_driver global_serial_driver = {0};

static int internal_is_tx_fifo_busy(
    meson_uart_regs_t *regs)
{
    /* check the TXFE (transmit buffer FIFO empty) flag, which is cleared
     * automatically when data is written to the TxFIFO. Even though the flag
     * is set, the actual data transmission via the UART's 32 byte FIFO buffer
     * might still be in progress.
     */
    return (0 == (regs->sr & AML_UART_TX_EMPTY));
}

/*
 * BaudRate = RefFreq / (16 * (BMR + 1)/(BIR + 1) )
 * BMR and BIR are 16 bit
 * Function taken from seL4 util_libs serial.c implementation for imx8mm
 */
static void set_baud(long bps)
{
    /* TODO: Fix buad rate setup */
    
    // meson_uart_regs_t *regs = (meson_uart_regs_t *) uart_base;

    // // Wait to clear transmit port
    // while (internal_is_tx_fifo_busy(regs)) {

    // }

    // // Caluclate baud rate
    // uint32_t val = 0;
    // val = DIV_ROUND_CLOSEST(UART_REF_CLK / 4, bps) - 1;
    // val |= AML_UART_BAUD_USE;

    // regs->reg5 = val;
}


int serial_configure(
    long bps,
    int char_size,
    enum serial_parity parity,
    int stop_bits, 
    int mode, 
    int echo)
{
    meson_uart_regs_t *regs = (meson_uart_regs_t *) uart_base;

    global_serial_driver.mode = mode;
    global_serial_driver.echo = echo;

    uint32_t cr;
    /* Character size */
    cr = regs->cr;
    if (char_size == 8) {
        cr |= AML_UART_DATA_LEN_8BIT;
    } else if (char_size == 7) {
        cr |= AML_UART_DATA_LEN_7BIT;
    } else {
        return -1;
    }
    /* Stop bits */
    if (stop_bits == 2) {
        cr |= AML_UART_STOP_BIT_2SB;
    } else if (stop_bits == 1) {
        cr |= AML_UART_STOP_BIT_1SB;
    } else {
        return -1;
    }

    /* Parity */
    if (parity == PARITY_NONE) {
        cr &= ~AML_UART_PARITY_EN;
    } else if (parity == PARITY_ODD) {
        /* ODD */
        cr |= AML_UART_PARITY_EN;
        cr |= AML_UART_PARITY_TYPE;
    } else if (parity == PARITY_EVEN) {
        /* Even */
        cr |= AML_UART_PARITY_EN;
        cr &= ~AML_UART_PARITY_TYPE;
    } else {
        return -1;
    }
    /* Apply the changes */
    regs->cr = cr;
    //sel4cp_dbg_puts("finished configuring the line, setting the baud rate\n");
    /* Now set the baud rate */
    set_baud(bps);

    // PPC to mux to register the mode
    sel4cp_mr_set(0, mode);
    sel4cp_ppcall(RX_CH, sel4cp_msginfo_new(0, 1));

    return 0;
}

int getchar()
{
    meson_uart_regs_t *regs = (meson_uart_regs_t *) uart_base;

    while (regs->sr & AML_UART_RX_EMPTY);
    return regs->rfifo;
}

// Putchar that is using the hardware FIFO buffers --> Switch to DMA later 
int putchar(int c) {

    meson_uart_regs_t *regs = (meson_uart_regs_t *) uart_base;

    while (regs->sr & AML_UART_TX_FULL);

    /* Add character to the buffer. */
    regs->wfifo = c & 0x7f;
    if (c == '\n') {
        putchar('\r');
    }

    return 0;
}

// Called from handle tx, write each character stored in the buffer to the serial port
static void
raw_tx(char *phys, unsigned int len, void *cookie)
{
    //sel4cp_dbg_puts("entering raw tx function\n");
    // This is byte by byte for now, switch to DMA use later
    for (int i = 0; i < len || phys[i] != '\0'; i++) {
        // Loop until the fifo queue is ready to transmit
        while (putchar(phys[i]) != 0);
    }
}

void handle_tx() {
    //sel4cp_dbg_puts("In the handle tx func\n");
    uintptr_t buffer = 0;
    unsigned int len = 0;
    void *cookie = 0;
    // Dequeue something from the Tx ring -> the server will have placed something in here, if its empty then nothing to do
    //sel4cp_dbg_puts("Dequeuing and printing everything currently in the ring buffer\n");
    while (!driver_dequeue(tx_ring.used_ring, &buffer, &len, &cookie)) {
        //sel4cp_dbg_puts("in the driver dequeue loop\n");
        // Buffer cointaining the bytes to write to serial
        char *phys = (char * )buffer;
        // Handle the tx
        raw_tx(phys, len, cookie);
        // Then enqueue this buffer back into the available queue, so that it can be collected and reused by the server
        enqueue_avail(&tx_ring, buffer, len, &cookie);
    }
    //sel4cp_dbg_puts("Finished handle_tx\n");
}

void handle_irq() {
    //sel4cp_dbg_puts("IN the irq func\n");
/* Here we have interrupted because a character has been inputted. We first want to get the 
    character from the hardware FIFO queue.

    Then we want to dequeue from the rx available ring, and populate it, then add to the rx used queue
    ready to be processed by the client server
    */
    int input = getchar();
    char input_char = (char) input;
    sel4cp_irq_ack(IRQ_CH);

    // Not sure if we should be printing this here or elsewhere? What is the expected behaviour?
    // putchar(input);


    if (input == -1) {
        // //sel4cp_dbg_puts(sel4cp_name);
        //sel4cp_dbg_puts(": invalid input when attempting to getchar\n");
        return;
    }

    int ret = 0;

    if (global_serial_driver.mode == RAW_MODE) {
        // Place characters straight into the buffer

        // Address that we will pass to dequeue to store the buffer address
        uintptr_t buffer = 0;
        // Integer to store the length of the buffer
        unsigned int buffer_len = 0; 

        void *cookie = 0;

        ret = dequeue_avail(&rx_ring, &buffer, &buffer_len, &cookie);

        if (ret != 0) {
            //sel4cp_dbg_puts(sel4cp_name);
            //sel4cp_dbg_puts(": unable to dequeue from the rx available ring\n");
            return;
        }

        ((char *) buffer)[0] = (char) input;

        // Now place in the rx used ring
        ret = enqueue_used(&rx_ring, buffer, 1, &cookie);
        sel4cp_notify(RX_CH);

        // Handle backspaces if echo is specified
    } else if (global_serial_driver.mode == LINE_MODE) {
        // Place in a buffer, until we reach a new line, ctrl+d/ctrl+c/enter (check what else can stop)
        sel4cp_dbg_puthex8(input_char);
        if (global_serial_driver.line_buffer == 0) {
            // We need to dequeue a buffer to use
            // Address that we will pass to dequeue to store the buffer address
            uintptr_t buffer = 0;
            // Integer to store the length of the buffer
            unsigned int buffer_len = 0; 

            void *cookie = 0;

            ret = dequeue_avail(&rx_ring, &buffer, &buffer_len, &cookie);
            sel4cp_dbg_puts("dequeuing new buffe\n");
            if (ret != 0) {
                //sel4cp_dbg_puts(sel4cp_name);
                //sel4cp_dbg_puts(": unable to dequeue from the rx available ring\n");
                return;
            }

            global_serial_driver.line_buffer = buffer;
            global_serial_driver.line_buffer_size = 0;


        } 

        // Check that the buffer is not full, and other exit conditions here
        if (global_serial_driver.line_buffer_size > BUFFER_SIZE ||
            input_char == EOT || 
            input_char == ETX || 
            input_char == LF ||
            input_char == SB ||
            input_char == CR) {
                char *char_arr = (char * ) global_serial_driver.line_buffer;
                void *cookie = 0;
                // Place the line end character into buffer
                char_arr[global_serial_driver.line_buffer_size] = input_char;
                global_serial_driver.line_buffer_size += 1;
                // Enqueue buffer back
                sel4cp_dbg_puts("This is the buffer size in the driver: ");
                sel4cp_dbg_puthex64(global_serial_driver.line_buffer_size);
                sel4cp_dbg_puts("\n");
                ret = enqueue_used(&rx_ring, global_serial_driver.line_buffer, global_serial_driver.line_buffer_size, &cookie);
                sel4cp_dbg_puts("enqueued buffer back in\n");
                // Zero out the driver states
                global_serial_driver.line_buffer = 0;
                global_serial_driver.line_buffer_size = 0;
                sel4cp_notify(RX_CH);

        } else {
            // Otherwise, add to the character array
            sel4cp_dbg_puts("adding character to arr\n");
            char *char_arr = (char * ) global_serial_driver.line_buffer;
    
            // Conduct any line editing as long as we have stuff in the buffer
            if (input_char == 0x7f && global_serial_driver.line_buffer_size > 0) {
                // Remove last character
                global_serial_driver.line_buffer_size -= 1;
                char_arr[global_serial_driver.line_buffer_size] = 0;
            } else {
                char_arr[global_serial_driver.line_buffer_size] = input;
                global_serial_driver.line_buffer_size += 1;
            }
        }
    }
    if (global_serial_driver.echo == ECHO_EN) {
        // // Special case for backspace
        if (input_char == '\b' || input_char == 0x7f) {
            // Backspace will move the cursor back, and space will erase the character
            putchar('\b');
            putchar(' ');
            putchar('\b');
        } else {
            putchar(input);
        }
    }

    // Line or Raw Mode differentiation here.

    if (ret != 0) {
        //sel4cp_dbg_puts(sel4cp_name);
        //sel4cp_dbg_puts(": unable to enqueue to the tx available ring\n");
        return;
    }

}

void init_post() {
    //sel4cp_dbg_puts(sel4cp_name);
    //sel4cp_dbg_puts(": init_post function running\n");

    // Init the shared ring buffers
    ring_init(&rx_ring, (ring_buffer_t *)rx_avail, (ring_buffer_t *)rx_used, NULL, 0);
    ring_init(&tx_ring, (ring_buffer_t *)tx_avail, (ring_buffer_t *)tx_used, NULL, 0);
    
    // Redundant right now, as a channel has not been set up for init calls
    // sel4cp_notify(INIT);
}

// Init function required by CP for every PD
void init(void) {
    //sel4cp_dbg_puts(sel4cp_name);
    //sel4cp_dbg_puts(": elf PD init function running\n");


    // Call init_post here to setup the ring buffer regions. The init_post case in the notified
    // switch statement may be redundant.
    init_post();

    meson_uart_regs_t *regs = (meson_uart_regs_t *) uart_base;

    // Software reset results in failed uart init, not too sure why
    /* Software reset */
    // regs->cr2 &= ~UART_CR2_SRST;
    // while (!(regs->cr2 & UART_CR2_SRST));

    global_serial_driver.regs = regs;
    global_serial_driver.rx_ring = rx_ring;
    global_serial_driver.tx_ring = tx_ring;

    //sel4cp_dbg_puts("Line configuration\n");

    /* Line configuration */
    int ret = serial_configure(115200, 8, PARITY_NONE, 1, LINE_MODE, ECHO_EN);

    if (ret != 0) {
        //sel4cp_dbg_puts("Error occured during line configuration\n");
    }

    //sel4cp_dbg_puts("Configured serial, enabling uart\n");

    // /* Enable the UART */
    uint32_t val;
    val = regs->cr;
    val |= AML_UART_CLEAR_ERR;
    regs->cr = val;
    val &= ~AML_UART_CLEAR_ERR;
    regs->cr = val;
    val |= (AML_UART_RX_EN | AML_UART_TX_EN);
    regs->cr = val;
    val |= (AML_UART_RX_INT_EN);
    regs->cr = val;
    val = (AML_UART_RECV_IRQ(1));
    regs->irqc = val;

    //sel4cp_dbg_puts("Enabled the uart, init the ring buffers\n");

}

// Entry point that is invoked on a serial interrupt, or notifications from the server using the TX and RX channels
void notified(sel4cp_channel ch) {
    //sel4cp_dbg_puts(sel4cp_name);
    //sel4cp_dbg_puts(": elf PD notified function running\n");

    switch(ch) {
        case IRQ_CH:
            handle_irq();
            return;
        case INIT:
            init_post();
            break;
        case TX_CH:
            //sel4cp_dbg_puts("Notified to print something\n");
            handle_tx();
            break;
        default:
            //sel4cp_dbg_puts("eth driver: received notification on unexpected channel\n");
            break;
    }
}