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

/* FUNCTIONS COPIED OVER FROM BASIS_FFI.C */

void int_to_byte8(int i, unsigned char *b){
    /* i is encoded on 8 bytes */
    /* i is cast to long long to ensure having 64 bits */
    /* assumes CHAR_BIT = 8. use static assertion checks? */
    b[0] = ((long long) i >> 56) & 0xFF;
    b[1] = ((long long) i >> 48) & 0xFF;
    b[2] = ((long long) i >> 40) & 0xFF;
    b[3] = ((long long) i >> 32) & 0xFF;
    b[4] = ((long long) i >> 24) & 0xFF;
    b[5] = ((long long) i >> 16) & 0xFF;
    b[6] = ((long long) i >> 8) & 0xFF;
    b[7] =  (long long) i & 0xFF;
}

int byte8_to_int(unsigned char *b){
    return (((long long) b[0] << 56) | ((long long) b[1] << 48) |
             ((long long) b[2] << 40) | ((long long) b[3] << 32) |
             (b[4] << 24) | (b[5] << 16) | (b[6] << 8) | b[7]);
}



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

void increment_num_chars(unsigned char *c, long clen, unsigned char *a, long alen) {
    global_serial_driver_data.num_to_get_chars += 1;
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
        a[0] = 1;
        return;
    }

    // From our serial driver, this function is only ever called in the handle irq function
    // to attempt to service all get char requests. Check here how many get char requests we 
    // have that are outstanding. If none are outstanding then return -1 in a array, 
    // otherwise continue with the dequeue.

    if (global_serial_driver_data.num_to_get_chars <= 0) {
        // We have no more get char requests to service. 
        a[0] = 1;
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
    if (a[0] != 0) {
        return;
    }

    // uintptr_t buffer_addr = &buffer;
    // a[0]= (buffer >> 24) & 0xff;
    // a[1]= (buffer >> 16) & 0xff;
    // a[2]= (buffer >> 8) & 0xff;
    // a[3]= buffer & 0xff;      

    // For now pass buffer addresses through the alen value  
    // alen = buffer;
    global_serial_driver_data.num_to_get_chars--;
}

void serial_enqueue_used(unsigned char *c, long clen, unsigned char *a, long alen) {
    
    if (clen != 1) {
        sel4cp_dbg_puts("There are no arguments supplied when args are expected");
        return;
    }

    bool rx_tx = c[0];
    int input = c[1];

    uintptr_t buffer = 0;

    ((char *) buffer)[0] = (char) input;

    // Integer to store the length of the buffer
    unsigned int buffer_len = 0; 
    
    void *cookie = 0;

    if (rx_tx == 0) {
        a[0] =  enqueue_used(&rx_ring, &buffer, &buffer_len, cookie);
    } else {
        a[0] =  enqueue_used(&tx_ring, &buffer, &buffer_len, cookie);
    }

}

void serial_driver_dequeue_used(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (clen != 1) {
        sel4cp_dbg_puts("There are no arguments supplied when args are expected\n");
        return;
    }

    if (alen != 2048) {
        // We always need the a array to be 2048 bytes long, the same length as the buffers 
        // in the ring buffers. 
        sel4cp_dbg_puts("Argument alen not of correct size\n");
        return;
    }
    bool rx_tx = c[0];

    void *cookie = 0;

    // Address that we will pass to dequeue to store the buffer address
    uintptr_t buffer = 0;
    // Integer to store the length of the buffer
    unsigned int buffer_len = 0; 
    int ret = 0;
    if (rx_tx == 0) {
        ret = driver_dequeue(rx_ring.used_ring, &buffer, &buffer_len, cookie);
    } else {
        ret = driver_dequeue(tx_ring.used_ring, &buffer, &buffer_len, cookie);
    }

    if (ret != 0) {
        // alen = 0;
        c[0] = 0;
        return;
    } else {
        memcpy(buffer, a);
        // clen = buffer;
        // alen = buffer_len;
        c[0] = 1;

        // Copy over the length of the buffer that is to be printed
        int_to_byte8(buffer_len, &c[1]);
    }
}

void serial_enqueue_avail(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (clen != 1) {
        sel4cp_dbg_puts("There are no arguments supplied when args are expected");
        return;
    }

    bool rx_tx = c[0];

    void *cookie = 0;

    // Address that we will pass to dequeue to store the buffer address
    uintptr_t buffer = 0;
    // alen = 0;

    // Integer to store the length of the buffer
    unsigned int buffer_len = 0; 

    if (rx_tx == 0) {
        a[0] = enqueue_avail(&rx_ring, &buffer, &buffer_len, cookie);
    } else {
        a[0] = enqueue_avail(&tx_ring, &buffer, &buffer_len, cookie);
    }
}

/*
Placing these functions in here for now. These are the entry points required by the core platform, however,
we can only have 1 entry point in our pancake program. So we will have to have these entry points in our c code.
*/

// Init function required by CP for every PD
void init(void) {
    sel4cp_dbg_puts(sel4cp_name);
    sel4cp_dbg_puts(": elf PD init function running\n");

    // Call init_post here to setup the ring buffer regions. The init_post case in the notified
    // switch statement may be redundant. Init post is now in the serial_driver_data file

    unsigned char c[1];
    long clen = 1;
    unsigned char a[1];
    long alen = 1;

    init_post(c, clen, a, alen);
}

// Entry point that is invoked on a serial interrupt, or notifications from the server using the TX and RX channels
void notified(sel4cp_channel ch) {
    sel4cp_dbg_puts(sel4cp_name);
    sel4cp_dbg_puts(": elf PD notified function running\n");

    handle_notified(ch);
}