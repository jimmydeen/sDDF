/*
  Implements the foreign function interface (FFI) used in the CakeML basis
  library, as a thin wrapper around the relevant system calls.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <stdint.h>
#include <sel4cp.h>
#include <string.h>
#include <sel4/sel4.h>
#include "serial.h"
#include "shared_ringbuffer.h"
#include "serial_driver_data.h"

#ifdef EVAL
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <signal.h>
#endif

/* This flag is on by default. It catches CakeML's out-of-memory exit codes
 * and prints a helpful message to stderr.
 * Note that this is not specified by the basis library.
 * */
#define STDERR_MEM_EXHAUST
#define CONFIG_ENABLE_BENCHMARKS
/* clFFI (command line) */


/* Shared Memory regions. These all have to be here to keep compiler happy */
// Ring handle components
uintptr_t rx_avail;
uintptr_t rx_used;
uintptr_t tx_avail;
uintptr_t tx_used;
uintptr_t shared_dma_vaddr;
// Base of the uart registers
uintptr_t uart_base;

char current_channel;

/* Pointers to shared_ringbuffers */
ring_handle_t rx_ring;
ring_handle_t tx_ring;

// Attempt to save the address that notified needs to return to
void *notified_return;

struct serial_driver global_serial_driver_data = {0};

static char cml_memory[1024*1024*2];

unsigned int argc;
char **argv;

/* exported in cake.S */
extern void cml_main(void);
extern void *cml_heap;
extern void *cml_stack;
extern void *cml_stackend;

extern char cake_text_begin;
extern char cake_codebuffer_begin;
extern char cake_codebuffer_end;

#ifdef EVAL

/* Signal handler for SIGINT */

/* This is set to 1 when the runtime traps a SIGINT */
volatile sig_atomic_t caught_sigint = 0;

void do_sigint(int sig_num)
{
    signal(SIGINT, do_sigint);
    caught_sigint = 1;
}

void ffipoll_sigint (unsigned char *c, long clen, unsigned char *a, long alen)
{
    if (alen < 1) {
        return;
    }
    a[0] = (unsigned char) caught_sigint;
    caught_sigint = 0;
}

void ffikernel_ffi (unsigned char *c, long clen, unsigned char *a, long alen) {
    for (long i = 0; i < clen; i++) {
        putc(c[i], stdout);
    }
}

#else

void ffipoll_sigint (unsigned char *c, long clen, unsigned char *a, long alen) { }

void ffikernel_ffi (unsigned char *c, long clen, unsigned char *a, long alen) { }

#endif

void ffiget_arg_count (unsigned char *c, long clen, unsigned char *a, long alen) {
  a[0] = (char) argc;
  a[1] = (char) (argc / 256);
}

void ffiget_arg_length (unsigned char *c, long clen, unsigned char *a, long alen) {
  int i = a[0] + (a[1] * 256);
  int k = 0;
  while (argv[i][k] != 0) { k++; }
  a[0] = (char) k;
  a[1] = (char) (k / 256);
}

void ffiget_arg (unsigned char *c, long clen, unsigned char *a, long alen) {
  int i = a[0] + (a[1] * 256);
  int k = 0;
  while (argv[i][k] != 0) {
    a[k] = argv[i][k];
    k++;
  }
}

void int_to_byte2(int i, unsigned char *b){
    /* i is encoded on 2 bytes */
    b[0] = (i >> 8) & 0xFF;
    b[1] = i & 0xFF;
}

int byte2_to_int(unsigned char *b){
    return ((b[0] << 8) | b[1]);
}

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

void cml_exit(int arg) {
    sel4cp_dbg_puts("CALLING CML_EXIT\n");

    if (current_channel == 1) {
        // irq ack
        sel4cp_irq_ack(current_channel);
    }

    current_channel = 0;
    void (*foo)(void) = (void (*)())notified_return;
    foo();
}

/* Need to come up with a replacement for this clear cache function. Might be worth testing just flushing the entire l1 cache, but might cause issues with returning to this file*/
void cml_clear() {
//   __builtin___clear_cache(&cake_codebuffer_begin, &cake_codebuffer_end);
    sel4cp_dbg_puts("Trying to clear cache\n");
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

/* Debugging FFI Calls*/

void ffitest(unsigned char *c, long clen, unsigned char *a, long alen) {
    // sel4cp_dbg_puts("We have made a successful ffi call from the pancake program\n");
}

void ffiloop_exit(unsigned char *c, long clen, unsigned char *a, long alen) {
    // sel4cp_dbg_puts("We have hit a break statement\n");
}

void ffireturn_call(unsigned char *c, long clen, unsigned char *a, long alen) {
    // sel4cp_dbg_puts("We have hit a return statement\n");
}

void ffiloop_continue(unsigned char *c, long clen, unsigned char *a, long alen) {
    // sel4cp_dbg_puts("We are continuing the loop\n");
}

void ffiputchar_loop(unsigned char *c, long clen, unsigned char *a, long alen) {
    // sel4cp_dbg_puts("We are now looping to attempt to call putchar if fifo is ready\n");
}

void ffireached_end(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("We have reached the end of the program\n");
}

void ffirawtx_loop(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("Entering raw tx loop\n");
}

void ffifinished_pnk(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("We have finished our pnk program, returning to c code\n");
}

void ffifinished_enqueue(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("Finsihed the serial enqueue function and back in pnk code\n");
}

void ffihandle_irq_fun(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("We are in the handle irq function\n");
}

void ffiget_channel(unsigned char *c, long clen, unsigned char *a, long alen) {
    if (alen != 1) {
        return;
    }

    a[0] = current_channel;
}

void ffiprint_int(unsigned char *c, long clen, unsigned char *a, long alen) {

    sel4cp_dbg_puts("The address of c is -- (");

    sel4cp_dbg_puts(c);

    sel4cp_dbg_puts("\n");

    char arg = c[0];

    sel4cp_dbg_puts("We received the following int --- (");
    sel4cp_dbg_puts(arg);
    sel4cp_dbg_puts(")\n");
}

void ffiinternal_is_tx_fifo_busy(unsigned char *c, long clen, unsigned char *a, long alen)
{

    // sel4cp_dbg_puts("Checking if the fifo buffer is busy\n");
    imx_uart_regs_t *regs = (imx_uart_regs_t *) uart_base;

    /* check the TXFE (transmit buffer FIFO empty) flag, which is cleared
     * automatically when data is written to the TxFIFO. Even though the flag
     * is set, the actual data transmission via the UART's 32 byte FIFO buffer
     * might still be in progress.
     */

    int ret = (0 == (regs->sr2 & UART_SR2_TXFIFO_EMPTY));
    // sel4cp_dbg_puts("Attempting to access a buffer\n");
    if (ret) {
        // sel4cp_dbg_puts("FIFO was busy\n");
        a[0] = 1;
    } else {
        // sel4cp_dbg_puts("FIFO was not busy\n");
        a[0] = 0;
    }
    // sel4cp_dbg_puts("Returning from tx fifo busy function\n");
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

    return 0;
}

void ffigetchar(unsigned char *c, long clen, unsigned char *a, long alen)
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
void ffiputchar_regs(unsigned char *c, long clen, unsigned char *a, long alen) {
    // sel4cp_dbg_puts("Entered putchar in serial_driver_data\n");
    // sel4cp_dbg_puts("\t");
    // sel4cp_dbg_puts(c[0]);
    // sel4cp_dbg_puts("\n");
    imx_uart_regs_t *regs = (imx_uart_regs_t *) uart_base;

    regs->txd = c[0];
    // sel4cp_dbg_puts("Finished putchar regs\n");
}

void ffiincrement_num_chars(unsigned char *c, long clen, unsigned char *a, long alen) {
    global_serial_driver_data.num_to_get_chars += 1;
}

void init_post(unsigned char *c, long clen, unsigned char *a, long alen) {
    // Setup the ring buffer mechanisms here as well as init the global serial driver data


    sel4cp_dbg_puts("Init the ring buffers\n");

    imx_uart_regs_t *regs = (imx_uart_regs_t *) uart_base;

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
}

void ffiserial_dequeue_avail(unsigned char *c, long clen, unsigned char *a, long alen) {
    // Dequeue from shared mem avail avail buffer
    sel4cp_dbg_puts("In serial dequeue avail function\n");
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
        sel4cp_dbg_puts("No requests for get char outstanding\n");
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
        a[0] = dequeue_avail(&rx_ring, &buffer, &buffer_len, &cookie);
    } else {
        a[0] = dequeue_avail(&tx_ring, &buffer, &buffer_len, &cookie);
    }
    if (a[0] != 0) {
        return;
    } 

    int_to_byte8(buffer, &a[1]);

    global_serial_driver_data.num_to_get_chars--;
    sel4cp_dbg_puts("Finished serial dequeue avail function\n");
}

void ffiserial_enqueue_used(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("Starting serial enqueue used function\n");
    if (clen <= 0) {
        sel4cp_dbg_puts("There are no arguments supplied when args are expected");
        return;
    }

    bool rx_tx = c[0];
    int input = c[1];

    uintptr_t buffer = byte8_to_int(&a[1]);

    ((char *) buffer)[0] = (char) input;

    unsigned int buffer_len = 0; 
    
    void *cookie = 0;

    if (rx_tx == 0) {
        a[0] =  enqueue_used(&rx_ring, buffer, 1, &cookie);
    } else {
        a[0] =  enqueue_used(&tx_ring, buffer, 1, &cookie);
    }

}

void ffiserial_driver_dequeue_used(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("In the serial driver dequeue used function\n");
    if (clen != 1) {
        sel4cp_dbg_puts("There are no arguments supplied when args are expected\n");
        return;
    }

    if (alen != BUFFER_SIZE) {
        // We always need the a array to be 1024 bytes long, the same length as the buffers 
        // in the ring buffers. 
        sel4cp_dbg_puts("Argument alen not of correct size\n");
        return;
    }
    bool rx_tx = c[0];

    void *cookie = 0;
    sel4cp_dbg_puts("Attempting driver dequeue\n");
    // Address that we will pass to dequeue to store the buffer address
    uintptr_t buffer = 0;
    // Integer to store the length of the buffer
    unsigned int buffer_len = 0; 
    int ret = 0;
    if (rx_tx == 0) {
        ret = driver_dequeue(rx_ring.used_ring, &buffer, &buffer_len, &cookie);
    } else {
        ret = driver_dequeue(tx_ring.used_ring, &buffer, &buffer_len, &cookie);
    }

    if (ret != 0) {
        // alen = 0;
        sel4cp_dbg_puts("Driver dequeue was unsuccessful\n");
        c[0] = 0;
        return;
    } else {
        sel4cp_dbg_puts("Driver dequeue successful, attempting memcpy\n");
        if (buffer_len >= BUFFER_SIZE) {
            sel4cp_dbg_puts("Buffer len too large\n");
            return;
        }
        int mem_ret = memcpy(a, (char *) buffer, buffer_len);
        // clen = buffer;
        // alen = buffer_len;

        c[0] = 1;

        // Copy over the length of the buffer that is to be printed
        int_to_byte8(buffer_len, &c[1]);
    }
    sel4cp_dbg_puts("Finished buffer dequeue\n");
}

void ffiserial_enqueue_avail(unsigned char *c, long clen, unsigned char *a, long alen) {
    sel4cp_dbg_puts("We are in the ffi serial enqueue available function\n");
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

    if (a[0] != 0) {
        sel4cp_dbg_puts("Error in serial enqueue used\n");
    }
}

void ffidequeue_enqueue_avail(unsigned char *c, long clen, unsigned char *a, long alen) {
    // Dequeue from shared mem avail avail buffer
    sel4cp_dbg_puts("In serial dequeue avail function\n");
    
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
        sel4cp_dbg_puts("No requests for get char outstanding\n");
        a[0] = -1;
        return;
    }

    void *cookie = 0;

    // Address that we will pass to dequeue to store the buffer address
    uintptr_t buffer = 0;
    // Integer to store the length of the buffer
    unsigned int buffer_len = 0; 

    a[0] = dequeue_avail(&rx_ring, &buffer, &buffer_len, &cookie);

    if (a[0] != 0) {
        return;
    }


    // For now pass buffer addresses through the alen value  
    // alen = buffer;
    global_serial_driver_data.num_to_get_chars--;
    sel4cp_dbg_puts("Finished serial dequeue avail function\n");

    int input = c[1];

    ((char *) buffer)[0] = (char) input;

    a[0] =  enqueue_used(&rx_ring, buffer, 1, &cookie);
    
}

void init_pancake_mem() {
    unsigned long sz = 1024*1024; // 1 MB unit\n",
    unsigned long cml_heap_sz = sz;    // Default: 1 MB heap\n", (* TODO: parameterise *)
    unsigned long cml_stack_sz = sz;   // Default: 1 MB stack\n", (* TODO: parameterise *)
    cml_heap = cml_memory;
    cml_stack = cml_heap + cml_heap_sz;
    cml_stackend = cml_stack + cml_stack_sz;
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
    init_pancake_mem();
    sel4cp_dbg_puts("Finished initing the pancake mem and the device driver\n");
}

// Entry point that is invoked on a serial interrupt, or notifications from the server using the TX and RX channels
void notified(sel4cp_channel ch) {
    notified_return = __builtin_return_address(0);

    sel4cp_dbg_puts(sel4cp_name);
    sel4cp_dbg_puts(": elf PD notified function running\n");

    sel4cp_dbg_puts("Attempting to jump to pancake main\n");


    // Here, we want to call to cakeml main - this will be our entry point into the pancake program.
    if (ch == 8 || ch == 1 || ch == 10) {
        current_channel = (char) ch;
        // char *heap_arr = (char *) cml_heap;

        // sel4cp_dbg_puts("This is the address of heap_arr -- ");
        // sel4cp_dbg_puts();
        // sel4cp_dbg_puts("\n");
        // heap_arr[1] = 8;

        cml_main();

    }

    sel4cp_dbg_puts("After main call, I'm not sure if we should ever get here\n");
}