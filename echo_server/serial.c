/*
* Sample serial driver for imx8mm based on the sDDF
*/

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <sel4cp.h>
#include <sel4/sel4.h>
#include "serial.h"
#include "shared_ringbuffer.h"
#include "serial_driver_data.h"

// Defines to manage interrupts and notifications
#define IRQ_CH 1
#define TX_CH  8
#define RX_CH  10
#define INIT   4

// Global serial_driver variable

struct serial_driver global_serial_driver = {0};

int putchar(int c) {
    sel4cp_dbg_puts("Entering putchar in serial.c\n");

    unsigned char c_arr[1];
    long clen = 1;
    unsigned char a_arr[1];
    long alen = 0;
    
    unsigned char temp_c[1];
    long temp_clen = 0;
    unsigned char temp_a[1];
    temp_a[0] = 0;
    long temp_alen = 0;

    sel4cp_dbg_puts("Checking if tx fifo is busy\n");
    internal_is_tx_fifo_busy(temp_c, temp_clen, temp_a, temp_alen);
    int ret_tx_fifo = temp_a[0];
    if (ret_tx_fifo) {
        // A transmit is probably in progress, we will have to wait
        return -1;
    } else {
        sel4cp_dbg_puts("Placing CR first\n");
        // Try and keep as much of the logic here as possible
        if (c == '\n') {
            // For now, by default we will have Auto-send CR(Carriage Return) enabled
            /* write CR first */
            c_arr[0] = '\r';
            sel4cp_dbg_puts("Attempting putchar\n");
            putchar_regs(c_arr, clen, a_arr, alen);
            
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
        while (1) {
            /* busy loop */
            // Kinda sketchy way to do this
            internal_is_tx_fifo_busy(temp_c, temp_clen, temp_a, temp_alen);
            if (!temp_a[0]) {
                break;
            }
        }


        c_arr[0] = c;

        putchar_regs(c_arr, clen, a_arr, alen);
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
    unsigned int len = 0;
    void *cookie = 0;
    // Dequeue something from the Tx ring -> the server will have placed something in here, if its empty then nothing to do
    sel4cp_dbg_puts("Dequeuing and printing everything currently in the ring buffer\n");
    
    unsigned char c_arr[1];
    c_arr[0] = 1;
    long clen = 1;
    // For now, can only accomodate for inputs of up to 2048 characters. The same size as the buffers
    unsigned char a_arr[2048];
    // a_arr[0] = 1;
    long alen = 2048;

    while (1) {
        serial_driver_dequeue_used(c_arr, clen, a_arr, alen);
        sel4cp_dbg_puts("in the driver dequeue loop\n");

        // In this case, the c array is used to carry the return value as the a array will be full with the characeters
        // from the buffer. 
        int ret = (int) c_arr[0];

        // Check if there was an error
        if (ret == 0) {
            break;
        }

        // The length of the buffer should start from index 1 in the c array
        int len = byte8_to_int(&c_arr[1]);

        raw_tx(a_arr, len, cookie);
        
        // Then enqueue this buffer back into the available queue, so that it can be collected and reused by the server
        c_arr[0] = 1;
        // alen = clen;
        clen = 1;
        serial_enqueue_avail(c_arr, clen, a_arr, alen);

        if (a_arr[0] != 0) {
            sel4cp_dbg_puts(sel4cp_name);
            sel4cp_dbg_puts(": failed to enqueue into the tx ring buffers\n");
            return;
        } 
    }
    sel4cp_dbg_puts("Finished handle_tx\n");
}

// Increment the number of chars that the server has requested us to get.
void handle_rx() {
    unsigned char c[1];
    long clen = 0;

    unsigned char a[1];
    long alen = 0;

    increment_num_chars(c, clen, a, alen);
    
}


void handle_irq() {
    /* Here we have interrupted because a character has been inputted. We first want to get the 
    character from the hardware FIFO queue.

    Then we want to dequeue from the rx available ring, and populate it, then add to the rx used queue
    ready to be processed by the client server
    */

    sel4cp_dbg_puts("Entering handle irq function\n");

    unsigned char getchar_c[1];
    long getchar_clen = 0;
    unsigned char getchar_a[1];
    long getchar_alen = 0;

    getchar(getchar_c, getchar_clen, getchar_a, getchar_alen);

    // int input = (getchar_a[0] >> 24) & 0xff;
    // input |= (getchar_a[1] >> 16) & 0xff;
    // input |= (getchar_a[2] >> 8) & 0xff;
    // input |= (getchar_a[3]) & 0xff;

    char input = getchar_a[0];

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

    // The break condition will now happen after the serial_dequeue_avail call
    while (1) {
        sel4cp_dbg_puts("In loop\n");
        // Address that we will pass to dequeue to store the buffer address
        // Integer to store the length of the buffer
        // unsigned int buffer_len = 0; 

        // Arguments to supply to the function
        unsigned char c[1];

        // Buffer Address
        // uintptr_t buffer_addr = &buffer;
        // c[0]= (buffer_addr >> 24) & 0xff;
        // c[1]= (buffer_addr >> 16) & 0xff;
        // c[2]= (buffer_addr >> 8) & 0xff;
        // c[3]= buffer_addr & 0xff;        
        // // Buffer len address
        // uintptr_t buffer_len_addr = &buffer_len;
        // c[4]= (buffer_len_addr >> 24) & 0xff;
        // c[5]= (buffer_len_addr >> 16) & 0xff;
        // c[6]= (buffer_len_addr >> 8) & 0xff;
        // c[7]= buffer_len_addr & 0xff; 
        // // Rx tx boolean
        // c[8] = 0;
        // long clen = 9;
        unsigned char a[1];
        a[0] = 0;
        long alen = 0;

        c[0] = 0;
        int clen = 1;

        serial_dequeue_avail(c, clen, a, alen);

        int ret = a[0];
        // Add in error checking for every function call

        // uintptr_t buffer = alen;
                
        // The ret value will only ever be -1 if there are no more get char requests to service
        // per the global variable stored in serial_driver_data
        if (ret == -1) {
            sel4cp_dbg_puts(sel4cp_name);
            sel4cp_dbg_puts(": no more getchar requests to service\n");
            break;
        } else if (ret != 0) {
            // Here something else has gone wrong with the dequeue, return from the function
            sel4cp_dbg_puts(sel4cp_name);
            sel4cp_dbg_puts(": unable to dequeue from the rx available ring\n");
            return;
        }

        // ((char *) buffer)[0] = (char) input;

        unsigned char enqueue_c_arr[2];
        enqueue_c_arr[0] = 0;
        enqueue_c_arr[1] = input;
        long enqueue_clen = 2;

        unsigned char enqueue_a_arr[1];
        // a[0] = -1;
        long enqueue_alen = 1;

        // Now place in the rx used ring
        serial_enqueue_used(enqueue_c_arr, enqueue_clen, enqueue_a_arr, enqueue_alen);

        ret = a[0];

        if (ret != 0) {
            sel4cp_dbg_puts(sel4cp_name);
            sel4cp_dbg_puts(": unable to enqueue to the tx available ring\n");
            return;
        }

        // We have serviced one getchar request, we can now decrement the count
        // This is now done in the sreial_driver_data file, probably not best in case of errors
        // in serial enqueue used, but for now we will assume that these function calls work
        // global_serial_driver.num_to_get_chars--;
    }

    sel4cp_dbg_puts("Finished handling the irq\n");
}

// Entry point that is invoked on a serial interrupt, or notifications from the server using the TX and RX channels
// This will be the main function in the pancake code. Not sure if there will be concurrency issues here, hopefully not -- revise mental model of driver. 
void handle_notified(int ch) {
    sel4cp_dbg_puts(sel4cp_name);
    sel4cp_dbg_puts(": elf PD notified function running\n");

    switch(ch) {
        case IRQ_CH:
            handle_irq();
            // Might have some issues with the irq ack here, potentially may have to move it to the ffi notified function
            sel4cp_irq_ack(ch);
            return;
        case INIT:
            // init_post();

            // For now we don't really need to do anything in here
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