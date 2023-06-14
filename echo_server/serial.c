/*
* Sample serial driver for imx8mm based on the sDDF
*/

#include <stdbool.h>
#include <stdint.h>
#include <sel4cp.h>
#include <sel4/sel4.h>
#include "serial.h"
#include "shared_ringbuffer.h"
#include "imx_dma.h"
#include "sdma.h"
#include <string.h>

#define BIT(nr) (1UL << (nr))

// Defines to manage interrupts and notifications
#define IRQ_CH 1
#define TX_CH  8
#define RX_CH  10
#define INIT   4

/* DMA Variables */
/* Array to map SDMA instance number to base pointer. */
static uintptr_t *const s_sdmaBases[] = SDMAARM_BASE_PTRS;

/*! @brief channel 0 Channel control blcok */
AT_NONCACHEABLE_SECTION_ALIGN(
    static sdma_channel_control_t s_SDMACCB[FSL_FEATURE_SOC_SDMA_COUNT][FSL_FEATURE_SDMA_MODULE_CHANNEL], 4);

/*! @brief channel 0 buffer descriptor */
AT_NONCACHEABLE_SECTION_ALIGN(
    static sdma_buffer_descriptor_t s_SDMABD[FSL_FEATURE_SOC_SDMA_COUNT][FSL_FEATURE_SDMA_MODULE_CHANNEL], 4);

/*! @brief Array to map SDMA instance number to IRQ number. */
static const IRQn_Type s_sdmaIRQNumber[FSL_FEATURE_SOC_SDMA_COUNT] = SDMAARM_IRQS;

AT_NONCACHEABLE_SECTION_ALIGN(sdma_context_data_t context_Tx, 4);
AT_NONCACHEABLE_SECTION_ALIGN(sdma_context_data_t context_Rx, 4);

AT_NONCACHEABLE_SECTION_ALIGN(sdma_handle_t g_uartTxSdmaHandle, 4);
AT_NONCACHEABLE_SECTION_ALIGN(sdma_handle_t g_uartRxSdmaHandle, 4);

/*! @brief Pointers to transfer handle for each SDMA channel. */
static sdma_handle_t *s_SDMAHandle[FSL_FEATURE_SOC_SDMA_COUNT][FSL_FEATURE_SDMA_MODULE_CHANNEL];

/* Memory regions. These all have to be here to keep compiler happy */

// Ring handle components
uintptr_t rx_avail;
uintptr_t rx_used;
uintptr_t tx_avail;
uintptr_t tx_used;
uintptr_t shared_dma_vaddr;
uintptr_t shared_dma_paddr;

// Base of the uart and dma controller registers 
uintptr_t uart_base;
uintptr_t dma_controller;
uintptr_t core_controller;

/* Pointers to shared_ringbuffers */
ring_handle_t rx_ring;
ring_handle_t tx_ring;

// Global serial_driver variable

struct serial_driver global_serial_driver = {0};

/* Functions to enable and disable DMA */
void serial_enable_dma() {
    imx_uart_regs_t *regs = (imx_uart_regs_t *) uart_base;

    // regs->cr1 |= UCR1_RXDMAEN | UCR1_TXDMAEN | UCR1_ATDMAEN;
    // Only enabling dma for tx for now
    regs->cr1 |= UCR1_TXDMAEN;

    return;
}

void serial_disable_dma() {
    imx_uart_regs_t *regs = (imx_uart_regs_t *) uart_base;

    // regs->cr1 &= ~(UCR1_RXDMAEN | UCR1_TXDMAEN | UCR1_ATDMAEN);
    // Only enabling dma for tx for now
    regs->cr1 &= ~(UCR1_TXDMAEN);

    return;
}

void enableIRQ(IRQn_Type IRQn)
{
    NVIC_Type *NVIC = (NVIC_Type *) core_controller + NVIC_OFFSET;

  if ((int32_t)(IRQn) >= 0)
  {
    __COMPILER_BARRIER();
    NVIC->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __COMPILER_BARRIER();
  }
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
    return 0;
}

int getchar()
{
    imx_uart_regs_t *regs = (imx_uart_regs_t *) uart_base;
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

    imx_uart_regs_t *regs = (imx_uart_regs_t *) uart_base;

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

    regs->txd = c;

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
    while (!driver_dequeue(tx_ring.used_ring, &buffer, &len, &cookie)) {
        sel4cp_dbg_puts("in the driver dequeue loop\n");
        // Buffer cointaining the bytes to write to serial
        char *phys = (char * )buffer;
        // Handle the tx
        raw_tx(phys, len, cookie);
        // Then enqueue this buffer back into the available queue, so that it can be collected and reused by the server
        enqueue_avail(&tx_ring, buffer, len, &cookie);
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

        int ret = dequeue_avail(&rx_ring, &buffer, &buffer_len, &cookie);

        if (ret != 0) {
            sel4cp_dbg_puts(sel4cp_name);
            sel4cp_dbg_puts(": unable to dequeue from the rx available ring\n");
            return;
        }

        ((char *) buffer)[0] = (char) input;

        // Now place in the rx used ring
        ret = enqueue_used(&rx_ring, buffer, 1, &cookie);

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

void init_post() {
    sel4cp_dbg_puts(sel4cp_name);
    sel4cp_dbg_puts(": init_post function running\n");

    // Init the shared ring buffers
    ring_init(&rx_ring, (ring_buffer_t *)rx_avail, (ring_buffer_t *)rx_used, NULL, 0);
    ring_init(&tx_ring, (ring_buffer_t *)tx_avail, (ring_buffer_t *)tx_used, NULL, 0);
    
    // Redundant right now, as a channel has not been set up for init calls
    // sel4cp_notify(INIT);
}

void SDMA_GetDefaultConfig(sdma_config_t *config)
{
    // assert(config != NULL);

    config->enableRealTimeDebugPin   = false;
    config->isSoftwareResetClearLock = true;
    config->ratio                    = kSDMA_HalfARMClockFreq;
}

static uint32_t SDMA_GetInstance(SDMAARM_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ARRAY_SIZE(s_sdmaBases); instance++)
    {
        if (s_sdmaBases[instance] == base)
        {
            break;
        }
    }

    if (instance < ARRAY_SIZE(s_sdmaBases)) {
        sel4cp_dbg_puts("Instance is less than array size\n");
    }

    return instance;
}

void SDMA_ResetModule(SDMAARM_Type *base)
{
    if (base == NULL) {
        sel4cp_dbg_puts("We have recieved a null base address for the sdma controller\n");
        return;
    }

    uint32_t i = 0, status;

    base->MC0PTR = 0;
    status       = base->INTR;
    SDMA_ClearChannelInterruptStatus(base, status);
    status = base->STOP_STAT;
    SDMA_ClearChannelStopStatus(base, status);
    base->EVTOVR  = 0;
    base->DSPOVR  = 0xFFFFFFFFU;
    base->HOSTOVR = 0;
    status        = base->EVTPEND;
    SDMA_ClearChannelPendStatus(base, status);
    base->INTRMASK = 0;

    /* Disable all events */
    for (i = 0; i < (uint32_t)FSL_FEATURE_SDMA_EVENT_NUM; i++)
    {
        SDMA_SetSourceChannel(base, i, 0);
    }

    /* Clear all channel priority */
    for (i = 0; i < (uint32_t)FSL_FEATURE_SDMA_MODULE_CHANNEL; i++)
    {
        SDMA_SetChannelPriority(base, i, 0);
    }
}


void init_dma(SDMAARM_Type *base, sdma_config_t *config) {
    // SDMA_init from imx8mm sdk

    uint32_t tmpreg;
    // See comment in SDMA_GetInstance Function. NEED TO UNDERSTAND WHAT THE INSTANCE INDEX IS

    /* This is an instance within a Channel Control Block. */
    uint32_t instance = SDMA_GetInstance(base);

    // Clear the channel Channel Control Block (CCB) for channel 0
    (void)memset(&s_SDMACCB[instance][0], 0,
                sizeof(sdma_channel_control_t) * (uint32_t)FSL_FEATURE_SDMA_MODULE_CHANNEL);

    // Reset all sDMA registers
    SDMA_ResetModule(base);

    // Init the CCB for channel 0

    /* Channel 0 is the boot channel. It holds a pointer to the array of buffer descriptors*/

    (void)memset(&s_SDMACCB[instance][0], 0,
                sizeof(sdma_channel_control_t));

    // Setting channel 0's CCB to have a pointer to the array of buffer descriptors (BD)
    s_SDMACCB[instance][0].currentBDAddr = (uint32_t)(&s_SDMABD[instance][0]);
    s_SDMACCB[instance][0].baseBDAddr    = (uint32_t)(&s_SDMABD[instance][0]);

    // Set the priority of channel 0
    SDMA_SetChannelPriority(base, 0, 7U);

    /* Set channel 0 ownership */
    base->HOSTOVR = 0U;
    base->EVTOVR  = 1U;

    /* Configure SDMA peripheral according to the configuration structure. */
    tmpreg = base->CONFIG;
    tmpreg &= ~(SDMAARM_CONFIG_ACR_MASK | SDMAARM_CONFIG_RTDOBS_MASK | SDMAARM_CONFIG_CSM_MASK);
    /* Channel 0 shall use static context switch method */
    tmpreg |= (SDMAARM_CONFIG_ACR(config->ratio) | SDMAARM_CONFIG_RTDOBS(config->enableRealTimeDebugPin) |
               SDMAARM_CONFIG_CSM(0U));
    base->CONFIG = tmpreg;

    tmpreg = base->SDMA_LOCK;
    tmpreg &= ~SDMAARM_SDMA_LOCK_SRESET_LOCK_CLR_MASK;
    tmpreg |= SDMAARM_SDMA_LOCK_SRESET_LOCK_CLR(config->isSoftwareResetClearLock);
    base->SDMA_LOCK = tmpreg;

    /* Set the context size to 32 bytes */
    base->CHN0ADDR = 0x4050U;

    base->MC0PTR                         = (uint32_t)(&s_SDMACCB[instance][0]);

}

void SDMA_CreateHandle(sdma_handle_t *handle, SDMAARM_Type *base, uint32_t channel, sdma_context_data_t *context) {
    uint32_t sdmaInstance;

    /* Zero the handle */
    (void)memset(handle, 0, sizeof(*handle));

    // Create the handle with the supplied arguments
    handle->base    = base;
    handle->channel = (uint8_t)channel;
    handle->bdCount = 1U;
    handle->context = context;

    /* Get the DMA instance number */
    sdmaInstance                        = SDMA_GetInstance(base);
    s_SDMAHandle[sdmaInstance][channel] = handle;

    // Set the CCB 
    s_SDMACCB[sdmaInstance][channel].baseBDAddr            = (uint32_t)(&s_SDMABD[sdmaInstance][channel]);
    s_SDMACCB[sdmaInstance][channel].currentBDAddr         = (uint32_t)(&s_SDMABD[sdmaInstance][channel]);
    /* Enable interrupt */
    (void)enableIRQ(s_sdmaIRQNumber[sdmaInstance]);
}


// Init function required by CP for every PD
void init(void) {
    sel4cp_dbg_puts(sel4cp_name);
    sel4cp_dbg_puts(": elf PD init function running\n");


    // Call init_post here to setup the ring buffer regions. The init_post case in the notified
    // switch statement may be redundant.
    init_post();

    imx_uart_regs_t *regs = (imx_uart_regs_t *) uart_base;
    SDMAARM_Type *base = (SDMAARM_Type *) dma_controller;
    // Software reset results in failed uart init, not too sure why
    /* Software reset */
    // regs->cr2 &= ~UART_CR2_SRST;
    // while (!(regs->cr2 & UART_CR2_SRST));

    global_serial_driver.regs = regs;
    global_serial_driver.rx_ring = rx_ring;
    global_serial_driver.tx_ring = tx_ring;
    global_serial_driver.num_to_get_chars = 0;

    sel4cp_dbg_puts("Line configuration\n");

    /* Line configuration */
    int ret = serial_configure(115200, 8, PARITY_NONE, 1);

    if (ret != 0) {
        sel4cp_dbg_puts("Error occured during line configuration\n");
    }

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

    sel4cp_dbg_puts("Enabled the uart, init the ring buffers\n");

    sel4cp_dbg_puts("Attempting to init dma controller\n");

    sdma_config_t sdmaConfig;

    /* Init the SDMA module */
    SDMA_GetDefaultConfig(&sdmaConfig);
    init_dma(base, &sdmaConfig);
    SDMA_CreateHandle(&g_uartTxSdmaHandle, base, UART_TX_DMA_CHANNEL, &context_Tx);
    SDMA_CreateHandle(&g_uartRxSdmaHandle, base, UART_RX_DMA_CHANNEL, &context_Rx);
    SDMA_SetChannelPriority(base, UART_TX_DMA_CHANNEL, 3U);
    SDMA_SetChannelPriority(base, UART_RX_DMA_CHANNEL, 4U);

    sel4cp_dbg_puts("Init'd the dma controller\n");

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