/*
* Sample serial driver for imx8mm based on the sDDF
*/

#include <stdbool.h>
#include <stdint.h>
#include <sel4cp.h>
#include <sel4/sel4.h>
#include "string.h"
#include "serial.h"
#include "shared_ringbuffer.h"
#include "sDMA_api/include/sdma.h"
#include "nvic.h"
#include "uart_reg_masks.h"

#define BIT(nr) (1UL << (nr))

// Defines to manage interrupts and notifications
#define IRQ_CH 1
#define TX_CH  8
#define RX_CH  10
#define INIT   4

AT_NONCACHEABLE_SECTION_ALIGN(sdma_context_data_t context_Tx, 4);
AT_NONCACHEABLE_SECTION_ALIGN(sdma_context_data_t context_Rx, 4);

AT_NONCACHEABLE_SECTION_ALIGN(uart_sdma_handle_t g_uartSdmaHandle, 4);
AT_NONCACHEABLE_SECTION_ALIGN(sdma_handle_t g_uartTxSdmaHandle, 4);
AT_NONCACHEABLE_SECTION_ALIGN(sdma_handle_t g_uartRxSdmaHandle, 4);


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
uintptr_t dma_controller;

/* Pointers to shared_ringbuffers */
ring_handle_t rx_ring;
ring_handle_t tx_ring;

// Global serial_driver variable

struct serial_driver global_serial_driver = {0};

static uintptr_t 
getPhysAddr(uintptr_t virtual)
{
    uint64_t offset = virtual - shared_dma_vaddr;
    uintptr_t phys;

    if (offset < 0) {
        print("getPhysAddr: offset < 0");
        return 0;
    }

    phys = shared_dma_paddr + offset;
    return phys;
}


/* fsl_uart_sdma.c from imx8mm sdk*/

/*!
 * brief Enables UART interrupts according to the provided mask.
 *
 * This function enables the UART interrupts according to the provided mask. The mask
 * is a logical OR of enumeration members. See ref _uart_interrupt_enable.
 * For example, to enable TX empty interrupt and RX data ready interrupt, do the following.
 * code
 *     UART_EnableInterrupts(UART1,kUART_TxEmptyEnable | kUART_RxDataReadyEnable);
 * endcode
 *
 * param base UART peripheral base address.
 * param mask The interrupts to enable. Logical OR of ref _uart_interrupt_enable.
 */
void UART_EnableInterrupts(imx_uart_regs_t *base, uint32_t mask)
{

    if ((0X3FU & mask) != 0U)
    {
        base->cr1 |= ((mask << UART_UCR1_ADEN_SHIFT) & UART_UCR1_ADEN_MASK) |
                      (((mask >> 1) << UART_UCR1_TRDYEN_SHIFT) & UART_UCR1_TRDYEN_MASK) |
                      (((mask >> 2) << UART_UCR1_IDEN_SHIFT) & UART_UCR1_IDEN_MASK) |
                      (((mask >> 3) << UART_UCR1_RRDYEN_SHIFT) & UART_UCR1_RRDYEN_MASK) |
                      (((mask >> 4) << UART_UCR1_TXMPTYEN_SHIFT) & UART_UCR1_TXMPTYEN_MASK) |
                      (((mask >> 5) << UART_UCR1_RTSDEN_SHIFT) & UART_UCR1_RTSDEN_MASK);
    }
    if ((0X700U & mask) != 0U)
    {
        base->cr1 |= (((mask >> 8) << UART_UCR2_ESCI_SHIFT) & UART_UCR2_ESCI_MASK) |
                      (((mask >> 9) << UART_UCR2_RTSEN_SHIFT) & UART_UCR2_RTSEN_MASK) |
                      (((mask >> 10) << UART_UCR2_ATEN_SHIFT) & UART_UCR2_ATEN_MASK);
    }
    if ((0x3FF000U & mask) != 0U)
    {
        base->cr1 |= (((mask >> 12) << UART_UCR3_DTREN_SHIFT) & UART_UCR3_DTREN_MASK) |
                      (((mask >> 13) << UART_UCR3_PARERREN_SHIFT) & UART_UCR3_PARERREN_MASK) |
                      (((mask >> 14) << UART_UCR3_FRAERREN_SHIFT) & UART_UCR3_FRAERREN_MASK) |
                      (((mask >> 15) << UART_UCR3_DCD_SHIFT) & UART_UCR3_DCD_MASK) |
                      (((mask >> 16) << UART_UCR3_RI_SHIFT) & UART_UCR3_RI_MASK) |
                      (((mask >> 17) << UART_UCR3_RXDSEN_SHIFT) & UART_UCR3_RXDSEN_MASK) |
                      (((mask >> 18) << UART_UCR3_AIRINTEN_SHIFT) & UART_UCR3_AIRINTEN_MASK) |
                      (((mask >> 19) << UART_UCR3_AWAKEN_SHIFT) & UART_UCR3_AWAKEN_MASK) |
                      (((mask >> 20) << UART_UCR3_DTRDEN_SHIFT) & UART_UCR3_DTRDEN_MASK) |
                      (((mask >> 21) << UART_UCR3_ACIEN_SHIFT) & UART_UCR3_ACIEN_MASK);
    }
    if ((0x7F000000U & mask) != 0U)
    {
        base->cr4 |= (((mask >> 24) << UART_UCR4_ENIRI_SHIFT) & UART_UCR4_ENIRI_MASK) |
                      (((mask >> 25) << UART_UCR4_WKEN_SHIFT) & UART_UCR4_WKEN_MASK) |
                      (((mask >> 26) << UART_UCR4_TCEN_SHIFT) & UART_UCR4_TCEN_MASK) |
                      (((mask >> 27) << UART_UCR4_BKEN_SHIFT) & UART_UCR4_BKEN_MASK) |
                      (((mask >> 28) << UART_UCR4_OREN_SHIFT) & UART_UCR4_OREN_MASK) |
                      (((mask >> 29) << UART_UCR4_DREN_SHIFT) & UART_UCR4_DREN_MASK) |
                      (((mask >> 30) << UART_UCR4_IDDMAEN_SHIFT) & UART_UCR4_IDDMAEN_MASK);
    }
}

/*!
 * brief Disables the UART interrupts according to the provided mask.
 *
 * This function disables the UART interrupts according to the provided mask. The mask
 * is a logical OR of enumeration members. See ref _uart_interrupt_enable.
 * For example, to disable TX empty interrupt and RX data ready interrupt do the following.
 * code
 *     UART_EnableInterrupts(UART1,kUART_TxEmptyEnable | kUART_RxDataReadyEnable);
 * endcode
 *
 * param base UART peripheral base address.
 * param mask The interrupts to disable. Logical OR of ref _uart_interrupt_enable.
 */
void UART_DisableInterrupts(imx_uart_regs_t *base, uint32_t mask)
{

    if ((0X3FU & mask) != 0U)
    {
        base->cr1 &= ~(((mask << UART_UCR1_ADEN_SHIFT) & UART_UCR1_ADEN_MASK) |
                        (((mask >> 1) << UART_UCR1_TRDYEN_SHIFT) & UART_UCR1_TRDYEN_MASK) |
                        (((mask >> 2) << UART_UCR1_IDEN_SHIFT) & UART_UCR1_IDEN_MASK) |
                        (((mask >> 3) << UART_UCR1_RRDYEN_SHIFT) & UART_UCR1_RRDYEN_MASK) |
                        (((mask >> 4) << UART_UCR1_TXMPTYEN_SHIFT) & UART_UCR1_TXMPTYEN_MASK) |
                        (((mask >> 5) << UART_UCR1_RTSDEN_SHIFT) & UART_UCR1_RTSDEN_MASK));
    }
    if ((0X700U & mask) != 0U)
    {
        base->cr2 &= ~((((mask >> 8) << UART_UCR2_ESCI_SHIFT) & UART_UCR2_ESCI_MASK) |
                        (((mask >> 9) << UART_UCR2_RTSEN_SHIFT) & UART_UCR2_RTSEN_MASK) |
                        (((mask >> 10) << UART_UCR2_ATEN_SHIFT) & UART_UCR2_ATEN_MASK));
    }
    if ((0x3FF000U & mask) != 0U)
    {
        base->cr3 &= ~((((mask >> 12) << UART_UCR3_DTREN_SHIFT) & UART_UCR3_DTREN_MASK) |
                        (((mask >> 13) << UART_UCR3_PARERREN_SHIFT) & UART_UCR3_PARERREN_MASK) |
                        (((mask >> 14) << UART_UCR3_FRAERREN_SHIFT) & UART_UCR3_FRAERREN_MASK) |
                        (((mask >> 15) << UART_UCR3_DCD_SHIFT) & UART_UCR3_DCD_MASK) |
                        (((mask >> 16) << UART_UCR3_RI_SHIFT) & UART_UCR3_RI_MASK) |
                        (((mask >> 17) << UART_UCR3_RXDSEN_SHIFT) & UART_UCR3_RXDSEN_MASK) |
                        (((mask >> 18) << UART_UCR3_AIRINTEN_SHIFT) & UART_UCR3_AIRINTEN_MASK) |
                        (((mask >> 19) << UART_UCR3_AWAKEN_SHIFT) & UART_UCR3_AWAKEN_MASK) |
                        (((mask >> 20) << UART_UCR3_DTRDEN_SHIFT) & UART_UCR3_DTRDEN_MASK) |
                        (((mask >> 21) << UART_UCR3_ACIEN_SHIFT) & UART_UCR3_ACIEN_MASK));
    }
    if ((0x7F000000U & mask) != 0U)
    {
        base->cr4 &= ~((((mask >> 24) << UART_UCR4_ENIRI_SHIFT) & UART_UCR4_ENIRI_MASK) |
                        (((mask >> 25) << UART_UCR4_WKEN_SHIFT) & UART_UCR4_WKEN_MASK) |
                        (((mask >> 26) << UART_UCR4_TCEN_SHIFT) & UART_UCR4_TCEN_MASK) |
                        (((mask >> 27) << UART_UCR4_BKEN_SHIFT) & UART_UCR4_BKEN_MASK) |
                        (((mask >> 28) << UART_UCR4_OREN_SHIFT) & UART_UCR4_OREN_MASK) |
                        (((mask >> 29) << UART_UCR4_DREN_SHIFT) & UART_UCR4_DREN_MASK) |
                        (((mask >> 30) << UART_UCR4_IDDMAEN_SHIFT) & UART_UCR4_IDDMAEN_MASK));
    }
}

/*!
 * @brief Enables or disables the UART transmitter DMA request.
 *
 * This function enables or disables the transmit request when the transmitter
 * has one or more slots available in the TxFIFO. The fill level in the TxFIFO
 * that generates the DMA request is controlled by the TXTL bits.
 *
 * @param base UART peripheral base address.
 * @param enable True to enable, false to disable.
 */
void UART_EnableTxDMA(imx_uart_regs_t *base, bool enable)
{
    if (enable)
    {
        base->cr1 |= UCR1_TXDMAEN;
    }
    else
    {
        base->cr1 &= ~UCR1_TXDMAEN;
    }
}

/*!
 * @brief Enables or disables the UART receiver DMA request.
 *
 * This function enables or disables the receive request when the receiver
 * has data in the RxFIFO. The fill level in the RxFIFO at which a DMA request
 * is generated is controlled by the RXTL bits .
 *
 * @param base UART peripheral base address.
 * @param enable True to enable, false to disable.
 */
void UART_EnableRxDMA(imx_uart_regs_t *base, bool enable)
{
    if (enable)
    {
        base->cr1 |= UCR1_RXDMAEN;
    }
    else
    {
        base->cr1 &= ~UCR1_RXDMAEN;
    }
}

/* UART user callback */
void UART_UserCallback(imx_uart_regs_t *base, uart_sdma_handle_t *handle, int32_t status, void *userData)
{
    // userData = userData;

    // if (kStatus_UART_TxIdle == status)
    // {
    //     txBufferFull = false;
    //     txOnGoing    = false;
    // }

    // if (kStatus_UART_RxIdle == status)
    // {
    //     rxBufferEmpty = false;
    //     rxOnGoing     = false;
    // }
    sel4cp_dbg_puts("We are in the uart user callback\n");
}


/*<! Structure definition for uart_sdma_private_handle_t. The structure is private. */
typedef struct _uart_sdma_private_handle
{
    imx_uart_regs_t *base;
    uart_sdma_handle_t *handle;
} uart_sdma_private_handle_t;

/* UART SDMA transfer handle. */
enum _uart_sdma_tansfer_states
{
    kUART_TxIdle, /* TX idle. */
    kUART_TxBusy, /* TX busy. */
    kUART_RxIdle, /* RX idle. */
    kUART_RxBusy  /* RX busy. */
};

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*<! Private handle only used for internally. */
static uart_sdma_private_handle_t s_sdmaPrivateHandle[1];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief UART SDMA send finished callback function.
 *
 * This function is called when UART SDMA send finished. It disables the UART
 * TX SDMA request and sends @ref kStatus_UART_TxIdle to UART callback.
 *
 * @param handle The SDMA handle.
 * @param param Callback function parameter.
 */
static void UART_SendSDMACallback(sdma_handle_t *handle, void *param, bool transferDone, uint32_t tcds);

/*!
 * @brief UART SDMA receive finished callback function.
 *
 * This function is called when UART SDMA receive finished. It disables the UART
 * RX SDMA request and sends @ref kStatus_UART_RxIdle to UART callback.
 *
 * @param handle The SDMA handle.
 * @param param Callback function parameter.
 */
static void UART_ReceiveSDMACallback(sdma_handle_t *handle, void *param, bool transferDone, uint32_t tcds);

/*******************************************************************************
 * Code
 ******************************************************************************/

static void UART_SendSDMACallback(sdma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    // assert(param != NULL);

    uart_sdma_private_handle_t *uartPrivateHandle = (uart_sdma_private_handle_t *)param;

    if (transferDone)
    {
        /* Disable UART TX SDMA. */
        UART_EnableTxDMA(uartPrivateHandle->base, false);

        /* Stop transfer. */
        SDMA_AbortTransfer(handle);

        /* Enable tx empty interrupt */
        UART_EnableInterrupts(uartPrivateHandle->base, (uint32_t)kUART_TxEmptyEnable);
    }
}

static void UART_ReceiveSDMACallback(sdma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    // assert(param != NULL);

    uart_sdma_private_handle_t *uartPrivateHandle = (uart_sdma_private_handle_t *)param;

    if (transferDone)
    {
        /* Disable transfer. */
        UART_TransferAbortReceiveSDMA(uartPrivateHandle->base, uartPrivateHandle->handle);

        if (uartPrivateHandle->handle->callback != NULL)
        {
            uartPrivateHandle->handle->callback(uartPrivateHandle->base, uartPrivateHandle->handle, kStatus_UART_RxIdle,
                                                uartPrivateHandle->handle->userData);
        }
    }
}

/*!
 * brief Initializes the UART handle which is used in transactional functions.
 * param base UART peripheral base address.
 * param handle Pointer to the uart_sdma_handle_t structure.
 * param callback UART callback, NULL means no callback.
 * param userData User callback function data.
 * param rxSdmaHandle User-requested DMA handle for RX DMA transfer.
 * param txSdmaHandle User-requested DMA handle for TX DMA transfer.
 * param eventSourceTx Eventsource for TX DMA transfer.
 * param eventSourceRx Eventsource for RX DMA transfer.
 */
void UART_TransferCreateHandleSDMA(imx_uart_regs_t *base,
                                   uart_sdma_handle_t *handle,
                                   uart_sdma_transfer_callback_t callback,
                                   void *userData,
                                   sdma_handle_t *txSdmaHandle,
                                   sdma_handle_t *rxSdmaHandle,
                                   uint32_t eventSourceTx,
                                   uint32_t eventSourceRx)
{
    // assert(handle != NULL);

    // We are currently only using UART1. No other UART instances will be running
    uint32_t instance = 0;

    (void)memset(handle, 0, sizeof(*handle));

    handle->rxState = (uint8_t)kUART_RxIdle;
    handle->txState = (uint8_t)kUART_TxIdle;

    if (rxSdmaHandle != NULL)
    {
        rxSdmaHandle->eventSource = eventSourceRx;
    }

    if (txSdmaHandle != NULL)
    {
        txSdmaHandle->eventSource = eventSourceTx;
    }

    handle->rxSdmaHandle = rxSdmaHandle;
    handle->txSdmaHandle = txSdmaHandle;

    handle->callback = callback;
    handle->userData = userData;

    s_sdmaPrivateHandle[instance].base = base;
    s_sdmaPrivateHandle[instance].handle = handle;

    /* Enable interrupt in NVIC. */
    (void)EnableIRQ(UART1_IRQn);

    /* Configure TX. */
    if (txSdmaHandle != NULL)
    {
        SDMA_SetCallback(handle->txSdmaHandle, UART_SendSDMACallback, &s_sdmaPrivateHandle[instance]);
    }

    /* Configure RX. */
    if (rxSdmaHandle != NULL)
    {
        SDMA_SetCallback(handle->rxSdmaHandle, UART_ReceiveSDMACallback, &s_sdmaPrivateHandle[instance]);
    }
}

/*!
 * brief Sends data using sDMA.
 *
 * This function sends data using sDMA. This is a non-blocking function, which returns
 * right away. When all data is sent, the send callback function is called.
 *
 * param base UART peripheral base address.
 * param handle UART handle pointer.
 * param xfer UART sDMA transfer structure. See #uart_transfer_t.
 * retval kStatus_Success if succeeded; otherwise failed.
 * retval kStatus_UART_TxBusy Previous transfer ongoing.
 * retval kStatus_InvalidArgument Invalid argument.
 */
int32_t UART_SendSDMA(imx_uart_regs_t *base, uart_sdma_handle_t *handle, uart_transfer_t *xfer)
{
    // assert(handle != NULL);
    // assert(handle->txSdmaHandle != NULL);
    // assert(xfer != NULL);
    // assert(xfer->data != NULL);
    // assert(xfer->dataSize != 0U);

    sdma_transfer_config_t xferConfig = {0U};
    int32_t status;
    sdma_peripheral_t perType = kSDMA_PeripheralTypeUART;

    /* If previous TX not finished. */
    if ((uint8_t)kUART_TxBusy == handle->txState)
    {
        status = kStatus_UART_TxBusy;
    }
    else
    {
        handle->txState = (uint8_t)kUART_TxBusy;
        handle->txDataSizeAll = xfer->dataSize;

        sdma_handle_t *txSdmaHandle = handle->txSdmaHandle;

        /* Prepare transfer. */
        SDMA_PrepareTransfer(&xferConfig, (uint32_t)xfer->data, (uint32_t) & (base->txd), sizeof(uint8_t),
                             sizeof(uint8_t), sizeof(uint8_t), (uint32_t)xfer->dataSize,
                             txSdmaHandle->eventSource, perType, kSDMA_MemoryToPeripheral);

        /* Submit transfer. */
        SDMA_SubmitTransfer(handle->txSdmaHandle, &xferConfig);

        SDMA_StartTransfer(handle->txSdmaHandle);

        /* Enable UART TX SDMA. */
        UART_EnableTxDMA(base, true);
        status = kStatus_Success;
    }

    return status;
}

/*!
 * brief Receives data using sDMA.
 *
 * This function receives data using sDMA. This is a non-blocking function, which returns
 * right away. When all data is received, the receive callback function is called.
 *
 * param base UART peripheral base address.
 * param handle Pointer to the uart_sdma_handle_t structure.
 * param xfer UART sDMA transfer structure. See #uart_transfer_t.
 * retval kStatus_Success if succeeded; otherwise failed.
 * retval kStatus_UART_RxBusy Previous transfer ongoing.
 * retval kStatus_InvalidArgument Invalid argument.
 */
int32_t UART_ReceiveSDMA(imx_uart_regs_t *base, uart_sdma_handle_t *handle, uart_transfer_t *xfer)
{
    // assert(handle != NULL);
    // assert(handle->rxSdmaHandle != NULL);
    // assert(xfer != NULL);
    // assert(xfer->data != NULL);
    // assert(xfer->dataSize != 0U);

    sdma_transfer_config_t xferConfig = {0U};
    int32_t status;
    sdma_peripheral_t perType = kSDMA_PeripheralTypeUART;

    /* If previous RX not finished. */
    if ((uint8_t)kUART_RxBusy == handle->rxState)
    {
        status = kStatus_UART_RxBusy;
    }
    else
    {
        handle->rxState = (uint8_t)kUART_RxBusy;
        handle->rxDataSizeAll = xfer->dataSize;

        sdma_handle_t *rxSdmaHandle = handle->rxSdmaHandle;

        /* Prepare transfer. */
        SDMA_PrepareTransfer(&xferConfig, (uint32_t) & (base->rxd), (uint32_t)xfer->data, sizeof(uint8_t),
                             sizeof(uint8_t), sizeof(uint8_t), (uint32_t)xfer->dataSize,
                             rxSdmaHandle->eventSource, perType, kSDMA_PeripheralToMemory);

        /* Submit transfer. */
        SDMA_SubmitTransfer(handle->rxSdmaHandle, &xferConfig);

        SDMA_StartTransfer(handle->rxSdmaHandle);

        /* Enable UART RX SDMA. */
        UART_EnableRxDMA(base, true);

        status = kStatus_Success;
    }

    return status;
}

/*!
 * brief Aborts the sent data using sDMA.
 *
 * This function aborts sent data using sDMA.
 *
 * param base UART peripheral base address.
 * param handle Pointer to the uart_sdma_handle_t structure.
 */
void UART_TransferAbortSendSDMA(imx_uart_regs_t *base, uart_sdma_handle_t *handle)
{
    // assert(handle != NULL);
    // assert(handle->txSdmaHandle != NULL);

    /* Disable UART TX SDMA. */
    UART_EnableTxDMA(base, false);

    /* Stop transfer. */
    SDMA_AbortTransfer(handle->txSdmaHandle);

    handle->txState = (uint8_t)kUART_TxIdle;
}

/*!
 * brief Aborts the receive data using sDMA.
 *
 * This function aborts receive data using sDMA.
 *
 * param base UART peripheral base address.
 * param handle Pointer to the uart_sdma_handle_t structure.
 */
void UART_TransferAbortReceiveSDMA(imx_uart_regs_t *base, uart_sdma_handle_t *handle)
{
    // assert(handle != NULL);
    // assert(handle->rxSdmaHandle != NULL);

    /* Disable UART RX SDMA. */
    UART_EnableRxDMA(base, false);

    /* Stop transfer. */
    SDMA_AbortTransfer(handle->rxSdmaHandle);

    handle->rxState = (uint8_t)kUART_RxIdle;
}

/*!
 * brief UART IRQ handle function.
 *
 * This function handles the UART transmit complete IRQ request and invoke user callback.
 *
 * param base UART peripheral base address.
 * param uartSdmaHandle UART handle pointer.
 */
void UART_TransferSdmaHandleIRQ(imx_uart_regs_t *base, void *uartSdmaHandle)
{
    // assert(uartSdmaHandle != NULL);

    uart_sdma_handle_t *handle = (uart_sdma_handle_t *)uartSdmaHandle;
    handle->txState = (uint8_t)kUART_TxIdle;

    /* Disable tx empty interrupt */
    UART_DisableInterrupts(base, (uint32_t)kUART_TxEmptyEnable);

    if (handle->callback != NULL)
    {
        handle->callback(base, handle, kStatus_UART_TxIdle, handle->userData);
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

static void raw_tx_dma(char *phys, unsigned int len, void *cookie) {
    uart_transfer_t xfer;

    // Might need to change this to a physical address
    xfer.data     =  getPhysAddr(phys);
    xfer.dataSize = len;
    UART_SendSDMA((imx_uart_regs_t *) uart_base, &g_uartSdmaHandle, &xfer);
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
        raw_tx_dma(phys, len, cookie);
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

    sel4cp_dbg_puts("Attempting to init the sdma controller\n");

    sdma_config_t sdmaConfig;

    /* Init the SDMA module */
    SDMA_GetDefaultConfig(&sdmaConfig);
    SDMA_Init(base, &sdmaConfig);
    SDMA_CreateHandle(&g_uartTxSdmaHandle, base, UART_TX_DMA_CHANNEL, &context_Tx);
    SDMA_CreateHandle(&g_uartRxSdmaHandle, base, UART_RX_DMA_CHANNEL, &context_Rx);
    SDMA_SetChannelPriority(base, UART_TX_DMA_CHANNEL, 3U);
    SDMA_SetChannelPriority(base, UART_RX_DMA_CHANNEL, 4U);
    
/* Create UART DMA handle. */
    UART_TransferCreateHandleSDMA(regs, &g_uartSdmaHandle, UART_UserCallback, NULL, &g_uartTxSdmaHandle,
                                  &g_uartRxSdmaHandle, UART_TX_DMA_REQUEST, UART_RX_DMA_REQUEST);

    sel4cp_dbg_puts("Finished initing the sdma controller\n");


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