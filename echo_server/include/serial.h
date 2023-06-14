#pragma once

#include "shared_ringbuffer.h"
#include "sdma.h"

#define BIT(nr) (1UL << (nr))

#define UART_SR1_RRDY          BIT( 9)
#define UART_SR1_TRDY          BIT(13)
/* CR1 */
#define UART_CR1_UARTEN        BIT( 0)
#define UART_CR1_RRDYEN        BIT( 9)
/* CR2 */
#define UART_CR2_SRST          BIT( 0)
#define UART_CR2_RXEN          BIT( 1)
#define UART_CR2_TXEN          BIT( 2)
#define UART_CR2_ATEN          BIT( 3)
#define UART_CR2_RTSEN         BIT( 4)
#define UART_CR2_WS            BIT( 5)
#define UART_CR2_STPB          BIT( 6)
#define UART_CR2_PROE          BIT( 7)
#define UART_CR2_PREN          BIT( 8)
#define UART_CR2_RTEC          BIT( 9)
#define UART_CR2_ESCEN         BIT(11)
#define UART_CR2_CTS           BIT(12)
#define UART_CR2_CTSC          BIT(13)
#define UART_CR2_IRTS          BIT(14)
#define UART_CR2_ESCI          BIT(15)
/* CR3 */
#define UART_CR3_RXDMUXDEL     BIT( 2)
/* FCR */
#define UART_FCR_RFDIV(x)      ((x) * BIT(7))
#define UART_FCR_RFDIV_MASK    UART_FCR_RFDIV(0x7)
#define UART_FCR_RXTL(x)       ((x) * BIT(0))
#define UART_FCR_RXTL_MASK     UART_FCR_RXTL(0x1F)
/* SR2 */
#define UART_SR2_RXFIFO_RDR    BIT(0)
#define UART_SR2_TXFIFO_EMPTY  BIT(14)
/* RXD */
#define UART_URXD_READY_MASK   BIT(15)
#define UART_BYTE_MASK         0xFF
 /* DMA */
#define UCR1_RXDMAEN	(1<<8)	/* Recv ready DMA enable */
#define UCR1_TXDMAEN	(1<<3)	/* Transmitter ready DMA enable */
#define USR1_TRDY	(1<<13) /* Transmitter ready interrupt/dma flag */
#define USR1_RRDY	(1<<9)	 /* Receiver ready interrupt/dma flag */
/* INTERRUPT FLAGS*/
#define USR1_PARITYERR	(1<<15) /* Parity error interrupt flag */
#define IRQ_MASK (USR1_TRDY | USR1_RRDY)

// Might need to copy over the platform specific files, for now copied into one serial.h

#define UART1_PADDR  0x30860000
#define UART2_PADDR  0x30890000
#define UART3_PADDR  0x30880000
#define UART4_PADDR  0x30a60000

#define UART1_IRQ    58
#define UART2_IRQ    59
#define UART3_IRQ    60
#define UART4_IRQ    61

#define UART_REF_CLK 12096000

// Move this into a seperate file in the future
#define NUM_BUFFERS 512
#define BUFFER_SIZE 2048

/* DMA DEFINITIONS */
#define UART_RX_DMA_CHANNEL       1U
#define UART_TX_DMA_CHANNEL       2U
#define UART_RX_DMA_REQUEST       (28)
#define UART_TX_DMA_REQUEST       (29)
#define ECHO_BUFFER_LENGTH 8

#define kStatus_UART_RxBusy 3
#define kStatus_UART_TxBusy 2800
#define kStatus_Success 0
#define kStatus_UART_RxIdle 2803
#define kStatus_UART_TxIdle 2802
#define kUART_TxEmptyEnable 16

/* UART - Peripheral instance base addresses */
/** Peripheral UART1 base address */
#define UART1_BASE                               (0x30860000u)
/** Peripheral UART1 base pointer */
#define UART1                                    ((UART_Type *)UART1_BASE)
/** Peripheral UART2 base address */
#define UART2_BASE                               (0x30890000u)
/** Peripheral UART2 base pointer */
#define UART2                                    ((UART_Type *)UART2_BASE)
/** Peripheral UART3 base address */
#define UART3_BASE                               (0x30880000u)
/** Peripheral UART3 base pointer */
#define UART3                                    ((UART_Type *)UART3_BASE)
/** Peripheral UART4 base address */
#define UART4_BASE                               (0x30A60000u)
/** Peripheral UART4 base pointer */
#define UART4                                    ((UART_Type *)UART4_BASE)
/** Array initializer of UART peripheral base addresses */
#define UART_BASE_ADDRS                          { 0u, UART1_BASE, UART2_BASE, UART3_BASE, UART4_BASE }
/** Array initializer of UART peripheral base pointers */
#define UART_BASE_PTRS                           { (UART_Type *)0u, UART1, UART2, UART3, UART4 }
/** Interrupt vectors for the UART peripheral type */
#define UART_IRQS                  

enum serial_parity {
    PARITY_NONE,
    PARITY_EVEN,
    PARITY_ODD
};

struct imx_uart_regs {
    uint32_t rxd;      /* 0x000 Receiver Register */
    uint32_t res0[15];
    uint32_t txd;      /* 0x040 Transmitter Register */
    uint32_t res1[15];
    uint32_t cr1;      /* 0x080 Control Register 1 */
    uint32_t cr2;      /* 0x084 Control Register 2 */
    uint32_t cr3;      /* 0x088 Control Register 3 */
    uint32_t cr4;      /* 0x08C Control Register 4 */
    uint32_t fcr;      /* 0x090 FIFO Control Register */
    uint32_t sr1;      /* 0x094 Status Register 1 */
    uint32_t sr2;      /* 0x098 Status Register 2 */
    uint32_t esc;      /* 0x09c Escape Character Register */
    uint32_t tim;      /* 0x0a0 Escape Timer Register */
    uint32_t bir;      /* 0x0a4 BRM Incremental Register */
    uint32_t bmr;      /* 0x0a8 BRM Modulator Register */
    uint32_t brc;      /* 0x0ac Baud Rate Counter Register */
    uint32_t onems;    /* 0x0b0 One Millisecond Register */
    uint32_t ts;       /* 0x0b4 Test Register */
};
typedef volatile struct imx_uart_regs imx_uart_regs_t;

/*
serial driver struct akin to patrick's implementation*/
struct serial_driver {
    imx_uart_regs_t *regs;

    ring_handle_t rx_ring;
    ring_handle_t tx_ring;

    int num_to_get_chars;
};

/* DMA FUNCTIONS AND DEFINITIONS*/

/*! @brief UART transfer structure. */
typedef struct _uart_transfer
{
    /*
     * Use separate TX and RX data pointer, because TX data is const data.
     * The member data is kept for backward compatibility.
     */
    union
    {
        uint8_t *data;         /*!< The buffer of data to be transfer.*/
        uint8_t *rxData;       /*!< The buffer to receive data. */
        const uint8_t *txData; /*!< The buffer of data to be sent. */
    };
    size_t dataSize; /*!< The byte count to be transfer. */
} uart_transfer_t;

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief UART SDMA driver version. */
#define FSL_UART_SDMA_DRIVER_VERSION (MAKE_VERSION(2, 3, 0))
/*@}*/

/* Forward declaration of the handle typedef. */
typedef struct _uart_sdma_handle uart_sdma_handle_t;

/*! @brief UART transfer callback function. */
typedef void (*uart_sdma_transfer_callback_t)(imx_uart_regs_t *base,
                                              uart_sdma_handle_t *handle,
                                              int32_t status,
                                              void *userData);

/*!
 * @brief UART sDMA handle
 */
struct _uart_sdma_handle
{
    uart_sdma_transfer_callback_t callback; /*!< Callback function. */
    void *userData;                         /*!< UART callback function parameter.*/
    size_t rxDataSizeAll;                   /*!< Size of the data to receive. */
    size_t txDataSizeAll;                   /*!< Size of the data to send out. */
    sdma_handle_t *txSdmaHandle;            /*!< The sDMA TX channel used. */
    sdma_handle_t *rxSdmaHandle;            /*!< The sDMA RX channel used. */
    volatile uint8_t txState;               /*!< TX transfer state. */
    volatile uint8_t rxState;               /*!< RX transfer state */
};

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @name sDMA transactional
 * @{
 */

/*!
 * @brief Initializes the UART handle which is used in transactional functions.
 * @param base UART peripheral base address.
 * @param handle Pointer to the uart_sdma_handle_t structure.
 * @param callback UART callback, NULL means no callback.
 * @param userData User callback function data.
 * @param rxSdmaHandle User-requested DMA handle for RX DMA transfer.
 * @param txSdmaHandle User-requested DMA handle for TX DMA transfer.
 * @param eventSourceTx Eventsource for TX DMA transfer.
 * @param eventSourceRx Eventsource for RX DMA transfer.
 */
void UART_TransferCreateHandleSDMA(imx_uart_regs_t *base,
                                   uart_sdma_handle_t *handle,
                                   uart_sdma_transfer_callback_t callback,
                                   void *userData,
                                   sdma_handle_t *txSdmaHandle,
                                   sdma_handle_t *rxSdmaHandle,
                                   uint32_t eventSourceTx,
                                   uint32_t eventSourceRx);

/*!
 * @brief Sends data using sDMA.
 *
 * This function sends data using sDMA. This is a non-blocking function, which returns
 * right away. When all data is sent, the send callback function is called.
 *
 * @param base UART peripheral base address.
 * @param handle UART handle pointer.
 * @param xfer UART sDMA transfer structure. See #uart_transfer_t.
 * @retval kStatus_Success if succeeded; otherwise failed.
 * @retval kStatus_UART_TxBusy Previous transfer ongoing.
 * @retval kStatus_InvalidArgument Invalid argument.
 */
int32_t UART_SendSDMA(imx_uart_regs_t *base, uart_sdma_handle_t *handle, uart_transfer_t *xfer);

/*!
 * @brief Receives data using sDMA.
 *
 * This function receives data using sDMA. This is a non-blocking function, which returns
 * right away. When all data is received, the receive callback function is called.
 *
 * @param base UART peripheral base address.
 * @param handle Pointer to the uart_sdma_handle_t structure.
 * @param xfer UART sDMA transfer structure. See #uart_transfer_t.
 * @retval kStatus_Success if succeeded; otherwise failed.
 * @retval kStatus_UART_RxBusy Previous transfer ongoing.
 * @retval kStatus_InvalidArgument Invalid argument.
 */
int32_t UART_ReceiveSDMA(imx_uart_regs_t *base, uart_sdma_handle_t *handle, uart_transfer_t *xfer);

/*!
 * @brief Aborts the sent data using sDMA.
 *
 * This function aborts sent data using sDMA.
 *
 * @param base UART peripheral base address.
 * @param handle Pointer to the uart_sdma_handle_t structure.
 */
void UART_TransferAbortSendSDMA(imx_uart_regs_t *base, uart_sdma_handle_t *handle);

/*!
 * @brief Aborts the receive data using sDMA.
 *
 * This function aborts receive data using sDMA.
 *
 * @param base UART peripheral base address.
 * @param handle Pointer to the uart_sdma_handle_t structure.
 */
void UART_TransferAbortReceiveSDMA(imx_uart_regs_t *base, uart_sdma_handle_t *handle);

/*!
 * @brief UART IRQ handle function.
 *
 * This function handles the UART transmit complete IRQ request and invoke user callback.
 *
 * @param base UART peripheral base address.
 * @param uartSdmaHandle UART handle pointer.
 */
void UART_TransferSdmaHandleIRQ(imx_uart_regs_t *base, void *uartSdmaHandle);


