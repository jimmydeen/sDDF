#pragma once

/* Subset of the sDMA API Provided by Freescale */

#include <stdint.h>
/* Definitions of the DMA Controller registers, taken from linux drivers/dma/imx-dma.h */

#define DMA_DCR     0x00		/* Control Register */
#define DMA_DISR    0x04		/* Interrupt status Register */
#define DMA_DIMR    0x08		/* Interrupt mask Register */
#define DMA_DBTOSR  0x0c		/* Burst timeout status Register */
#define DMA_DRTOSR  0x10		/* Request timeout Register */
#define DMA_DSESR   0x14		/* Transfer Error Status Register */
#define DMA_DBOSR   0x18		/* Buffer overflow status Register */
#define DMA_DBTOCR  0x1c		/* Burst timeout control Register */
#define DMA_WSRA    0x40		/* W-Size Register A */
#define DMA_XSRA    0x44		/* X-Size Register A */
#define DMA_YSRA    0x48		/* Y-Size Register A */
#define DMA_WSRB    0x4c		/* W-Size Register B */
#define DMA_XSRB    0x50		/* X-Size Register B */
#define DMA_YSRB    0x54		/* Y-Size Register B */
#define DMA_SAR(x)  (0x80 + ((x) << 6))	/* Source Address Registers */
#define DMA_DAR(x)  (0x84 + ((x) << 6))	/* Destination Address Registers */
#define DMA_CNTR(x) (0x88 + ((x) << 6))	/* Count Registers */
#define DMA_CCR(x)  (0x8c + ((x) << 6))	/* Control Registers */
#define DMA_RSSR(x) (0x90 + ((x) << 6))	/* Request source select Registers */
#define DMA_BLR(x)  (0x94 + ((x) << 6))	/* Burst length Registers */
#define DMA_RTOR(x) (0x98 + ((x) << 6))	/* Request timeout Registers */
#define DMA_BUCR(x) (0x98 + ((x) << 6))	/* Bus Utilization Registers */
#define DMA_CCNR(x) (0x9C + ((x) << 6))	/* Channel counter Registers */

#define UART_DMA_TX_CH 22 /*DMA TX Channel for UART0*/
#define UART_DMA_RX_CH 21 /*DMA RX Channel for UART0*/

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))


/* SDMAARM - Peripheral instance base addresses */
/** Peripheral SDMAARM1 base address */
#define SDMAARM1_BASE                            (0x30BD0000u)
/** Peripheral SDMAARM1 base pointer */
#define SDMAARM1                                 ((SDMAARM_Type *)SDMAARM1_BASE)
/** Peripheral SDMAARM2 base address */
#define SDMAARM2_BASE                            (0x302C0000u)
/** Peripheral SDMAARM2 base pointer */
#define SDMAARM2                                 ((SDMAARM_Type *)SDMAARM2_BASE)
/** Peripheral SDMAARM3 base address */
#define SDMAARM3_BASE                            (0x302B0000u)
/** Peripheral SDMAARM3 base pointer */
#define SDMAARM3                                 ((SDMAARM_Type *)SDMAARM3_BASE)
/** Array initializer of SDMAARM peripheral base addresses */
#define SDMAARM_BASE_ADDRS                       { SDMAARM1_BASE, SDMAARM2_BASE, SDMAARM3_BASE }
/** Array initializer of SDMAARM peripheral base pointers */
#define SDMAARM_BASE_PTRS                        { SDMAARM1, SDMAARM2, SDMAARM3 }
/** Interrupt vectors for the SDMAARM peripheral type */
#define SDMAARM_IRQS             

#define FSL_FEATURE_SOC_SDMA_COUNT (3)


/* SDMA module features */

/* @brief SDMA module channel number. */
#define FSL_FEATURE_SDMA_MODULE_CHANNEL (32)
/* @brief SDMA module event number. */
#define FSL_FEATURE_SDMA_EVENT_NUM (48)
/* @brief SDMA ROM memory to memory script start address. */
#define FSL_FEATURE_SDMA_M2M_ADDR (644)
/* @brief SDMA ROM peripheral to memory script start address. */
#define FSL_FEATURE_SDMA_P2M_ADDR (685)
/* @brief SDMA ROM memory to peripheral script start address. */
#define FSL_FEATURE_SDMA_M2P_ADDR (749)
/* @brief SDMA ROM uart to memory script start address. */
#define FSL_FEATURE_SDMA_UART2M_ADDR (819)
/* @brief SDMA ROM peripheral on SPBA to memory script start address. */
#define FSL_FEATURE_SDMA_SHP2M_ADDR (893)
/* @brief SDMA ROM memory to peripheral on SPBA script start address. */
#define FSL_FEATURE_SDMA_M2SHP_ADDR (962)
/* @brief SDMA ROM UART on SPBA to memory script start address. */
#define FSL_FEATURE_SDMA_UARTSH2M_ADDR (1034)
/* @brief SDMA ROM SPDIF to memory script start address. */
#define FSL_FEATURE_SDMA_SPDIF2M_ADDR (1102)
/* @brief SDMA ROM memory to SPDIF script start address. */
#define FSL_FEATURE_SDMA_M2SPDIF_ADDR (1136)
/* @brief SDMA ROM memory to MULTI_FIFO_SAI_TX script start address. */
#define FSL_FEATURE_SDMA_MULTI_FIFO_SAI_TX_ADDR (6235)
/* @brief SDMA ROM memory to MULTI_FIFO_SAI_RX script start address. */
#define FSL_FEATURE_SDMA_MULTI_FIFO_SAI_RX_ADDR (6729)

/*! @brief SDMA core clock frequency ratio to the ARM DMA interface. */
typedef enum _sdma_clock_ratio
{
    kSDMA_HalfARMClockFreq = 0x0U, /*!< SDMA core clock frequency half of ARM platform */
    kSDMA_ARMClockFreq,            /*!< SDMA core clock frequency equals to ARM platform */
} sdma_clock_ratio_t;

/*! @brief SDMA global configuration structure.*/
typedef struct _sdma_config
{
    bool enableRealTimeDebugPin;   /*!< If enable real-time debug pin, default is closed to reduce power consumption.*/
    bool isSoftwareResetClearLock; /*!< If software reset clears the LOCK bit which prevent writing SDMA scripts into
                                      SDMA.*/
    sdma_clock_ratio_t ratio;      /*!< SDMA core clock ratio to ARM platform DMA interface */
} sdma_config_t;

/*!
 * @brief SDMA buffer descriptor structure.
 *
 * This structure is a buffer descriptor, this structure describes the buffer start address and other options
 */
typedef struct _sdma_buffer_descriptor
{
    uint32_t count : 16;       /*!< Bytes of the buffer length for this buffer descriptor. */
    uint32_t status : 8;       /*!< E,R,I,C,W,D status bits stored here */
    uint32_t command : 8;      /*!< command mostlky used for channel 0 */
    uint32_t bufferAddr;       /*!< Buffer start address for this descriptor. */
    uint32_t extendBufferAddr; /*!< External buffer start address, this is an optional for a transfer. */
} sdma_buffer_descriptor_t;

/*!
 * @brief SDMA channel control descriptor structure.
 */
typedef struct _sdma_channel_control
{
    uint32_t currentBDAddr; /*!< Address of current buffer descriptor processed  */
    uint32_t baseBDAddr;    /*!< The start address of the buffer descriptor array */
    uint32_t channelDesc;   /*!< Optional for transfer */
    uint32_t status;        /*!< Channel status */
} sdma_channel_control_t;


/** SDMAARM - Register Layout Typedef */
typedef struct {
    uint32_t MC0PTR;                            /**< Arm platform Channel 0 Pointer, offset: 0x0 */
    uint32_t INTR;                              /**< Channel Interrupts, offset: 0x4 */
    uint32_t STOP_STAT;                         /**< Channel Stop/Channel Status, offset: 0x8 */
    uint32_t HSTART;                            /**< Channel Start, offset: 0xC */
    uint32_t EVTOVR;                            /**< Channel Event Override, offset: 0x10 */
    uint32_t DSPOVR;                            /**< Channel BP Override, offset: 0x14 */
    uint32_t HOSTOVR;                           /**< Channel Arm platform Override, offset: 0x18 */
    uint32_t EVTPEND;                           /**< Channel Event Pending, offset: 0x1C */
    uint8_t RESERVED_0[4];
    uint32_t RESET;                             /**< Reset Register, offset: 0x24 */
    uint32_t EVTERR;                            /**< DMA Request Error Register, offset: 0x28 */
    uint32_t INTRMASK;                          /**< Channel Arm platform Interrupt Mask, offset: 0x2C */
    uint32_t PSW;                               /**< Schedule Status, offset: 0x30 */
    uint32_t EVTERRDBG;                         /**< DMA Request Error Register, offset: 0x34 */
    uint32_t CONFIG;                            /**< Configuration Register, offset: 0x38 */
    uint32_t SDMA_LOCK;                         /**< SDMA LOCK, offset: 0x3C */
    uint32_t ONCE_ENB;                          /**< OnCE Enable, offset: 0x40 */
    uint32_t ONCE_DATA;                         /**< OnCE Data Register, offset: 0x44 */
    uint32_t ONCE_INSTR;                        /**< OnCE Instruction Register, offset: 0x48 */
    uint32_t ONCE_STAT;                         /**< OnCE Status Register, offset: 0x4C */
    uint32_t ONCE_CMD;                          /**< OnCE Command Register, offset: 0x50 */
    uint8_t RESERVED_1[4];
    uint32_t ILLINSTADDR;                       /**< Illegal Instruction Trap Address, offset: 0x58 */
    uint32_t CHN0ADDR;                          /**< Channel 0 Boot Address, offset: 0x5C */
    uint32_t EVT_MIRROR;                        /**< DMA Requests, offset: 0x60 */
    uint32_t EVT_MIRROR2;                       /**< DMA Requests 2, offset: 0x64 */
    uint8_t RESERVED_2[8];
    uint32_t XTRIG_CONF1;                       /**< Cross-Trigger Events Configuration Register 1, offset: 0x70 */
    uint32_t XTRIG_CONF2;                       /**< Cross-Trigger Events Configuration Register 2, offset: 0x74 */
    uint8_t RESERVED_3[136];
    uint32_t SDMA_CHNPRI[32];                   /**< Channel Priority Registers, array offset: 0x100, array step: 0x4 */
    uint8_t RESERVED_4[128];
    uint32_t CHNENBL[48];                       /**< Channel Enable RAM, array offset: 0x200, array step: 0x4 */
    uint8_t RESERVED_5[3392];
    uint32_t DONE0_CONFIG;                      /**< SDMA DONE0 Configuration, offset: 0x1000 */
    uint32_t DONE1_CONFIG;                      /**< SDMA DONE1 Configuration, offset: 0x1004 */
} SDMAARM_Type;

/*!
 * @brief Gets the SDMA interrupt status of all channels.
 *
 * @param base SDMA peripheral base address.
 * @return The interrupt status for all channels. Check the relevant bits for specific channel.
 */
static inline uint32_t SDMA_GetChannelInterruptStatus(SDMAARM_Type *base)
{
    return base->INTR;
}

/*!
 * @brief Clear the SDMA channel interrupt status of specific channels.
 *
 * @param base SDMA peripheral base address.
 * @param mask The interrupt status need to be cleared.
 */
static inline void SDMA_ClearChannelInterruptStatus(SDMAARM_Type *base, uint32_t mask)
{
    base->INTR = mask;
}

/*!
 * @brief Gets the SDMA stop status of all channels.
 *
 * @param base SDMA peripheral base address.
 * @return The stop status for all channels. Check the relevant bits for specific channel.
 */
static inline uint32_t SDMA_GetChannelStopStatus(SDMAARM_Type *base)
{
    return base->STOP_STAT;
}

/*!
 * @brief Clear the SDMA channel stop status of specific channels.
 *
 * @param base SDMA peripheral base address.
 * @param mask The stop status need to be cleared.
 */
static inline void SDMA_ClearChannelStopStatus(SDMAARM_Type *base, uint32_t mask)
{
    base->STOP_STAT = mask;
}

/*!
 * @brief Gets the SDMA channel pending status of all channels.
 *
 * @param base SDMA peripheral base address.
 * @return The pending status for all channels. Check the relevant bits for specific channel.
 */
static inline uint32_t SDMA_GetChannelPendStatus(SDMAARM_Type *base)
{
    return base->EVTPEND;
}

/*!
 * @brief Clear the SDMA channel pending status of specific channels.
 *
 * @param base SDMA peripheral base address.
 * @param mask The pending status need to be cleared.
 */
static inline void SDMA_ClearChannelPendStatus(SDMAARM_Type *base, uint32_t mask)
{
    base->EVTPEND = mask;
}


/*!
 * @brief Set SDMA request source mapping channel.
 *
 * This function sets which channel will be triggered by the dma request source.
 *
 * @param base SDMA peripheral base address.
 * @param source SDMA dma request source number.
 * @param channelMask SDMA channel mask. 1 means channel 0, 2 means channel 1, 4 means channel 3. SDMA supports
 * an event trigger multi-channel. A channel can also be triggered by several source events.
 */
static inline void SDMA_SetSourceChannel(SDMAARM_Type *base, uint32_t source, uint32_t channelMask)
{
    base->CHNENBL[source] = channelMask;
}

/*!
 * @brief Set SDMA channel priority.
 *
 * This function sets the channel priority. The default value is 0 for all channels, priority 0 will prevents
 * channel from starting, so the priority must be set before start a channel.
 *
 * @param base SDMA peripheral base address.
 * @param channel SDMA channel number.
 * @param priority SDMA channel priority.
 */
static inline void SDMA_SetChannelPriority(SDMAARM_Type *base, uint32_t channel, uint8_t priority)
{
    base->SDMA_CHNPRI[channel] = priority;
}