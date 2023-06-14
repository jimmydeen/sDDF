#pragma once

/* Subset of the sDMA API Provided by Freescale */

#include <stdint.h>

#define UART_RX_DMA_CHANNEL       1U
#define UART_TX_DMA_CHANNEL       2U
#define UART_RX_DMA_REQUEST       (28)
#define UART_TX_DMA_REQUEST       (29)
#define ECHO_BUFFER_LENGTH 8

#define __COMPILER_BARRIER()      asm volatile("":::"memory")

#define NVIC_OFFSET 0x0100

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

/* following defines should be used for structure members */
#define     __IM     volatile const      /*! Defines 'read only' structure member permissions */
#define     __OM     volatile            /*! Defines 'write only' structure member permissions */
#define     __IOM    volatile            /*! Defines 'read / write' structure member permissions */

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
 * @brief SDMA context structure for each channel. This structure can be load into SDMA core, with this structure, SDMA
 * scripts can start work.
 */
typedef struct _sdma_context_data
{
    uint32_t PC : 14;
    uint32_t unused1 : 1;
    uint32_t T : 1;
    uint32_t RPC : 14;
    uint32_t unused0 : 1;
    uint32_t SF : 1;
    uint32_t SPC : 14;
    uint32_t unused2 : 1;
    uint32_t DF : 1;
    uint32_t EPC : 14;
    uint32_t LM : 2;
    uint32_t GeneralReg[8]; /*!< 8 general regsiters used for SDMA RISC core */
    uint32_t MDA;
    uint32_t MSA;
    uint32_t MS;
    uint32_t MD;
    uint32_t PDA;
    uint32_t PSA;
    uint32_t PS;
    uint32_t PD;
    uint32_t CA;
    uint32_t CS;
    uint32_t DDA;
    uint32_t DSA;
    uint32_t DS;
    uint32_t DD;
    uint32_t Scratch0;
    uint32_t Scratch1;
    uint32_t Scratch2;
    uint32_t Scratch3;
    uint32_t Scratch4;
    uint32_t Scratch5;
    uint32_t Scratch6;
    uint32_t Scratch7;
} sdma_context_data_t;

/*! @brief Define callback function for SDMA. */
typedef void (*sdma_callback)(struct _sdma_handle *handle, void *userData, bool transferDone, uint32_t bdIndex);

/*! @brief SDMA transfer handle structure */
typedef struct _sdma_handle
{
    sdma_callback callback;           /*!< Callback function for major count exhausted. */
    void *userData;                   /*!< Callback function parameter. */
    SDMAARM_Type *base;               /*!< SDMA peripheral base address. */
    sdma_buffer_descriptor_t *BDPool; /*!< Pointer to memory stored BD arrays. */
    uint32_t bdCount;                 /*!< How many buffer descriptor   */
    uint32_t bdIndex;                 /*!< How many buffer descriptor   */
    uint32_t eventSource;             /*!< Event source count for the channel */
    uint32_t eventSource1;            /*!< Event source 1 count for the channel */
    sdma_context_data_t *context;     /*!< Channel context to exectute in SDMA */
    uint8_t channel;                  /*!< SDMA channel number. */
    uint8_t priority;                 /*!< SDMA channel priority */
    uint8_t flags;                    /*!< The status of the current channel. */
} sdma_handle_t;

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


/* NVIC STUFF - MOVE THIS OVER EVENTUALLY */

/**
  \brief  Structure type to access the Nested Vectored Interrupt Controller (NVIC).
 */
typedef struct
{
  __IOM uint32_t ISER[8U];               /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
        uint32_t RESERVED0[24U];
  __IOM uint32_t ICER[8U];               /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
        uint32_t RESERVED1[24U];
  __IOM uint32_t ISPR[8U];               /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
        uint32_t RESERVED2[24U];
  __IOM uint32_t ICPR[8U];               /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
        uint32_t RESERVED3[24U];
  __IOM uint32_t IABR[8U];               /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
        uint32_t RESERVED4[56U];
  __IOM uint8_t  IP[240U];               /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
        uint32_t RESERVED5[644U];
  __OM  uint32_t STIR;                   /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
}  NVIC_Type;

typedef enum IRQn {
  /* Auxiliary constants */
  NotAvail_IRQn                = -128,             /**< Not available device specific interrupt */

  /* Core interrupts */
  NonMaskableInt_IRQn          = -14,              /**< Non Maskable Interrupt */
  HardFault_IRQn               = -13,              /**< Cortex-M4 SV Hard Fault Interrupt */
  MemoryManagement_IRQn        = -12,              /**< Cortex-M4 Memory Management Interrupt */
  BusFault_IRQn                = -11,              /**< Cortex-M4 Bus Fault Interrupt */
  UsageFault_IRQn              = -10,              /**< Cortex-M4 Usage Fault Interrupt */
  SVCall_IRQn                  = -5,               /**< Cortex-M4 SV Call Interrupt */
  DebugMonitor_IRQn            = -4,               /**< Cortex-M4 Debug Monitor Interrupt */
  PendSV_IRQn                  = -2,               /**< Cortex-M4 Pend SV Interrupt */
  SysTick_IRQn                 = -1,               /**< Cortex-M4 System Tick Interrupt */

  /* Device specific interrupts */
  GPR_IRQ_IRQn                 = 0,                /**< GPR Interrupt. Used to notify cores on exception condition while boot. */
  DAP_IRQn                     = 1,                /**< DAP Interrupt */
  SDMA1_IRQn                   = 2,                /**< AND of all 48 SDMA1 interrupts (events) from all the channels */
  GPU3D_IRQn                   = 3,                /**< GPU3D Interrupt */
  SNVS_IRQn                    = 4,                /**< ON-OFF button press shorter than 5 seconds (pulse event) */
  LCDIF_IRQn                   = 5,                /**< LCDIF Interrupt */
  SPDIF1_IRQn                  = 6,                /**< SPDIF1 RZX/TX Interrupt */
  VPU_G1_IRQn                  = 7,                /**< VPU G1 Decoder Interrupt */
  VPU_G2_IRQn                  = 8,                /**< VPU G2 Decoder Interrupt */
  QOS_IRQn                     = 9,                /**< QOS interrupt */
  WDOG3_IRQn                   = 10,               /**< Watchdog Timer reset */
  HS_CP1_IRQn                  = 11,               /**< HS Interrupt Request */
  APBHDMA_IRQn                 = 12,               /**< GPMI operation channel 0-3 description complete interrupt */
  Reserved29_IRQn              = 13,               /**< Reserved */
  BCH_IRQn                     = 14,               /**< BCH operation complete interrupt */
  GPMI_IRQn                    = 15,               /**< GPMI operation TIMEOUT ERROR interrupt */
  CSI1_IRQn                    = 16,               /**< CSI Interrupt */
  MIPI_CSI1_IRQn               = 17,               /**< MIPI CSI Interrupt */
  MIPI_DSI_IRQn                = 18,               /**< MIPI DSI Interrupt */
  SNVS_Consolidated_IRQn       = 19,               /**< SRTC Consolidated Interrupt. Non TZ. */
  SNVS_Security_IRQn           = 20,               /**< SRTC Security Interrupt. TZ. */
  CSU_IRQn                     = 21,               /**< CSU Interrupt Request. Indicates to the processor that one or more alarm inputs were asserted. */
  USDHC1_IRQn                  = 22,               /**< uSDHC1 Enhanced SDHC Interrupt Request */
  USDHC2_IRQn                  = 23,               /**< uSDHC2 Enhanced SDHC Interrupt Request */
  USDHC3_IRQn                  = 24,               /**< uSDHC3 Enhanced SDHC Interrupt Request */
  GPU2D_IRQn                   = 25,               /**< GPU2D Interrupt */
  UART1_IRQn                   = 26,               /**< UART-1 ORed interrupt */
  UART2_IRQn                   = 27,               /**< UART-2 ORed interrupt */
  UART3_IRQn                   = 28,               /**< UART-3 ORed interrupt */
  UART4_IRQn                   = 29,               /**< UART-4 ORed interrupt */
  VPU_H1_IRQn                  = 30,               /**< VPU H1 Encoder Interrupt */
  ECSPI1_IRQn                  = 31,               /**< ECSPI1 interrupt request line to the core. */
  ECSPI2_IRQn                  = 32,               /**< ECSPI2 interrupt request line to the core. */
  ECSPI3_IRQn                  = 33,               /**< ECSPI3 interrupt request line to the core. */
  SDMA3_IRQn                   = 34,               /**< AND of all 48 SDMA3 interrupts (events) from all the channels */
  I2C1_IRQn                    = 35,               /**< I2C-1 Interrupt */
  I2C2_IRQn                    = 36,               /**< I2C-2 Interrupt */
  I2C3_IRQn                    = 37,               /**< I2C-3 Interrupt */
  I2C4_IRQn                    = 38,               /**< I2C-4 Interrupt */
  RDC_IRQn                     = 39,               /**< RDC interrupt */
  USB1_IRQn                    = 40,               /**< USB1 Interrupt */
  USB2_IRQn                    = 41,               /**< USB1 Interrupt */
  Reserved58_IRQn              = 42,               /**< Reserved interrupt */
  Reserved59_IRQn              = 43,               /**< Reserved interrupt */
  PDM_HWVAD_EVENT_IRQn         = 44,               /**< Digital Microphone interface voice activity detector event interrupt */
  PDM_HWVAD_ERROR_IRQn         = 45,               /**< Digital Microphone interface voice activity detector error interrupt */
  GPT6_IRQn                    = 46,               /**< OR of GPT Rollover interrupt line, Input Capture 1 and 2 lines, Output Compare 1, 2, and 3 Interrupt lines */
  SCTR_IRQ0_IRQn               = 47,               /**< System Counter Interrupt 0 */
  SCTR_IRQ1_IRQn               = 48,               /**< System Counter Interrupt 1 */
  TEMPMON_LOW_IRQn             = 49,               /**< TempSensor (Temperature low alarm). */
  I2S3_IRQn                    = 50,               /**< SAI3 Receive / Transmit Interrupt */
  GPT5_IRQn                    = 51,               /**< OR of GPT Rollover interrupt line, Input Capture 1 and 2 lines, Output Compare 1, 2, and 3 Interrupt lines */
  GPT4_IRQn                    = 52,               /**< OR of GPT Rollover interrupt line, Input Capture 1 and 2 lines, Output Compare 1, 2, and 3 Interrupt lines */
  GPT3_IRQn                    = 53,               /**< OR of GPT Rollover interrupt line, Input Capture 1 and 2 lines, Output Compare 1, 2, and 3 Interrupt lines */
  GPT2_IRQn                    = 54,               /**< OR of GPT Rollover interrupt line, Input Capture 1 and 2 lines, Output Compare 1, 2, and 3 Interrupt lines */
  GPT1_IRQn                    = 55,               /**< OR of GPT Rollover interrupt line, Input Capture 1 and 2 lines, Output Compare 1, 2, and 3 Interrupt lines */
  GPIO1_INT7_IRQn              = 56,               /**< Active HIGH Interrupt from INT7 from GPIO */
  GPIO1_INT6_IRQn              = 57,               /**< Active HIGH Interrupt from INT6 from GPIO */
  GPIO1_INT5_IRQn              = 58,               /**< Active HIGH Interrupt from INT5 from GPIO */
  GPIO1_INT4_IRQn              = 59,               /**< Active HIGH Interrupt from INT4 from GPIO */
  GPIO1_INT3_IRQn              = 60,               /**< Active HIGH Interrupt from INT3 from GPIO */
  GPIO1_INT2_IRQn              = 61,               /**< Active HIGH Interrupt from INT2 from GPIO */
  GPIO1_INT1_IRQn              = 62,               /**< Active HIGH Interrupt from INT1 from GPIO */
  GPIO1_INT0_IRQn              = 63,               /**< Active HIGH Interrupt from INT0 from GPIO */
  GPIO1_Combined_0_15_IRQn     = 64,               /**< Combined interrupt indication for GPIO1 signal 0 throughout 15 */
  GPIO1_Combined_16_31_IRQn    = 65,               /**< Combined interrupt indication for GPIO1 signal 16 throughout 31 */
  GPIO2_Combined_0_15_IRQn     = 66,               /**< Combined interrupt indication for GPIO2 signal 0 throughout 15 */
  GPIO2_Combined_16_31_IRQn    = 67,               /**< Combined interrupt indication for GPIO2 signal 16 throughout 31 */
  GPIO3_Combined_0_15_IRQn     = 68,               /**< Combined interrupt indication for GPIO3 signal 0 throughout 15 */
  GPIO3_Combined_16_31_IRQn    = 69,               /**< Combined interrupt indication for GPIO3 signal 16 throughout 31 */
  GPIO4_Combined_0_15_IRQn     = 70,               /**< Combined interrupt indication for GPIO4 signal 0 throughout 15 */
  GPIO4_Combined_16_31_IRQn    = 71,               /**< Combined interrupt indication for GPIO4 signal 16 throughout 31 */
  GPIO5_Combined_0_15_IRQn     = 72,               /**< Combined interrupt indication for GPIO5 signal 0 throughout 15 */
  GPIO5_Combined_16_31_IRQn    = 73,               /**< Combined interrupt indication for GPIO5 signal 16 throughout 31 */
  Reserved90_IRQn              = 74,               /**< Reserved interrupt */
  Reserved91_IRQn              = 75,               /**< Reserved interrupt */
  Reserved92_IRQn              = 76,               /**< Reserved interrupt */
  Reserved93_IRQn              = 77,               /**< Reserved interrupt */
  WDOG1_IRQn                   = 78,               /**< Watchdog Timer reset */
  WDOG2_IRQn                   = 79,               /**< Watchdog Timer reset */
  Reserved96_IRQn              = 80,               /**< Reserved interrupt */
  PWM1_IRQn                    = 81,               /**< Cumulative interrupt line. OR of Rollover Interrupt line, Compare Interrupt line and FIFO Waterlevel crossing interrupt line. */
  PWM2_IRQn                    = 82,               /**< Cumulative interrupt line. OR of Rollover Interrupt line, Compare Interrupt line and FIFO Waterlevel crossing interrupt line. */
  PWM3_IRQn                    = 83,               /**< Cumulative interrupt line. OR of Rollover Interrupt line, Compare Interrupt line and FIFO Waterlevel crossing interrupt line. */
  PWM4_IRQn                    = 84,               /**< Cumulative interrupt line. OR of Rollover Interrupt line, Compare Interrupt line and FIFO Waterlevel crossing interrupt line. */
  CCM_IRQ1_IRQn                = 85,               /**< CCM Interrupt Request 1 */
  CCM_IRQ2_IRQn                = 86,               /**< CCM Interrupt Request 2 */
  GPC_IRQn                     = 87,               /**< GPC Interrupt Request 1 */
  MU_A53_IRQn                  = 88,               /**< Interrupt to A53 */
  SRC_IRQn                     = 89,               /**< SRC interrupt request */
  I2S56_IRQn                   = 90,               /**< SAI5/6 Receive / Transmit Interrupt */
  RTIC_IRQn                    = 91,               /**< RTIC Interrupt */
  CPU_PerformanceUnit_IRQn     = 92,               /**< Performance Unit Interrupts from Cheetah (interrnally: PMUIRQ[n] */
  CPU_CTI_Trigger_IRQn         = 93,               /**< CTI trigger outputs (internal: nCTIIRQ[n] */
  SRC_Combined_IRQn            = 94,               /**< Combined CPU wdog interrupts (4x) out of SRC. */
  I2S1_IRQn                    = 95,               /**< SAI1 Receive / Transmit Interrupt */
  I2S2_IRQn                    = 96,               /**< SAI2 Receive / Transmit Interrupt */
  MU_M4_IRQn                   = 97,               /**< Interrupt to M4 */
  DDR_PerformanceMonitor_IRQn  = 98,               /**< ddr Interrupt for performance monitor */
  DDR_IRQn                     = 99,               /**< ddr Interrupt */
  Reserved116_IRQn             = 100,              /**< Reserved interrupt */
  CPU_Error_AXI_IRQn           = 101,              /**< CPU Error indicator for AXI transaction with a write response error condition */
  CPU_Error_L2RAM_IRQn         = 102,              /**< CPU Error indicator for L2 RAM double-bit ECC error */
  SDMA2_IRQn                   = 103,              /**< AND of all 48 SDMA2 interrupts (events) from all the channels */
  SJC_IRQn                     = 104,              /**< Interrupt triggered by SJC register */
  CAAM_IRQ0_IRQn               = 105,              /**< CAAM interrupt queue for JQ */
  CAAM_IRQ1_IRQn               = 106,              /**< CAAM interrupt queue for JQ */
  QSPI_IRQn                    = 107,              /**< QSPI Interrupt */
  TZASC_IRQn                   = 108,              /**< TZASC (PL380) interrupt */
  PDM_EVENT_IRQn               = 109,              /**< Digital Microphone interface interrupt */
  PDM_ERROR_IRQn               = 110,              /**< Digital Microphone interface error interrupt */
  Reserved127_IRQn             = 111,              /**< Reserved interrupt */
  PERFMON1_IRQn                = 112,              /**< General Interrupt */
  PERFMON2_IRQn                = 113,              /**< General Interrupt */
  CAAM_IRQ2_IRQn               = 114,              /**< CAAM interrupt queue for JQ */
  CAAM_ERROR_IRQn              = 115,              /**< Recoverable error interrupt */
  HS_CP0_IRQn                  = 116,              /**< HS Interrupt Request */
  Reserved133_IRQn             = 117,              /**< Reserved interrupt */
  ENET1_MAC0_Rx_Tx_Done1_IRQn  = 118,              /**< MAC 0 Receive / Trasmit Frame / Buffer Done */
  ENET1_MAC0_Rx_Tx_Done2_IRQn  = 119,              /**< MAC 0 Receive / Trasmit Frame / Buffer Done */
  ENET1_IRQn                   = 120,              /**< MAC 0 IRQ */
  ENET1_1588_Timer_IRQn        = 121,              /**< MAC 0 1588 Timer Interrupt - synchronous */
  PCIE_CTRL1_IRQ0_IRQn         = 122,              /**< Coming from GLUE logic, of set / reset FF, driven by PCIE signals. */
  PCIE_CTRL1_IRQ1_IRQn         = 123,              /**< Coming from GLUE logic, of set / reset FF, driven by PCIE signals. */
  PCIE_CTRL1_IRQ2_IRQn         = 124,              /**< Coming from GLUE logic, of set / reset FF, driven by PCIE signals. */
  PCIE_CTRL1_IRQ3_IRQn         = 125,              /**< Coming from GLUE logic, of set / reset FF, driven by PCIE signals. */
  Reserved142_IRQn             = 126,              /**< Reserved */
  PCIE_CTRL1_IRQn              = 127               /**< Channels [63:32] interrupts requests */
} IRQn_Type;

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