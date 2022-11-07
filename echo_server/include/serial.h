#pragma once


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