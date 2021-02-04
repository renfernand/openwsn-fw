#ifndef __UART_H
#define __UART_H

/**
\addtogroup BSP
\{
\addtogroup uart
\{

\brief Cross-platform declaration "uart" bsp module.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, February 2012.
*/

#include "stdint.h"
#include "board.h"

//=========================== define ==========================================

#define XOFF            0x13
#define XON             0x11
#define XONXOFF_ESCAPE  0x12
#define XONXOFF_MASK    0x10

/*
 * define if the Debug using serial port will be through the UART0(default) or UART1
 */

//SENS_ITF UART1 PINS
#define OSENS_UART_BASE           UART1_BASE
#define OSENS_UART_ENABLE_BM      SYS_CTRL_PERIPH_UART1
#define OSENS_UART_BUS_BASE       GPIO_A_BASE
#define OSENS_UART_RXD_BASE       BSP_UART_BUS_BASE
#define OSENS_UART_RXD            GPIO_PIN_5      //!< PA5
#define OSENS_UART_TXD_BASE       BSP_UART_BUS_BASE
#define OSENS_UART_TXD            GPIO_PIN_4      //!< PA4
#define OSENS_MUX_UART_TXD        IOC_MUX_OUT_SEL_UART1_TXD
#define OSENS_MUX_UART_RXD        IOC_UARTRXD_UART1
#define OSENS_INT_UART            INT_UART1


//Define the DEBUG_UART
#define PIN_UART_RXD            GPIO_PIN_0 // PA0 is UART RX
#define PIN_UART_TXD            GPIO_PIN_1 // PA1 is UART TX

#define DBG_UART_BASE           UART0_BASE
#define DBG_UART_ENABLE_BM      SYS_CTRL_PERIPH_UART0
#define DBG_UART_BUS_BASE       GPIO_A_BASE
#define DBG_UART_RXD_BASE       BSP_UART_BUS_BASE
#define DBG_UART_RXD            PIN_UART_RXD      //!< PA0 is UART RX
#define DBG_UART_TXD_BASE       BSP_UART_BUS_BASE
#define DBG_UART_TXD            PIN_UART_TXD      //!< PA1 is UART TX
#define DBG_MUX_UART_TXD        IOC_MUX_OUT_SEL_UART0_TXD
#define DBG_MUX_UART_RXD        IOC_UARTRXD_UART0
#define DBG_INT_UART            INT_UART0

#if (DBG_USING_UART1 == 1)
#define DBG_UART1_BASE           UART1_BASE
#define DBG_UART1_ENABLE_BM      SYS_CTRL_PERIPH_UART1
#define DBG_UART1_BUS_BASE       GPIO_A_BASE
#define DBG_UART1_RXD_BASE       BSP_UART_BUS_BASE
#define DBG_UART1_RXD            GPIO_PIN_5      //!< PA5
#define DBG_UART1_TXD_BASE       BSP_UART_BUS_BASE
#define DBG_UART1_TXD            GPIO_PIN_4      //!< PA4
#define DBG_MUX_UART1_TXD        IOC_MUX_OUT_SEL_UART1_TXD
#define DBG_MUX_UART1_RXD        IOC_UARTRXD_UART1
#define DBG_INT_UART1            INT_UART1

#endif

//=========================== typedef =========================================

typedef enum {
   UART_EVENT_THRES,
   UART_EVENT_OVERFLOW,
} uart_event_t;

typedef void (*uart_tx_cbt)(void);
typedef uint8_t (*uart_rx_cbt)(void);

//=========================== variables =======================================

//=========================== prototypes ======================================

void    uart_init(void);
void    uart_setCallbacks(uart_tx_cbt txCb, uart_rx_cbt rxCb);
void    uart_enableInterrupts(void);
void    uart_disableInterrupts(void);
void    uart_clearRxInterrupts(void);
void    uart_clearTxInterrupts(void);
void    uart_setCTS(bool state);
void    uart_writeByte(uint8_t byteToWrite);
#ifdef FASTSIM
void    uart_writeCircularBuffer_FASTSIM(uint8_t* buffer, uint16_t* outputBufIdxR, uint16_t* outputBufIdxW);
#endif
uint8_t uart_readByte(void);

// interrupt handlers
kick_scheduler_t uart_tx_isr(void);
kick_scheduler_t uart_rx_isr(void);

#if 1 // (DBG_USING_UART1 == 1)
void    uart1_setCallbacks(uart_tx_cbt txCb, uart_rx_cbt rxCb);
void    uart1_init(void);
void    uart1_setCallbacks(uart_tx_cbt txCb, uart_rx_cbt rxCb);
void    uart1_enableInterrupts(void);
void    uart1_disableInterrupts(void);
void    uart1_clearRxInterrupts(void);
void    uart1_clearTxInterrupts(void);
void    uart1_writeByte(uint8_t byteToWrite);
uint8_t uart1_readByte(void);

void bspSpiInit(void);
#endif
/**
\}
\}
*/

#endif
