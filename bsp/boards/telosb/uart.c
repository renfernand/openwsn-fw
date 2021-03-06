/**
\brief TelosB-specific definition of the "uart" bsp module.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, February 2012.
*/

#include "msp430f1611.h"
#include "uart.h"
#include "board.h"

//=========================== defines =========================================

//=========================== variables =======================================

typedef struct {
    uart_tx_cbt txCb;
    uart_rx_cbt rxCb;
    bool        fXonXoffEscaping;
    uint8_t     xonXoffEscapedByte;
} uart_vars_t;

uart_vars_t uart_vars;

//=========================== prototypes ======================================

//=========================== public ==========================================

void uart_init(void) {
    
    memset(&uart_vars,0,sizeof(uart_vars_t));
    
    P3SEL                    |=  0xc0;            // P3.6,7 = UART1TX/RX
    
    UCTL1                     =  SWRST;           // hold UART1 module in reset
    UCTL1                    |=  CHAR;            // 8-bit character
    
    /*
    //   9600 baud, clocked from 32kHz ACLK
    UTCTL1                   |=  SSEL0;           // clocking from ACLK
    UBR01                     =  0x03;            // 32768/9600 = 3.41
    UBR11                     =  0x00;            //
    UMCTL1                    =  0x4A;            // modulation
    */
    
    // 115200 baud, clocked from 4.8MHz SMCLK
    UTCTL1                   |=  SSEL1;           // clocking from SMCLK
    UBR01                     =  41;              // 4.8MHz/115200 - 41.66
    UBR11                     =  0x00;            //
    UMCTL1                    =  0x4A;            // modulation
    
    ME2                      |=  UTXE1 + URXE1;   // enable UART1 TX/RX
    UCTL1                    &= ~SWRST;           // clear UART1 reset bit
}

void uart_setCallbacks(uart_tx_cbt txCb, uart_rx_cbt rxCb) {
    uart_vars.txCb = txCb;
    uart_vars.rxCb = rxCb;
}

void    uart_enableInterrupts(void) {
    IE2 |=  (URXIE1 | UTXIE1);  
}

void    uart_disableInterrupts(void) {
    IE2 &= ~(URXIE1 | UTXIE1);
}

void    uart_clearRxInterrupts(void) {
    IFG2   &= ~URXIFG1;
}

void    uart_clearTxInterrupts(void) {
    IFG2   &= ~UTXIFG1;
}

void uart_setCTS(bool state) {
    if (state==0x01) {
        U1TXBUF = XON;
    } else {
        U1TXBUF = XOFF;
    }
}

void uart_writeByte(uint8_t byteToWrite){
    if (byteToWrite==XON || byteToWrite==XOFF || byteToWrite==XONXOFF_ESCAPE) {
        uart_vars.fXonXoffEscaping     = 0x01;
        uart_vars.xonXoffEscapedByte   = byteToWrite;
        U1TXBUF = XONXOFF_ESCAPE;
    } else {
        U1TXBUF = byteToWrite;
    }
}

uint8_t uart_readByte(void) {
    return U1RXBUF;
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================

kick_scheduler_t uart_tx_isr(void) {
    uart_clearTxInterrupts(); // TODO: do not clear, but disable when done
    if (uart_vars.fXonXoffEscaping==0x01) {
        uart_vars.fXonXoffEscaping = 0x00;
        U1TXBUF = uart_vars.xonXoffEscapedByte^XONXOFF_MASK;
    } else {
        uart_vars.txCb();
    }
    return DO_NOT_KICK_SCHEDULER;
}

kick_scheduler_t uart_rx_isr(void) {
   uart_clearRxInterrupts(); // TODO: do not clear, but disable when done
   return (kick_scheduler_t)uart_vars.rxCb();
}
