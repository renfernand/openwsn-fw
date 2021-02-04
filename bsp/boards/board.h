#ifndef __BOARD_H
#define __BOARD_H

/**
\addtogroup BSP
\{
\addtogroup board
\{

\brief Cross-platform declaration "board" bsp module.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, February 2012.
*/

#include "board_info.h"
#include "toolchain_defs.h"

//=========================== define ==========================================

typedef enum {
   DO_NOT_KICK_SCHEDULER = 0,
   KICK_SCHEDULER        = 1,
} kick_scheduler_t;

//enable the log using UART0 - cannot be used for DAG ROOT
#define DEBUG_VIA_SERIAL 0

#define OSENS_UART_INT  0


/*
 * HABILITA OS SENSORES JA UTILIZADOS NESTA PLACA CC2538EM
 * MYLINKXS_LIGHT_CONTROL - Renato Fernandes - sem necessidade de placa sensora.
 * MYLINKXS_REMOTE_CONTROL - Renato Fernandes - com arduino acionando controle remoto
 * OSENS - Original do marcelo Barros com placa remota Freescale
 *
 **/
//#define MYLINKXS_SENSORS 0

//*****************************************************************************
//
// SPI defines (Common for LCD, SD reader and accelerometer)
//*****************************************************************************
#if (USE_SPI_INTERFACE == 1)
#define  USE_SPI_INTERRUPT 0

#define BSP_SPI_SSI_BASE        SSI0_BASE
//! Bitmask to enable SSI module.
#define BSP_SPI_SSI_ENABLE_BM   SYS_CTRL_PERIPH_SSI0
#define BSP_SPI_BUS_BASE        GPIO_A_BASE
#define BSP_SPI_SCK             GPIO_PIN_2      //!< PA2
#define BSP_SPI_FSS             GPIO_PIN_3      //!< PA3
#define BSP_SPI_MOSI            GPIO_PIN_4      //!< PA4
#define BSP_SPI_MISO            GPIO_PIN_5      //!< PA5

//! Default SPI clock speed. 8 MHz is supported by all SmartRF06EB peripherals.
#define BSP_SPI_CLK_SPD         1000000UL


#if SENSOR_ACCEL
// Board accelerometer defines
#define BSP_ACC_PWR_BASE        GPIO_D_BASE
#define BSP_ACC_PWR             GPIO_PIN_4      //!< PD4
#define BSP_ACC_INT_BASE        GPIO_D_BASE
#define BSP_ACC_INT             GPIO_PIN_2      //!< PD2
#define BSP_ACC_INT1_BASE       BSP_ACC_INT_BASE
#define BSP_ACC_INT1            BSP_ACC_INT     //!< ACC_INT1 == ACC_INT
#define BSP_ACC_INT2_BASE       GPIO_D_BASE
#define BSP_ACC_INT2            GPIO_PIN_2      //!< PD1
#define BSP_ACC_CS_BASE         GPIO_D_BASE
#define BSP_ACC_CS              GPIO_PIN_5      //!< PD5
#define BSP_ACC_SCK_BASE        BSP_SPI_BUS_BASE
#define BSP_ACC_SCK             BSP_SPI_SCK     //!< PA2
#define BSP_ACC_MOSI_BASE       BSP_SPI_BUS_BASE
#define BSP_ACC_MOSI            BSP_SPI_MOSI    //!< PA4
#define BSP_ACC_MISO_BASE       BSP_SPI_BUS_BASE
#define BSP_ACC_MISO            BSP_SPI_MISO    //!< PA5
#endif


#else
//*****************************************************************************
//
// UART interface
//*****************************************************************************

// UART backchannel defines
#if 0
#define BSP_UART_BASE           UART0_BASE
#define BSP_UART_ENABLE_BM      SYS_CTRL_PERIPH_UART0
#define BSP_UART_BUS_BASE       GPIO_A_BASE
#define BSP_UART_RXD_BASE       BSP_UART_BUS_BASE
#define BSP_UART_RXD            GPIO_PIN_0      //!< PA0
#define BSP_UART_TXD_BASE       BSP_UART_BUS_BASE
#define BSP_UART_TXD            GPIO_PIN_1      //!< PA1
#define BSP_UART_CTS_BASE       GPIO_B_BASE
#define BSP_UART_CTS            GPIO_PIN_0      //!< PB0
#define BSP_UART_RTS_BASE       GPIO_D_BASE
#define BSP_UART_RTS            GPIO_PIN_3      //!< PD3
#define BSP_UART_INT_BM         0xF0            //!< Interrupts handled by bsp uart
#define BSP_INT_UART            INT_UART0
#define BSP_MUX_UART_TXD        IOC_MUX_OUT_SEL_UART0_TXD
#define BSP_MUX_UART_RXD        IOC_UARTRXD_UART0
#else
#define BSP_UART_BASE           UART1_BASE
#define BSP_UART_ENABLE_BM      SYS_CTRL_PERIPH_UART1
#define BSP_UART_BUS_BASE       GPIO_A_BASE
#define BSP_UART_RXD_BASE       BSP_UART_BUS_BASE
#define BSP_UART_RXD            GPIO_PIN_5      //!< PA5
#define BSP_UART_TXD_BASE       BSP_UART_BUS_BASE
#define BSP_UART_TXD            GPIO_PIN_4      //!< PA4
#define BSP_UART_CTS_BASE       GPIO_B_BASE
#define BSP_UART_CTS            GPIO_PIN_0      //!< PB0
#define BSP_UART_RTS_BASE       GPIO_D_BASE
#define BSP_UART_RTS            GPIO_PIN_3      //!< PD3
#define BSP_UART_INT_BM         0xF0            //!< Interrupts handled by bsp uart
#define BSP_INT_UART            INT_UART1
#define BSP_MUX_UART_TXD        IOC_MUX_OUT_SEL_UART1_TXD
#define BSP_MUX_UART_RXD        IOC_UARTRXD_UART1
#endif

#endif

//*****************************************************************************
//
// SPI defines (Common for LCD, SD reader and accelerometer)
//*****************************************************************************
#if (USE_SPI_INTERFACE == 1)
#define  USE_SPI_INTERRUPT 0

#define BSP_SPI_SSI_BASE        SSI0_BASE
//! Bitmask to enable SSI module.
#define BSP_SPI_SSI_ENABLE_BM   SYS_CTRL_PERIPH_SSI0
#define BSP_SPI_BUS_BASE        GPIO_A_BASE
#define BSP_SPI_SCK             GPIO_PIN_2      //!< PA2
#define BSP_SPI_FSS             GPIO_PIN_3      //!< PA3
#define BSP_SPI_MOSI            GPIO_PIN_4      //!< PA4
#define BSP_SPI_MISO            GPIO_PIN_5      //!< PA5

//! Default SPI clock speed. 8 MHz is supported by all SmartRF06EB peripherals.
#define BSP_SPI_CLK_SPD         1000000UL

#endif


#if (SENSOR_ACCEL == 1)
// Board accelerometer defines
#define BSP_ACC_PWR_BASE        GPIO_D_BASE
#define BSP_ACC_PWR             GPIO_PIN_4      //!< PD4
#define BSP_ACC_INT_BASE        GPIO_D_BASE
#define BSP_ACC_INT             GPIO_PIN_2      //!< PD2
#define BSP_ACC_INT1_BASE       BSP_ACC_INT_BASE
#define BSP_ACC_INT1            BSP_ACC_INT     //!< ACC_INT1 == ACC_INT
#define BSP_ACC_INT2_BASE       GPIO_D_BASE
#define BSP_ACC_INT2            GPIO_PIN_2      //!< PD1
#define BSP_ACC_CS_BASE         GPIO_D_BASE
#define BSP_ACC_CS              GPIO_PIN_5      //!< PD5
#define BSP_ACC_SCK_BASE        BSP_SPI_BUS_BASE
#define BSP_ACC_SCK             BSP_SPI_SCK     //!< PA2
#define BSP_ACC_MOSI_BASE       BSP_SPI_BUS_BASE
#define BSP_ACC_MOSI            BSP_SPI_MOSI    //!< PA4
#define BSP_ACC_MISO_BASE       BSP_SPI_BUS_BASE
#define BSP_ACC_MISO            BSP_SPI_MISO    //!< PA5
#endif
//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

void board_init(void);
void board_sleep(void);
void board_reset(void);

/**
\}
\}
*/

#endif
