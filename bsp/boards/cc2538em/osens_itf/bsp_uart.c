#include "board.h"
#include "stdio.h"
#include <string.h>
#include "hw_ioc.h"             // Access to IOC register defines
#include "hw_ssi.h"             // Access to SSI register defines
#include "hw_sys_ctrl.h"        // Clocking control
#include "hw_memmap.h"
#include "ioc.h"                // Access to driverlib ioc fns
#include "gpio.h"               // Access to driverlib gpio fns
#include "osens.h"
#include "osens_itf.h"
#include "opendefs.h"
#include "opentimers.h"
#include "scheduler.h"
#include "board.h"
#include "sys_ctrl.h"
#include "osens_itf_mote.h"
#include "hw_ints.h"
#include  "uarthal.h"
#if USE_SPI_INTERFACE
#include "ssi.h"
#if SENSOR_ACCEL
#include "acc_bma250.h"
#endif
#else
#include "uart.h"
#include "uarthal.h"
#endif
#include "leds.h"

extern osens_mote_sm_state_t sm_state;


uint8_t rx_frame[OSENS_MAX_FRAME_SIZE];
uint8_t tx_frame[OSENS_MAX_FRAME_SIZE];
static tBuBuf sBuBufRx;

#define BSP_UART_ISR 1

#if (USE_SPI_INTERFACE == 0)
static tBuBuf sBuBufTx;
static void buBufFlush(tBuBuf *psBuf);
static void buBufPopByte(void);
static void buBufPushByte(void);

/******************************************************************************
* FUNCTIONS
*/
/**************************************************************************//**
* @brief    This function returns the number of bytes available in the BSP UART
*           TX buffer.
*
* @return   Returns the free space in bytes of the BSP UART TX buffer.
******************************************************************************/
uint16_t
bspUartTxSpaceAvail(void)
{
    uint16_t ui16Space;
    bool bIntDisabled = IntMasterDisable();
    ui16Space = BU_GET_FREE_SPACE(&sBuBufTx);
    if(!bIntDisabled)
    {
        IntMasterEnable();
    }
    return ui16Space;
}
/**************************************************************************//**
* @brief    This function pops a byte from the BSP UART TX buffer to the UART
*           TX FIFO. The function handles TX buffer tail wrap-around, but does
*           not handle buffer underrun. The function modifies volatile
*           variables and should only be called when interrupts are disabled.
*
* @return   None
******************************************************************************/
#if 0
static void
buBufPopByte(void)
{
    //
    // Send byte to TX FIFO (wait for space to become available)
    //
    UARTCharPut(BSP_OSENS_BASE, (*sBuBufTx.pui8Tail++));

    //
    // Update byte count
    //
    sBuBufTx.ui16NumBytes--;

    //
    // Manage ringbuffer wrap-around
    //
    if(sBuBufTx.pui8Tail == sBuBufTx.pui8End)
    {
        sBuBufTx.pui8Tail = sBuBufTx.pui8Start;
    }
}
#endif
/**************************************************************************//**
* @brief    This function pushes a byte from the UART RX FIFO to the BSP UART
*           RX buffer. The function handles RX buffer wrap-around, but does not
*           handle RX buffer overflow. The function should only be called if
*           there is data in the UART RX FIFO. It modifies volatile variables
*           and should only be called when interrupts are disabled.
*
* @brief    None
******************************************************************************/
#if 1
static void
buBufPushByte(void)
{
    //
    // Push byte from RX FIFO to buffer
    //
    *sBuBufRx.pui8Head++ = UARTCharGetNonBlocking(BSP_UART_BASE);

    //
    // Update byte count
    //
    sBuBufRx.ui16NumBytes++;

    //
    // Manage wrap-around
    //
    if(sBuBufRx.pui8Head == sBuBufRx.pui8End)
    {
        sBuBufRx.pui8Head = sBuBufRx.pui8Start;
    }
}
#endif
/**************************************************************************
* @brief    This function flushes the ringbuffer control structure specified
*           by \e psBuf.
*
* @param    psBuf       is a pointer to a \e tBuBuf ringbuffer structure.
*
* @return   None
******************************************************************************/
static void
buBufFlush(tBuBuf *psBuf)
{
    //
    // Start of critical section
    //
    bool bIntDisabled = IntMasterDisable();

    psBuf->pui8Head = psBuf->pui8Start;
    psBuf->pui8Tail = psBuf->pui8Start;
    psBuf->ui16NumBytes = 0;

    //
    // Return to previous interrupt state
    //
    if(!bIntDisabled)
    {
        IntMasterEnable();
    }
}


/**************************************************************************
* @brief    This function initializes buffers used by BSP UART module.
*
* @param    pui8TxBuf       is a pointer to the TX buffer.
* @param    ui16TxByteAlloc is the size of the TX buffer in bytes.
* @param    pui8RxBuf       is a pointer to the RX buffer.
* @param    ui16RxByteAlloc is the size of the RX buffer in bytes.
*
* @return   Returns BSP_UART_SUCCESS on success.
* @return   Returns BSP_UART_ERROR on configuration error.
******************************************************************************/
uint8_t bspUartBufInit(uint8_t *pui8TxBuf, uint16_t ui16TxByteAlloc,
               uint8_t *pui8RxBuf, uint16_t ui16RxByteAlloc)
{
    //
    // Check input arguments
    //
    if((pui8RxBuf == 0) || (ui16RxByteAlloc == 0) ||                          \
            (pui8TxBuf == 0) || (ui16TxByteAlloc == 0))
    {
        return BSP_UART_ERROR;
    }

    //
    // Store pointers
    //
    sBuBufTx.pui8Start = pui8TxBuf;
    sBuBufTx.pui8End = pui8TxBuf + ui16TxByteAlloc;
    sBuBufRx.pui8Start = pui8RxBuf;
    sBuBufRx.pui8End = pui8RxBuf + ui16RxByteAlloc;

    //
    // Reset control structure
    //
    buBufFlush(&sBuBufTx);
    buBufFlush(&sBuBufRx);

    //
    // Return status
    //
    return BSP_UART_SUCCESS;
}

void bspUartInit(void)
{
	// reset local variables
	// memset(&uart_vars,0,sizeof(uart_vars_t));
	//bspUartBufInit(tx_frame,sizeof(tx_frame),rx_frame,sizeof(tx_frame));

	//SysCtrlPeripheralEnable(BSP_UART_ENABLE_BM);

	UARTDisable(BSP_UART_BASE);

	UARTIntDisable(BSP_UART_BASE, 0x1FFF);

	UARTClockSourceSet(BSP_UART_BASE, UART_CLOCK_PIOSC);

	//
	// Map UART signals to the correct GPIO pins and configure them as
	// hardware controlled.
	//
	IOCPinConfigPeriphOutput(BSP_UART_BUS_BASE, BSP_UART_TXD, BSP_MUX_UART_TXD);
	GPIOPinTypeUARTOutput(BSP_UART_BUS_BASE, BSP_UART_TXD);
	IOCPinConfigPeriphInput(BSP_UART_BUS_BASE, BSP_UART_RXD, BSP_MUX_UART_RXD);
	GPIOPinTypeUARTInput(BSP_UART_BUS_BASE, BSP_UART_RXD);

	//
	// Configure the UART for 115,200, 8-N-1 operation.
	// This function uses SysCtrlClockGet() to get the system clock
	// frequency.  This could be also be a variable or hard coded value
	// instead of a function call.
	//
	UARTConfigSetExpClk(BSP_UART_BASE, SysCtrlClockGet(), 9600,
						(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
						 UART_CONFIG_PAR_NONE));
	UARTEnable(BSP_UART_BASE);

#ifdef BSP_UART_ISR
	UARTFIFODisable(BSP_UART_BASE);

	UARTIntRegister(BSP_UART_BASE, &bspUartIsrHandler);

	// Raise interrupt at end of tx (not by fifo)
	UARTTxIntModeSet(BSP_UART_BASE, UART_TXINT_MODE_EOT);

	UARTIntEnable  (BSP_UART_BASE, (UART_INT_RX | UART_INT_RT));
#endif

}

#else
/*
 * Config SPI 2 - PINS PA2 - SSI0CLK; PA3 - SSI0Fss; PA4 - SSI0Rx ; PA5 - SSI0Tx
 *
 */
void bspSpiInit(void)
{
    uint32_t ui32Dummy;

    SysCtrlPeripheralEnable(BSP_SPI_SSI_ENABLE_BM);

    SSIDisable(BSP_SPI_SSI_BASE);

    SSIClockSourceSet(BSP_SPI_SSI_BASE, SSI_CLOCK_PIOSC);

	IOCPinConfigPeriphOutput(BSP_SPI_BUS_BASE, BSP_SPI_FSS , IOC_MUX_OUT_SEL_SSI0_FSSOUT);
    IOCPinConfigPeriphOutput(BSP_SPI_BUS_BASE, BSP_SPI_SCK , IOC_MUX_OUT_SEL_SSI0_CLKOUT);
    IOCPinConfigPeriphOutput(BSP_SPI_BUS_BASE, BSP_SPI_MOSI, IOC_MUX_OUT_SEL_SSI0_TXD);
    IOCPinConfigPeriphInput (BSP_SPI_BUS_BASE, BSP_SPI_MISO, IOC_SSIRXD_SSI0);

    GPIOPinTypeSSI(BSP_SPI_BUS_BASE, (BSP_SPI_MOSI | BSP_SPI_MISO | BSP_SPI_SCK | BSP_SPI_FSS));

    /* config de acordo com placa sonoma14 Manual MAX78615+LMU pag 54 -
	 * Slave in mode 3 (CPOL=1 e CPHA=1) - first bit eh MSB
     * barramento SPI2 eh de 42MHZ - 42/64 =  656K
	 * sodoma tem limite de 1MHZ de clock (Manual MAX78615+LMU pag 6)
	 */

    SSIConfigSetExpClk(BSP_SPI_SSI_BASE, SysCtrlIOClockGet(), SSI_FRF_MOTO_MODE_3,
                       SSI_MODE_MASTER,BSP_SPI_CLK_SPD , 8);


    SSIEnable(BSP_SPI_SSI_BASE);

#if (USE_SPI_INTERRUPT == 1)
    // Raise interrupt at end of RX timeout
    SSIIntEnable(SSI0_BASE,SSI_RXTO);

    // Register isr in the nvic and enable isr at the nvic
    SSIIntRegister(SSI0_BASE, ssi_isr_private);

    // Enable the INT_SSI0 interrupt
    IntEnable(INT_SSI0);
#endif

    while(SSIDataGetNonBlocking(BSP_SPI_SSI_BASE, &ui32Dummy))
    {
    }

}

#endif

/*
 * Enable the serial interface with the slave board
 * it is provide SPI or UART interface
 * UART1 is for the Sensor Board COmmunication
 */
void bspserial(void)
{
#if USE_SPI_INTERFACE
   bspSpiInit();
#else
   bspUartInit();
#endif
}

#if USE_SPI_INTERFACE

// Serial or SPI interrupt, called when a new byte is received
#if (USE_SPI_INTERRUPT == 1)
void ssi_isr_private(void)
{
    uint8_t value;
    uint32_t u32Value=0;

	if (SSIIntStatus(BSP_SPI_SSI_BASE,SSI_TXFF) == 1)
	{
		SSIDataPut(BSP_SPI_SSI_BASE, tx_frame[num_tx_bytes] );
	    //leds_radio_toggle();
			 = value;

		num_rx_bytes++;
		if (num_rx_bytes >= OSENS_MAX_FRAME_SIZE)
			num_rx_bytes = 0;
	}

	if (SSIIntStatus(BSP_SPI_SSI_BASE,SSI_RXTO) == 1)
	{
	    //leds_radio_toggle();

	    SSIDataGetNonBlocking(BSP_SPI_SSI_BASE, &u32Value);

		if (num_rx_bytes < OSENS_MAX_FRAME_SIZE)
			rx_frame[num_rx_bytes] = value;

		num_rx_bytes++;
		if (num_rx_bytes >= OSENS_MAX_FRAME_SIZE)
			num_rx_bytes = 0;
	}

}

uint8_t osens_mote_send_frame(uint8_t *frame, uint8_t size)
{
    int8_t sent=size;
	uint32_t ui32Data;

    leds_debug_toggle();

    memcpy(tx_frame,frame, size);

    while(SSIBusy(BSP_SPI_SSI_BASE))
	{
	}

	while(SSIDataGetNonBlocking(BSP_SPI_SSI_BASE, &ui32Data));
	{
	}

	SSIDataPut(BSP_SPI_SSI_BASE, tx_frame[0] );
    tx_data_len = size-1;
    SSIIntEnable(BSP_SPI_SSI_BASE,SSI_TXFF);

    return (sent < 0 ? 0 : (uint8_t) size); // CHECK AGAIN
}

#else
/*-------------------------------------
 * Rotina para Ler um frame na SPI
 */
static uint8_t receive_spi(osens_mote_sm_state_t *st, uint8_t size)
{
	uint8_t NumByteToRead = size;
	uint32_t ui32Data=0;
	volatile uint8_t buf[10];
	uint8_t count=0;


    while(SSIBusy(BSP_SPI_SSI_BASE))
	{
	}

	while(SSIDataGetNonBlocking(BSP_SPI_SSI_BASE, &ui32Data));
	{
	}

	while (NumByteToRead > 0)
	{
		SSIDataPut(BSP_SPI_SSI_BASE, DUMMY_BYTE);
		SSIDataGet(BSP_SPI_SSI_BASE, &ui32Data);

	    //if (count < OSENS_MAX_FRAME_SIZE)
		if (count < 10)
	    	buf[count] = (uint8_t) (ui32Data & 0xFF);

	    count++;

	    //if (count >= OSENS_MAX_FRAME_SIZE)
	    //	count = 0;

		NumByteToRead--;
	}

	st->frame_arrived = 1;

	return size;
}

uint8_t osens_mote_send_frame(uint8_t *frame, uint8_t size)
{
    int8_t sent=size;
	uint32_t ui32Data;

    //leds_debug_toggle();

#if ( SENSOR_ACCEL == 1)

    frame[0] = ACC_X_LSB;

	while(SSIBusy(BSP_SPI_SSI_BASE))
	{
	}

	while(SSIDataGetNonBlocking(BSP_SPI_SSI_BASE, &ui32Data));
	{
	}

	GPIOPinWrite(BSP_ACC_CS_BASE, BSP_ACC_CS, 0);  // Set CSn active

	while (sent > 0)
	{
		SSIDataPut(BSP_SPI_SSI_BASE, (*frame | ACC_READ_M));
		SSIDataGet(BSP_SPI_SSI_BASE, &ui32Data);

		frame++;
		sent--;
	}
#else

    while(SSIBusy(BSP_SPI_SSI_BASE))
	{
	}

	while(SSIDataGetNonBlocking(BSP_SPI_SSI_BASE, &ui32Data));
	{
	}

	while (sent > 0)
	{
		SSIDataPut(BSP_SPI_SSI_BASE, *frame );
		SSIDataGet(BSP_SPI_SSI_BASE, &ui32Data);
		frame++;
		sent--;
	}
#endif

    return (sent < 0 ? 0 : (uint8_t) size); // CHECK AGAIN
}
#endif
#else //UART



#ifdef BSP_UART_ISR
/**************************************************************************//**
* @brief    This function handles BSP UART interrupts. This function clears all
*           handled interrupt flags.
*
* @return   None
******************************************************************************/
void
bspUartIsrHandler(void)
{
    uint32_t ui32IntBm = UARTIntStatus(BSP_UART_BASE, 1);

    UARTIntClear(BSP_UART_BASE, (ui32IntBm & BSP_UART_INT_BM));

    //
    // RX or RX timeout
    //
    if(ui32IntBm & (UART_INT_RX | UART_INT_RT))
    {
        //while(UARTCharsAvail(BSP_UART_BASE) && !BU_IS_BUF_FULL(&sBuBufRx))
        while(UARTCharsAvail(BSP_UART_BASE))
        {
            buBufPushByte();
        }

        //when occur receive timeout indicate end of frame
        if (ui32IntBm & UART_INT_RT)
        {
        	sm_state.frame_arrived = true;
        }
    }

    //
    // TX
    //
    if(ui32IntBm & UART_INT_TX)
    {
        //
        // Fill fifo with bytes from buffer
        //
        //while(UARTSpaceAvail(BSP_UART_BASE) && (!BU_IS_BUF_EMPTY(&sBuBufTx)))
        //{
        //    buBufPopByte();
        //}
    }
}
#endif // BSP_UART_ISR

#endif

/**************************************************************************//**
* @brief    This function reads up to \e ui16Length bytes from the BSP UART RX
*           buffer into the buffer specified by \e pui8Data.
*
*           If \b BSP_UART_ALL_OR_NOTHING is defined, data is read only if
*           \e ui16Length or more bytes are available in the BSP UART RX
*           buffer.
*
* @param    pui8Data        is a pointer to the destination buffer.
* @param    ui16Length      is the number of bytes to transfer.
*
* @return   Returns the number of bytes read from the BSP UART RX buffer.
******************************************************************************/
uint16_t
bspUartDataGet(uint8_t *pui8Data, uint16_t ui16Length)
{
    bool bIntDisabled;
    register uint_fast16_t ui1N;
    uint_fast16_t ui1Bytes;

    //
    // Data available in RX buffer?
    //
    ui1Bytes = bspUartRxCharsAvail();

    //
    // Return early if no data available/requested
    //
    if(!ui1Bytes || !ui16Length)
    {
        return 0;
    }

    if(ui16Length <= ui1Bytes)
    {
        //
        // Limit by input argument
        //
        ui1Bytes = ui16Length;
    }
#ifdef BSP_UART_ALL_OR_NOTHING
    else
    {
        //
        // All or nothing. Not enough data in buffer, fetching nothing.
        //
        return 0;
    }
#endif

    //
    // Copy data to application buffer
    //
    for(ui1N = 0; ui1N < ui1Bytes; ui1N++)
    {
        pui8Data[ui1N] = *sBuBufRx.pui8Tail++;

        //
        // Handle ringbuffer wrap-around
        //
        if(sBuBufRx.pui8Tail == sBuBufRx.pui8End)
        {
            sBuBufRx.pui8Tail = sBuBufRx.pui8Start;
        }
    }

    //
    // Critical section. Update volatile count variable
    //
    bIntDisabled = IntMasterDisable();
    sBuBufRx.ui16NumBytes -= ui1N;
    if(!bIntDisabled)
    {
        IntMasterEnable();
    }

    //
    // End of critical section
    //

    //
    // Return number of bytes read
    //
    return ui1N;
}


/**************************************************************************//**
* @brief    This function returns the number of data bytes available in the BSP
*           UART RX buffer.
*
* @return   Returns the number data bytes available in the BSP UART RX buffer.
******************************************************************************/
uint16_t bspUartRxCharsAvail(void)
{
    uint_fast16_t ui16Space;

    //
    // Critical section. Read volatile variable from RX control structure.
    //
    bool bIntDisabled = IntMasterDisable();
    //ui16Space = BU_GET_USED_SPACE(&sBuBufRx);

    //
    // Critical section end.
    //
    if(!bIntDisabled)
    {
        IntMasterEnable();
    }

    return ui16Space;
}

#if ( SENSOR_ACCEL == 1)
/**************************************************************************//**
* @brief    This function reads one or more accelerometer registers.
*
* @param    ui8Addr     is the register start address.
* @param    pui8Buf     is a pointer to the destination buffer.
* @param    ui8Len      is the number of registers to read.
*
* @return   None
******************************************************************************/
void accReadReg2(uint8_t ui8Addr, uint8_t *pui8Buf, uint8_t ui8Len)
{
    uint32_t ui32Data;

    //
    // Wait for ongoing transfers to complete and then clear in fifo before
    // pulling CSn low. This makes sure that accelerometer only retrieves data
    // intended for it.
    //
    while(SSIBusy(BSP_SPI_SSI_BASE))
    {
    }
    while(SSIDataGetNonBlocking(BSP_SPI_SSI_BASE, &ui32Data));
    {
    }

    //
    // Set CSn active
    //
    GPIOPinWrite(BSP_ACC_CS_BASE, BSP_ACC_CS, 0);

    //
    // Send address byte to SSI FIFO and read dummy data
    //
    SSIDataPut(BSP_SPI_SSI_BASE, (ui8Addr | ACC_READ_M));
    SSIDataGet(BSP_SPI_SSI_BASE, &ui32Data);

    while(ui8Len--)
    {
        //
        // Send dummy byte and read returned data from SPI FIFO
        //
        SSIDataPut(BSP_SPI_SSI_BASE, 0x00);
        SSIDataGet(BSP_SPI_SSI_BASE, &ui32Data);

        //
        // Store read data to buffer
        //
        *pui8Buf++ = (ui32Data & 0xFF);
    }

    //
    // Clear CSn
    //
    GPIOPinWrite(BSP_ACC_CS_BASE, BSP_ACC_CS, BSP_ACC_CS);
}


#endif

#if (SONOMA14 == 1)

/**
* \brief       Reads the value of a register
* \par         Details
*
* \param[in]   uchRegAddr      - Register Address
*
* \retval      Register Value
*/
uint32_t read_reg(uint8_t uchRegAddr)
{
	uint8_t uchRegValue[3];
	uint8_t uchSPIWBuffer[5];
	uint32_t ui32Index;
	uint32_t ui32Data;

	//preparing read command
	uchSPIWBuffer[0] = uchRegAddr>>6;
	uchSPIWBuffer[0] = (uchSPIWBuffer[0]<<2)+1;
	uchSPIWBuffer[1] = 0x3F & uchRegAddr;
	uchSPIWBuffer[1] = uchSPIWBuffer[1]<<2;
	uchSPIWBuffer[2] = 0;
	uchSPIWBuffer[3] = 0;
	uchSPIWBuffer[4] = 0;


    //
    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
    // because the SPI SSI mode is full-duplex, which allows you to send and
    // receive at the same time.  The SSIDataGetNonBlocking function returns
    // "true" when data was returned, and "false" when no data was returned.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    //
    while(SSIDataGetNonBlocking(BSP_SPI_SSI_BASE, &ui32Data))
    {
    }

	/*Leitura da spi do sonoma 14 - manual pag 55
	 * 1o byte=0x1 ;
	 * 2o byte = endereco (6bits msb)
	 * 3o,4o,5o bytes = 0x00
	 */
    for (ui32Index = 0; ui32Index < 5; ui32Index++)
    {
        SSIDataPut(BSP_SPI_SSI_BASE, uchSPIWBuffer[ui32Index]);
    }

    //
    // Wait until SSI0 is done transferring all the data in the transmit FIFO.
    //
    while(SSIBusy(BSP_SPI_SSI_BASE))
    {
    }

    for(ui32Index = 0; ui32Index < 5; ui32Index++)
    {
        //
        // Receive the data using the "blocking" Get function. This function
        // will wait until there is data in the receive FIFO before returning.
        //
        SSIDataGet(BSP_SPI_SSI_BASE, &ui32Data);

        if (ui32Index > 1)
        {
        	uchRegValue[ui32Index-2] = (ui32Data & 0xFF);
        }

    }

	return (uchRegValue[0]<<16) + (uchRegValue[1]<<8) + uchRegValue[2];

}


#endif


