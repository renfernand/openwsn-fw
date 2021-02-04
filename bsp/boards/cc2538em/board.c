/**
 * Author: Xavier Vilajosana (xvilajosana@eecs.berkeley.edu)
 *         Pere Tuset (peretuset@openmote.com)
 * Date:   July 2013
 * Description: CC2538-specific definition of the "board" bsp module.
 */

#include "hw_ioc.h"            
#include "hw_memmap.h"
#include "hw_ssi.h"
#include "hw_sys_ctrl.h"
#include "hw_types.h"

#include "flash.h"
#include "interrupt.h"
#include "ioc.h"
#include "gpio.h"   
#include "gptimer.h"
#include "sys_ctrl.h"

#include "board.h"
#include "debugpins.h"

#include "leds.h"
#include "radio.h"
#include "sensors.h"
#include "sctimer.h"
#include "uart.h"
#include "cryptoengine.h"

#include "hw_ints.h"
#include  "uarthal.h"

#include "osens_app.h"

//=========================== variables =======================================

#define BSP_RADIO_BASE              ( GPIO_D_BASE )
#define BSP_BUTTON_BASE             ( GPIO_C_BASE )
#define BSP_RADIO_INT               ( GPIO_PIN_5 )
#define BSP_RADIO_EXT               ( GPIO_PIN_4 )
#define BSP_USER_BUTTON             ( GPIO_PIN_3 )

#define BSP_SPI_CLK_SPD             8000000UL

//=========================== prototypes ======================================

void board_timer_init(void);
uint32_t board_timer_get(void);
bool board_timer_expired(uint32_t future);

static void clock_init(void);
static void gpio_init(void);


static void SysCtrlDeepSleepSetting(void);
static void SysCtrlSleepSetting(void);
static void SysCtrlRunSetting(void);
static void SysCtrlWakeupSetting(void);

void GPIO_C_Isr_Handler(void);

void antenna_init(void);
void antenna_internal(void);
void antenna_external(void);
void button_init(void);


void ssi_isr_private(void);
//=========================== main ============================================

extern int mote_main(void);

int main(void) {
   return mote_main();
}

//=========================== public ==========================================

void board_init(void) {
   gpio_init();
   clock_init();
   board_timer_init();
   leds_init();
   debugpins_init();
  // button_init();
   antenna_init();
   antenna_external();
   leds_all_off();
   sctimer_init(); 
   uart_init();
#if (USE_SPI_INTERFACE == 1)
   bspSpiInit();
#endif
   radio_init();
// i2c_init();  
   sensors_init();
   cryptoengine_init();
   radio_init();
   osens_app_init();
  
}

/**
 * Puts the board to sleep
 */
void board_sleep(void) {
    SysCtrlPowerModeSet(SYS_CTRL_PM_NOACTION);
    SysCtrlSleep();
}

/**
 * Timer runs at 32 MHz and is 32-bit wide
 * The timer is divided by 32, whichs gives a 1 microsecond ticks
 */
void board_timer_init(void) {
    // Configure the timer
    TimerConfigure(GPTIMER2_BASE, GPTIMER_CFG_PERIODIC_UP);
    
    // Enable the timer
    TimerEnable(GPTIMER2_BASE, GPTIMER_BOTH);
}

/**
 * Returns the current value of the timer
 * The timer is divided by 32, whichs gives a 1 microsecond ticks
 */
uint32_t board_timer_get(void) {
    uint32_t current;
    
    current = TimerValueGet(GPTIMER2_BASE, GPTIMER_A) >> 5;
    
    return current;
}

/**
 * Returns true if the timer has expired
 * The timer is divided by 32, whichs gives a 1 microsecond ticks
 */
bool board_timer_expired(uint32_t future) {
    uint32_t current;
    int32_t remaining;

    current = TimerValueGet(GPTIMER2_BASE, GPTIMER_A) >> 5;

    remaining = (int32_t) (future - current);
    
    if (remaining > 0) {
        return false;
    } else {
        return true;
    }
}

/**
 * Resets the board
 */
void board_reset(void) {
	SysCtrlReset();
}

//=========================== private =========================================

static void gpio_init(void) {
    /* Set GPIOs as output */
    GPIOPinTypeGPIOOutput(GPIO_A_BASE, 0xFF);
    GPIOPinTypeGPIOOutput(GPIO_B_BASE, 0xFF);
    GPIOPinTypeGPIOOutput(GPIO_C_BASE, 0xFF);
    GPIOPinTypeGPIOOutput(GPIO_D_BASE, 0xFF);

    /* Initialize GPIOs to low */
    GPIOPinWrite(GPIO_A_BASE, 0xFF, 0x00);
    GPIOPinWrite(GPIO_B_BASE, 0xFF, 0x00);
    GPIOPinWrite(GPIO_C_BASE, 0xFF, 0x00);
    GPIOPinWrite(GPIO_D_BASE, 0xFF, 0x00);
}

static void clock_init(void) {
    /* Disable global interrupts */
    bool bIntDisabled = IntMasterDisable();

    /* Configure the 32 kHz pins, PD6 and PD7, for crystal operation */
    /* By default they are configured as GPIOs */
    GPIODirModeSet(GPIO_D_BASE, 0x40, GPIO_DIR_MODE_IN);
    GPIODirModeSet(GPIO_D_BASE, 0x80, GPIO_DIR_MODE_IN);
    IOCPadConfigSet(GPIO_D_BASE, 0x40, IOC_OVERRIDE_ANA);
    IOCPadConfigSet(GPIO_D_BASE, 0x80, IOC_OVERRIDE_ANA);

    /* Set the real-time clock to use the 32 kHz external crystal */
    /* Set the system clock to use the external 32 MHz crystal */
    /* Set the system clock to 32 MHz */
    SysCtrlClockSet(true, false, SYS_CTRL_SYSDIV_32MHZ);

    /* Set the IO clock to operate at 16 MHz */
    /* This way peripherals can run while the system clock is gated */
    SysCtrlIOClockSet(SYS_CTRL_SYSDIV_16MHZ);

    /* Wait until the selected clock configuration is stable */
    while (!((HWREG(SYS_CTRL_CLOCK_STA)) & (SYS_CTRL_CLOCK_STA_XOSC_STB)));

   /* Define what peripherals run in each mode */
    SysCtrlRunSetting();
    SysCtrlSleepSetting();
    SysCtrlDeepSleepSetting();
    SysCtrlWakeupSetting();

    /* Re-enable interrupt if initially enabled */
    if (!bIntDisabled) {
        IntMasterEnable();
    }
}

/**
 * Configures the user button as input source
 */
void button_init(void){
	GPIOPinTypeGPIOInput(BSP_BUTTON_BASE, BSP_USER_BUTTON);
	GPIOIntTypeSet(BSP_BUTTON_BASE,BSP_USER_BUTTON,GPIO_FALLING_EDGE);
	GPIOPortIntRegister(BSP_BUTTON_BASE,GPIO_C_Isr_Handler);
	GPIOPinIntClear(BSP_BUTTON_BASE, BSP_USER_BUTTON);
	GPIOPinIntEnable(BSP_BUTTON_BASE, BSP_USER_BUTTON);
}

static void SysCtrlRunSetting(void) {
  /* Disable General Purpose Timers 0, 1, 2, 3 when running */
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_GPT0);
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_GPT1);
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_GPT2);
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_GPT3);

  /* Disable SSI 0, 1 when running */
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_SSI0);
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_SSI1);

  /* Disable UART1 when running */
  //SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_UART1);    //RFF

  /* Disable I2C, AES and PKA when running */
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_I2C);
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_PKA);
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_AES);

  /* Enable UART0 and RFC when running */
  SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_UART0);
  SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_UART1);
  SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_RFC);
}

static void SysCtrlSleepSetting(void) {
  /* Disable General Purpose Timers 0, 1, 2, 3 during sleep */
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_GPT0);
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_GPT1);
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_GPT2);
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_GPT3);

  /* Disable SSI 0, 1 during sleep */
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_SSI0);
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_SSI1);

  /* Disable UART 0, 1 during sleep */
  //SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_UART1);    //RFF

  /* Disable I2C, PKA, AES during sleep */
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_I2C);
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_PKA);
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_AES);

  /* Enable UART and RFC during sleep */
  SysCtrlPeripheralSleepEnable(SYS_CTRL_PERIPH_UART0);
  SysCtrlPeripheralSleepEnable(SYS_CTRL_PERIPH_UART1);    //RFF
  SysCtrlPeripheralSleepEnable(SYS_CTRL_PERIPH_RFC);
}

static void SysCtrlDeepSleepSetting(void) {
  /* Disable General Purpose Timers 0, 1, 2, 3 during deep sleep */
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_GPT0);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_GPT1);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_GPT2);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_GPT3);

  /* Disable SSI 0, 1 during deep sleep */
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_SSI0);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_SSI1);

  /* Disable UART 0, 1 during deep sleep */
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_UART0);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_UART1);

  /* Disable I2C, PKA, AES during deep sleep */
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_I2C);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_PKA);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_AES);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_RFC);
}

static void SysCtrlWakeupSetting(void) {
  /* Allow the SMTimer to wake up the processor */
  GPIOIntWakeupEnable(GPIO_IWE_SM_TIMER);
}

//=========================== interrupt handlers ==============================

/**
 * GPIO_C interrupt handler. User button is GPIO_C_3
 * Erases a Flash sector to trigger the bootloader backdoor
 */
void GPIO_C_Isr_Handler(){

	GPIOPinIntClear(GPIO_C_BASE, BSP_USER_BUTTON);

}

/**
 * Configures the antenna using a RF switch
 * INT is the internal antenna (chip) configured through ANT1_SEL (V1)
 * EXT is the external antenna (connector) configured through ANT2_SEL (V2)
 */
void antenna_init(void) {
    // Configure the ANT1 and ANT2 GPIO as output
    GPIOPinTypeGPIOOutput(BSP_RADIO_BASE, BSP_RADIO_INT);
    GPIOPinTypeGPIOOutput(BSP_RADIO_BASE, BSP_RADIO_EXT);

    // By default the chip antenna is selected as the default
    GPIOPinWrite(BSP_RADIO_BASE, BSP_RADIO_INT, BSP_RADIO_INT);
    GPIOPinWrite(BSP_RADIO_BASE, BSP_RADIO_EXT, ~BSP_RADIO_EXT);
}

/**
 * Selects the external (connector) antenna
 */
void antenna_external(void) {
    GPIOPinWrite(BSP_RADIO_BASE, BSP_RADIO_EXT, BSP_RADIO_EXT);
    GPIOPinWrite(BSP_RADIO_BASE, BSP_RADIO_INT, ~BSP_RADIO_INT);
}

/**
 * Selects the internal (chip) antenna
 */
void antenna_internal(void) {
    GPIOPinWrite(BSP_RADIO_BASE, BSP_RADIO_EXT, ~BSP_RADIO_EXT);
    GPIOPinWrite(BSP_RADIO_BASE, BSP_RADIO_INT, BSP_RADIO_INT);
}

