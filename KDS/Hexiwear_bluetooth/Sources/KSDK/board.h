/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#if !defined(__BOARD_H__)
#define __BOARD_H__

#include <stdint.h>
#include "pin_mux.h"
#include "gpio_pins.h"

/*************************************************************************************
**************************************************************************************
* Public macros
**************************************************************************************
*************************************************************************************/

/* The board name */
#define BOARD_NAME                      "FRDM-KW40Z"

#define CLOCK_VLPR    1U
#define CLOCK_RUN_16  2U
#define CLOCK_RUN_32  3U
#define CLOCK_NUMBER_OF_CONFIGURATIONS 3U

#ifndef CLOCK_INIT_CONFIG
#define CLOCK_INIT_CONFIG CLOCK_RUN_32
#endif

#if (CLOCK_INIT_CONFIG == CLOCK_RUN_32)
    #define CORE_CLOCK_FREQ 32000000U
#elif (CLOCK_INIT_CONFIG == CLOCK_RUN_16)
    #define CORE_CLOCK_FREQ 16000000U
#else
    #define CORE_CLOCK_FREQ 4000000U    
#endif

#define BOARD_USE_LPUART 1

/* Connectivity */
#ifndef APP_SERIAL_INTERFACE_TYPE
#define APP_SERIAL_INTERFACE_TYPE (gSerialMgrLpuart_c)
#endif

#ifndef APP_SERIAL_INTERFACE_INSTANCE
#define APP_SERIAL_INTERFACE_INSTANCE (0)
#endif

/* DCDC driver default mode */
#ifndef APP_DCDC_MODE
#define APP_DCDC_MODE    (gDCDC_Mode_Buck_c)
#endif

/* Disable all pins when entering Low Power */
#ifndef APP_DISABLE_PINS_IN_LOW_POWER
#define APP_DISABLE_PINS_IN_LOW_POWER    (0)
#endif
   
/* Vbat monitor Interval in Ms */
#ifndef APP_DCDC_VBAT_MONITOR_INTERVAL
#define APP_DCDC_VBAT_MONITOR_INTERVAL    (50000)
#endif

/* EXTAL0 PTA18 */
#define EXTAL0_PORT   PORTB
#define EXTAL0_PIN    17
#define EXTAL0_PINMUX kPortPinDisabled

/* XTAL0 PTA19 */
#define XTAL0_PORT   PORTB
#define XTAL0_PIN    16
#define XTAL0_PINMUX kPortPinDisabled

/* RTC external clock configuration. */
#define RTC_XTAL_FREQ   32768U
#define RTC_SC2P_ENABLE_CONFIG       false
#define RTC_SC4P_ENABLE_CONFIG       false
#define RTC_SC8P_ENABLE_CONFIG       false
#define RTC_SC16P_ENABLE_CONFIG      false
#define RTC_OSC_ENABLE_CONFIG        true

/* RTC_CLKIN PTB16 */
#define RTC_CLKIN_PORT   PORTB
#define RTC_CLKIN_PIN    16
#define RTC_CLKIN_PINMUX kPortMuxAsGpio

/* The UART to use for debug messages. */
#ifndef BOARD_DEBUG_UART_INSTANCE
    #define BOARD_DEBUG_UART_INSTANCE   0
    #define BOARD_DEBUG_UART_BASEADDR   LPUART0
#endif
#ifndef BOARD_DEBUG_UART_BAUD
    #define BOARD_DEBUG_UART_BAUD       115200
#endif
#ifndef BOARD_LPUART_CLOCK_SOURCE
    #define BOARD_LPUART_CLOCK_SOURCE   kClockLpuartSrcMcgFllClk
#endif

/* Usage or pins for UART */
#define BOARD_USE_DEBUG_UART        0
#define BOARD_USE_FRDM_HDR_UART     1
#define BOARD_USE_FRDM_HDR_UART_ALT 2
  
#ifndef BOARD_UART_CONFIG  
    #define BOARD_UART_CONFIG  BOARD_USE_DEBUG_UART
#endif
   
/* Define the port interrupt number for the board switches */
#define BOARD_SW_GPIO               kGpioSW3
#define BOARD_SW_IRQ_NUM            PORTA_IRQn
#define BOARD_SW_IRQ_HANDLER        PORTA_IRQHandler
#define BOARD_SW_NAME               "SW3"
/* Define print statement to inform user which switch to press for
 * power_manager_hal_demo and power_manager_rtos_demo
 */
#define PRINT_LLWU_SW_NUM \
  PRINTF("SW3")

/* Defines the llwu pin number for board switch which is used in power_manager_demo. */
#define BOARD_SW_HAS_LLWU_PIN        1
#define BOARD_SW_LLWU_EXT_PIN        kLlwuWakeupPin6
/* Switch port base address and IRQ handler name. Used by power_manager_demo */
#define BOARD_SW_LLWU_PIN            18
#define BOARD_SW_LLWU_BASE           PORTA
#define BOARD_SW_LLWU_IRQ_HANDLER    PORTA_IRQHandler
#define BOARD_SW_LLWU_IRQ_NUM        PORTA_IRQn

#define BOARD_I2C_GPIO_SCL              GPIO_MAKE_PIN(GPIOC_IDX, 2)
#define BOARD_I2C_GPIO_SDA              GPIO_MAKE_PIN(GPIOC_IDX, 3)

/* The instances of peripherals used for dac_adc_demo */
#define BOARD_DAC_DEMO_DAC_INSTANCE     0U
#define BOARD_DAC_DEMO_ADC_INSTANCE     0U
#define BOARD_DAC_DEMO_ADC_CHANNEL      4U

/* The i2c instance used for i2c connection by default */
#define BOARD_I2C_INSTANCE              1

/* The dspi instance used for dspi example */
#define BOARD_DSPI_INSTANCE             1

/* The TPM instance/channel used for board */
#define BOARD_TPM_INSTANCE              0
#define BOARD_TPM_CHANNEL               1

/* TSI electrodes mapping */
#define BOARD_TSI_ELECTRODE_1 10
#define BOARD_TSI_ELECTRODE_2 11
#define BOARD_TSI_ELECTRODE_CNT 2
    
/* deafult ADC channel for hw trigger demo */
#define BOARD_ADC_HW_TRIGGER_CHAN       3

/* The CMP instance used for board. */
#define BOARD_CMP_INSTANCE              0
/* The CMP channel used for board. */
#define BOARD_CMP_CHANNEL               1

/* The rtc instance used for rtc_func */
#define BOARD_RTC_FUNC_INSTANCE         0

#define BOARD_LTC_INSTANCE              0

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/************************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
************************************************************************************/

void hardware_init(void);
void dbg_uart_init(void);
/* Function to initialize clock base on board configuration. */
void BOARD_ClockInit(void);

/* Function to initialize OSC0 base on board configuration. */
void BOARD_InitOsc0(void);

/* Function to initialize RTC external clock base on board configuration. */
void BOARD_InitRtcOsc(void);

/* Function to initialize ADC on board configuration. */
void BOARD_InitAdc(void);

/* Function to initialize DCDC on board configuration. */
void BOARD_DCDCInit(void);

/* Function to read battery level on board configuration. */
uint8_t BOARD_GetBatteryLevel(void);

/* Function to read potentiometer level on board configuration. */
uint16_t BOARD_GetPotentiometerLevel(void);

/* Function to install callbacks for actions before and after entering low power. */
extern void BOARD_InstallLowPowerCallbacks(void);

/* Function called by low power module prior to entering low power. */
extern void BOARD_EnterLowPowerCb(void);

/* Function called by low power module after exiting low power. */
extern void BOARD_ExitLowPowerCb(void);


#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* __BOARD_H__ */
