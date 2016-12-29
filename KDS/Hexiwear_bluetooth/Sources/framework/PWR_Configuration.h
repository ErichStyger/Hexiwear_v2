/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PWR_Configuraion.h
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


#ifndef _PWR_CONFIGURATION_H_
#define _PWR_CONFIGURATION_H_

/*****************************************************************************
 *                               INCLUDED HEADERS                            *
 *---------------------------------------------------------------------------*
 * Add to this section all the headers that this module needs to include.    *
 * Note that it is not a good practice to include header files into header   *
 * files, so use this section only if there is no other better solution.     *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
#include "TimersManager.h"


/************************************************************************************
*************************************************************************************
* Module configuration constants
*************************************************************************************
************************************************************************************/

/*********************/
/* LVD configuration */
/*********************/

//-----------------------------------------------------------------------------
// The use of Low Voltage detection has the following possibilities:
//   0: Don't use Low voltage detection at all
//   1: Use polled => Check made each time the function is called.
//   2: A minutes software timer used for handling when to poll LVD, according
//      to the cPWR_LVD_Ticks constant
//   3: LVDRE  are set to hold MCU in reset while VLVDL  condition is detected
// PEX Settings: - in modes 0,1,2 the property CPU->Internal peripherals->Power management controller->LVDreset must be Disabled
//               - in mode 3 the property CPU->Internal peripherals->Power management controller->LVDreset must be Enabled  
//The the propery refers to LVDRE bit in the PMC_LVDSC1 register which is a write once bit, so it cannot be modified after that.

#ifndef cPWR_LVD_Enable
  #define cPWR_LVD_Enable                         0
#endif

//-----------------------------------------------------------------------------
// How often to check the LVD level when cPWR_LVD_Enable == 2
// This is the number of minutes before voltage is checked (Consumes
// current and time)

#ifndef cPWR_LVD_Ticks
  #define cPWR_LVD_Ticks                          60
#endif

//-----------------------------------------------------------------------------
// To enable/disable all of the code in this PWR/PWRLib files.
//   TRUE =  1: Use PowerDown functions (Normal)
//   FALSE = 0: Don't use PowerDown. Will cut variables and code out. But
//              functions still exist. Useful for debugging and test purposes
#ifndef cPWR_UsePowerDownMode
  #define cPWR_UsePowerDownMode                   FALSE
#endif


#ifndef cPWR_BLE_LL_Enable
#define cPWR_BLE_LL_Enable               FALSE
#endif
#ifndef cPWR_Zigbee_Enable
#define cPWR_Zigbee_Enable               FALSE
#endif

//-----------------------------------------------------------------------------
// The way that DeepSleep mode are handled. Following possibilities exist:
//*****************************************************************************
//   
//*****************************************************************************
//   1: MCU/Radio low power modes:
//        MCU in LLS3 mode.
//        BLE_LL in DSM.  
//      Wakeup sources:
//        GPIO (push button) interrupt using LLWU module(SW2(PTA18),SW3(PTA19)). 
//        BLE_LL wake up interrupt(BLE_LL reference clock reaches wake up instance register)  using LLWU module.
//          - BTE_LL wakeup timeout: controlled by the BLE stack(SoC must be awake before next BLE action).
//          - BTE_LL reference clock source:   32Khz oscillator
//          - BTE_LL reference clock resolution:     625us
//        LPTMR wakeup timeout: cPWR_DeepSleepDurationMs by default. Use PWR_SetDeepSleepTimeInMs  
//                                  to change it at run time. Maximum timeout is 65535000 ms (18.2 h).
//          - LPTMR clock source:   32Khz oscillator
//          - LPTMR resolution:     modified at run time to meet timeout value.
//          - LPTMR resolution:     modified at run time to meet timeout value.                                  
//*****************************************************************************
//*****************************************************************************
//   2: MCU/Radio low power modes:
//        MCU in LLS3 mode.
//        BLE_LL in DSM.  
//      Wakeup sources:
//        GPIO (push button) interrupt using LLWU module(SW2(PTA18),SW3(PTA19)). 
//        BLE_LL wake up interrupt(BLE_LL reference clock reaches wake up instance register)  using LLWU module.
//          - BTE_LL wakeup timeout: cPWR_DeepSleepDurationMs by default. Use PWR_SetDeepSleepTimeInMs  
//                                  to change it at run time. Maximum timeout is 40959 ms. BLE suppose to be idle(no action scheduled).
//          - BTE_LL reference clock source:   32Khz oscillator
//          - BTE_LL reference clock resolution:     625us//        LPTMR interrupt.
//*****************************************************************************
//*****************************************************************************
//   3: MCU/Radio low power modes:
//        MCU in LLS3 mode.
//        BLE_LL in idle.  
//      Wakeup sources:
//        GPIO (push button) interrupt using LLWU module(SW2(PTA18),SW3(PTA19)). 
//        DCDC PowerSwitch - available in buck mode only. 
//        LPTMR interrupt using LLWU module    
//          - LPTMR wakeup timeout: cPWR_DeepSleepDurationMs by default. Use PWR_SetDeepSleepTimeInMs  
//                                  to change it at run time. Maximum timeout is 65535000 ms (18.2 h).
//          - LPTMR clock source:   32Khz oscillator
//          - LPTMR resolution:     modified at run time to meet timeout value.
//*****************************************************************************
//*****************************************************************************
//   4: MCU/Radio low power modes:
//        MCU in VLLS0/1 mode(VLLS0 if DCDC bypassed/ VLLS1 otherwise ).
//        BLE_LL in idle.  
//      Wakeup sources:
//        GPIO (push button) interrupt using LLWU module(SW2(PTA18),SW3(PTA19)). 
//        DCDC PowerSwitch - available in buck mode only. 
//*****************************************************************************
//*****************************************************************************
//   5: MCU/Radio low power modes:
//        MCU in VLLS2 (4k Ram retention (0x20000000- 0x20000fff)).
//        BLE_LL in idle.  
//      Wakeup sources:
//        GPIO (push button) interrupt using LLWU module(SW2(PTA18),SW3(PTA19)). 
//        DCDC PowerSwitch - available in buck mode only. 
//*****************************************************************************
//*****************************************************************************
//   6: MCU/Radio low power modes:
//        MCU in STOP.
//      Wakeup sources:
//        GPIO (push button) interrupt using LLWU module(SW2(PTA18),SW3(PTA19)). 
//        DCDC PowerSwitch - available in buck mode only. 
//        LPTMR wakeup timeout: cPWR_DeepSleepDurationMs by default. Use PWR_SetDeepSleepTimeInMs  
//                                  to change it at run time. Maximum timeout is 65535000 ms (18.2 h).
//          - LPTMR clock source:   32Khz oscillator
//          - LPTMR resolution:     modified at run time to meet timeout value.
//          - LPTMR resolution:     modified at run time to meet timeout value.
//        Radio interrupt LL or 802.15.4
//        UART
//*****************************************************************************
#ifndef cPWR_DeepSleepMode
  #define cPWR_DeepSleepMode                     4
#endif

//-----------------------------------------------------------------------------
// The way that Sleep mode are handled. Following possibilities exist:
//   0: No Sleep done, but application can set modes
//   1: MCU/Radio low power modes:
//        MCU in WAIT mode.
//        Radio in normal mode.

#ifndef cPWR_SleepMode
#define cPWR_SleepMode                            1
#endif

//-----------------------------------------------------------------------------
// The deep sleep duration in ms. 
#ifndef cPWR_DeepSleepDurationMs
  #define cPWR_DeepSleepDurationMs                3000
#endif

//-----------------------------------------------------------------------------
// Enabling of external call to a procedure each time that DeepSleep are exited
//   0: Don't call any functions after DeepSleep (MAC)
//   1: Call a function after DeepSleep (Stack)
#ifndef cPWR_CallWakeupStackProcAfterDeepSleep
  #define cPWR_CallWakeupStackProcAfterDeepSleep  FALSE
#endif

//-----------------------------------------------------------------------------
// The extra function to call every time RTI clock run's out. Used by Stack.
#if (cPWR_CallWakeupStackProcAfterDeepSleep == 0)
  #define cPWR_DeepSleepWakeupStackProc           ;
#else
  extern void                                     DeepSleepWakeupStackProc(void);
  #define cPWR_DeepSleepWakeupStackProc           DeepSleepWakeupStackProc();  
#endif

//-----------------------------------------------------------------------------

// BLE LL DEEP SLEEP MODE DEFINES
/* Number of slots(625us) before the wake up instant before which the hardware needs to exit from deep sleep mode. */
#ifndef cPWR_BLE_LL_OffsetToWakeupInstant
#define cPWR_BLE_LL_OffsetToWakeupInstant                   (3)
#endif

/*Oscillator stabilization/startup delay. This is in X.Y for-mat where X is in terms of number of BT slots (625 us) and Y is in terms of number of clock periods of 16KHz clock input, required for RF oscillator to stabilize the clock output to the controller on its output pin, after os-cillator is turned ON. In this period the clock is assumed to be unstable, and so the controller does not turn on the clock to internal logic till this period is over. This means, the wake up from deep sleep mode must ac-count for this delay before the wakeup instant.
Osc_startup_delay[7:5] is number of slots(625us)
Osc_startup_delay[4:0 is number of clock periods of 16KHz clock
(Warning: Max. value of Osc_startup_delay [4:0] sup-ported is 9. Therefore do not program value greater than 9)*/

#define cPWR_BLE_LL_OscStartupDelay                         (0x42)

#define cPWR_POR_DisabledInVLLS0                            (1)

#define cPWR_DCDC_InBypass                                  (0)
//-----------------------------------------------------------------------------

#if (cPWR_LVD_Enable > 3)
  #error "*** ERROR: Illegal value in cPWR_LVD_Enable"
#endif

#if (cPWR_LVD_Enable == 2)
  #if (gTMR_Enabled_d != TRUE) 
    #error "*** ERROR: Illegal value in cPWR_LVD_Enable"
  #endif
#endif

#if (cPWR_UsePowerDownMode > 1)
  #error "*** ERROR: Illegal value in cPWR_UsePowerDownMode"
#endif

#if (cPWR_CallWakeupStackProcAfterDeepSleep > 1)
  #error "*** ERROR: Illegal value in cPWR_CallWakeupStackProcAfterDeepSleep"
#endif

#if (cPWR_DeepSleepMode > 6 )
  #error "*** ERROR: Illegal value in cPWR_DeepSleepMode"
#endif

#if (cPWR_SleepMode > 1)
  #error "*** ERROR: Illegal value in cPWR_SleepMode"
#endif

#endif /* _PWR_CONFIGURATION_H_ */
