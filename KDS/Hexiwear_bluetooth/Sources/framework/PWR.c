/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PWR.c
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

/*****************************************************************************
 *                               INCLUDED HEADERS                            *
 *---------------------------------------------------------------------------*
 * Add to this section all the headers that this module needs to include.    *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
#include "EmbeddedTypes.h"
#include "PWR_Configuration.h"
#include "PWRLib.h"
#include "PWR_Interface.h"
#include "TimersManager.h"
#include "fsl_device_registers.h"
#include "fsl_os_abstraction.h"
#if cPWR_Zigbee_Enable   
#include "MacInterface.h"
#endif

#if (defined(CPU_MKW30Z160VHT4) && (cPWR_Zigbee_Enable))
 #error "No Zigbee on this CPU!"
#endif

#if (cPWR_BLE_LL_Enable)
#include "PWR_BLE.h"
#endif
/*****************************************************************************
 *                             PRIVATE MACROS                                *
 *---------------------------------------------------------------------------*
 * Add to this section all the access macros, registers mappings, bit access *
 * macros, masks, flags etc ...                                              *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
#define mBLETimoutMaxMs_c 40959
   


/*****************************************************************************
 *                               PUBLIC VARIABLES                            *
 *---------------------------------------------------------------------------*
 * Add to this section all the variables and constants that have global      *
 * (project) scope.                                                          *
 * These variables / constants can be accessed outside this module.          *
 * These variables / constants shall be preceded by the 'extern' keyword in  *
 * the interface header.                                                     *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

/*****************************************************************************
 *                           PRIVATE FUNCTIONS PROTOTYPES                    *
 *---------------------------------------------------------------------------*
 * Add to this section all the functions prototypes that have local (file)   *
 * scope.                                                                    *
 * These functions cannot be accessed outside this module.                   *
 * These declarations shall be preceded by the 'static' keyword.             *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
#if (cPWR_UsePowerDownMode)

static void PWR_HandleDeepSleepMode_1(void);
static void PWR_HandleDeepSleepMode_2(void);
static void PWR_HandleDeepSleepMode_3(void);
static void PWR_HandleDeepSleepMode_4(void);
static void PWR_HandleDeepSleepMode_5(void);   
static void PWR_HandleDeepSleepMode_6(void);      

#endif


/*****************************************************************************
 *                        PUBLIC TYPE DEFINITIONS                            *
 *---------------------------------------------------------------------------*
 * Add to this section all the data types definitions: stuctures, unions,    *
 * enumerations, typedefs ...                                                *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

typedef enum 
{
  PWR_Run = 77,
  PWR_Sleep,
  PWR_DeepSleep,
  PWR_Reset,
  PWR_OFF
} PWR_CheckForAndEnterNewPowerState_t;
 typedef  void (*pfHandleDeepSleepFunc_t)(void);


/*****************************************************************************
 *                               PRIVATE VARIABLES                           *
 *---------------------------------------------------------------------------*
 * Add to this section all the variables and constants that have local       *
 * (file) scope.                                                             *
 * Each of this declarations shall be preceded by the 'static' keyword.      *
 * These variables / constants cannot be accessed outside this module.       *
 *---------------------------------------------------------------------------*
 *****************************************************************************/


uint8_t mLPMFlag = gAllowDeviceToSleep_c;

#if (cPWR_UsePowerDownMode)
static uint32_t mPWR_DeepSleepTimeMs = cPWR_DeepSleepDurationMs;
const pfHandleDeepSleepFunc_t maHandleDeepSleep[]={PWR_HandleDeepSleepMode_1,
                                                    PWR_HandleDeepSleepMode_2,
                                                    PWR_HandleDeepSleepMode_3,
                                                    PWR_HandleDeepSleepMode_4,
                                                    PWR_HandleDeepSleepMode_5,
                                                    PWR_HandleDeepSleepMode_6
                                                   };
static pfPWRCallBack_t gpfPWR_LowPowerEnterCb;
static pfPWRCallBack_t gpfPWR_LowPowerExitCb;

#endif //(cPWR_UsePowerDownMode)
/*****************************************************************************
 *                             PRIVATE FUNCTIONS                             *
 *---------------------------------------------------------------------------*
 * Add to this section all the functions that have local (file) scope.       *
 * These functions cannot be accessed outside this module.                   *
 * These definitions shall be preceded by the 'static' keyword.              *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

#define REGS_BLE_BASE           0x4005B000
#define BLE_LL_REG(offset)  (*((volatile uint16_t*)(REGS_BLE_BASE + offset))) 

/*---------------------------------------------------------------------------
 * Name: PWR_HandleDeepSleepMode_1
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
#if (cPWR_UsePowerDownMode)
static void PWR_HandleDeepSleepMode_1(void)
{
#if cPWR_BLE_LL_Enable
#if (gTMR_EnableLowPowerTimers_d)   
  uint32_t notCountedTicksBeforeSleep= 0;
#endif  
  uint8_t clkMode;
  uint32_t lptmrTicks;
  uint32_t lptmrFreq;
  bool_t enterLowPower = TRUE;
  PWRLib_MCU_WakeupReason.AllBits = 0;  
  PWRLib_LPTMR_GetTimeSettings(mPWR_DeepSleepTimeMs ,&clkMode ,&lptmrTicks, &lptmrFreq);
  PWRLib_LPTMR_ClockStart(clkMode,lptmrTicks);
#if (gTMR_EnableLowPowerTimers_d)
  /* if more low power timers are running, stop the hardware timer
  and save the spend time in ticks that wasn't counted.  */
  notCountedTicksBeforeSleep = TMR_NotCountedTicksBeforeSleep();
#endif
  
  if(RSIM_BRD_CONTROL_BLE_RF_OSC_REQ_STAT(RSIM))// if false it means that BLE_LL is already in dsm
  {
    uint16_t instantTimer;
    uint16_t distanceToNextInstant;
    uint16_t nextInstant;
    uint16_t offsetBeforeDsmExit;   
    
    if(BLE_LL_REG(0xC4) & (uint16_t)(1<<7))
    {                  
        RSIM_BWR_CONTROL_BLE_DEEP_SLEEP_EXIT(RSIM, 0);
        instantTimer = PWRLib_BLL_GetInstantTimer();
        while(instantTimer == PWRLib_BLL_GetInstantTimer());
        instantTimer = PWRLib_BLL_GetInstantTimer();
        
        distanceToNextInstant = PWR_BLE_GetNearestInstant( &nextInstant);
        offsetBeforeDsmExit = PWRLib_BLL_GetOffsetSlotsBeforeDsmExit();
        if(distanceToNextInstant > offsetBeforeDsmExit + 1 )
        {
            PWRLib_BLL_SetWakeupInstant(nextInstant);
            PWRLib_BLL_EnterDSM();
        }  
        else
        {
            enterLowPower = FALSE;
        }
    }
    else
    {
        enterLowPower = FALSE;
    }
  }  
  
  if(enterLowPower)
  {
    if(gpfPWR_LowPowerEnterCb != NULL)
    {
      gpfPWR_LowPowerEnterCb();
    }
    PWRLib_MCU_Enter_LLS3();
    if(gpfPWR_LowPowerExitCb != NULL)
    {
      gpfPWR_LowPowerExitCb();
    }
    PWRLib_LLWU_UpdateWakeupReason();
    if(RSIM_BRD_CONTROL_BLE_RF_OSC_REQ_INT(RSIM) == 0)
    {
      RSIM_BWR_CONTROL_BLE_DEEP_SLEEP_EXIT(RSIM, 1);
    }
    PWRLib_BLL_WaitForDSMExit();
    RSIM_BWR_CONTROL_BLE_DEEP_SLEEP_EXIT(RSIM, 0);
  }      
  PWRLib_LPTMR_ClockStop();
  /* Sync. the low power timers */
#if (gTMR_EnableLowPowerTimers_d)
  {
    uint64_t timerTicks;
    timerTicks = ((uint64_t)PWRLib_LPTMR_ClockCheck()*TMR_GetTimerFreq())/lptmrFreq;
    timerTicks += notCountedTicksBeforeSleep;
    TMR_SyncLpmTimers((uint32_t)timerTicks);
  }
#endif     
#else
  PWRLib_MCU_WakeupReason.AllBits = 0;  
#endif  
}
#endif

/*---------------------------------------------------------------------------
 * Name: PWR_HandleDeepSleepMode_2
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
#if (cPWR_UsePowerDownMode)
static void PWR_HandleDeepSleepMode_2(void)
{
#if cPWR_BLE_LL_Enable
#if (gTMR_EnableLowPowerTimers_d)   
  uint32_t notCountedTicksBeforeSleep= 0;
#endif  
  uint8_t clkMode;
  uint32_t lptmrTicks;
  uint32_t lptmrFreq;
  bool_t enterLowPower = TRUE;
  PWRLib_MCU_WakeupReason.AllBits = 0;  
  PWRLib_LPTMR_GetTimeSettings(50000 ,&clkMode ,&lptmrTicks, &lptmrFreq);
  PWRLib_LPTMR_ClockStart(clkMode,lptmrTicks);
#if (gTMR_EnableLowPowerTimers_d)
  /* if more low power timers are running, stop the hardware timer
  and save the spend time in ticks that wasn't counted.  */
  notCountedTicksBeforeSleep = TMR_NotCountedTicksBeforeSleep();
#endif
  
  if(RSIM_BRD_CONTROL_BLE_RF_OSC_REQ_STAT(RSIM))// if false it means that BLE_LL is already in dsm
  {
    uint16_t instantTimer;
    uint16_t offsetBeforeDsmExit;
    uint32_t ticksToSleep;
    RSIM_BWR_CONTROL_BLE_DEEP_SLEEP_EXIT(RSIM, 0);
    ticksToSleep = (mPWR_DeepSleepTimeMs < mBLETimoutMaxMs_c)?(mPWR_DeepSleepTimeMs):(mBLETimoutMaxMs_c);
    ticksToSleep = (ticksToSleep * 1600 )/1000;
    instantTimer = PWRLib_BLL_GetInstantTimer();
    while(instantTimer == PWRLib_BLL_GetInstantTimer());
    instantTimer = PWRLib_BLL_GetInstantTimer();
    offsetBeforeDsmExit = PWRLib_BLL_GetOffsetSlotsBeforeDsmExit();
    if(ticksToSleep > offsetBeforeDsmExit + 1 )
    {
      PWRLib_BLL_SetWakeupInstant((uint16_t)(ticksToSleep + instantTimer));
      PWRLib_BLL_EnterDSM();
    }  
    else
    {
      enterLowPower = FALSE;
    }
  }
  if(enterLowPower)
  {
    if(gpfPWR_LowPowerEnterCb != NULL)
    {
      gpfPWR_LowPowerEnterCb();
    }
    PWRLib_MCU_Enter_LLS3();
    if(gpfPWR_LowPowerExitCb != NULL)
    {
      gpfPWR_LowPowerExitCb();
    }
    PWRLib_LLWU_UpdateWakeupReason();
    if(RSIM_BRD_CONTROL_BLE_RF_OSC_REQ_INT(RSIM) == 0)
    {
      RSIM_BWR_CONTROL_BLE_DEEP_SLEEP_EXIT(RSIM, 1);
    }
    PWRLib_BLL_WaitForDSMExit();
    RSIM_BWR_CONTROL_BLE_DEEP_SLEEP_EXIT(RSIM, 0);
  }      
  
  PWRLib_LPTMR_ClockStop();
  /* Sync. the low power timers */
#if (gTMR_EnableLowPowerTimers_d)
  {
    uint64_t timerTicks;
    timerTicks = ((uint64_t)PWRLib_LPTMR_ClockCheck()*TMR_GetTimerFreq())/lptmrFreq;
    timerTicks += notCountedTicksBeforeSleep;
    TMR_SyncLpmTimers((uint32_t)timerTicks);
  }
#endif
#else
  PWRLib_MCU_WakeupReason.AllBits = 0;  
#endif  
}
#endif
/*---------------------------------------------------------------------------
 * Name: PWR_HandleDeepSleepMode_3
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
#if (cPWR_UsePowerDownMode)
static void PWR_HandleDeepSleepMode_3(void)
{
  
#if (gTMR_EnableLowPowerTimers_d)   
  uint32_t notCountedTicksBeforeSleep= 0;
#endif  
  uint8_t clkMode;
  uint32_t lptmrTicks;
  uint32_t lptmrFreq;
#if cPWR_BLE_LL_Enable  
  uint16_t bleEnabledInt;
#endif  
  PWRLib_MCU_WakeupReason.AllBits = 0;  
#if cPWR_BLE_LL_Enable   
  if(RSIM_BRD_CONTROL_BLE_RF_OSC_REQ_STAT(RSIM)== 0)// BLL in DSM
  {
    return;
  }
#endif  
  PWRLib_LPTMR_GetTimeSettings(mPWR_DeepSleepTimeMs ,&clkMode ,&lptmrTicks, &lptmrFreq);
  PWRLib_LPTMR_ClockStart(clkMode,lptmrTicks);
#if (gTMR_EnableLowPowerTimers_d)
  /* if more low power timers are running, stop the hardware timer
  and save the spend time in ticks that wasn't counted.  */
  notCountedTicksBeforeSleep = TMR_NotCountedTicksBeforeSleep();
#endif
#if cPWR_BLE_LL_Enable 
  /* At this point the BLE_LL should be in idle state. BLE stack should assure that 
  before allowing device to sleep */
  bleEnabledInt = PWRLib_BLL_GetIntEn();
  PWRLib_BLL_ClearInterrupts(bleEnabledInt);    
  PWRLib_BLL_DisableInterrupts(bleEnabledInt);
#endif  
  if(gpfPWR_LowPowerEnterCb != NULL)
  {
    gpfPWR_LowPowerEnterCb();
  }
  PWRLib_MCU_Enter_LLS3();
  if(gpfPWR_LowPowerExitCb != NULL)
  {
    gpfPWR_LowPowerExitCb();
  }
  PWRLib_LLWU_UpdateWakeupReason();
#if cPWR_BLE_LL_Enable        
  PWRLib_BLL_EnableInterrupts(bleEnabledInt); 
#endif       
  PWRLib_LPTMR_ClockStop();
  
  /* Sync. the low power timers */
#if (gTMR_EnableLowPowerTimers_d)
  {
    uint64_t timerTicks;
    timerTicks = ((uint64_t)PWRLib_LPTMR_ClockCheck()*TMR_GetTimerFreq())/lptmrFreq;
    timerTicks += notCountedTicksBeforeSleep;
    TMR_SyncLpmTimers((uint32_t)timerTicks);
  }
#endif     
}
#endif

/*---------------------------------------------------------------------------
 * Name: PWR_HandleDeepSleepMode_4
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
#if (cPWR_UsePowerDownMode)
static void PWR_HandleDeepSleepMode_4(void)
{
#if cPWR_BLE_LL_Enable
  uint16_t bleEnabledInt;
#endif  
  PWRLib_MCU_WakeupReason.AllBits = 0;  
#if cPWR_BLE_LL_Enable  
  if(RSIM_BRD_CONTROL_BLE_RF_OSC_REQ_STAT(RSIM)== 0)// BLL in DSM
  {
    return;
  }
  bleEnabledInt = PWRLib_BLL_GetIntEn();
  PWRLib_BLL_ClearInterrupts(bleEnabledInt);    
  PWRLib_BLL_DisableInterrupts(bleEnabledInt);
#endif  
  if(gpfPWR_LowPowerEnterCb != NULL)
  {
    gpfPWR_LowPowerEnterCb();
  }
#if cPWR_DCDC_InBypass  
  PWRLib_MCU_Enter_VLLS0();
#else
  PWRLib_MCU_Enter_VLLS1();
#endif
  if(gpfPWR_LowPowerExitCb != NULL)
  {
    gpfPWR_LowPowerExitCb();
  }
#if cPWR_BLE_LL_Enable  
  PWRLib_BLL_EnableInterrupts(bleEnabledInt);        
#endif  
}
#endif

/*---------------------------------------------------------------------------
 * Name: PWR_HandleDeepSleepMode_5
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
#if (cPWR_UsePowerDownMode)
static void PWR_HandleDeepSleepMode_5(void)
{
#if cPWR_BLE_LL_Enable
  uint16_t bleEnabledInt;
#endif  
  PWRLib_MCU_WakeupReason.AllBits = 0;  
#if cPWR_BLE_LL_Enable  
  if(RSIM_BRD_CONTROL_BLE_RF_OSC_REQ_STAT(RSIM)== 0)// BLL in DSM
  {
    return;
  }
  bleEnabledInt = PWRLib_BLL_GetIntEn();
  PWRLib_BLL_ClearInterrupts(bleEnabledInt);    
  PWRLib_BLL_DisableInterrupts(bleEnabledInt);
#endif  
  if(gpfPWR_LowPowerEnterCb != NULL)
  {
    gpfPWR_LowPowerEnterCb();
  }
  PWRLib_MCU_Enter_VLLS2();
  if(gpfPWR_LowPowerExitCb != NULL)
  {
    gpfPWR_LowPowerExitCb();
  }
#if cPWR_BLE_LL_Enable  
  PWRLib_BLL_EnableInterrupts(bleEnabledInt);        
#endif  
}
#endif

/*---------------------------------------------------------------------------
 * Name: PWR_HandleDeepSleepMode_6
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
#if (cPWR_UsePowerDownMode)
static void PWR_HandleDeepSleepMode_6(void)
{
  
#if (gTMR_EnableLowPowerTimers_d)   
  uint32_t notCountedTicksBeforeSleep= 0;
#endif  
  uint8_t clkMode;
  uint32_t lptmrTicks;
  uint32_t lptmrFreq;
  uint32_t sysTickCtrl;
  PWRLib_MCU_WakeupReason.AllBits = 0;  
#if cPWR_BLE_LL_Enable   
  if(RSIM_BRD_CONTROL_BLE_RF_OSC_REQ_STAT(RSIM)== 0)// BLL in DSM
  {
    return;
  }
#endif  
  PWRLib_LPTMR_GetTimeSettings(mPWR_DeepSleepTimeMs ,&clkMode ,&lptmrTicks, &lptmrFreq);
  PWRLib_LPTMR_ClockStart(clkMode,lptmrTicks);
#if (gTMR_EnableLowPowerTimers_d)
  /* if more low power timers are running, stop the hardware timer
  and save the spend time in ticks that wasn't counted.  */
  notCountedTicksBeforeSleep = TMR_NotCountedTicksBeforeSleep();
#endif
  /* disable SysTick counter and interrupt */
  sysTickCtrl = SysTick->CTRL & (SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
  SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
  {
    uint32_t rfOscEn = RSIM_BRD_CONTROL_RF_OSC_EN(RSIM);
    RSIM_BWR_CONTROL_RF_OSC_EN(RSIM, 3);
    PWRLib_MCU_Enter_Stop();
    RSIM_BWR_CONTROL_RF_OSC_EN(RSIM, rfOscEn);
  }
  
  /* restore the state of SysTick */
  SysTick->CTRL |= sysTickCtrl; 
  /* checks sources of wakeup */
  PWRLib_StopUpdateWakeupReason();
  PWRLib_LPTMR_ClockStop();
  
  /* Sync. the low power timers */
#if (gTMR_EnableLowPowerTimers_d)
  {
    uint64_t timerTicks;
    timerTicks = ((uint64_t)PWRLib_LPTMR_ClockCheck()*TMR_GetTimerFreq())/lptmrFreq;
    timerTicks += notCountedTicksBeforeSleep;
    TMR_SyncLpmTimers((uint32_t)timerTicks);
  }
  
#endif     
}
#endif
/*---------------------------------------------------------------------------
 * Name: PWR_HandleDeepSleep
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
static PWRLib_WakeupReason_t PWR_HandleDeepSleep(void)
{
  PWRLib_MCU_WakeupReason.AllBits = 0;  
#if (cPWR_UsePowerDownMode)
  {
    uint8_t lpMode;
    lpMode = PWRLib_GetDeepSleepMode();
    if(lpMode)
    {
      maHandleDeepSleep[lpMode-1]();      
    }
  }
#endif
  return PWRLib_MCU_WakeupReason;            
}

/*---------------------------------------------------------------------------
 * Name: PWR_HandleSleep
 * Description: - 
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
static PWRLib_WakeupReason_t PWR_HandleSleep
(
)
{
  PWRLib_WakeupReason_t  Res;
  
  Res.AllBits = 0;
    
#if (cPWR_UsePowerDownMode)
  /*---------------------------------------------------------------------------*/
#if (cPWR_SleepMode==0)
  return Res;
  
  /*---------------------------------------------------------------------------*/
#elif (cPWR_SleepMode==1)
  /* radio in autodoze mode by default. mcu in wait mode */
  PWRLib_MCU_WakeupReason.AllBits = 0;
  PWRLib_MCU_Enter_Sleep();
  Res.Bits.SleepTimeout = 1;
  PWRLib_MCU_WakeupReason.Bits.SleepTimeout = 1;
  return Res;
  /*---------------------------------------------------------------------------*/
#else
#error "*** ERROR: Not a valid cPWR_SleepMode chosen"
#endif
#else  /* #if (cPWR_UsePowerDownMode) else */
  /* Last part to avoid unused warning */
  PWRLib_MCU_WakeupReason.AllBits = 0;
  return Res;          /* (PWRLib_WakeupReason_t) DozeDuration */
#endif  /* #if (cPWR_UsePowerDownMode) end */
}

/*---------------------------------------------------------------------------
 * Name: PWR_SleepAllowed
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
static bool_t PWR_SleepAllowed
(
void
)
{
#if (cPWR_UsePowerDownMode)
#if cPWR_BLE_LL_Enable
#endif  
#if cPWR_Zigbee_Enable
  if((PWRLib_GetCurrentZigbeeStackPowerState != StackPS_Sleep) &&  \
    (PWRLib_GetCurrentZigbeeStackPowerState != StackPS_DeepSleep) )
  {
    return FALSE;
  }
  if( PWRLib_GetMacStateReq()== gMacStateBusy_c)
  {
    return FALSE;
  }
#endif
  return TRUE;
#else
  return TRUE;
#endif  /* #if (cPWR_UsePowerDownMode) else */
}

/*---------------------------------------------------------------------------
 * Name: PWR_DeepSleepAllowed
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
static bool_t PWR_DeepSleepAllowed
(
void
)
{
#if (cPWR_UsePowerDownMode)
#if cPWR_BLE_LL_Enable
#endif  
#if cPWR_Zigbee_Enable  
  if (PWRLib_GetCurrentZigbeeStackPowerState != StackPS_DeepSleep)
  {
    return FALSE;
  }
  if ( PWRLib_GetMacStateReq() != gMacStateIdle_c)
  {
    if(PWRLib_GetDeepSleepMode() == 6)
    {
      if ( PWRLib_GetMacStateReq() != gMacIdleRx_c)
      {
        return FALSE;
      }
    }
    else
    {
    return FALSE;
    }
    
  }
#endif  
  return TRUE;
#else
  return TRUE;
#endif  /* #if (cPWR_UsePowerDownMode)*/
}

/*---------------------------------------------------------------------------
 * Name: PWR_CheckForAndEnterNewPowerState
 * Description: - 
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
static PWRLib_WakeupReason_t PWR_CheckForAndEnterNewPowerState
(
PWR_CheckForAndEnterNewPowerState_t NewPowerState
)
{
  PWRLib_WakeupReason_t ReturnValue;
  ReturnValue.AllBits = 0;
  
#if (cPWR_UsePowerDownMode)
  if ( NewPowerState == PWR_Run)
  {
    /* ReturnValue = 0; */
  }
  else if( NewPowerState == PWR_OFF)
  {
    // disable all wake up sources
    void PWRLib_LLWU_DisableAllWakeupSources();
    /* configure MCU in VLLS0 mode */
    PWR_HandleDeepSleepMode_4();
    /* Never returns */
    for(;;){}
    
  }
  else if( NewPowerState == PWR_Reset)
  {
    /* Never returns */
    PWR_SystemReset();
  }
  
  else if( NewPowerState == PWR_DeepSleep )
  {
    if(PWR_CheckIfDeviceCanGoToSleep() && PWR_DeepSleepAllowed())
    {
      ReturnValue = PWR_HandleDeepSleep();
    }
  } 
  else if( NewPowerState == PWR_Sleep )
  {
    if(PWR_CheckIfDeviceCanGoToSleep() && PWR_SleepAllowed())
    {
      ReturnValue = PWR_HandleSleep();
    }
  }
  else
  {
    /* ReturnValue = FALSE; */
  }
  /* Clear wakeup reason */
  
#else
  /* To remove warning for variabels in functioncall */
  (void)NewPowerState;
#endif  /* #if (cPWR_UsePowerDownMode) */
  
  return ReturnValue;
}

/*****************************************************************************
 *                             PUBLIC FUNCTIONS                              *
 *---------------------------------------------------------------------------*
 * Add to this section all the functions that have global (project) scope.   *
 * These functions can be accessed outside this module.                      *
 * These functions shall have their declarations (prototypes) within the     *
 * interface header file and shall be preceded by the 'extern' keyword.      *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
 
/*---------------------------------------------------------------------------
 * Name: PWR_Init
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWR_Init
(
  void
)
{
#if (cPWR_UsePowerDownMode)

  PWRLib_Init();

#endif  /* #if (cPWR_UsePowerDownMode) */
}
/*---------------------------------------------------------------------------
 * Name: PWR_GetSystemResetStatus
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint16_t PWR_GetSystemResetStatus
(
  void
)
{
  uint16_t resetStatus = 0;
  resetStatus = (uint16_t) (RCM_SRS0);
  resetStatus |= (uint16_t)(RCM_SRS1 << 8);
  return resetStatus;
}
/*---------------------------------------------------------------------------
 * Name: PWR_SystemReset
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWR_SystemReset
(
void
)
{
  SCB->AIRCR =  ((uint32_t)0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
  while(1);
}
/*---------------------------------------------------------------------------
 * Name: PWR_SetDeepSleepTimeInMs
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWR_SetDeepSleepTimeInMs
(
uint32_t deepSleepTimeMs
)
{
#if (cPWR_UsePowerDownMode)
  if(deepSleepTimeMs == 0) 
  {
    return;
  }
  mPWR_DeepSleepTimeMs = deepSleepTimeMs; 
#else
  (void) deepSleepTimeMs;
#endif
}

/*---------------------------------------------------------------------------
 * Name: PWR_SetDeepSleepTimeInSymbols
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWR_SetDeepSleepTimeInSymbols
(
uint32_t deepSleepTimeSymbols
)
{
#if (cPWR_UsePowerDownMode)
  if(deepSleepTimeSymbols == 0) 
  {
    return;
  }
  mPWR_DeepSleepTimeMs = (deepSleepTimeSymbols*16)/1000; 
#else
  (void) deepSleepTimeSymbols;
#endif
}

/*---------------------------------------------------------------------------
 * Name: PWR_AllowDeviceToSleep
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWR_AllowDeviceToSleep
(
void
)
{
#if (cPWR_UsePowerDownMode)
  OSA_EnterCritical(kCriticalDisableInt);
  
  if( mLPMFlag != 0 ){    
    mLPMFlag--;
  }
  OSA_ExitCritical(kCriticalDisableInt);
#endif
}

/*---------------------------------------------------------------------------
 * Name: PWR_DisallowDeviceToSleep
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWR_DisallowDeviceToSleep
(
void
)
{
#if (cPWR_UsePowerDownMode)
  uint8_t prot;
  OSA_EnterCritical(kCriticalDisableInt);
  prot = mLPMFlag + 1;
  if(prot != 0)
  {
    mLPMFlag++;
  }
  OSA_ExitCritical(kCriticalDisableInt);
#endif
}

/*---------------------------------------------------------------------------
 * Name: PWR_CheckIfDeviceCanGoToSleep
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
bool_t PWR_CheckIfDeviceCanGoToSleep
(
void
)
{
  bool_t   returnValue;
  OSA_EnterCritical(kCriticalDisableInt);
  returnValue = mLPMFlag == 0 ? TRUE : FALSE;
  OSA_ExitCritical(kCriticalDisableInt);
  return returnValue;
}



/*---------------------------------------------------------------------------
 * Name: PWR_ChangeDeepSleepMode
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
bool_t PWR_ChangeDeepSleepMode
(
uint8_t dsMode
)
{
  
#if (cPWR_UsePowerDownMode)
  if(dsMode > 6)
  {
    return FALSE;
  }
  PWRLib_SetDeepSleepMode(dsMode);
  PWRLib_ConfigLLWU(dsMode);
#if (cPWR_BLE_LL_Enable)  
  PWRLib_BLL_ConfigDSM(dsMode);
  PWRLib_ConfigRSIM(dsMode);
#endif  
  return TRUE;
#else
  return TRUE;
#endif  /* #if (cPWR_UsePowerDownMode)*/
} 

/*---------------------------------------------------------------------------
 * Name: PWR_GetDeepSleepMode
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t PWR_GetDeepSleepMode(void)
{
  
#if (cPWR_UsePowerDownMode)
   return PWRLib_GetDeepSleepMode();
#else
  return 0;
#endif  /* #if (cPWR_UsePowerDownMode)*/
}



/*---------------------------------------------------------------------------
 * Name: PWR_EnterPowerOff
 * Description: - Radio on Reset, MCU on VLLS1
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWR_EnterPowerOff(void)
{
  OSA_EnterCritical(kCriticalDisableInt);
  (void)PWR_CheckForAndEnterNewPowerState(PWR_OFF);
  OSA_ExitCritical(kCriticalDisableInt);
}
/*---------------------------------------------------------------------------
 * Name: PWR_BLE_EnterDSM
 * Description: - 
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWR_BLE_EnterDSM(uint16_t wakeupInstant)
{
#if cPWR_BLE_LL_Enable
  uint8_t dsMode;
  dsMode = PWRLib_GetDeepSleepMode();
  if((dsMode == 1) || (dsMode == 2))
  {
    if(RSIM_BRD_CONTROL_BLE_RF_OSC_REQ_STAT(RSIM))
    {
      RSIM_BWR_CONTROL_BLE_DEEP_SLEEP_EXIT(RSIM, 0);
      PWRLib_BLL_SetWakeupInstant(wakeupInstant);
      PWRLib_BLL_EnterDSM();
    }
  }

#endif
}
/*---------------------------------------------------------------------------
 * Name: PWR_BLE_GetReferenceClock
 * Description: - 
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint16_t PWR_BLE_GetReferenceClock(void)
{
  #if cPWR_BLE_LL_Enable
      return PWRLib_BLL_GetInstantTimer();
  #else
      return 0;  
  #endif
}
/*---------------------------------------------------------------------------
 * Name: PWR_BLE_ExitDSM
 * Description: - 
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWR_BLE_ExitDSM(void)
{
  #if cPWR_BLE_LL_Enable
  if(RSIM_BRD_CONTROL_BLE_RF_OSC_REQ_STAT(RSIM)== 0)
  {
    RSIM_BWR_CONTROL_BLE_DEEP_SLEEP_EXIT(RSIM, 1);
  }

  #endif
}
/*---------------------------------------------------------------------------
 * Name: PWR_LVD_ReportLevel
 * Description: - 
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
PWRLib_LVD_VoltageLevel_t PWR_LVD_ReportLevel
(
void
)
{
  PWRLib_LVD_VoltageLevel_t   Level;
#if ((cPWR_LVD_Enable == 0) || (cPWR_LVD_Enable == 3))
  Level = PWR_ABOVE_LEVEL_3_0V;
#elif (cPWR_LVD_Enable==1)
  Level = PWRLib_LVD_CollectLevel();
#elif (cPWR_LVD_Enable==2)
  Level = PWRLib_LVD_SavedLevel;
#else
#error "*** ERROR: Illegal value for cPWR_LVD_Enable"
#endif /* #if (cPWR_LVD_Enable) */
  return Level;
}

/*---------------------------------------------------------------------------
 * Name: PWR_EnterLowPower
 * Description: - 
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
PWRLib_WakeupReason_t PWR_EnterLowPower
(
void
)
{
  
  PWRLib_WakeupReason_t ReturnValue;  
#if (gTMR_EnableLowPowerTimers_d) 
  bool_t unlockTMRThread = FALSE;
#endif
  ReturnValue.AllBits = 0;
  
  if (PWR_LVD_ReportLevel() == PWR_LEVEL_CRITICAL)
  {
    /* Voltage <= 1.8V so enter power-off state - to disable false Tx'ing(void)*/
    ReturnValue = PWR_CheckForAndEnterNewPowerState( PWR_OFF);
  }
  OSA_EnterCritical(kCriticalDisableInt);
  
#if cPWR_Zigbee_Enable    
  PWRLib_SetCurrentZigbeeStackPowerState(StackPS_DeepSleep);
#endif  
  if (
      TMR_AreAllTimersOff()
     )  /*No timer running*/
  {
    /* if power lib is enabled */	
#if (cPWR_UsePowerDownMode)
    /* if Low Power Capability is enabled */
#if (gTMR_EnableLowPowerTimers_d) 
    /* if more low power timers are running, stop the hardware timer
    and save the spend time in ticks that wasn't counted.
    */
    unlockTMRThread = TRUE;
#endif /* #if (gTMR_EnableLowPowerTimers_d)  */
#endif /* #if (cPWR_UsePowerDownMode)  */
    
    ReturnValue = PWR_CheckForAndEnterNewPowerState (PWR_DeepSleep);
  }
  else /*timers are running*/
  { 	 
    ReturnValue = PWR_CheckForAndEnterNewPowerState (PWR_Sleep);
  }
  OSA_ExitCritical(kCriticalDisableInt);
  
#if (gTMR_EnableLowPowerTimers_d)
  if(unlockTMRThread)
  {
    TMR_MakeTMRThreadReady();
  }
  
#endif    
  return ReturnValue;
}

/*---------------------------------------------------------------------------
 * Name: PWR_RegisterLowPowerEnterCallback
 * Description: - 
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/

void PWR_RegisterLowPowerEnterCallback
(
pfPWRCallBack_t lowPowerEnterCallback
)
{
 #if (cPWR_UsePowerDownMode)
  gpfPWR_LowPowerEnterCb = lowPowerEnterCallback;
 #endif 
}

/*---------------------------------------------------------------------------
 * Name: PWR_RegisterLowPowerExitCallback
 * Description: - 
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/

void PWR_RegisterLowPowerExitCallback
(
pfPWRCallBack_t lowPowerExitCallback
)
{
 #if (cPWR_UsePowerDownMode)
  gpfPWR_LowPowerExitCb = lowPowerExitCallback;
 #endif 
}