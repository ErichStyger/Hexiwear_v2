/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file TimersManager.h
* TIMER interface file for the ARM CORTEX-M4 processor
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

#ifndef __TMR_INTERFACE_H__
#define __TMR_INTERFACE_H__

#include "EmbeddedTypes.h"


/*****************************************************************************
******************************************************************************
* Public macros
******************************************************************************
*****************************************************************************/

/*
 * NAME: gTMR_Enabled_d
 * DESCRIPTION: Enables / Disables the Timer platform component
 * VALID RANGE: TRUE/FALSE
 */
#ifndef gTMR_Enabled_d
#define gTMR_Enabled_d    1
#endif

/*
 * NAME: gTimestamp_Enabled_d
 * DESCRIPTION: Enables / Disables the timestamp feature of the timers platform component
 * VALID RANGE: TRUE/FALSE
 */
#ifndef gTimestamp_Enabled_d 
#define gTimestamp_Enabled_d 1
#endif

/*
 * NAME: gTMR_PIT_Timestamp_Enabled_d
 * DESCRIPTION: Enables / Disables the PIT based timestamp feature of the timers platform component
 * VALID RANGE: TRUE/FALSE
*/
#ifndef gTMR_PIT_Timestamp_Enabled_d 
#define gTMR_PIT_Timestamp_Enabled_d  1
#endif


#if gTMR_PIT_Timestamp_Enabled_d
/*
 * NAME: gTMR_PIT_FreqMultipleOfMHZ_d
 * DESCRIPTION: Set this define TRUE if the PIT frequency is an integer number of MHZ.
                The function TMR_PITGetTimestamp will be optimised and will have no jitter.
 * VALID RANGE: TRUE/FALSE
*/
#ifndef gTMR_PIT_FreqMultipleOfMHZ_d 
#define gTMR_PIT_FreqMultipleOfMHZ_d 1
#endif
#endif

/*
 * NAME: gPrecision_Timers_Enabled_d
 * DESCRIPTION: Enables / Disables the precision timers platform component
 * VALID RANGE: TRUE/FALSE
 */
#ifndef gPrecision_Timers_Enabled_d 
#define gPrecision_Timers_Enabled_d 0
#endif

/*
 * NAME: gTmrTaskStackSize_c
 * DESCRIPTION: Configures the timer task stack size.
 * VALID RANGE:
 */
#ifndef gTmrTaskStackSize_c 
#define gTmrTaskStackSize_c 600
#endif

/*
 * NAME: gTmrTaskPriority_c
 * DESCRIPTION: Configures the timer task priority.
 * VALID RANGE:
 */
#ifndef gTmrTaskPriority_c
#define gTmrTaskPriority_c                 2
#endif

/*
 * NAME: gTMR_FTM_CNx
 * DESCRIPTION: Specifies which FTM CHANNEL is used
 * VALID RANGE: 0..7
 */
#ifndef gTMR_FTM_CNx
#define gTMR_FTM_CNx    0
#endif

/*
 * NAME: gTMR_EnableLowPowerTimers_d
 * DESCRIPTION: Enable/Disable Low Power Timer
 * VALID RANGE: TRUE/FALSE
 */
#ifndef gTMR_EnableLowPowerTimers_d
#define gTMR_EnableLowPowerTimers_d    (1)
#endif

/*
 * NAME: gTMR_EnableHWLowPowerTimers_d
 * DESCRIPTION: Enables the low power timers based on the 
 * LPTMR hardware module. Setting, available ONLY in 
 * deep sleep mode 14 and 15.
 */

#ifndef gTMR_EnableHWLowPowerTimers_d
#define gTMR_EnableHWLowPowerTimers_d    (0)
#endif

#if ((gTMR_EnableHWLowPowerTimers_d == 1) && (gTMR_EnableLowPowerTimers_d == 1))
#error "*** ERROR: gTMR_EnableLowPowerTimers_d needs to be set to 0 if hardware low-power timers are enabled"
#endif

/*
 * NAME: gTMR_EnableMinutesSecondsTimers_d
 * DESCRIPTION:  Enable/Disable Minutes and Seconds Timers
 * VALID RANGE: TRUE/FALSE
 */
#ifndef gTMR_EnableMinutesSecondsTimers_d
#define gTMR_EnableMinutesSecondsTimers_d   (1)
#endif

/*
 * NAME: gTmrApplicationTimers_c
 * DESCRIPTION: Number of timers needed by the application
 * VALID RANGE: user defined
 */
#ifndef gTmrApplicationTimers_c
#define gTmrApplicationTimers_c 4
#endif

/*
 * NAME: gTmrStackTimers_c
 * DESCRIPTION: Number of timers needed by the protocol stack
 * VALID RANGE: user defined
 */
#ifndef gTmrStackTimers_c
#define gTmrStackTimers_c   4
#endif

/*
 * NAME: gTmrTotalTimers_c
 * DESCRIPTION: Total number of timers
 * VALID RANGE: sum of application and stack timers
 */
#ifndef gTmrTotalTimers_c
#define gTmrTotalTimers_c   ( gTmrApplicationTimers_c + gTmrStackTimers_c )
#endif

/*
 * NAME: TmrMilliseconds()
 * DESCRIPTION: Typecast the macro argument into milliseconds
 * VALID RANGE: -
 */
#define TmrMilliseconds( n )    ( (tmrTimeInMilliseconds_t) (n) )
   
/*
 * NAME: TmrSecondsToMicroseconds()
 * DESCRIPTION: Converts the macro argument from seconds to microseconds
 * VALID RANGE: -
 */
#define TmrSecondsToMicroseconds( n )   ( (uint64_t) ((n) * 1000000) )
   
/*
 * NAME: TmrMicroecondsToSeconds()
 * DESCRIPTION: Converts the macro argument from microseconds to seconds
 * VALID RANGE: -
 */
#define TmrMicrosecondsToSeconds( n )   ( ((n) + 500000) / 1000000 )

/*
 * NAME: TmrSeconds()
 * DESCRIPTION: Converts the macro argument (i.e. seconds) into milliseconds
 * VALID RANGE: -
 */
#define TmrSeconds( n )         ( (tmrTimeInMilliseconds_t) (TmrMilliseconds(n) * 1000) )

/*
 * NAME: TmrMinutes()
 * DESCRIPTION: Converts the macro argument (i.e. minutes) into milliseconds
 * VALID RANGE: -
 */
#define TmrMinutes( n )         ( (tmrTimeInMilliseconds_t) (TmrSeconds(n) * 60) )

/*
 * NAME: gTmrInvalidTimerID_c
 * DESCRIPTION: Reserved for invalid timer id
 * VALID RANGE: 0xFF
 */
#define gTmrInvalidTimerID_c    0xFF

/*
 * NAME: gTmrSingleShotTimer_c, gTmrIntervalTimer_c,
 *       gTmrSetMinuteTimer_c, gTmrSetSecondTimer_c,
 *       gTmrLowPowerTimer_c
 * DESCRIPTION: Timer types coded values
 * VALID RANGE: see definitions below
 */
#define gTmrSingleShotTimer_c   0x01
#define gTmrIntervalTimer_c     0x02
#define gTmrSetMinuteTimer_c    0x04
#define gTmrSetSecondTimer_c    0x08
#define gTmrLowPowerTimer_c     0x10

/*
 * NAME: gTmrMinuteTimer_c
 * DESCRIPTION: Minute timer definition
 * VALID RANGE: see definition below
 */
#define gTmrMinuteTimer_c       ( gTmrSetMinuteTimer_c )

/*
 * NAME: gTmrSecondTimer_c
 * DESCRIPTION: Second timer definition
 * VALID RANGE: see definition below
 */
#define gTmrSecondTimer_c       ( gTmrSetSecondTimer_c )

/*
 * NAME: See below
 * DESCRIPTION: LP minute/second/millisecond timer definitions
 * VALID VALUES: See definitions below
 */
#define gTmrLowPowerMinuteTimer_c           ( gTmrMinuteTimer_c | gTmrLowPowerTimer_c )
#define gTmrLowPowerSecondTimer_c           ( gTmrSecondTimer_c | gTmrLowPowerTimer_c )
#define gTmrLowPowerSingleShotMillisTimer_c ( gTmrSingleShotTimer_c | gTmrLowPowerTimer_c )
#define gTmrLowPowerIntervalMillisTimer_c   ( gTmrIntervalTimer_c | gTmrLowPowerTimer_c )

/*****************************************************************************
******************************************************************************
* Public type definitions
******************************************************************************
*****************************************************************************/

typedef enum tmrErrCode_tag{
    gTmrSuccess_c,
    gTmrInvalidId_c,
    gTmrOutOfRange_c
}tmrErrCode_t;

/*
 * NAME: tmrTimerTicks_t
 * DESCRIPTION: 16-bit timer ticks type definition
 * VALID VALUES: see definition
 */
typedef uint16_t tmrTimerTicks16_t;

/*
 * NAME: tmrTimerTicks32_t
 * DESCRIPTION: 32-bit timer ticks type definition
 * VALID VALUES: see definition
 */
typedef uint32_t tmrTimerTicks32_t;

/*
 * NAME: tmrTimerTicks64_t
 * DESCRIPTION: 64-bit timer ticks type definition
 * VALID VALUES: see definition
 */
typedef uint64_t tmrTimerTicks64_t;

/*
 * NAME: tmrTimeInMilliseconds_t
 * DESCRIPTION: Times specified in milliseconds (max 0xFFFFFFFF)
 */
typedef uint32_t    tmrTimeInMilliseconds_t;

/*
 * NAME: tmrTimeInMinutes_t
 * DESCRIPTION: Times specified in minutes (up to 0xFFFFFFFF/60000)
 */
typedef uint32_t    tmrTimeInMinutes_t;

/*
 * NAME: tmrTimeInSeconds_t
 * DESCRIPTION: Times specified in seconds (up to 0xFFFFFFFF/1000)
 */
typedef uint32_t    tmrTimeInSeconds_t;

/*
 * NAME: tmrTimerType_t
 * DESCRIPTION: Timer type
 */
typedef uint8_t     tmrTimerID_t;

/*
 * NAME: tmrTimerType_t
 * DESCRIPTION: Timer type
 */
typedef uint8_t     tmrTimerType_t;

/*
 * NAME: pfTmrCallBack_t
 * DESCRIPTION: Timer callback function
 */
typedef void ( *pfTmrCallBack_t ) ( void * );


/*****************************************************************************
******************************************************************************
* Public memory declarations
******************************************************************************
*****************************************************************************/

/* none */

/*****************************************************************************
******************************************************************************
* Public prototypes
******************************************************************************
*****************************************************************************/

#if gTMR_Enabled_d

/*---------------------------------------------------------------------------
 * NAME: TMR_Init
 * DESCRIPTION: initialize the timer module
 * PARAMETERS: -
 * RETURN: -
 *---------------------------------------------------------------------------*/
void TMR_Init
(
    void
);

/*---------------------------------------------------------------------------
 * NAME: TMR_NotifyClkChanged
 * DESCRIPTION: This function is called when the clock is changed
 * PARAMETERS: IN: clkKhz (uint32_t) - new clock
 * RETURN: -
 *---------------------------------------------------------------------------*/
void TMR_NotifyClkChanged
(
    uint32_t clkKhz
);

/*---------------------------------------------------------------------------
 * NAME: TMR_AllocateTimer
 * DESCRIPTION: allocate a timer
 * PARAMETERS: -
 * RETURN: timer ID
 *---------------------------------------------------------------------------*/
tmrTimerID_t TMR_AllocateTimer
(
    void
);

/*---------------------------------------------------------------------------
 * NAME: TMR_AreAllTimersOff
 * DESCRIPTION: Check if all timers except the LP timers are OFF.
 * PARAMETERS: -
 * RETURN: TRUE if there are no active non-low power timers, FALSE otherwise
 *---------------------------------------------------------------------------*/
bool_t TMR_AreAllTimersOff
(
    void
);

/*---------------------------------------------------------------------------
 * NAME: TMR_FreeTimer
 * DESCRIPTION: Free a timer
 * PARAMETERS:  IN: timerID - the ID of the timer
 * RETURN: -
 * NOTES: Safe to call even if the timer is running.
 *        Harmless if the timer is already free.
 *---------------------------------------------------------------------------*/
tmrErrCode_t TMR_FreeTimer
(
    tmrTimerID_t timerID
);

/*---------------------------------------------------------------------------
 * NAME: TMR_IsTimerActive
 * DESCRIPTION: Check if a specified timer is active
 * PARAMETERS: IN: timerID - the ID of the timer
 * RETURN: TRUE if the timer (specified by the timerID) is active,
 *         FALSE otherwise
 *---------------------------------------------------------------------------*/
bool_t TMR_IsTimerActive
(
    tmrTimerID_t timerID
);

/*---------------------------------------------------------------------------
 * NAME: TMR_IsTimerReady
 * DESCRIPTION: Check if a specified timer is ready
 * PARAMETERS: IN: timerID - the ID of the timer
 * RETURN: TRUE if the timer (specified by the timerID) is ready,
 *         FALSE otherwise
 *---------------------------------------------------------------------------*/
bool_t TMR_IsTimerReady
(
    tmrTimerID_t timerID
);

/*---------------------------------------------------------------------------
 * NAME: TMR_GetRemainingTime
 * DESCRIPTION: Returns the remaining time until timeout, for the specified
 *              timer
 * PARAMETERS: IN: timerID - the ID of the timer
 * RETURN: remaining time in milliseconds until timer timeouts.
 *---------------------------------------------------------------------------*/
uint32_t TMR_GetRemainingTime
(
    tmrTimerID_t tmrID
);

/*---------------------------------------------------------------------------
 * NAME: TMR_StartTimer (BeeStack or application)
 * DESCRIPTION: Start a specified timer
 * PARAMETERS: IN: timerId - the ID of the timer
 *             IN: timerType - the type of the timer
 *             IN: timeInMilliseconds - time expressed in millisecond units
 *             IN: pfTmrCallBack - callback function
  *            IN: param - parameter to callback function
 * RETURN: -
 * NOTES: When the timer expires, the callback function is called in
 *        non-interrupt context. If the timer is already running when
 *        this function is called, it will be stopped and restarted.
 *---------------------------------------------------------------------------*/
tmrErrCode_t TMR_StartTimer
(
    tmrTimerID_t timerID,
    tmrTimerType_t timerType,
    tmrTimeInMilliseconds_t timeInMilliseconds,
    void (*pfTimerCallBack)(void *),
    void *param
);

/*---------------------------------------------------------------------------
 * NAME: TMR_StartLowPowerTimer
 * DESCRIPTION: Start a low power timer. When the timer goes off, call the
 *              callback function in non-interrupt context.
 *              If the timer is running when this function is called, it will
 *              be stopped and restarted.
 *              Start the timer with the following timer types:
 *                          - gTmrLowPowerMinuteTimer_c
 *                          - gTmrLowPowerSecondTimer_c
 *                          - gTmrLowPowerSingleShotMillisTimer_c
 *                          - gTmrLowPowerIntervalMillisTimer_c
 *              The MCU can enter in low power if there are only active low
 *              power timers.
 * PARAMETERS: IN: timerId - the ID of the timer
 *             IN: timerType - the type of the timer
 *             IN: timeIn - time in ticks
 *             IN: pfTmrCallBack - callback function
 *             IN: param - parameter to callback function
 * RETURN: type/DESCRIPTION
 *---------------------------------------------------------------------------*/
tmrErrCode_t TMR_StartLowPowerTimer
(
    tmrTimerID_t timerId,
    tmrTimerType_t timerType,
    uint32_t timeIn,
    void (*pfTmrCallBack)(void *),
    void *param
);

/*---------------------------------------------------------------------------
 * NAME: TMR_StartMinuteTimer
 * DESCRIPTION: Starts a minutes timer
 * PARAMETERS:  IN: timerId - the ID of the timer
 *              IN: timeInMinutes - time expressed in minutes
 *              IN: pfTmrCallBack - callback function
 *              IN: param - parameter to callback function
 * RETURN: None
 * NOTES: Customized form of TMR_StartTimer(). This is a single shot timer.
 *        There are no interval minute timers.
 *---------------------------------------------------------------------------*/
#if gTMR_EnableMinutesSecondsTimers_d
tmrErrCode_t TMR_StartMinuteTimer
(
    tmrTimerID_t timerId,
    tmrTimeInMinutes_t timeInMinutes,
    void (*pfTmrCallBack)(void *),
    void *param
);
#endif

/*---------------------------------------------------------------------------
 * NAME: TMR_StartSecondTimer
 * DESCRIPTION: Starts a seconds timer
 * PARAMETERS:  IN: timerId - the ID of the timer
 *              IN: timeInSeconds - time expressed in seconds
 *              IN: pfTmrCallBack - callback function
 *              IN: param - parameter to callback function
 * RETURN: None
 * NOTES: Customized form of TMR_StartTimer(). This is a single shot timer.
 *        There are no interval seconds timers.
 *---------------------------------------------------------------------------*/
#if gTMR_EnableMinutesSecondsTimers_d
tmrErrCode_t TMR_StartSecondTimer
(
    tmrTimerID_t timerId,
    tmrTimeInSeconds_t timeInSeconds,
    void (*pfTmrCallBack)(void *),
    void *param
);
#endif

/*---------------------------------------------------------------------------
 * NAME: TMR_StartIntervalTimer
 * DESCRIPTION: Starts an interval count timer
 * PARAMETERS:  IN: timerId - the ID of the timer
 *              IN: timeInMilliseconds - time expressed in milliseconds
 *              IN: pfTmrCallBack - callback function
 *              IN: param - parameter to callback function
 * RETURN: None
 * NOTES: Customized form of TMR_StartTimer()
 *---------------------------------------------------------------------------*/
tmrErrCode_t TMR_StartIntervalTimer
(
    tmrTimerID_t timerID,
    tmrTimeInMilliseconds_t timeInMilliseconds,
    void (*pfTimerCallBack)(void *),
    void *param
);

/*---------------------------------------------------------------------------
 * NAME: TMR_StartSingleShotTimer
 * DESCRIPTION: Starts an single-shot timer
 * PARAMETERS:  IN: timerId - the ID of the timer
 *              IN: timeInMilliseconds - time expressed in milliseconds
 *              IN: pfTmrCallBack - callback function
 *              IN: param - parameter to callback function
 * RETURN: None
 * NOTES: Customized form of TMR_StartTimer()
 *---------------------------------------------------------------------------*/
tmrErrCode_t TMR_StartSingleShotTimer
(
    tmrTimerID_t timerID,
    tmrTimeInMilliseconds_t timeInMilliseconds,
    void (*pfTimerCallBack)(void *),
    void *param
);

/*---------------------------------------------------------------------------
 * NAME: TMR_StopTimer
 * DESCRIPTION: Stop a timer
 * PARAMETERS:  IN: timerID - the ID of the timer
 * RETURN: None
 * NOTES: Associated timer callback function is not called, even if the timer
 *        expires. Does not frees the timer. Safe to call anytime, regardless
 *        of the state of the timer.
 *---------------------------------------------------------------------------*/
tmrErrCode_t TMR_StopTimer
(
    tmrTimerID_t timerID
);


/*---------------------------------------------------------------------------
 * NAME: TMR_EnableTimer
 * DESCRIPTION: Enable the specified timer
 * PARAMETERS:  IN: tmrID - the timer ID
 * RETURN: None
 * NOTES: none
 *---------------------------------------------------------------------------*/
void TMR_EnableTimer
(
    tmrTimerID_t tmrID
);

/*---------------------------------------------------------------------------
 * NAME: TMR_NotCountedMillisTimeBeforeSleep
 * DESCRIPTION: This function is called by Low Power module;
 * Also this function stops the hardware timer.
 * PARAMETERS:  none
 * RETURN: uint32 - time in millisecond that wasn't counted before
 *        entering in sleep
 * NOTES: none
 *---------------------------------------------------------------------------*/
uint16_t TMR_NotCountedTicksBeforeSleep
(
    void
);

/*---------------------------------------------------------------------------
 * NAME: TMR_SyncLpmTimers
 * DESCRIPTION: This function is called by the Low Power module
 * each time the MCU wakes up.
 * PARAMETERS:  sleep duration in milliseconds
 * RETURN: none
 * NOTES: none
 *---------------------------------------------------------------------------*/
void TMR_SyncLpmTimers
(
    uint32_t sleepDurationTmrTicks
);
/*---------------------------------------------------------------------------
 * NAME: TMR_MakeTMRThreadReady
 * DESCRIPTION: This function is called by the Low Power module
 * each time the MCU wakes up after low power timers synchronisation.
 * PARAMETERS:  
 * RETURN: none
 * NOTES: none
 *---------------------------------------------------------------------------*/
void TMR_MakeTMRThreadReady
(
 void   
);
/*---------------------------------------------------------------------------
 * NAME: TmrTicksFromMilliseconds
 * DESCRIPTION: Convert milliseconds to ticks
 * PARAMETERS:  IN: milliseconds
 * RETURN: tmrTimerTicks64_t - ticks number
 * NOTES: none
 *---------------------------------------------------------------------------*/
tmrTimerTicks64_t TmrTicksFromMilliseconds
(
    tmrTimeInMilliseconds_t milliseconds
);

/*---------------------------------------------------------------------------
 * NAME: TMR_GetTimerFreq
 * DESCRIPTION: 
 * PARAMETERS: -
 * RETURN: -
 *---------------------------------------------------------------------------*/
uint32_t TMR_GetTimerFreq(void);

/*---------------------------------------------------------------------------
 * NAME: TMR_TimeStampInit
 * DESCRIPTION: 
 * PARAMETERS: -
 * RETURN: -
 *---------------------------------------------------------------------------*/
void TMR_TimeStampInit(void);

/*---------------------------------------------------------------------------
 * NAME: TMR_GetTimestamp
 * DESCRIPTION: 
 * PARAMETERS: -
 * RETURN: -
 *---------------------------------------------------------------------------*/
uint64_t TMR_GetTimestamp(void);


/*---------------------------------------------------------------------------
 * NAME: TMR_AllocateMinuteTimer
 * DESCRIPTION: Reserve a minute timer
 * PARAMETERS: -
 * RETURN: gTmrInvalidTimerID_c if there are no timers available
 * NOTES: none
 *---------------------------------------------------------------------------*/
#define TMR_AllocateMinuteTimer() TMR_AllocateTimer()


/*---------------------------------------------------------------------------
 * NAME: TMR_AllocateSecondTimer
 * DESCRIPTION: Reserve a second timer
 * PARAMETERS: -
 * RETURN: gTmrInvalidTimerID_c if there are no timers available
 * NOTES: none
 *---------------------------------------------------------------------------*/
#define TMR_AllocateSecondTimer() TMR_AllocateTimer()

/*---------------------------------------------------------------------------
 * NAME: TMR_FreeMinuteTimer
 * DESCRIPTION: Free a minute timer. Safe to call even if the timer is running
 * PARAMETERS: timer ID
 * RETURN: -
 * NOTES: none
 *---------------------------------------------------------------------------*/
#define TMR_FreeMinuteTimer(timerID) TMR_FreeTimer(timerID)

/*---------------------------------------------------------------------------
 * NAME: TMR_FreeSecondTimer
 * DESCRIPTION: Free a second timer. Safe to call even if the timer is running
 * PARAMETERS: timer ID
 * RETURN: -
 * NOTES: none
 *---------------------------------------------------------------------------*/
#define TMR_FreeSecondTimer(timerID) TMR_FreeTimer(timerID)

/*---------------------------------------------------------------------------
 * NAME: TMR_StopMinuteTimer
 * DESCRIPTION: Stop a timer started by TMR_StartMinuteTimer()
 * PARAMETERS: timer ID
 * RETURN: TRUE if there are no active non-low power timers, FALSE otherwise
 *         Used by power management
 * NOTES: none
 *---------------------------------------------------------------------------*/
#define TMR_StopMinuteTimer(timerID)  TMR_StopTimer(timerID)

/*  */
/*---------------------------------------------------------------------------
 * NAME: TMR_StopSecondTimer
 * DESCRIPTION: stop a timer started by TMR_StartSecondTimer()
 * PARAMETERS: timer ID
 * RETURN: TRUE if there are no active non-low power timers, FALSE otherwise
 *         Used by power management
 * NOTES: none
 *---------------------------------------------------------------------------*/
#define TMR_StopSecondTimer(timerID)  TMR_StopTimer(timerID)

#else /* stub functions */

#define TMR_Init()
#define TMR_NotifyClkChanged(clkKhz)
#define TMR_AllocateTimer()         gTmrInvalidTimerID_c
#define TMR_AreAllTimersOff()       1
#define TMR_FreeTimer(timerID)      0
#define TMR_IsTimerActive(timerID)  0
#define TMR_StartTimer(timerID,timerType,timeInMilliseconds, pfTimerCallBack, param) 0
#define TMR_StartLowPowerTimer(timerId,timerType,timeIn,pfTmrCallBack,param) 0
#if gTMR_EnableMinutesSecondsTimers_d
#define TMR_StartMinuteTimer(timerId,timeInMinutes,pfTmrCallBack,param) 0
#endif
#if gTMR_EnableMinutesSecondsTimers_d
#define TMR_StartSecondTimer(timerId,timeInSeconds,pfTmrCallBack,param) 0
#endif
#define TMR_StartIntervalTimer(timerID,timeInMilliseconds,pfTimerCallBack,param) 0
#define TMR_StartSingleShotTimer(timerID,timeInMilliseconds,pfTimerCallBack,param) 0
#define TMR_StopTimer(timerID)              0
#define TMR_Thread(events)
#define TMR_EnableTimer(tmrID)
#define TMR_NotCountedTicksBeforeSleep()    0
#define TMR_SyncLpmTimers(sleepDurationTmrTicks)
#define TMR_MakeTMRThreadReady()
#define TmrTicksFromMilliseconds(milliseconds)      0
#define TMR_GetTimerFreq()                          0
#define TMR_GetRemainingTime(tmrID)                 0
#define TMR_AllocateMinuteTimer()     TMR_AllocateTimer()
#define TMR_AllocateSecondTimer()     TMR_AllocateTimer()
#define TMR_FreeMinuteTimer(timerID)  TMR_FreeTimer(timerID)
#define TMR_FreeSecondTimer(timerID)  TMR_FreeTimer(timerID)
#define TMR_StopMinuteTimer(timerID)  TMR_StopTimer(timerID)
#define TMR_StopSecondTimer(timerID)  TMR_StopTimer(timerID)
#define TMR_TimeStampInit()
#define TMR_GetTimestamp()                          0

#endif /* gTMR_Enabled_d */

#if gPrecision_Timers_Enabled_d

/*---------------------------------------------------------------------------
 * NAME: TMR_PrecisionTimerInit
 * DESCRIPTION: initialize the precision timer module
 * PARAMETERS: -
 * RETURN: -
 *---------------------------------------------------------------------------*/
void TMR_PrecisionTimerInit(void);

/*---------------------------------------------------------------------------
 * NAME: TMR_GetAbsoluteTimeus
 * DESCRIPTION: Gets the absolute time in microseconds.
 * PARAMETERS:  -
 * RETURN: Time in microseconds
 *---------------------------------------------------------------------------*/
uint64_t TMR_GetAbsoluteTimeus(void);

#else /*stub functions*/

#define TMR_PrecisionTimerInit() 0

#define TMR_GetAbsoluteTimeus() 0

#endif /*gPrecision_Timers_Enabled_d*/
   
/*---------------------------------------------------------------------------
 * NAME: TMR_RTCIsOscStarted
 * DESCRIPTION: returns the state of the RTC oscillator
 * PARAMETERS: -
 * RETURN: -
 *---------------------------------------------------------------------------*/
bool_t TMR_RTCIsOscStarted(void);

#if gTimestamp_Enabled_d

/*---------------------------------------------------------------------------
 * NAME: TMR_RTCInit
 * DESCRIPTION: initialize the RTC part of the timer module
 * PARAMETERS: -
 * RETURN: -
 *---------------------------------------------------------------------------*/
void TMR_RTCInit(void);

/*---------------------------------------------------------------------------
 * NAME: TMR_RTCGetTimestamp
 * DESCRIPTION: Returns the absolute time at the moment of the call.
 * PARAMETERS: -
 * RETURN: Absolute time at the moment of the call in microseconds.
 *---------------------------------------------------------------------------*/
uint64_t TMR_RTCGetTimestamp(void);

/*---------------------------------------------------------------------------
 * NAME: TMR_RTCSetTime
 * DESCRIPTION: Sets the absolute time.
 * PARAMETERS: Time in microseconds.
 * RETURN: -
 *---------------------------------------------------------------------------*/
void TMR_RTCSetTime(uint64_t microseconds);

/*---------------------------------------------------------------------------
 * NAME: TMR_RTCSetAlarm
 * DESCRIPTION: Sets the alarm absolute time in seconds.
 * PARAMETERS: Time in seconds for the alarm. Callback function pointer. Parameter for callback.
 * RETURN: -
 *---------------------------------------------------------------------------*/
void TMR_RTCSetAlarm(uint64_t seconds, pfTmrCallBack_t callback, void *param);

/*---------------------------------------------------------------------------
 * NAME: TMR_RTCSetAlarmRelative
 * DESCRIPTION: Sets the alarm relative time in seconds.
 * PARAMETERS: Time in seconds for the alarm. Callback function pointer. Parameter for callback.
 * RETURN: -
 *---------------------------------------------------------------------------*/
void TMR_RTCSetAlarmRelative(uint32_t seconds, pfTmrCallBack_t callback, void *param);

#else /*stub functions*/

#define TMR_RTCInit()
#define TMR_RTCGetTimestamp() 0
#define TMR_RTCSetTime()
#define TMR_RTCSetAlarm()
#define TMR_RTCSetAlarmRelative()

#endif /*gTimestamp_Enabled_d*/
#if gTMR_PIT_Timestamp_Enabled_d
/*---------------------------------------------------------------------------
 * NAME: TMR_PITInit
 * DESCRIPTION: initialize the PIT part of the timer module
 * PARAMETERS: -
 * RETURN: -
 *---------------------------------------------------------------------------*/
void TMR_PITInit(void);

/*---------------------------------------------------------------------------
 * NAME: TMR_PITGetTimestamp
 * DESCRIPTION: Returns the absolute time at the moment of the call.
 * PARAMETERS: -
 * RETURN: Absolute time at the moment of the call in microseconds.
 *---------------------------------------------------------------------------*/
uint64_t TMR_PITGetTimestamp(void);

#else
#define TMR_PITInit()
#define TMR_PITGetTimestamp() 0
#endif


#endif /* #ifndef __TMR_INTERFACE_H__ */

/*****************************************************************************
 *                               <<< EOF >>>                                 *
 *****************************************************************************/
