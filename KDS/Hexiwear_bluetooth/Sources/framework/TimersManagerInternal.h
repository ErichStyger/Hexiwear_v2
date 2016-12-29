/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file TimersManagerInternal.h
* TIMER header file for the ARM CORTEX-M4 processor
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

 #ifndef __TIMER_H__
 #define __TIMER_H__

#include "TimersManager.h"
#include "EmbeddedTypes.h"

/*****************************************************************************
******************************************************************************
* Private macros
******************************************************************************
*****************************************************************************/

/*
 * NAME: mTmrStatusFree_c
 * DESCRIPTION: self explanatory
 */
#define mTmrStatusFree_c    0x00

/*
 * NAME: mTmrStatusActive_c
 * DESCRIPTION: self explanatory
 */
#define mTmrStatusActive_c    0x20

/*
 * NAME: mTmrStatusReady_c
 * DESCRIPTION: self explanatory
 */
#define mTmrStatusReady_c    0x40

/*
 * NAME: mTmrStatusInactive_c
 * DESCRIPTION: self explanatory
 */
#define mTmrStatusInactive_c    0x80

/*
 * NAME: mTimerStatusMask_c
 * DESCRIPTION: timer status mask
 */
#define mTimerStatusMask_c      ( mTmrStatusActive_c \
| mTmrStatusReady_c \
| mTmrStatusInactive_c)

/*
 * NAME: TMR_IsTimerAllocated()
 * DESCRIPTION: checks if a specified timer is allocated
 */
#define TMR_IsTimerAllocated(timerID)   (maTmrTimerStatusTable[(timerID)])

/*
 * NAME: TMR_MarkTimerFree()
 * DESCRIPTION: marks the specified timer as free
 */
#define TMR_MarkTimerFree(timerID)       maTmrTimerStatusTable[(timerID)] = 0

/*
 * NAME: IsLowPowerTimer()
 * DESCRIPTION: Detect if the timer is a low-power timer
 */
#define IsLowPowerTimer(type)           ((type) & gTmrLowPowerTimer_c)

/*
 * NAME: mTimerType_c
 * DESCRIPTION: timer types
 */
#define mTimerType_c            ( gTmrSingleShotTimer_c \
    | gTmrSetSecondTimer_c \
    | gTmrSetMinuteTimer_c \
    | gTmrIntervalTimer_c \
    | gTmrLowPowerTimer_c )

/*
 * NAME: mTMR_Event_c
 * DESCRIPTION: TMR_Thread() event flag. Only one event is needed.
 */
#define mTMR_Event_c    ( 1 << 0 )

/*
 * NAME: TmrIntDisableAll()
 * DESCRIPTION: Disable interrupts
 */
#define TmrIntDisableAll() OSA_EnterCritical(kCriticalDisableInt);

/*
 * NAME: TmrIntRestoreAll()
 * DESCRIPTION: Enable interrupts
 */
#define TmrIntRestoreAll() OSA_ExitCritical(kCriticalDisableInt);

/*****************************************************************************
******************************************************************************
* Private type definitions
******************************************************************************
*****************************************************************************/

/*
 * Type name: tmrTimerStatus_t
 * Type description: The status and type are bitfields, to save RAM.
 *                   This costs some code space, though.
 * Members: N/A
 */
typedef uint8_t tmrTimerStatus_t;

/*
 * NAME: tmrStatus_t
 * DESCRIPTION: timer status - see the status macros.
 *              If none of these flags are on, the timer is not allocated.
 *              For allocated timers, exactly one of these flags will be set.
 *              mTmrStatusActive_c - Timer has been started and has not yet expired.
 *              mTmrStatusReady_c - TMR_StartTimer() has been called for this timer, but
 *                                  the timer thread has not yet actually started it. The
 *                                  timer is considered to be active.
 *              mTmrStatusInactive_c Timer is allocated, but is not active.
 */
typedef uint8_t tmrStatus_t;



/*
 * Type name: tmrTimerTableEntry_tag
 * Type description: One entry in the main timer table.
 * Members: intervalInTicks - The timer's original duration, in ticks.
 *                            Used to reset intervnal timers.
 *
 *          countDown - When a timer is started, this is set to the duration.
 *                      The timer thread decrements this value. When it reaches
 *                      zero, the timer has expired.
 *          pfCallBack - Pointer to the callback function
 *          param - Parameter to the callback function
 */
typedef struct tmrTimerTableEntry_tag {
  tmrTimerTicks64_t intervalInTicks;
  tmrTimerTicks64_t remainingTicks;
  pfTmrCallBack_t pfCallBack;
  void *param;
  tmrTimerTicks16_t timestamp; /* all HW counters are 16-bit wide */
} tmrTimerTableEntry_t;

#endif /* #ifndef __TIMER_H__ */

 /*****************************************************************************
 *                               <<< EOF >>>                                  *
 ******************************************************************************/
