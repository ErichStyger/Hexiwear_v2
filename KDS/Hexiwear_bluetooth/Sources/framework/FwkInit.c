/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file FwkInit.c
* TIMER implementation file for the ARM CORTEX-M4 processor
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


/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include "EmbeddedTypes.h"
#include "fsl_os_abstraction.h"
#include "TimersManager.h"
#include "SerialManager.h"
#include "Panic.h"

#if defined(FWK_SMALL_RAM_CONFIG)

/*! *********************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
********************************************************************************** */
#if gTmrTaskStackSize_c > gSerialTaskStackSize_c
#define gFwkCommonStackSize_c gTmrTaskStackSize_c
#else
#define gFwkCommonStackSize_c gSerialTaskStackSize_c
#endif


/*! *********************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
********************************************************************************** */
OSA_TASK_DEFINE( FWK, gFwkCommonStackSize_c );
event_t  gFwkCommonEvent;
task_handler_t gFwkCommonTaskId = 0;

extern const uint8_t gUseRtos_c;

/*! *********************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
********************************************************************************** */
extern void SerialManagerTask(task_param_t argument);
extern void TMR_Task(task_param_t param);
static void FwkTask(task_param_t argument);


/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */
void FwkInit(void)
{
    osa_status_t status;
    
    if( gFwkCommonTaskId )
        return;

    status = OSA_EventCreate( &gFwkCommonEvent, kEventAutoClear);
    if( kStatus_OSA_Success != status )
    {
        panic(0,0,0,0);
        return;
    }

    status = OSA_TaskCreate(FwkTask, "FWK_Task", gFwkCommonStackSize_c, FWK_stack,
                            gSerialTaskPriority_c, (task_param_t)NULL, FALSE, &gFwkCommonTaskId);
    if( kStatus_OSA_Success != status )
    {
        panic(0,0,0,0);
        return;
    }
}


static void FwkTask(task_param_t argument)
{
    while(1)
    {
        event_flags_t  ev;
        (void)OSA_EventWait(&gFwkCommonEvent, 0x00FFFFFF, FALSE, OSA_WAIT_FOREVER ,&ev);

#if gTMR_Enabled_d
        if(ev >> 16)
        {
            TMR_Task(0);
        }
#endif

#if gSerialManagerMaxInterfaces_c
        ev |= 0x0000FFFF;
        
        if( ev )
        {
            SerialManagerTask(0);
        }
#endif

        /* For BareMetal break the while(1) after 1 run */
        if (gUseRtos_c == 0)
        {
            break;
        }
    }
}

#endif /* FWK_SMALL_RAM_CONFIG */