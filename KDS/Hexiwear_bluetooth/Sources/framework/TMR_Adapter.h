/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file TMR_Adapter.h
* TMR export interface file for ARM CORTEX processor
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

#ifndef _TMR_ADAPTER_H_
#define _TMR_ADAPTER_H_


/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include "EmbeddedTypes.h"
#include "fsl_device_registers.h"

#if defined(HW_FTM_INSTANCE_COUNT)
  #include "fsl_ftm_driver.h"
  #include "fsl_ftm_hal.h"
#else
  #include "fsl_tpm_driver.h"
  #include "fsl_tpm_hal.h"
#endif

/*! *********************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
********************************************************************************** */

#ifndef gStackTimerInstance_c
#define gStackTimerInstance_c  0
#endif

#ifndef gStackTimerChannel_c
#define gStackTimerChannel_c   0
#endif

#ifndef gLptmrInstance_c
#define gLptmrInstance_c       0
#endif

#ifndef gTmrRtcInstance_c
#define gTmrRtcInstance_c      0
#endif

#ifndef gTmrPitInstance_c
#define gTmrPitInstance_c      0
#endif

#define gStackTimer_IsrPrio_c (0x80)

/*! *********************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
********************************************************************************** */
void StackTimer_Init(void (*cb)(void));
void StackTimer_Enable(void);
void StackTimer_Disable(void);
void StackTimer_ClearIntFlag(void);
uint32_t StackTimer_GetInputFrequency(void);
uint32_t StackTimer_GetCounterValue(void);
void StackTimer_SetOffsetTicks(uint32_t offset);

void LPTMR_Init(void (*cb)(void));
void LPTMR_Enable(void);
void LPTMR_Disable(void);
uint32_t LPTMR_GetInputFrequency(void);
uint32_t LPTMR_GetCounterValue(void);
void LPTMR_SetOffsetTicks(uint32_t offset);

#endif /* _TMR_ADAPTER_H_ */