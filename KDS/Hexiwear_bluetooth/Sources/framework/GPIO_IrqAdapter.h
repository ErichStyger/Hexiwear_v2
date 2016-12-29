/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file GPIO_IrqAdapter.h
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

#ifndef __GPIO_IRQ_ADAPTER_H__
#define __GPIO_IRQ_ADAPTER_H__

/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include "EmbeddedTypes.h"
#include "board.h"


/*! *********************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
********************************************************************************** */
#define gGpioMaxIsrEntries_c   (5)
#define gGpioDefaultNvicPrio_c (0x80)

#define gGpioIsrPrioHigh_c     (0)
#define gGpioIsrPrioNormal_c   (7)
#define gGpioIsrPrioLow_c      (15)

/*! *********************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
********************************************************************************** */
typedef void (*pfGpioIsrCb_t)(void);

typedef struct gpioIsr_tag{
    pfGpioIsrCb_t callback;
    uint32_t      pinMask;
    IRQn_Type     irqId;
    uint8_t       port;
    uint8_t       prio;
}gpioIsr_t;

typedef enum gpioStatus_tag{
    gpio_success,
    gpio_outOfMemory,
    gpio_notFound,
    gpio_error
}gpioStatus_t;

/*! *********************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
********************************************************************************** */
gpioStatus_t GpioInstallIsr( pfGpioIsrCb_t cb, uint8_t priority, uint8_t nvicPriority, uint32_t pinDef );
gpioStatus_t GpioUninstallIsr( uint32_t pinDef );


#endif /* __GPIO_IRQ_ADAPTER_H__ */