/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file GPIO_IrqAdapter.c
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
#include "fsl_gpio_driver.h"
#include "fsl_os_abstraction.h"

#include "GPIO_IrqAdapter.h"
#include "FunctionLib.h"

/*! *********************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
********************************************************************************** */
#define gGpio_FoundPortIsr_c  0x01
#define gGpio_FoundSimilar_c  0x02


/*! *********************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
********************************************************************************** */


/*! *********************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
********************************************************************************** */
static void Gpio_CommonIsr(void);
static gpioStatus_t Gpio_InstallPortISR(IRQn_Type irqId, uint32_t nvicPrio);


/*! *********************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
********************************************************************************** */
static gpioIsr_t mGpioIsr[gGpioMaxIsrEntries_c];
static uint16_t  mGpioIsrCount=0;


/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
* \brief  Uninstall the callback for the specified Pin Definition
*
* \param[in]  cb              The callback function to be installed
* \param[in]  priority        The priority used by the GPIO_IrqAdapter
* \param[in]  nvicPriority    The priority to be set in NVIC. Only most significant bits are used!
* \param[in]  pinDef          The KSDK pin definition
*
* \return  install status
*
********************************************************************************** */
gpioStatus_t GpioInstallIsr( pfGpioIsrCb_t cb, uint8_t priority, uint8_t nvicPriority, uint32_t pinDef )
{
    uint32_t i;
    uint8_t  found   = 0;
    uint8_t  pos     = mGpioIsrCount;
    uint8_t  portId  = GPIO_EXTRACT_PORT(pinDef);
    uint32_t pinMask = 1 << GPIO_EXTRACT_PIN(pinDef);
    IRQn_Type irqNo  = g_portIrqId[portId];

    for( i=0; i<mGpioIsrCount; i++ )
    {
        /* search for port ISR already installed */
        if( mGpioIsr[i].port == portId )
        {
            found |= gGpio_FoundPortIsr_c;
        }
        /* search for insert position */
        if( (pos == mGpioIsrCount) && (mGpioIsr[i].prio >= priority) ) 
        {
            pos = i;
        }
        /* search for an entry with the same callback installed for the same port with the same priority */
        if( (mGpioIsr[i].callback == cb) && (mGpioIsr[i].port == portId) && (mGpioIsr[i].prio == priority) )
        {
            pos = i;
            found |= gGpio_FoundSimilar_c;
            break;
        }
    }

    if( found & gGpio_FoundSimilar_c )
    {
        /* found the same ISR installed for the same port, but other pins */
        mGpioIsr[pos].pinMask |= pinMask;
    }
    else
    {
        if( mGpioIsrCount >= gGpioMaxIsrEntries_c )
            return gpio_outOfMemory;

        if( pos != mGpioIsrCount )
        {
            OSA_EnterCritical(kCriticalDisableInt);
            /* Shift all entries to the left, to obtain a sorted list */
            for( i=mGpioIsrCount; i>pos; i-- )
            {
                mGpioIsr[i] = mGpioIsr[i-1];
            }
            OSA_ExitCritical(kCriticalDisableInt);
        }
        /* install new callback */
        mGpioIsr[pos].callback = cb;
        mGpioIsr[pos].prio     = priority;
        mGpioIsr[pos].port     = portId;
        mGpioIsr[pos].irqId    = g_portIrqId[portId];
        mGpioIsr[pos].pinMask  = pinMask;
        mGpioIsrCount++;
    }

    if( found )
    {
        /* The PORT ISR was already installed. Update NVIC priority if higher than the old one! */
        nvicPriority = nvicPriority >> (8 - __NVIC_PRIO_BITS);
        i = NVIC_GetPriority(irqNo);
        if( i > nvicPriority )
        {
            NVIC_SetPriority(irqNo, nvicPriority);
        }
        return gpio_success;
    }
    else
    {
        /* Install common PORT ISR */
        return Gpio_InstallPortISR(irqNo, nvicPriority);
    }
}

/*! *********************************************************************************
* \brief  Uninstall the callback for the specified Pin Definition
*
* \param[in]  pinDef    The KSDK pin definition
*
* \return  uninstall status
*
********************************************************************************** */
gpioStatus_t GpioUninstallIsr( uint32_t pinDef )
{
    IRQn_Type irqNo;
    uint32_t  i, j;
    uint8_t port      = GPIO_EXTRACT_PORT(pinDef);
    uint32_t pinMask  = 1 << GPIO_EXTRACT_PIN(pinDef);    
    
    for( i=0; i<mGpioIsrCount; i++ )
    {
        if( (mGpioIsr[i].port == port) && (mGpioIsr[i].pinMask & pinMask) )
        {
            OSA_EnterCritical(kCriticalDisableInt);
            /* uninstall ISR only for specified pins */
            mGpioIsr[i].pinMask &= ~pinMask;
            /* if no more pins are active, uninstall handler function */
            if( !mGpioIsr[i].pinMask )
            {
                irqNo = mGpioIsr[i].irqId;
                mGpioIsr[i].callback = NULL;

                /* Shift next entries to the left */
                for( j=i; j<mGpioIsrCount-1; j++ )
                {
                    mGpioIsr[j] = mGpioIsr[j+1];
                }
                mGpioIsrCount--;

                /* Search for other ISR installed for the same IRQ */
                for( j=0; j<mGpioIsrCount; j++ )
                {
                    if( irqNo == mGpioIsr[j].irqId )
                    {
                        irqNo = NotAvail_IRQn;
                        break;
                    }
                }

                /* If no other ISR was installed for this IRQ, disable IRQ in NVIC */
                if( irqNo != NotAvail_IRQn )
                {
                    NVIC_DisableIRQ(irqNo);
                }
            }
            OSA_ExitCritical(kCriticalDisableInt);
            return gpio_success;
        }
    }

    return gpio_notFound;
}

/*! *********************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
********************************************************************************* */

/*! *********************************************************************************
* \brief  Common GPIO ISR.
*
********************************************************************************** */
static void Gpio_CommonIsr(void)
{
    uint32_t i;
    IRQn_Type irqId = (IRQn_Type)(__get_IPSR() - 16);

    /* Search for the highest priority ISR installed for the current PORT */
    for( i=0; i<mGpioIsrCount; i++ )
    {
        if( (mGpioIsr[i].irqId == irqId) && 
            (mGpioIsr[i].pinMask & PORT_HAL_GetPortIntFlag(g_portBase[mGpioIsr[i].port]))
          )
        {
            mGpioIsr[i].callback();
            /* If other lower priority IRSs need to run, this common ISR will run again! */
            return;
        }
    }
}

/*! *********************************************************************************
* \brief  Install the Gpio_CommonIsr() ISR for the specified IRQ
*
* \param[in]  irqId    The CMSIS irq Id
* \param[in]  nvicPrio The priority to be set in NVIC
*
* \return  install status
*
********************************************************************************** */
static gpioStatus_t Gpio_InstallPortISR(IRQn_Type irqId, uint32_t nvicPrio)
{
    if( irqId != NotAvail_IRQn )
    {
        if( NULL == OSA_InstallIntHandler(irqId, Gpio_CommonIsr) )
            return gpio_error;

        /* Enable IRQ in NVIC and set priority */
        NVIC_ClearPendingIRQ(irqId);
        NVIC_EnableIRQ(irqId);
        NVIC_SetPriority(irqId, nvicPrio >> (8 - __NVIC_PRIO_BITS));
    }
    return gpio_success;
}