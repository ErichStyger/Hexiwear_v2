/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file SPI_adapter.c
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
#include "SPI_Adapter.h"
#include "pin_mux.h"
#include "panic.h"
#include "fsl_clock_manager.h"
#include "fsl_os_abstraction.h"

#if FSL_FEATURE_SOC_DSPI_COUNT
#include "fsl_dspi_hal.h"
#else
#include "fsl_spi_hal.h"
#endif


/*! *********************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
********************************************************************************** */
#ifndef gXcvrSpiInstance_c
#define gXcvrSpiInstance_c (0xFF)
#endif


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
static void SPIx_ISR(void);
static bool_t Spi_IsMaster(SPI_Type*);
static void Spi_SetIntState(SPI_Type*, bool_t);
static uint32_t Spi_ReadData(SPI_Type* baseAddr);
static void Spi_WriteData(SPI_Type*, spiState_t*, uint32_t);


/*! *********************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
********************************************************************************** */
/*! @brief Pointer to runtime state structure.*/
spiState_t * gpaSpiState[SPI_INSTANCE_COUNT];
/*! @brief Table of base pointers for SPI instances. */
static SPI_Type * const mSpiBase[SPI_INSTANCE_COUNT] = SPI_BASE_PTRS;
/*! @brief Table to save SPI IRQ enum numbers defined in CMSIS files. */
static const IRQn_Type mSpiIrqId[] = SPI_IRQS;
/*! @brief Table of SPI FIFO sizes per instance. */
#if FSL_FEATURE_SPI_FIFO_SIZE
const uint32_t mSpi_FifoSize[SPI_INSTANCE_COUNT] = FSL_FEATURE_SPI_FIFO_SIZEx;
#endif
/*! @brief Default DSPI configuration.*/
#if FSL_FEATURE_SOC_DSPI_COUNT
static const dspi_command_config_t mDefaultSpiCfg = {
    .isChipSelectContinuous = FALSE,
    .whichCtar = kDspiCtar0,
    .whichPcs = kDspiPcs0,
    .isEndOfQueue = FALSE,
    .clearTransferCount = TRUE
};
#endif


/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
* \brief   This function will initialize the SPI module
*
* \param[in] instance     The SPI module number
* \param[in] pSpiState    Pointer to a location where to store the SPI state
* \param[in] pfCallback   Pointer to a Callback function
* \param[in] cbParam      Parameter to be used in Callback function
*
* \return error code
*
********************************************************************************** */
spiStatus_t Spi_Init(uint32_t instance, spiState_t* pSpiState, pfSPIx_TRxCB_t pfCallback, void* cbParam)
{
    IRQn_Type spiIrq = mSpiIrqId[instance];
    SPI_Type *baseAddr = mSpiBase[instance];

    if( !pSpiState || (instance == gXcvrSpiInstance_c) || (instance >= SPI_INSTANCE_COUNT))
    {
        panic(0,(uint32_t)Spi_Init,0,0);
        return spiInvalidParameter;
    }

    /* set SPI Pin Mux */    
    configure_spi_pins(instance);
    /* Enable SPI clock */
    CLOCK_SYS_EnableSpiClock(instance);
    /* Basic SPI initialization */
#if FSL_FEATURE_SOC_DSPI_COUNT
    DSPI_HAL_Init(baseAddr);
    DSPI_HAL_SetContinuousSckCmd(baseAddr, FALSE);
    DSPI_HAL_SetPcsPolarityMode(baseAddr, kDspiPcs0, kDspiPcs_ActiveLow);
    DSPI_HAL_SetFifoCmd(baseAddr, FALSE, FALSE);
    DSPI_HAL_SetFlushFifoCmd(baseAddr, TRUE, TRUE);
    DSPI_HAL_Enable(baseAddr);
#else
    SPI_HAL_Init(baseAddr);
#if FSL_FEATURE_SPI_FIFO_SIZE
    SPI_HAL_SetFifoMode(baseAddr, FALSE, kSpiTxFifoOneFourthEmpty, kSpiRxFifoOneHalfFull);
#endif
    SPI_HAL_Enable(baseAddr);
#endif
    /* Store Callback function and parameter */
    gpaSpiState[instance] = pSpiState;
    pSpiState->cb = pfCallback;
    pSpiState->callbackParam = cbParam;
    pSpiState->pRxData = NULL;
    pSpiState->pTxData = NULL;
    pSpiState->trxByteCount = 0;
    pSpiState->txByteCount  = 0;
    /* Overwrite old ISR */
    (void)OSA_InstallIntHandler(spiIrq, SPIx_ISR);
    /* set interrupt priority */
    NVIC_SetPriority(spiIrq, gSpi_IsrPrio_c >> (8 - __NVIC_PRIO_BITS));
    NVIC_ClearPendingIRQ(spiIrq);
    NVIC_EnableIRQ(spiIrq);
    return spiSuccess;
}


/*! *********************************************************************************
* \brief   This function will configure the SPI module
*
* \param[in] instance     The SPI module number
* \param[in] pConfig      Pointer to the configuration structure
*
* \return error code
*
********************************************************************************** */
spiStatus_t Spi_Configure(uint32_t instance, spiBusConfig_t* pConfig)
{
    bool_t intState;
    SPI_Type *baseAddr = mSpiBase[instance];
    spiState_t* pState = gpaSpiState[instance];
    /* Default SPI configuration */
#if FSL_FEATURE_SOC_DSPI_COUNT
    dspi_data_format_config_t dspiCfg = {
        .bitsPerFrame = 8,
        .clkPolarity  = kDspiClockPolarity_ActiveHigh,
        .clkPhase     = kDspiClockPhase_FirstEdge,
        .direction    = kDspiMsbFirst
    };
#else
    spi_clock_polarity_t clkPol     = kSpiClockPolarity_ActiveHigh;
    spi_clock_phase_t clkPhase      = kSpiClockPhase_FirstEdge;
    spi_shift_direction_t direction = kSpiMsbFirst;
#endif
    
    if( !pConfig || (instance >= SPI_INSTANCE_COUNT) )
    {
        return spiInvalidParameter;
    }

    /* Enable SPI IRQ is configured as Slave OR if Rx byte signaling is enabled*/
    intState = !pConfig->master || pState->signalRxByte;

#if FSL_FEATURE_SOC_DSPI_COUNT
    DSPI_HAL_StopTransfer(baseAddr);
    /* SPI Clock Polarity */    
    if( !pConfig->clkActiveHigh )
    {
        dspiCfg.clkPolarity = kDspiClockPolarity_ActiveLow;
    }

    /* SPI Clock Phase */
    if( !pConfig->clkPhaseFirstEdge )
    {
        dspiCfg.clkPhase = kDspiClockPhase_SecondEdge;
    }

    /* SPI shift direction */
    if( !pConfig->MsbFirst )
    {
        dspiCfg.direction = kDspiLsbFirst;
    }

    /* Apply DSPI configuration */
    DSPI_HAL_SetDataFormat(baseAddr, kDspiCtar0, &dspiCfg);
    (void)DSPI_HAL_CalculateDelay(mSpiBase[instance], kDspiCtar0, kDspiAfterTransfer, CLOCK_SYS_GetSpiFreq(instance), 4000);

    if( pConfig->master )
    {
        DSPI_HAL_SetMasterSlaveMode(baseAddr, kDspiMaster);
        DSPI_HAL_SetBaudRate(baseAddr, kDspiCtar0,  pConfig->bitsPerSec, CLOCK_SYS_GetSpiFreq(instance));
        pState->dspiCmd = DSPI_HAL_GetFormattedCommand(baseAddr, (dspi_command_config_t*)&mDefaultSpiCfg );
    }
    else
    {
        DSPI_HAL_SetMasterSlaveMode(baseAddr, kDspiSlave);
        DSPI_HAL_WriteDataSlavemode(baseAddr, gSpi_DummyChar_d);
    }

    DSPI_HAL_StartTransfer(baseAddr);
    
#else /* FSL_FEATURE_SOC_DSPI_COUNT */
    
    /* SPI Clock Polarity */    
    if( !pConfig->clkActiveHigh )
    {
        clkPol = kSpiClockPolarity_ActiveLow;
    }

    /* SPI Clock Phase */
    if( !pConfig->clkPhaseFirstEdge )
    {
        clkPhase = kSpiClockPhase_SecondEdge;
    }

    /* SPI shift direction */
    if( !pConfig->MsbFirst )
    {
        direction = kSpiLsbFirst;
    }

    /* Apply SPI configuration */
    SPI_HAL_SetDataFormat(baseAddr, clkPol, clkPhase, direction);

#if FSL_FEATURE_SPI_16BIT_TRANSFERS
    SPI_HAL_Set8or16BitMode(baseAddr, kSpi8BitMode);
#endif

    if( pConfig->master )
    {
        SPI_HAL_SetMasterSlave(baseAddr, kSpiMaster);
        SPI_HAL_SetBaud(baseAddr, pConfig->bitsPerSec, CLOCK_SYS_GetSpiFreq(instance));
    }
    else
    {
        SPI_HAL_SetMasterSlave(baseAddr, kSpiSlave);
    }
#endif /* FSL_FEATURE_SOC_DSPI_COUNT */

    Spi_SetIntState(baseAddr, intState);
    
    return spiSuccess;
}


/*! *********************************************************************************
* \brief   This function transfer data synchronously over SPI.
*          The Callback function will not run since the transfer ends when this function returns
*
* \param[in] instance     The SPI module number
* \param[in] pTxData      Pointer to the data to be sent over SPI
                          Can be NULL if there is no data to TX (RX only)
* \param[in] pRxData      Pointer to a location where received data will be stored.
                          Can be NULL if no RX data is expected (TX only).
* \param[in] size         Number of bytes o be transfered over the SPI
*
* \return error code
*
********************************************************************************** */
spiStatus_t Spi_SyncTransfer(uint32_t instance, uint8_t* pTxData, uint8_t* pRxData, uint16_t size)
{
    volatile uint32_t data;
    SPI_Type *baseAddr = mSpiBase[instance];
    spiState_t* pState = gpaSpiState[instance];
    bool_t intState = !Spi_IsMaster(baseAddr) || pState->signalRxByte;

    /* Parameter validation */
    if( (!pTxData && !pRxData) || (instance >= SPI_INSTANCE_COUNT) )
    {
        return spiInvalidParameter;
    }

    /* Check if SPI is Busy */
    OSA_EnterCritical(kCriticalDisableInt);
    if( pState->trxByteCount )
    {
        OSA_ExitCritical(kCriticalDisableInt);
        return spiBusy;
    }

    /* Mark SPI as busy*/
    pState->trxByteCount = size;
    /* Disable SPI interrupts */
    Spi_SetIntState(baseAddr, FALSE);
#if FSL_FEATURE_SOC_DSPI_COUNT    
    DSPI_HAL_ClearStatusFlag(baseAddr, kDspiTxComplete);
    //DSPI_HAL_SetFlushFifoCmd(baseAddr, TRUE, TRUE);
#endif
    OSA_ExitCritical(kCriticalDisableInt);
    
    while( size-- )
    {
        if( pTxData )
        {
            data = *pTxData;
            pTxData++;
        }
        else
        {
            data = gSpi_DummyChar_d;
        }

        /* Start SPI transfer */
#if FSL_FEATURE_SOC_DSPI_COUNT
        
        Spi_WriteData(baseAddr, pState, data);
        while( !DSPI_HAL_GetStatusFlag(baseAddr, kDspiTxComplete) );
        DSPI_HAL_ClearStatusFlag(baseAddr, kDspiTxComplete);

#else /* FSL_FEATURE_SOC_DSPI_COUNT */

        while( SPI_HAL_IsTxBuffEmptyPending(baseAddr) );
        Spi_WriteData(baseAddr, pState, data);
        while( SPI_HAL_IsTxBuffEmptyPending(baseAddr) );
#endif /* FSL_FEATURE_SOC_DSPI_COUNT */

        data = Spi_ReadData(baseAddr);

        if( pRxData )
        {
            *pRxData = data;
            pRxData++;
        }
    }

    /* Mark SPI as Idle */
    pState->trxByteCount = 0;
    /* Restore SPI IRQ state */
    Spi_SetIntState(baseAddr, intState);

    return spiSuccess;
}

/*! *********************************************************************************
* \brief   This function transfer data asynchronously over SPI.
*          If a Callback function was registered, it will run at the end of the transfer
*
* \param[in] instance     The SPI module number
* \param[in] pTxData      Pointer to the data to be sent over SPI
                          Can be NULL if there is no data to TX (RX only)
* \param[in] pRxData      Pointer to a location where received data will be stored.
                          Can be NULL if no RX data is expected (TX only).
* \param[in] size         Number of bytes o be transfered over the SPI
*
* \return error code
*
********************************************************************************** */
spiStatus_t Spi_AsyncTransfer(uint32_t instance, uint8_t* pTxData, uint8_t* pRxData, uint16_t size)
{
    SPI_Type *baseAddr = mSpiBase[instance];
    spiState_t* pState = gpaSpiState[instance];
    
    /* Parameter validation */
    if( (!pTxData && !pRxData) || (instance >= SPI_INSTANCE_COUNT) )
    {
        return spiInvalidParameter;
    }

    /* Check if SPI is Busy */
    OSA_EnterCritical(kCriticalDisableInt);
    if( pState->trxByteCount )
    {
        OSA_ExitCritical(kCriticalDisableInt);
        return spiBusy;
    }

    /* Mark SPI as busy*/
    pState->trxByteCount = size;
    OSA_ExitCritical(kCriticalDisableInt);

    /* Fill transfer parameters */
    pState->pTxData = pTxData;
    pState->pRxData = pRxData;
    pState->txByteCount = size;

#if FSL_FEATURE_SOC_DSPI_COUNT
    /* Fill the SPI Tx FIFO (if available, only Slave mode) */
    DSPI_HAL_SetFlushFifoCmd(baseAddr, TRUE, TRUE);
    DSPI_HAL_ClearStatusFlag(baseAddr, kDspiTxFifoFillRequest);

    while( DSPI_HAL_GetStatusFlag(baseAddr, kDspiTxFifoFillRequest) && pState->txByteCount )
    {
        --pState->txByteCount;

        if( pState->pTxData )
        {
            ((uint8_t*)&pState->dspiCmd)[0] = *pState->pTxData;
            ++pState->pTxData;
        }
        else
        {
            ((uint8_t*)&pState->dspiCmd)[0] = gSpi_DummyChar_d;
        }

        /* Start SPI transfer */
        if( DSPI_HAL_IsMaster(baseAddr) )
        {
            DSPI_HAL_WriteCmdDataMastermode(baseAddr, pState->dspiCmd);
            break;
        }
        else
        {
            DSPI_HAL_WriteDataSlavemode(baseAddr, pState->dspiCmd);
        }

        DSPI_HAL_ClearStatusFlag(baseAddr, kDspiTxFifoFillRequest);
    }
#else /* FSL_FEATURE_SOC_DSPI_COUNT */

#if FSL_FEATURE_SPI_FIFO_SIZE
    while( !SPI_HAL_GetFifoStatusFlag(baseAddr, kSpiTxFifoFull) && pState->txByteCount )
#endif
    {
        uint8_t data;

        --pState->txByteCount;

        if( pState->pTxData )
        {
            data = *pState->pTxData;
            ++pState->pTxData;
        }
        else
        {
            data = gSpi_DummyChar_d;
        }

        Spi_WriteData(baseAddr, pState, data);
#if FSL_FEATURE_SPI_FIFO_SIZE
        if( !SPI_HAL_GetFifoCmd(baseAddr) || Spi_IsMaster(baseAddr) )
        {
            break;
        }
#endif
    }
#endif /* FSL_FEATURE_SOC_DSPI_COUNT */

    /* Enable SPI IRQ */
    Spi_SetIntState(baseAddr, TRUE);
    
    return spiSuccess;
}


/*! *********************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
********************************************************************************* */

/*! *********************************************************************************
* \brief   This is the ISR of the SPI module
*
********************************************************************************** */
static void SPIx_ISR(void)
{
    uint32_t    irq = __get_IPSR() - 16;
    spiState_t *pState;
    SPI_Type   *baseAddr;
    uint32_t    instance;
    uint32_t    data;
    uint32_t    flags;

    /* Get SPI instance */
    for( instance=0; instance<SPI_INSTANCE_COUNT; instance++ )
    {
        if( irq == mSpiIrqId[instance] )
            break;
    }

    pState = gpaSpiState[instance];
    baseAddr = mSpiBase[instance];

    /* Data was transfered */
    /* Clear status and read received byte */
    while(
#if FSL_FEATURE_SOC_DSPI_COUNT
          DSPI_HAL_GetStatusFlag(baseAddr, kDspiRxFifoDrainRequest)
#else
          SPI_HAL_IsReadBuffFullPending(baseAddr)
#endif
         )
    {
        flags = 0;
        data  = Spi_ReadData(baseAddr);
#if FSL_FEATURE_SOC_DSPI_COUNT
        DSPI_HAL_ClearStatusFlag(baseAddr, kDspiRxFifoDrainRequest);
#endif

        if( pState->signalRxByte )
        {
            flags |= gSPI_ByteRxFlag_d;
        }
        
        if( pState->pRxData )
        {
            *pState->pRxData = data;
            ++pState->pRxData;
        }

        if( pState->trxByteCount )
        {
            --pState->trxByteCount;
            /* Check if transfer is complete */
            if( !pState->trxByteCount )
            {
                if( Spi_IsMaster(baseAddr) )
                {
                    /* Disable SPI IRQ if continuous SPI RX is not enabled */
                    Spi_SetIntState(baseAddr, pState->signalRxByte);
                }
                else
                {
                    Spi_WriteData(baseAddr, pState, gSpi_DummyChar_d);
                }
                
                if( pState->pTxData )
                {
                    pState->pTxData = NULL;
                    flags |= gSPI_TxEndFlag_d;
                }

                if( pState->pRxData )
                {
                    pState->pRxData = NULL;
                    flags |= gSPI_RxEndFlag_d;
                }
            }
            /* Get next byte to transmit */
            else 
            {
                data = gSpi_DummyChar_d;
                
                if( pState->txByteCount )
                {
                    --pState->txByteCount;

                    if( pState->pTxData )
                    {
                        data = *pState->pTxData;
                        ++pState->pTxData;
                    }
                    
                    Spi_WriteData(baseAddr, pState, data);
                }
                else if( Spi_IsMaster(baseAddr) )
                {
                    Spi_WriteData(baseAddr, pState, data);
                }
            }
        }
 
        /* Run callback function */
        if( pState->cb && flags )
        {
            pState->cb(flags, pState);
        }
        
    }/* while(...) */
    
}

/*! *********************************************************************************
* \brief   SPI driver helper function used to write data into SPI HW
*
* \param[in] baseAddr  pointer to the SPI registers
* \param[in] pState    pointer to the SPI driver data
* \param[in] data      data to be sent over SPI
*
********************************************************************************** */
static void Spi_WriteData(SPI_Type* baseAddr, spiState_t *pState, uint32_t data)
{
#if FSL_FEATURE_SOC_DSPI_COUNT
    if( DSPI_HAL_IsMaster(baseAddr) )
    {
        ((uint8_t*)&pState->dspiCmd)[0] = (uint8_t)data;
        DSPI_HAL_WriteCmdDataMastermode(baseAddr, pState->dspiCmd);
    }
    else
    {
        DSPI_HAL_WriteDataSlavemode(baseAddr, data);
    }
    
#else
    #if FSL_FEATURE_SPI_16BIT_TRANSFERS
        SPI_HAL_WriteDataLow(baseAddr, data);
    #else
        SPI_HAL_WriteData(baseAddr, data);
    #endif
#endif    
}

/*! *********************************************************************************
* \brief   SPI driver helper function used to read data drom SPI HW
*
* \param[in] baseAddr  pointer to the SPI registers
*
* \return data from SPI HW.
*
********************************************************************************** */
static uint32_t Spi_ReadData(SPI_Type* baseAddr)
{
#if FSL_FEATURE_SOC_DSPI_COUNT
    return DSPI_HAL_ReadData(baseAddr);
    
#else
    #if FSL_FEATURE_SPI_16BIT_TRANSFERS
        return SPI_HAL_ReadDataLow(baseAddr);
    #else
        return SPI_HAL_ReadData(baseAddr);
    #endif
#endif    
}

/*! *********************************************************************************
* \brief   SPI driver helper function used to determine the SPI role (Master/Slave)
*
* \param[in] baseAddr  pointer to the SPI registers
*
* \return TRUE is SPI is configured as Master, else FALSE.
*
********************************************************************************** */
static bool_t Spi_IsMaster(SPI_Type* baseAddr)
{
#if FSL_FEATURE_SOC_DSPI_COUNT                
    return  DSPI_HAL_IsMaster(baseAddr);
#else
    return SPI_HAL_IsMaster(baseAddr);
#endif
}

/*! *********************************************************************************
* \brief   SPI driver helper function used to enable/disable SPI interrupts
*
* \param[in] baseAddr  pointer to the SPI registers
* \param[in] state     the state of the SPI interrupts
*
********************************************************************************** */
static void Spi_SetIntState(SPI_Type* baseAddr, bool_t state)
{
#if FSL_FEATURE_SOC_DSPI_COUNT
    DSPI_HAL_SetIntMode(baseAddr, kDspiRxFifoDrainRequest, state);
#else
    SPI_HAL_SetReceiveAndFaultIntCmd(baseAddr, state);
#endif
}