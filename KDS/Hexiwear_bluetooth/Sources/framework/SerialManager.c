/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file SerialManager.c
* This is the source file for the Serial Manager.
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

#include "SerialManager.h"
#include "Panic.h"
#include "MemManager.h"
#include "Messaging.h"
#include "FunctionLib.h"
#include "Gpio_IrqAdapter.h"

#include "fsl_device_registers.h"
#include "fsl_os_abstraction.h"
#include "fsl_gpio_driver.h"
#include "pin_mux.h"
#include <string.h>

#if gSerialMgr_DisallowMcuSleep_d
  #include "PWR_Interface.h"
#endif

#if (gSerialMgrUseUart_c)
#if FSL_FEATURE_SOC_UART_COUNT
  #include "fsl_uart_driver.h"
  #include "fsl_uart_hal.h"
#endif
#if FSL_FEATURE_SOC_LPUART_COUNT
  #include "fsl_lpuart_driver.h"
  #include "fsl_lpuart_hal.h"
#endif
#if FSL_FEATURE_SOC_LPSCI_COUNT
  #include "fsl_lpsci_driver.h"
  #include "fsl_lpsci_hal.h"
#endif
  #include "fsl_clock_manager.h"
#endif

#if (gSerialMgrUseIIC_c)
  #include "fsl_i2c_master_driver.h"
  #include "fsl_i2c_slave_driver.h"
  #include "fsl_i2c_hal.h"
#endif

#if (gSerialMgrUseSPI_c)
  #include "SPI_Adapter.h"
#endif

#if (gSerialMgrUseUSB_c)
  #include "VirtualComInterface.h"
#endif

#if gSerialMgrUseFSCIHdr_c
  #include "FSCIInterface.h"
  #include "FsciCommunication.h"
#endif

/*! *********************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
********************************************************************************** */
#ifndef gSMGR_UseOsSemForSynchronization_c
#define gSMGR_UseOsSemForSynchronization_c  (USE_RTOS)
#endif

#define mSerial_IncIdx_d(idx, max) if( ++(idx) >= (max) ) { (idx) = 0; }

#define mSerial_DecIdx_d(idx, max) if( (idx) > 0 ) { (idx)--; } else  { (idx) = (max) - 1; }

#define gSMRxBufSize_c (gSerialMgrRxBufSize_c + 1)

#define mSMGR_DapIsrPrio_c    (0x80)
#define mSMGR_I2cIsrPrio_c    (0x40)
#define mSMGR_UartIsrPrio_c   (0x40)
#define mSMGR_LpuartIsrPrio_c (0x40)
#define mSMGR_LpsciIsrPrio_c  (0x40)

#if gSerialMgrUseFSCIHdr_c
#define mSMGR_FSCIHdrLen_c  sizeof(clientPacketHdr_t)
#endif

/*! *********************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
********************************************************************************** */
/* 
 * Set the size of the Rx buffer indexes 
 */
#if gSMRxBufSize_c < 255
typedef uint8_t bufIndex_t;
#else
typedef uint16_t bufIndex_t;
#endif

/* 
 * Defines events recognized by the SerialManager's Task
 * Message used to enque async tx data 
 */
typedef struct SerialManagetMsg_tag{
    pSerialCallBack_t txCallback;
    void             *pTxParam;
    uint8_t          *pData;
    uint16_t          dataSize;
}SerialMsg_t;

/* 
 * Defines the serial interface structure 
 */
typedef struct serial_tag{
    serialInterfaceType_t  serialType;
    uint8_t                serialChannel;
    /* Rx parameters */
    bufIndex_t             rxIn;
    volatile bufIndex_t    rxOut;
    pSerialCallBack_t      rxCallback;
    void                  *pRxParam;
    uint8_t                rxBuffer[gSMRxBufSize_c];
    /* Tx parameters */
    SerialMsg_t            txQueue[gSerialMgrTxQueueSize_c];
#if gSMGR_UseOsSemForSynchronization_c
    semaphore_t            txSyncSem;
#if gSerialMgr_BlockSenderOnQueueFull_c
    semaphore_t            txQueueSem;
    uint8_t                txBlockedTasks;
#endif
#endif
#if gSerialMgrUseFSCIHdr_c
    fsciLen_t              rxFsciIn;
    fsciLen_t              rxFsciLen;
    uint8_t                rxFsciPkt;                 
#endif
    uint8_t                txIn;
    uint8_t                txOut;
    uint8_t                txCurrent;
    uint8_t                events;
    uint8_t                state;
}serial_t;

/* 
 * SMGR task event flags 
 */
typedef enum{
    gSMGR_Rx_c     = (1<<0),
    gSMGR_TxDone_c = (1<<1),
    gSMGR_TxNew_c  = (1<<2)
}serialEventType_t;

/*
 * IIC driver specific data structures
 */
#if (gSerialMgrUseIIC_c)
typedef struct smgrI2CSlaveData_tag{
    i2c_slave_state_t state;
}smgrI2CSlaveData_t;

typedef struct smgrI2CMasterData_tag{
  i2c_master_state_t state;
  i2c_device_t bus;
}smgrI2CMasterData_t;
#endif

/*
 * Common driver data structure union
 */
typedef union smgrDrvData_tag
{
#if (gSerialMgrUseUart_c)
#if FSL_FEATURE_SOC_UART_COUNT
  uart_state_t uartState;
#endif
#if FSL_FEATURE_SOC_LPUART_COUNT
  lpuart_state_t lpuartState;
#endif
#if FSL_FEATURE_SOC_LPSCI_COUNT
  lpsci_state_t lpsciState;
#endif
#endif /* #if (gSerialMgrUseUart_c) */
#if (gSerialMgrUseIIC_c)
  smgrI2CSlaveData_t  i2cSlave;
  smgrI2CMasterData_t i2cMaster;
#endif
#if (gSerialMgrUseSPI_c)
  spiState_t spiState;
#endif
  void *pDrvData;
}smgrDrvData_t;

/*! *********************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
********************************************************************************** */
#if (gSerialManagerMaxInterfaces_c)
/*
 * SMGR internal functions
 */
void SerialManagerTask(task_param_t argument);
void  SerialManager_RxNotify(uint32_t interfaceId);
void  SerialManager_TxNotify(uint32_t interfaceId);
#if gSMGR_UseOsSemForSynchronization_c
static void  Serial_SyncTxCallback(void *pSer);
#endif
static void  Serial_TxQueueMaintenance(serial_t *pSer);
static serialStatus_t Serial_WriteInternal (uint8_t InterfaceId);
#if (gSerialMgrUseSPI_c) || (gSerialMgrUseIIC_c)
static uint32_t Serial_GetInterfaceIdFromType(serialInterfaceType_t type);
static void Serial_ConfigureMasterDap(uint32_t pinName, void (*pfISR)(void));
static void Serial_ConfigureSlaveDap (uint32_t pinName);
#endif

/*
 * UART, LPUART and LPSCI specific functions
 */
#if (gSerialMgrUseUart_c)
#if FSL_FEATURE_SOC_UART_COUNT
void UART_IRQHandler(void);
extern void UART_DRV_IRQHandler(uint32_t instance);
void Serial_UartRxCb(uint32_t instance, void* state);
void Serial_UartTxCb(uint32_t instance, void* state);
#endif
#if FSL_FEATURE_SOC_LPUART_COUNT
void LPUART_IRQHandler(void);
extern void LPUART_DRV_IRQHandler(uint32_t instance);
void Serial_LpuartRxCb(uint32_t instance, void* state);
void Serial_LpuartTxCb(uint32_t instance, void* state);
#endif
#if FSL_FEATURE_SOC_LPSCI_COUNT
void LPSCI_IRQHandler(void);
extern void LPSCI_DRV_IRQHandler(uint32_t instance);
void Serial_LpsciRxCb(uint32_t instance, void* state);
void Serial_LpsciTxCb(uint32_t instance, void* state);
#endif
#endif

/*
 * SPI specific functions
 */
#if (gSerialMgrUseSPI_c)
static void SpiMasterDapISR(void);
static void SpiCallback(uint32_t, void*);
#endif

/*
 * IIC specific functions
 */
#if (gSerialMgrUseIIC_c)
extern void  I2C_DRV_IRQHandler(uint32_t instance);
extern const IRQn_Type g_i2cIrqId[];
static void  I2Cx_ISR(void);    
static void  I2cMasterDapISR(void);
static void  I2cMasterCb(uint8_t instance);
static void  I2cSlaveCb (uint8_t instance,i2c_slave_event_t slaveEvent,void *userData);
#endif

#endif /* #if (gSerialManagerMaxInterfaces_c) */

#if defined(FWK_SMALL_RAM_CONFIG)
void FwkInit(void);
#endif

/*! *********************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
********************************************************************************** */
extern const uint8_t gUseRtos_c;

/*! *********************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
********************************************************************************** */
#if gSerialManagerMaxInterfaces_c

/*
 * RTOS objects definition
 */
#if defined(FWK_SMALL_RAM_CONFIG)
extern event_t  gFwkCommonEvent;
extern task_handler_t gFwkCommonTaskId;
#define gSerialManagerTaskId gFwkCommonTaskId
#define mSMTaskEvent gFwkCommonEvent

#else

OSA_TASK_DEFINE( SMGR, gSerialTaskStackSize_c );
task_handler_t gSerialManagerTaskId;
event_t        mSMTaskEvent;
#endif /* defined(FWK_SMALL_RAM_CONFIG) */

/*
 * SMGR internal data
 */
static serial_t      mSerials[gSerialManagerMaxInterfaces_c];
static smgrDrvData_t mDrvData[gSerialManagerMaxInterfaces_c];

/*
 * Default configuration for IIC driver
 */
#if (gSerialMgrUseIIC_c)
const i2c_slave_user_config_t gI2cSlaveCfg = {
    .address = gSerialMgrIICAddress_c,
    .slaveListening   = false,
    .slaveCallback    = I2cSlaveCb,
    .callbackParam    = NULL,
#if FSL_FEATURE_I2C_HAS_START_STOP_DETECT
    .startStopDetect  = false,
#endif
#if FSL_FEATURE_I2C_HAS_STOP_DETECT
    .stopDetect       = false,
#endif
};
#endif /* #if (gSerialMgrUseIIC_c) */

/*
 * Default configuration for SPI driver
 */
#if (gSerialMgrUseSPI_c)
uint8_t mSPI_dummyData = gSpi_DummyChar_d;
spiBusConfig_t gSpiConfig = {
    .bitsPerSec = 1000000,
    .master = FALSE,
    .clkActiveHigh = TRUE,
    .clkPhaseFirstEdge = TRUE,
    .MsbFirst = TRUE
};
#endif

/*
 * Default configuration for UART, LPUART and LPSCI drivers
 */
#if (gSerialMgrUseUart_c)
#if FSL_FEATURE_SOC_UART_COUNT
uart_user_config_t mSmgr_UartCfg = {
    .baudRate = 115200,
    .parityMode = kUartParityDisabled,
    .stopBitCount = kUartOneStopBit,
    .bitCountPerChar = kUart8BitsPerChar
};
#endif
#if FSL_FEATURE_SOC_LPUART_COUNT
const lpuart_user_config_t mSmgr_LpuartCfg = {
    .clockSource = kClockLpuartSrcOsc0erClk,
    .baudRate = 115200,
    .parityMode = kLpuartParityDisabled,
    .stopBitCount = kLpuartOneStopBit,
    .bitCountPerChar = kLpuart8BitsPerChar
};
#endif
#if FSL_FEATURE_SOC_LPSCI_COUNT
const lpsci_user_config_t mSmgr_LpsciCfg = {
    .clockSource = kClockLpsciSrcPllFllSel,         
    .baudRate = 115200,
    .parityMode = kLpsciParityDisabled,
    .stopBitCount = kLpsciOneStopBit,
    .bitCountPerChar = kLpsci8BitsPerChar
};
#endif
#endif /* #if (gSerialMgrUseUart_c) */

#endif /* #if gSerialManagerMaxInterfaces_c */

/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
* \brief   Creates the SerialManager's task and initializes internal data structures
*
********************************************************************************** */
void SerialManager_Init( void )
{
#if (gSerialManagerMaxInterfaces_c)       
    static uint8_t initialized = FALSE;

    /* Check if SMGR is already initialized */
    if( initialized )
        return;

    initialized = TRUE;

    /* Fill the structure with zeros */
    FLib_MemSet( mSerials, 0x00, sizeof(mSerials) );
#if defined(FWK_SMALL_RAM_CONFIG)
    FwkInit();
#else
    osa_status_t status;
    
    status = OSA_EventCreate( &mSMTaskEvent, kEventAutoClear);
    if( kStatus_OSA_Success != status )
    {
        panic(0,0,0,0);
        return;
    }

    status = OSA_TaskCreate(SerialManagerTask, "SMGR_Task", gSerialTaskStackSize_c, SMGR_stack,
                            gSerialTaskPriority_c, (task_param_t)NULL, FALSE, &gSerialManagerTaskId);
    if( kStatus_OSA_Success != status )
    {
        panic(0,0,0,0);
        return;
    }
#endif /* #if defined(FWK_SMALL_RAM_CONFIG) */
#endif /* #if (gSerialManagerMaxInterfaces_c) */
}

/*! *********************************************************************************
* \brief   The main task of the Serial Manager
*
* \param[in] initialData unused
*
********************************************************************************** */
#if (gSerialManagerMaxInterfaces_c)
void SerialManagerTask(task_param_t argument)
{
    uint16_t i;
    uint8_t ev;    

#if defined(FWK_SMALL_RAM_CONFIG)
    {
#else
    event_flags_t  mSMTaskEventFlags;        

    while( 1 )
    {
        /* Wait for an event. The task will block here. */
        (void)OSA_EventWait(&mSMTaskEvent, 0x00FFFFFF, FALSE, OSA_WAIT_FOREVER ,&mSMTaskEventFlags);
#endif
        for( i = 0; i < gSerialManagerMaxInterfaces_c; i++ )
        {
            OSA_EnterCritical(kCriticalDisableInt);
            ev = mSerials[i].events;
            mSerials[i].events = 0;
            OSA_ExitCritical(kCriticalDisableInt);

            if ( (ev & gSMGR_Rx_c) &&
                 (NULL != mSerials[i].rxCallback) )
            {
                mSerials[i].rxCallback( mSerials[i].pRxParam );
            }

            if( ev & gSMGR_TxDone_c )
            {
                Serial_TxQueueMaintenance(&mSerials[i]);
            }

            /* If the Serial is IDLE and there is data to tx */
            if( (mSerials[i].state == 0) && mSerials[i].txQueue[mSerials[i].txCurrent].dataSize )
            {
                (void)Serial_WriteInternal( i );
            }
#if gSerialMgrUseSPI_c
            /* If the SPI Slave has more data to transmit, restart the transfer */
            if( (mSerials[i].serialType == gSerialMgrSPIMaster_c) && GPIO_DRV_ReadPinInput(kGpioSpiDAP) && !mSerials[i].state )
            {
                if( (0 == mSerials[i].txQueue[mSerials[i].txIn].dataSize) && (NULL == mSerials[i].txQueue[mSerials[i].txIn].txCallback) )
                {
                    Serial_AsyncWrite( i, &mSPI_dummyData, 1, NULL, NULL );
                }
            }
#endif
        
        }
         
#if !defined(FWK_SMALL_RAM_CONFIG)    
        /* For BareMetal break the while(1) after 1 run */
        if (gUseRtos_c == 0)
        {
            break;
        }
#endif
    } /* while(1) */
}
#endif

/*! *********************************************************************************
* \brief   Initialize a communication interface.
*
* \param[in] pInterfaceId   pointer to a location where the interface Id will be stored
* \param[in] interfaceType  the type of the interface: UART/SPI/IIC/USB
* \param[in] instance       the instance of the HW module (ex: if UART1 is used, this value should be 1)
*
* \return The interface number if success or gSerialManagerInvalidInterface_c if an error occured.
*
********************************************************************************** */
serialStatus_t Serial_InitInterface( uint8_t *pInterfaceId,
                                     serialInterfaceType_t interfaceType,
                                     uint8_t instance )
{
#if gSerialManagerMaxInterfaces_c
    uint8_t i;
    serial_t *pSer;

    *pInterfaceId = gSerialMgrInvalidIdx_c;

    for ( i=0; i<gSerialManagerMaxInterfaces_c; i++ )
    {
        pSer = &mSerials[i];

        if ( (pSer->serialType == interfaceType) &&
            (pSer->serialChannel == instance) )
        {
            /* The Interface is allready opened. */
            return gSerial_InterfaceInUse_c;
        }
        else if ( pSer->serialType == gSerialMgrNone_c )
        {
            OSA_EnterCritical(kCriticalDisableInt);
            pSer->serialChannel = instance;
            switch ( interfaceType )
            {
            case gSerialMgrUart_c:
#if gSerialMgrUseUart_c && FSL_FEATURE_SOC_UART_COUNT
                {
                    IRQn_Type irq = g_uartRxTxIrqId[instance];

                    configure_uart_pins(instance);
                    NVIC_SetPriority(irq, mSMGR_UartIsrPrio_c >> (8 - __NVIC_PRIO_BITS));
                    OSA_InstallIntHandler(irq, UART_IRQHandler);
                    UART_DRV_Init(instance, &mDrvData[i].uartState, &mSmgr_UartCfg);
                    UART_DRV_InstallRxCallback(instance, Serial_UartRxCb, &pSer->rxBuffer[pSer->rxIn], (void*)i, TRUE);
                    UART_DRV_InstallTxCallback(instance, Serial_UartTxCb, NULL, (void*)i);
                }
#endif
                break;

            case gSerialMgrLpuart_c:
#if gSerialMgrUseUart_c && FSL_FEATURE_SOC_LPUART_COUNT
                {
                    IRQn_Type irq = g_lpuartRxTxIrqId[instance];

                    configure_lpuart_pins(instance);
                    NVIC_SetPriority(irq, mSMGR_LpuartIsrPrio_c >> (8 - __NVIC_PRIO_BITS));
                    OSA_InstallIntHandler(irq, LPUART_IRQHandler);
                    LPUART_DRV_Init(instance, &mDrvData[i].lpuartState, &mSmgr_LpuartCfg);
                    LPUART_DRV_InstallRxCallback(instance, Serial_LpuartRxCb, &pSer->rxBuffer[pSer->rxIn], (void*)i, TRUE);
                    LPUART_DRV_InstallTxCallback(instance, Serial_LpuartTxCb, NULL, (void*)i);
                }
#endif
                break;

            case gSerialMgrLpsci_c:
#if gSerialMgrUseUart_c && FSL_FEATURE_SOC_LPSCI_COUNT
                {
                    IRQn_Type irq = g_lpsciRxTxIrqId[instance];

                    configure_lpsci_pins(instance);
                    NVIC_SetPriority(irq, mSMGR_LpsciIsrPrio_c >> (8 - __NVIC_PRIO_BITS));
                    OSA_InstallIntHandler(irq, LPSCI_IRQHandler);
                    LPSCI_DRV_Init(instance, &mDrvData[i].lpsciState, &mSmgr_LpsciCfg);
                    LPSCI_DRV_InstallRxCallback(instance, Serial_LpsciRxCb, &pSer->rxBuffer[pSer->rxIn], (void*)i, TRUE);
                    LPSCI_DRV_InstallTxCallback(instance, Serial_LpsciTxCb, NULL, (void*)i);
                }
#endif
                break;

            case gSerialMgrUSB_c:
#if gSerialMgrUseUSB_c
                mDrvData[i].pDrvData = VirtualCom_Init(i);
                if (NULL == mDrvData[i].pDrvData)
                {
                    OSA_ExitCritical(kCriticalDisableInt);
                    return gSerial_InternalError_c;
                }
#endif
                break;

            case gSerialMgrIICMaster_c:
#if gSerialMgrUseIIC_c
                mDrvData[i].i2cMaster.bus.address = gSerialMgrIICAddress_c;
                mDrvData[i].i2cMaster.bus.baudRate_kbps = 50;
                configure_i2c_pins(instance);
                OSA_InstallIntHandler(g_i2cIrqId[instance], I2Cx_ISR);
                NVIC_SetPriority(g_i2cIrqId[instance], mSMGR_I2cIsrPrio_c >> (8 - __NVIC_PRIO_BITS));
                NVIC_ClearPendingIRQ(g_i2cIrqId[instance]);
                NVIC_EnableIRQ(g_i2cIrqId[instance]);
                I2C_DRV_MasterInit(instance, &(mDrvData[i].i2cMaster.state));
                Serial_ConfigureMasterDap(kGpioI2cDAP, I2cMasterDapISR);
#endif
                break;                

            case gSerialMgrIICSlave_c:
#if gSerialMgrUseIIC_c
                configure_i2c_pins(instance);
                Serial_ConfigureSlaveDap(kGpioI2cDAP);
                OSA_InstallIntHandler(g_i2cIrqId[instance], I2Cx_ISR);
                NVIC_SetPriority(g_i2cIrqId[instance], mSMGR_I2cIsrPrio_c >> (8 - __NVIC_PRIO_BITS));
                NVIC_ClearPendingIRQ(g_i2cIrqId[instance]);
                NVIC_EnableIRQ(g_i2cIrqId[instance]);
                I2C_DRV_SlaveInit(instance, &gI2cSlaveCfg, &(mDrvData[i].i2cSlave.state));
                I2C_DRV_SlaveReceiveData(instance, &pSer->rxBuffer[pSer->rxIn], 1 );
#endif
                break;

            case gSerialMgrSPIMaster_c:
#if gSerialMgrUseSPI_c
                Spi_Init(instance, &(mDrvData[i].spiState), SpiCallback, (void*)i );
                gSpiConfig.master = TRUE;
                mDrvData[i].spiState.signalRxByte = TRUE;
                mDrvData[i].spiState.pRxData = &pSer->rxBuffer[pSer->rxIn];
                Spi_Configure(instance, &gSpiConfig);
                Serial_ConfigureMasterDap(kGpioSpiDAP, SpiMasterDapISR);
#endif
                break;

            case gSerialMgrSPISlave_c:
#if gSerialMgrUseSPI_c
                Spi_Init(instance, &(mDrvData[i].spiState), SpiCallback, (void*)i );
                gSpiConfig.master = FALSE;
                mDrvData[i].spiState.signalRxByte = TRUE;
                mDrvData[i].spiState.pRxData = &pSer->rxBuffer[pSer->rxIn];
                Spi_Configure(instance, &gSpiConfig);
                Serial_ConfigureSlaveDap(kGpioSpiDAP);
#endif
                break;

            default:
                OSA_ExitCritical(kCriticalDisableInt);
                return gSerial_InvalidInterface_c;
            }

#if gSMGR_UseOsSemForSynchronization_c
            if( kStatus_OSA_Success != OSA_SemaCreate(&pSer->txSyncSem, 0) )
            {
                OSA_ExitCritical(kCriticalDisableInt);
                return gSerial_SemCreateError_c;
            }

#if gSerialMgr_BlockSenderOnQueueFull_c
            if( kStatus_OSA_Success != OSA_SemaCreate(&pSer->txQueueSem, 0) )
            {
                OSA_ExitCritical(kCriticalDisableInt);
                return gSerial_SemCreateError_c;
            }
#endif /* gSerialMgr_BlockSenderOnQueueFull_c */
#endif /* gSMGR_UseOsSemForSynchronization_c */

            pSer->serialType = interfaceType;
            *pInterfaceId = i;
            OSA_ExitCritical(kCriticalDisableInt);
            return gSerial_Success_c;
        }
    }

    /* There are no more free interfaces. */
    return gSerial_MaxInterfacesReached_c;
#else
    (void)interfaceType;
    (void)instance;
    (void)pInterfaceId;
    return gSerial_Success_c;
#endif
}

/*! *********************************************************************************
* \brief   Transmit a data buffer asynchronously
*
* \param[in] InterfaceId the interface number
* \param[in] pBuf pointer to data location
* \param[in] bufLen the number of bytes to be sent
* \param[in] pSerialRxCallBack pointer to a function that will be called when
*            a new char is available
*
* \return The status of the operation
*
********************************************************************************** */
serialStatus_t Serial_AsyncWrite( uint8_t id,
                                  uint8_t *pBuf,
                                  uint16_t bufLen,
                                  pSerialCallBack_t cb,
                                  void *pTxParam )
{
#if gSerialManagerMaxInterfaces_c
    SerialMsg_t *pMsg = NULL;
    serial_t *pSer = &mSerials[id];

#if gSerialMgr_ParamValidation_d
    if( (NULL == pBuf) || (0 == bufLen)       ||
        (id >= gSerialManagerMaxInterfaces_c) ||
        (pSer->serialType == gSerialMgrNone_c) )
    {
        return gSerial_InvalidParameter_c;
    }
#endif
    task_handler_t taskHandler = OSA_TaskGetHandler();

#if (gSerialMgr_BlockSenderOnQueueFull_c == 0)
    if( taskHandler == gSerialManagerTaskId )
    {
        Serial_TxQueueMaintenance(pSer);
    }
#endif

    /* Check if slot is free */
#if gSerialMgr_BlockSenderOnQueueFull_c    
    while(1)
#endif      
    {
        OSA_EnterCritical(kCriticalDisableInt);
        if( (0 == pSer->txQueue[pSer->txIn].dataSize) && (NULL == pSer->txQueue[pSer->txIn].txCallback) )
        {
            pMsg = &pSer->txQueue[pSer->txIn];
            pMsg->dataSize   = bufLen;
            pMsg->pData      = (void*)pBuf;
            pMsg->txCallback = cb;
            pMsg->pTxParam   = pTxParam;
            mSerial_IncIdx_d(pSer->txIn, gSerialMgrTxQueueSize_c);
        }
#if (gSerialMgr_BlockSenderOnQueueFull_c && gSMGR_UseOsSemForSynchronization_c)
        else
        {
            if(taskHandler != gSerialManagerTaskId)
            {
                pSer->txBlockedTasks++;
            }
        }
#endif      
        OSA_ExitCritical(kCriticalDisableInt);

        if( pMsg )
        {
            return Serial_WriteInternal( id );
        }
#if gSerialMgr_BlockSenderOnQueueFull_c
        else
        {
#if gSMGR_UseOsSemForSynchronization_c              
            if(taskHandler != gSerialManagerTaskId)
            {
                (void)OSA_SemaWait(&pSer->txQueueSem, OSA_WAIT_FOREVER);
            }
            else
#endif
            {
                Serial_TxQueueMaintenance(pSer); 
            }   
        }
#endif      
    }
    
#if (gSerialMgr_BlockSenderOnQueueFull_c == 0)
    return gSerial_OutOfMemory_c;
#endif  
#else
    (void)id;
    (void)pBuf;
    (void)bufLen;
    (void)cb;
    (void)pTxParam;
    return gSerial_Success_c;
#endif /* gSerialManagerMaxInterfaces_c */
}


/*! *********************************************************************************
* \brief Transmit a data buffer synchronously. The task will block until the Tx is done
*
* \param[in] pBuf pointer to data location
* \param[in] bufLen the number of bytes to be sent
* \param[in] InterfaceId the interface number
*
* \return The status of the operation
*
********************************************************************************** */
serialStatus_t Serial_SyncWrite( uint8_t InterfaceId,
                                 uint8_t *pBuf,
                                 uint16_t bufLen )
{
    serialStatus_t status = gSerial_Success_c;
#if gSerialManagerMaxInterfaces_c
    pSerialCallBack_t cb = NULL;
    volatile serial_t *pSer = &mSerials[InterfaceId];

#if gSMGR_UseOsSemForSynchronization_c
    /* If the calling task is SMGR do not block on semaphore */
    if( OSA_TaskGetHandler() != gSerialManagerTaskId )
         cb = Serial_SyncTxCallback;
#endif

    status  = Serial_AsyncWrite(InterfaceId, pBuf, bufLen, cb, (void*)pSer);

    if( gSerial_Success_c == status )
    {
        /* Wait until Tx finishes. The sem will be released by the SMGR task */
#if gSMGR_UseOsSemForSynchronization_c
        if( cb )
        {
            (void)OSA_SemaWait((semaphore_t*)&pSer->txSyncSem, OSA_WAIT_FOREVER);
        }
        else
#endif
        {
            while(pSer->state);
        }
    }
#else
    (void)pBuf;
    (void)bufLen;
    (void)InterfaceId;
#endif /* gSerialManagerMaxInterfaces_c */
    return status;
}

/*! *********************************************************************************
* \brief   Returns a specified number of characters from the Rx buffer
*
* \param[in] InterfaceId the interface number
* \param[out] pData pointer to location where to store the characters
* \param[in] dataSize the number of characters to be read
* \param[out] bytesRead the number of characters read
*
* \return The status of the operation
*
********************************************************************************** */
serialStatus_t Serial_Read( uint8_t InterfaceId,
                            uint8_t *pData,
                            uint16_t dataSize,
                            uint16_t *bytesRead )
{
#if (gSerialManagerMaxInterfaces_c)
    serial_t *pSer = &mSerials[InterfaceId];
    serialStatus_t status = gSerial_Success_c;
    uint16_t i, bytes;

#if gSerialMgr_ParamValidation_d
    if ( (InterfaceId >= gSerialManagerMaxInterfaces_c) ||
        (NULL == pData) || (0 == dataSize) )
        return gSerial_InvalidParameter_c;
#endif

    /* Copy bytes from the SMGR Rx buffer */
    Serial_RxBufferByteCount(InterfaceId, &bytes);

    if( bytes > 0 )
    {
        if( bytes > dataSize )
            bytes = dataSize;

        /* Copy data */
        for( i=0; i<bytes; i++ )
        {
           OSA_EnterCritical(kCriticalDisableInt);          
           *pData++ = pSer->rxBuffer[pSer->rxOut++];
            if ( pSer->rxOut >= gSMRxBufSize_c )
            {
                pSer->rxOut = 0;
            }
           OSA_ExitCritical(kCriticalDisableInt);
        }

        dataSize -= bytes;
    }

    /* Aditional processing depending on interface */
    switch ( pSer->serialType )
    {
#if gSerialMgrUseUSB_c
    case gSerialMgrUSB_c:
        VirtualCom_SMReadNotify( mDrvData[InterfaceId].pDrvData );
        break;
#endif

#if gSerialMgrUseIIC_c
    case gSerialMgrIICMaster_c:
        if( !dataSize )
        {
            break;
        }
        /* Check if the I2C Slave has data available */
        if( !GPIO_DRV_ReadPinInput(kGpioI2cDAP) )
        {
            break;
        }
        /* Read remaining bytes from the IIC slave */        
        if(kStatus_I2C_Success != I2C_DRV_MasterReceiveDataBlocking(pSer->serialChannel, 
                                                                    &mDrvData[InterfaceId].i2cMaster.bus, 
                                                                    NULL, 0, pData, dataSize, OSA_WAIT_FOREVER) )
        {
            status = gSerial_InternalError_c;
            break;
        }
        bytes += dataSize;
        break;
#endif

    default:
        break;
    }

    if( bytesRead )
        *bytesRead = bytes;

    return status;
#else
    (void)InterfaceId;
    (void)pData;
    (void)dataSize;
    (void)bytesRead;
    return gSerial_InvalidInterface_c;
#endif
}

/*! *********************************************************************************
* \brief   Returns a the number of bytes available in the RX buffer
*
* \param[in] InterfaceId the interface number
* \param[out] bytesCount the number of bytes available
*
* \return The status of the operation
*
********************************************************************************** */
serialStatus_t Serial_RxBufferByteCount( uint8_t InterfaceId, uint16_t *bytesCount )
{
#if (gSerialManagerMaxInterfaces_c)
#if gSerialMgr_ParamValidation_d
    if ( (InterfaceId >= gSerialManagerMaxInterfaces_c) ||
        (NULL == bytesCount) )
        return  gSerial_InvalidParameter_c;
#endif

    OSA_EnterCritical(kCriticalDisableInt);

    if( mSerials[InterfaceId].rxIn >= mSerials[InterfaceId].rxOut )
    {
        *bytesCount = mSerials[InterfaceId].rxIn - mSerials[InterfaceId].rxOut;
    }
    else
    {
        *bytesCount = gSMRxBufSize_c - mSerials[InterfaceId].rxOut + mSerials[InterfaceId].rxIn;
    }

    OSA_ExitCritical(kCriticalDisableInt);
#else
    (void)bytesCount;
    (void)InterfaceId;
#endif
    return gSerial_Success_c;
}

/*! *********************************************************************************
* \brief   Sets a pointer to a function that will be called when data is received
*
* \param[in] InterfaceId the interface number
* \param[in] pfCallBack pointer to the function to be called
* \param[in] pRxParam pointer to a parameter which will be passed to the CB function
*
* \return The status of the operation
*
********************************************************************************** */
serialStatus_t Serial_SetRxCallBack( uint8_t InterfaceId, pSerialCallBack_t cb, void *pRxParam )
{
#if (gSerialManagerMaxInterfaces_c)
#if gSerialMgr_ParamValidation_d
    if ( InterfaceId >= gSerialManagerMaxInterfaces_c )
        return gSerial_InvalidParameter_c;
#endif
    mSerials[InterfaceId].rxCallback = cb;
    mSerials[InterfaceId].pRxParam = pRxParam;
#else
    (void)InterfaceId;
    (void)cb;
    (void)pRxParam;
#endif
    return gSerial_Success_c;
}

/*! *********************************************************************************
* \brief   Set the communication speed for an interface
*
* \param[in] baudRate communication speed
* \param[in] InterfaceId the interface number
*
* \return The status of the operation
*
********************************************************************************** */
serialStatus_t Serial_SetBaudRate( uint8_t InterfaceId, uint32_t baudRate  )
{
    serialStatus_t status = gSerial_Success_c;
#if gSerialManagerMaxInterfaces_c

#if gSerialMgr_ParamValidation_d
    if ( (InterfaceId >= gSerialManagerMaxInterfaces_c) || (0 == baudRate) )
        return gSerial_InvalidParameter_c;
#endif

    switch ( mSerials[InterfaceId].serialType )
    {
#if (gSerialMgrUseUart_c)
#if FSL_FEATURE_SOC_UART_COUNT
    case gSerialMgrUart_c:
        {
            uint32_t instance = mSerials[InterfaceId].serialChannel;
            UART_HAL_SetBaudRate(g_uartBase[instance], CLOCK_SYS_GetUartFreq(instance), baudRate);
        }
        break;
#endif
#if FSL_FEATURE_SOC_LPUART_COUNT
    case gSerialMgrLpuart_c:
        {
            uint32_t instance = mSerials[InterfaceId].serialChannel;
            LPUART_HAL_SetBaudRate(g_lpuartBase[instance], CLOCK_SYS_GetLpuartFreq(instance), baudRate);
        }
        break;
#endif
#if FSL_FEATURE_SOC_LPSCI_COUNT
    case gSerialMgrLpsci_c:
        {
            uint32_t instance = mSerials[InterfaceId].serialChannel;
            LPSCI_HAL_SetBaudRate(g_lpsciBase[instance], CLOCK_SYS_GetLpsciFreq(instance), baudRate);
        }
        break;
#endif
#endif /* #if (gSerialMgrUseUart_c) */
#if gSerialMgrUseIIC_c
    case gSerialMgrIICMaster_c:
        mDrvData[InterfaceId].i2cMaster.bus.baudRate_kbps = baudRate/1000;
        break;
#endif
#if gSerialMgrUseSPI_c
    case gSerialMgrSPIMaster_c:
        gSpiConfig.bitsPerSec = baudRate;
        Spi_Configure(mSerials[InterfaceId].serialChannel, &gSpiConfig);
        break;
#endif
#if gSerialMgrUseUSB_c
    case gSerialMgrUSB_c:
        /* Nothing to do here. */
        break;
#endif
    default:
        status = gSerial_InvalidInterface_c;
    }
#endif
    return status;
}

/*! *********************************************************************************
* \brief   Prints a string to the serial interface
*
* \param[in] InterfaceId the interface number
* \param[in] pString pointer to the string to be printed
* \param[in] allowToBlock specify if the task will wait for the tx to finish or not.
*
* \return The status of the operation
*
********************************************************************************** */
serialStatus_t Serial_Print( uint8_t InterfaceId, char* pString, serialBlock_t allowToBlock )
{
#if gSerialManagerMaxInterfaces_c
    if ( allowToBlock )
    {
        return Serial_SyncWrite( InterfaceId, (uint8_t*)pString, strlen(pString) );
    }
    else
    {
        return Serial_AsyncWrite( InterfaceId, (uint8_t*)pString, strlen(pString), NULL, NULL );
    }
#else
    (void)pString;
    (void)allowToBlock;
    (void)InterfaceId;
    return gSerial_Success_c;
#endif
}

/*! *********************************************************************************
* \brief   Prints an number in hedadecimal format to the serial interface
*
* \param[in] InterfaceId the interface number
* \param[in] hex pointer to the number to be printed
* \param[in] len the number ob bytes of the number
* \param[in] flags specify display options: comma, space, new line
*
* \return The status of the operation
*
* \remarks The task will waituntil the tx has finished
*
********************************************************************************** */
serialStatus_t Serial_PrintHex( uint8_t InterfaceId,
                                uint8_t *hex,
                                uint8_t len,
                                uint8_t flags )
{
#if (gSerialManagerMaxInterfaces_c)
    uint8_t i=0;
    serialStatus_t status;
    uint8_t hexString[6]; /* 2 bytes  - hexadecimal display
    1 byte   - separator ( comma)
    1 byte   - separator ( space)
    2 bytes  - new line (\n\r)  */

    if ( !(flags & gPrtHexBigEndian_c) )
        hex = hex + (len-1);

    while ( len )
    {
        /* start preparing the print of a new byte */
        i=0;
        hexString[i++] = HexToAscii( (*hex)>>4 );
        hexString[i++] = HexToAscii( *hex );

        if ( flags & gPrtHexCommas_c )
        {
            hexString[i++] = ',';
        }
        if ( flags & gPrtHexSpaces_c )
        {
            hexString[i++] = ' ';
        }
        hex = hex + (flags & gPrtHexBigEndian_c ? 1 : -1);
        len--;

        if ( (len == 0) && (flags & gPrtHexNewLine_c) )
        {
            hexString[i++] = '\n';
            hexString[i++] = '\r';
        }

        /* transmit formatted byte */
        status = Serial_SyncWrite( InterfaceId, (uint8_t*)hexString, (uint8_t)i) ;
        if ( gSerial_Success_c != status )
            return status;
    }
#else
    /* Avoid compiler warning */
    (void)hex;
    (void)len;
    (void)InterfaceId;
    (void)flags;
#endif
    return gSerial_Success_c;
}

/*! *********************************************************************************
* \brief   Prints an unsigned integer to the serial interface
*
* \param[in] InterfaceId the interface number
* \param[in] nr the number to be printed
*
* \return The status of the operation
*
* \remarks The task will waituntil the tx has finished
*
********************************************************************************** */
serialStatus_t Serial_PrintDec( uint8_t InterfaceId, uint32_t nr )
{
#if (gSerialManagerMaxInterfaces_c)
#define gDecStringLen_d 12
    uint8_t i = gDecStringLen_d-1;
    uint8_t decString[gDecStringLen_d];

    if ( nr == 0 )
    {
        decString[i] = '0';
    }
    else
    {
        while ( nr )
        {
            decString[i] = '0' + (uint8_t)(nr % 10);
            nr = nr / 10;
            i--;
        }
        i++;
    }

    /* transmit formatted byte */
    return Serial_SyncWrite( InterfaceId, (uint8_t*)&decString[i], gDecStringLen_d-i );
#else
    (void)nr;
    (void)InterfaceId;
    return gSerial_Success_c;
#endif
}


/*! *********************************************************************************
* \brief   Configures the enabled hardware modules of the given interface type as a wakeup source from STOP mode  
*
* \param[in] interface type of the modules to configure
*
* \return  gSerial_Success_c if there is at least one module to configure
*          gSerial_InvalidInterface_c otherwise 
* \pre
*
* \post
*
* \remarks 
*
********************************************************************************** */

serialStatus_t Serial_EnableLowPowerWakeup( serialInterfaceType_t interfaceType )
{
    serialStatus_t status = gSerial_Success_c;
#if gSerialManagerMaxInterfaces_c
    uint8_t uartIdx = 0;

    switch(interfaceType)
    {
#if (gSerialMgrUseUart_c)
#if FSL_FEATURE_SOC_UART_COUNT
    case gSerialMgrUart_c:
        while( uartIdx <= FSL_FEATURE_SOC_UART_COUNT-1 )
        {
            if(CLOCK_SYS_GetUartGateCmd(uartIdx))
            {
                UART_HAL_SetIntMode(g_uartBase[uartIdx], kUartIntRxActiveEdge, FALSE);
                UART_HAL_ClearStatusFlag(g_uartBase[uartIdx], kUartRxActiveEdgeDetect);
                UART_HAL_SetIntMode(g_uartBase[uartIdx], kUartIntRxActiveEdge, TRUE);
            }
            uartIdx++;
        }
        break;
#endif
#if FSL_FEATURE_SOC_LPUART_COUNT
    case gSerialMgrLpuart_c:
        while( uartIdx <= FSL_FEATURE_SOC_LPUART_COUNT-1 )
        {
            if(CLOCK_SYS_GetLpuartGateCmd(uartIdx))
            {
                LPUART_HAL_SetIntMode(g_lpuartBase[uartIdx], kLpuartIntRxActiveEdge, FALSE);
                LPUART_HAL_ClearStatusFlag(g_lpuartBase[uartIdx], kLpuartRxActiveEdgeDetect);
                LPUART_HAL_SetIntMode(g_lpuartBase[uartIdx], kLpuartIntRxActiveEdge, TRUE);
            }
            uartIdx++;
        }
        break;
#endif
#if FSL_FEATURE_SOC_LPSCI_COUNT
    case gSerialMgrLpsci_c:
        while( uartIdx <= FSL_FEATURE_SOC_LPSCI_COUNT-1 )
        {
            if(CLOCK_SYS_GetLpsciGateCmd(uartIdx))
            {
                LPSCI_HAL_SetIntMode(g_lpsciBase[uartIdx], kLpsciIntRxActiveEdge, FALSE);
                LPSCI_HAL_ClearStatusFlag(g_lpsciBase[uartIdx], kLpsciRxActiveEdgeDetect);
                LPSCI_HAL_SetIntMode(g_lpsciBase[uartIdx], kLpsciIntRxActiveEdge, TRUE);
            }
            uartIdx++;
        }
        break;
#endif
#endif /* #if (gSerialMgrUseUart_c) */
    default:
        status = gSerial_InvalidInterface_c;
        break;
    }
#endif /* #if gSerialManagerMaxInterfaces_c */
    return status;
}

/*! *********************************************************************************
* \brief   Configures the enabled hardware modules of the given interface type as modules without wakeup capabilities  
*
* \param[in] interface type of the modules to configure
*
* \return  gSerial_Success_c if there is at least one module to configure 
*          gSerial_InvalidInterface_c otherwise 
* \pre
*
* \post
*
* \remarks 
*
********************************************************************************** */

serialStatus_t Serial_DisableLowPowerWakeup( serialInterfaceType_t interfaceType )
{
    serialStatus_t status = gSerial_Success_c;
#if gSerialManagerMaxInterfaces_c
    uint8_t uartIdx = 0;

    switch(interfaceType)
    {
#if (gSerialMgrUseUart_c)
#if FSL_FEATURE_SOC_UART_COUNT
    case gSerialMgrUart_c:
        while( uartIdx <= FSL_FEATURE_SOC_UART_COUNT-1 )
        {
            if(CLOCK_SYS_GetUartGateCmd(uartIdx))
            {
                UART_HAL_SetIntMode(g_uartBase[uartIdx], kUartIntRxActiveEdge, FALSE);
                UART_HAL_ClearStatusFlag(g_uartBase[uartIdx], kUartRxActiveEdgeDetect);
            }
            uartIdx++;
        }
        break;
#endif
#if FSL_FEATURE_SOC_LPUART_COUNT
    case gSerialMgrLpuart_c:
        while( uartIdx <= FSL_FEATURE_SOC_LPUART_COUNT-1 )
        {
            if(CLOCK_SYS_GetLpuartGateCmd(uartIdx))
            {
                LPUART_HAL_SetIntMode(g_lpuartBase[uartIdx], kLpuartIntRxActiveEdge, FALSE);
                LPUART_HAL_ClearStatusFlag(g_lpuartBase[uartIdx], kLpuartRxActiveEdgeDetect);
            }
            uartIdx++;
        }
        break;
#endif
#if FSL_FEATURE_SOC_LPSCI_COUNT
    case gSerialMgrLpsci_c:
        while( uartIdx <= FSL_FEATURE_SOC_LPSCI_COUNT-1 )
        {
            if(CLOCK_SYS_GetLpsciGateCmd(uartIdx))
            {
                LPSCI_HAL_SetIntMode(g_lpsciBase[uartIdx], kLpsciIntRxActiveEdge, FALSE);
                LPSCI_HAL_ClearStatusFlag(g_lpsciBase[uartIdx], kLpsciRxActiveEdgeDetect);
            }
            uartIdx++;
        }
        break;
#endif
#endif /* #if (gSerialMgrUseUart_c) */
    default:
        status = gSerial_InvalidInterface_c;
        break;
    }
#endif /* #if gSerialManagerMaxInterfaces_c */
    return status;
}

/*! *********************************************************************************
* \brief   Decides whether a enabled hardware module of the given interface type woke up the CPU from STOP mode.  
*
* \param[in] interface type of the modules to be evaluated as wakeup source.
*
* \return  TRUE if a module of the given interface type was the wakeup source
*          FALSE otherwise 
* \pre
*
* \post
*
* \remarks 
*
********************************************************************************** */

bool_t Serial_IsWakeUpSource( serialInterfaceType_t interfaceType)
{
#if gSerialManagerMaxInterfaces_c
    uint8_t uartIdx = 0;

    switch(interfaceType)
    {
#if (gSerialMgrUseUart_c)
#if FSL_FEATURE_SOC_UART_COUNT
    case gSerialMgrUart_c:
        while( uartIdx <= FSL_FEATURE_SOC_UART_COUNT-1 )
        {
            if(CLOCK_SYS_GetUartGateCmd(uartIdx))
            {
                if( UART_HAL_GetStatusFlag(g_uartBase[uartIdx], kUartRxActiveEdgeDetect) )
                {
                    return TRUE;
                }  
            }
            uartIdx++;
        }
        break;
#endif
#if FSL_FEATURE_SOC_LPUART_COUNT
    case gSerialMgrLpuart_c:
        while( uartIdx <= FSL_FEATURE_SOC_LPUART_COUNT-1 )
        {
            if(CLOCK_SYS_GetLpuartGateCmd(uartIdx))
            {
                if( LPUART_HAL_GetStatusFlag(g_lpuartBase[uartIdx], kLpuartRxActiveEdgeDetect) )
                {
                    return TRUE;
                }  
            }
            uartIdx++;
        }
        break;
#endif
#if FSL_FEATURE_SOC_LPSCI_COUNT
    case gSerialMgrLpsci_c:
        while( uartIdx <= FSL_FEATURE_SOC_LPSCI_COUNT-1 )
        {
            if(CLOCK_SYS_GetLpsciGateCmd(uartIdx))
            {
                if( LPSCI_HAL_GetStatusFlag(g_lpsciBase[uartIdx], kLpsciRxActiveEdgeDetect) )
                {
                    return TRUE;
                }  
            }
            uartIdx++;
        }
        break;
#endif
#endif /* #if (gSerialMgrUseUart_c) */
    default:
        break;
    }
#else
    (void)interfaceType;
#endif
    return FALSE;
}


/*! *********************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
********************************************************************************* */
#if (gSerialManagerMaxInterfaces_c)
/*! *********************************************************************************
* \brief Transmit a data buffer to the specified interface.
*
* \param[in] InterfaceId the interface number
*
* \return The status of the operation
*
********************************************************************************** */
static serialStatus_t Serial_WriteInternal( uint8_t InterfaceId )
{
    serialStatus_t status = gSerial_Success_c;
    serial_t *pSer = &mSerials[InterfaceId];
    uint16_t idx;

    OSA_EnterCritical(kCriticalDisableInt);
    if( pSer->state == 0 )
    {
        pSer->state = 1;
#if gSerialMgr_DisallowMcuSleep_d
        PWR_DisallowDeviceToSleep();
#endif
    }
    else
    {
        /* The interface is busy transmitting!
         * The current data will be transmitted after the previous transmissions end. 
         */
        OSA_ExitCritical(kCriticalDisableInt);
        return gSerial_Success_c;
    }
    OSA_ExitCritical(kCriticalDisableInt);

    idx = pSer->txCurrent;
    if(pSer->txQueue[idx].dataSize == 0)
    {
#if gSerialMgr_DisallowMcuSleep_d
        PWR_AllowDeviceToSleep();
#endif
        pSer->state = 0;
        return gSerial_Success_c;
    }

    switch ( mSerials[InterfaceId].serialType )
    {
#if (gSerialMgrUseUart_c)
#if FSL_FEATURE_SOC_UART_COUNT
    case gSerialMgrUart_c:
        if( kStatus_UART_Success != UART_DRV_SendData( pSer->serialChannel, 
                                                       pSer->txQueue[idx].pData, 
                                                       pSer->txQueue[idx].dataSize ) )
        {
            status = gSerial_InternalError_c;
        }
        break;
#endif
#if FSL_FEATURE_SOC_LPUART_COUNT
    case gSerialMgrLpuart_c:
        if( kStatus_LPUART_Success != LPUART_DRV_SendData( pSer->serialChannel, 
                                                           pSer->txQueue[idx].pData, 
                                                           pSer->txQueue[idx].dataSize ) )
        {
            status = gSerial_InternalError_c;
        }
        break;
#endif
#if FSL_FEATURE_SOC_LPSCI_COUNT
    case gSerialMgrLpsci_c:
        if( kStatus_LPSCI_Success != LPSCI_DRV_SendData( pSer->serialChannel, 
                                                         pSer->txQueue[idx].pData, 
                                                         pSer->txQueue[idx].dataSize ) )
        {
            status = gSerial_InternalError_c;
        }
        break;
#endif
#endif /* #if (gSerialMgrUseUart_c) */

#if gSerialMgrUseUSB_c
    case gSerialMgrUSB_c:
        if( VirtualCom_Write(mDrvData[InterfaceId].pDrvData, 
                             pSer->txQueue[idx].pData, 
                             pSer->txQueue[idx].dataSize) )
        {
            status = gSerial_InternalError_c;
        }
        break;
#endif

#if gSerialMgrUseIIC_c
    case gSerialMgrIICMaster_c:
        if( kStatus_I2C_Success != I2C_DRV_MasterSendData( pSer->serialChannel, 
                                                           &mDrvData[InterfaceId].i2cMaster.bus, NULL, 0,
                                                           pSer->txQueue[idx].pData, pSer->txQueue[idx].dataSize ) )
        {
            status = gSerial_InternalError_c;
        }
        break;

    case gSerialMgrIICSlave_c:
        /* Notify IIC Master that we have data to send */
        if( kStatus_I2C_Success != I2C_DRV_SlaveSendData(pSer->serialChannel, 
                                                         pSer->txQueue[idx].pData, 
                                                         pSer->txQueue[idx].dataSize) )
        {
            status = gSerial_InternalError_c;
            break;
        }
        GPIO_DRV_SetPinOutput(kGpioI2cDAP);
        break;
#endif

#if gSerialMgrUseSPI_c
    case gSerialMgrSPISlave_c:
        /* Notify SPI Master that we have data to send */
        if( Spi_AsyncTransfer(pSer->serialChannel,
                                   pSer->txQueue[idx].pData,
                                   &pSer->rxBuffer[pSer->rxIn],
                                   pSer->txQueue[idx].dataSize) )
        {
          status = gSerial_InternalError_c;
          break;
        }
        GPIO_DRV_SetPinOutput(kGpioSpiDAP);
        break;

    case gSerialMgrSPIMaster_c:
        if( Spi_AsyncTransfer(pSer->serialChannel,
                                    pSer->txQueue[idx].pData,
                                    &pSer->rxBuffer[pSer->rxIn],
                                    pSer->txQueue[idx].dataSize) )
        {
          status = gSerial_InternalError_c;
        }
        break;
#endif
    default:
        status = gSerial_InternalError_c;
    }

    if( status != gSerial_Success_c )
    {
#if gSerialMgr_DisallowMcuSleep_d
        PWR_AllowDeviceToSleep();
#endif
        pSer->txQueue[idx].dataSize = 0;
        pSer->txQueue[idx].txCallback = NULL;
        mSerial_IncIdx_d(pSer->txCurrent, gSerialMgrTxQueueSize_c);
        pSer->state = 0;
    }

    return status;
}

/*! *********************************************************************************
* \brief Inform the Serial Manager task that new data is available
*
* \param[in] pData The id interface
*
* \return none
*
* \remarks Called from usb task
*
********************************************************************************** */

#if gSerialMgrUseUSB_c
void SerialManager_VirtualComRxNotify(uint8_t* pData, uint16_t dataSize, uint8_t interface)
{

  while(dataSize)
  {
    OSA_EnterCritical(kCriticalDisableInt);
    mSerials[interface].rxBuffer[mSerials[interface].rxIn] = *pData++;
    mSerial_IncIdx_d(mSerials[interface].rxIn, gSMRxBufSize_c);
    if(mSerials[interface].rxIn == mSerials[interface].rxOut)
    {
      mSerial_IncIdx_d(mSerials[interface].rxOut, gSMRxBufSize_c);
    }
    OSA_ExitCritical(kCriticalDisableInt);
    dataSize--;
  }
  
   mSerials[interface].events |= gSMGR_Rx_c;
   (void)OSA_EventSet(&mSMTaskEvent, gSMGR_Rx_c);    
  
}
#endif
/*! *********************************************************************************
* \brief Inform the Serial Manager task that new data is available
*
* \param[in] pData The id interface
*
* \return none
*
* \remarks Called from ISR
*
********************************************************************************** */
void SerialManager_RxNotify( uint32_t i )
{
    serial_t *pSer = &mSerials[i];
#if gSerialMgrUseFSCIHdr_c
    uint8_t rxByte = pSer->rxBuffer[pSer->rxIn];
    uint8_t slaveDapRxEnd = 0;
#endif    

    mSerial_IncIdx_d(pSer->rxIn, gSMRxBufSize_c);
    if(pSer->rxIn == pSer->rxOut)
    {
        mSerial_IncIdx_d(pSer->rxOut, gSMRxBufSize_c);
    }

    switch( pSer->serialType )
    {
        /* Uart driver is in continuous Rx. No need to restart reception. */
#if gSerialMgrUseSPI_c
    case gSerialMgrSPISlave_c:
        /* No need to restart RX since SPI is in continuous RX mode */
        break;
    case gSerialMgrSPIMaster_c:
#if gSerialMgrUseFSCIHdr_c
        if( (0 == pSer->rxFsciPkt) && (gFSCI_StartMarker_c == rxByte) )
        {
            pSer->rxFsciPkt = 1;
        }
        
        if( pSer->rxFsciPkt )
        {
            pSer->rxFsciIn++;
            
#if gFsciLenHas2Bytes_c            
            if( (mSMGR_FSCIHdrLen_c - 1) == pSer->rxFsciIn )
            {
                pSer->rxFsciLen = rxByte + 1; /* CRC */
            }
            else if( mSMGR_FSCIHdrLen_c == pSer->rxFsciIn )
            {
                pSer->rxFsciLen += rxByte << 8;
            }
#else       
            if( mSMGR_FSCIHdrLen_c == pSer->rxFsciIn )
            {
                pSer->rxFsciLen = rxByte + 1; /* CRC */
            }
#endif
            if( pSer->rxFsciLen == (pSer->rxFsciIn - mSMGR_FSCIHdrLen_c) )
            {
                pSer->rxFsciPkt = 0;
                pSer->rxFsciIn = 0;
                pSer->rxFsciLen = 0;
                
                slaveDapRxEnd = GPIO_DRV_ReadPinInput(kGpioSpiDAP);
            }
        }
        
        /* If more bytes need to be received */
        if( (pSer->rxFsciPkt || slaveDapRxEnd) && !pSer->state )
        {
#if gSMGR_UseOsSemForSynchronization_c
            if( (0 == pSer->txQueue[pSer->txIn].dataSize) && (NULL == pSer->txQueue[pSer->txIn].txCallback) )
#endif
            {          
                Serial_AsyncWrite( i, &mSPI_dummyData, 1, NULL, NULL );
            }    
        }
#else /* gSerialMgrUseFSCIHdr_c */   
        /* If the SPI Slave has more data to transmit, restart the transfer */
        if( GPIO_DRV_ReadPinInput(kGpioSpiDAP) && !pSer->state )
        {
#if gSMGR_UseOsSemForSynchronization_c
            if( (0 == pSer->txQueue[pSer->txIn].dataSize) && (NULL == pSer->txQueue[pSer->txIn].txCallback) )
#endif
            {
                Serial_AsyncWrite( i, &mSPI_dummyData, 1, NULL, NULL );
            }
        }
#endif
        break;
#endif

#if gSerialMgrUseIIC_c
    case gSerialMgrIICSlave_c:
        /* Restart I2C RX */
        I2C_DRV_SlaveReceiveData(pSer->serialChannel, &pSer->rxBuffer[pSer->rxIn], 1 );
        break;

    case gSerialMgrIICMaster_c:
        pSer->rxIn = 0;
        /* Just notify the application that the I2C Slave has data to transmit*/
        break;
#endif
    default:
        break;
    }

    /* Signal SMGR task if not allready done */
    if( !(pSer->events & gSMGR_Rx_c) )
    {
        pSer->events |= gSMGR_Rx_c;
        (void)OSA_EventSet(&mSMTaskEvent, gSMGR_Rx_c);
    }
}

/*! *********************************************************************************
* \brief Inform the Serial Manager task that a transmission has finished
*
* \param[in] pData the Id interface
*
* \return none
*
* \remarks Called from ISR
*
********************************************************************************** */
void SerialManager_TxNotify( uint32_t i )
{
  
    serial_t *pSer = &mSerials[i];

    OSA_EnterCritical(kCriticalDisableInt);
    pSer->events |= gSMGR_TxDone_c;
    pSer->txQueue[pSer->txCurrent].dataSize = 0; //Mark as transmitted
    mSerial_IncIdx_d(pSer->txCurrent, gSerialMgrTxQueueSize_c);
#if gSerialMgr_DisallowMcuSleep_d
    PWR_AllowDeviceToSleep();
#endif
    pSer->state = 0;
    OSA_ExitCritical(kCriticalDisableInt);

    /* Transmit next block if available */
    if( pSer->txCurrent != pSer->txIn )
    {
       if( pSer->serialType != gSerialMgrIICMaster_c && pSer->serialType != gSerialMgrIICSlave_c )
        {
            (void)Serial_WriteInternal(i);
        }
    }
    else
    {
#if (gSerialMgrUseIIC_c)
      
      if( pSer->serialType == gSerialMgrIICMaster_c &&
         GPIO_DRV_ReadPinInput(kGpioI2cDAP) )
      {
        /* Signal SMGR task if not allready done */
        if( !(pSer->events & gSMGR_Rx_c) )
        {
          pSer->events |= gSMGR_Rx_c;
          (void)OSA_EventSet(&mSMTaskEvent, gSMGR_Rx_c);
        }
      }
        if( pSer->serialType == gSerialMgrIICSlave_c )
        {
            GPIO_DRV_ClearPinOutput(kGpioI2cDAP);
        }
#endif
#if (gSerialMgrUseSPI_c)
        if( pSer->serialType == gSerialMgrSPISlave_c )
        {
            GPIO_DRV_ClearPinOutput(kGpioSpiDAP);
        }
#endif
    }
    (void)OSA_EventSet(&mSMTaskEvent, gSMGR_TxDone_c);
}


/*! *********************************************************************************
* \brief   This function will mark all finished TX queue entries as empty.
*          If a calback was provided, it will be run.
*
* \param[in] pSer pointer to the serial interface internal structure
*
********************************************************************************** */
static void Serial_TxQueueMaintenance(serial_t *pSer)
{
    uint32_t i;

    while( pSer->txQueue[pSer->txOut].dataSize == 0 )
    {
        i = pSer->txOut;
        mSerial_IncIdx_d(pSer->txOut, gSerialMgrTxQueueSize_c);
        
        /* Run Calback */
        if( pSer->txQueue[i].txCallback )
        {
            pSer->txQueue[i].txCallback( pSer->txQueue[i].pTxParam );
            pSer->txQueue[i].txCallback = NULL;
        }

#if gSerialMgr_BlockSenderOnQueueFull_c && gSMGR_UseOsSemForSynchronization_c
        OSA_EnterCritical(kCriticalDisableInt);        
        if( pSer->txBlockedTasks )
        {
            pSer->txBlockedTasks--;
            OSA_ExitCritical(kCriticalDisableInt);
            (void)OSA_SemaPost(&pSer->txQueueSem);
        }
        else
        {
          OSA_ExitCritical(kCriticalDisableInt);
        }
#endif
        if( pSer->txOut == pSer->txIn )
            break;
//#if defined(FWK_SMALL_RAM_CONFIG)
//        if( pSer->txQueue[pSer->txOut].dataSize )
//            (void)OSA_EventSet(&mSMTaskEvent, gSMGR_TxDone_c);
//        return;
//#endif
    }
}

/*! *********************************************************************************
* \brief   This function will unblock the task who called Serial_SyncWrite().
*
* \param[in] pSer pointer to the serial interface internal structure
*
********************************************************************************** */
#if gSMGR_UseOsSemForSynchronization_c
static void Serial_SyncTxCallback(void *pSer)
{
    (void)OSA_SemaPost( &((serial_t *)pSer)->txSyncSem );
}
#endif

/*! *********************************************************************************
* \brief   This function will return the interfaceId for the specified interface
*
* \param[in] type     the interface type
* \param[in] channel  the instance of the interfacte
*
* \return The mSerials index for the specified interface type and channel
*
********************************************************************************** */
uint32_t Serial_GetInterfaceId(serialInterfaceType_t type, uint32_t channel)
{
    uint32_t i;
    
    for(i=0; i<gSerialManagerMaxInterfaces_c; i++)
    {
        if( (mSerials[i].serialType == type) && 
            (mSerials[i].serialChannel == channel) )
            return i;
    }

    return gSerialMgrInvalidIdx_c;
}

/*! *********************************************************************************
* \brief   This function will return the first interfaceId for the specified interface type
*
* \param[in] type     the interface type
*
* \return The mSerials index for the specified interface type
*
*
********************************************************************************** */
#if (gSerialMgrUseSPI_c) || (gSerialMgrUseIIC_c)
static uint32_t Serial_GetInterfaceIdFromType(serialInterfaceType_t type)
{
    uint32_t i;
    
    for(i=0; i<gSerialManagerMaxInterfaces_c; i++)
    {
        if( mSerials[i].serialType == type )
            return i;
    }

    return gSerialMgrInvalidIdx_c;
}
#endif

/*! *********************************************************************************
* \brief   SPI transfet complete ISR callback
*
* \param[in] instance     the instance of the SPI module
*
********************************************************************************** */
#if (gSerialMgrUseSPI_c)
static void SpiCallback(uint32_t flags, void* pSpiState)
{
    spiState_t *pState = (spiState_t*)pSpiState;
    uint32_t    instance = (uint32_t)pState->callbackParam;
    serial_t   *pSer = &mSerials[instance];

    if( flags & gSPI_TxEndFlag_d )
    {
        /* SPI Tx sequence end */
        SerialManager_TxNotify(instance);
    }

    if( flags & (gSPI_RxEndFlag_d | gSPI_ByteRxFlag_d) )
    {
        /* SPI Rx sequence end OR new byte received */
        SerialManager_RxNotify(instance);
    }

    if( flags & gSPI_ByteRxFlag_d )
    {
        /* Update data pointer for next SPI Rx*/
        pState->pRxData = &pSer->rxBuffer[pSer->rxIn];  
    }
}
#endif

/*! *********************************************************************************
* \brief   SPI Master data available pin ISR
*
********************************************************************************** */
#if (gSerialMgrUseSPI_c)
static void SpiMasterDapISR(void)
{
    PORT_Type *port = g_portBase[GPIO_EXTRACT_PORT(kGpioSpiDAP)];
    uint32_t pin  = GPIO_EXTRACT_PIN(kGpioSpiDAP);
    uint32_t i = Serial_GetInterfaceIdFromType(gSerialMgrSPIMaster_c);

    if( PORT_HAL_IsPinIntPending(port, pin) )
    {
        if( GPIO_DRV_ReadPinInput(kGpioSpiDAP) )
        {
            /* Change IRQ logic to detect when SPI Slave has no more data to send */
            PORT_HAL_SetPinIntMode( port, pin, kPortIntLogicZero);

            /* SPI Master will start a dummy transfer to receive data from SPI Slave */
            if( mSerials[i].state == 0 )
            {
                Serial_AsyncWrite( i, &mSPI_dummyData, 1, NULL, NULL );
            }
        }
        else
        {
            /* Change IRQ logic to detect when SPI Slave has new data to send */
            PORT_HAL_SetPinIntMode( port, pin, kPortIntLogicOne);
        }

        PORT_HAL_ClearPinIntFlag(port, pin);
    }
}
#endif

/*! *********************************************************************************
* \brief   I2C ISR wrapper
*
********************************************************************************** */
#if (gSerialMgrUseIIC_c)
static void I2Cx_ISR(void)
{
    uint32_t irq = __get_IPSR() - 16;
    uint32_t instance;
    uint8_t isMaster;
    uint8_t idle = FALSE;

    for( instance=0; instance<I2C_INSTANCE_COUNT; instance++ )
    {
        if( irq == g_i2cIrqId[instance] )
            break;
    }

    isMaster = I2C_HAL_IsMaster(g_i2cBase[instance]);

    I2C_DRV_IRQHandler(instance);

    if( isMaster )
    {
        idle = ((i2c_master_state_t *)g_i2cStatePtr[instance])->i2cIdle;
    }
//    else
//    {
//        idle = !((i2c_slave_state_t *)g_i2cStatePtr[instance])->isTxBusy &&
//               !((i2c_slave_state_t *)g_i2cStatePtr[instance])->isRxBusy;
//    }
    
    if( idle )
    {
        I2cMasterCb(instance);
    }
}
#endif

/*! *********************************************************************************
* \brief   I2C Master transfet complete ISR callback
*
* \param[in] instance     the instance of the I2C module
*
********************************************************************************** */
#if (gSerialMgrUseIIC_c)
static void I2cMasterCb(uint8_t instance)
{
    instance = Serial_GetInterfaceId(gSerialMgrIICMaster_c, instance);

    if( mSerials[instance].state )
        SerialManager_TxNotify(instance);
    else
        SerialManager_RxNotify(instance);
}
#endif

/*! *********************************************************************************
* \brief   I2C Slave transfet complete ISR callback
*
* \param[in] instance     the instance of the I2C module
* \param[in] slaveEvent   the I2C event
* \param[in] userData     pointer to user parameter
*
********************************************************************************** */
#if (gSerialMgrUseIIC_c)
static void I2cSlaveCb(uint8_t instance, i2c_slave_event_t slaveEvent, void *userData)
{
    instance = Serial_GetInterfaceId(gSerialMgrIICSlave_c, instance);

    if( slaveEvent == kI2CSlaveTxEmpty )
    {
        SerialManager_TxNotify(instance);
    }

    if( slaveEvent == kI2CSlaveRxFull )
    {
        SerialManager_RxNotify(instance);
    }
}
#endif

/*! *********************************************************************************
* \brief   I2C Master data available pin ISR
*
********************************************************************************** */
#if (gSerialMgrUseIIC_c)
static void I2cMasterDapISR(void)
{
    PORT_Type *port  = g_portBase[GPIO_EXTRACT_PORT(kGpioI2cDAP)];
    uint32_t pin     = GPIO_EXTRACT_PIN(kGpioI2cDAP);
    uint32_t i       = Serial_GetInterfaceIdFromType(gSerialMgrIICMaster_c);

    if( PORT_HAL_IsPinIntPending(port, pin) )
    {
        if( GPIO_DRV_ReadPinInput(kGpioI2cDAP) )
        {
            PORT_HAL_SetPinIntMode( port, pin, kPortIntLogicZero);
 
            if( !(mSerials[i].events & gSMGR_Rx_c) )
            {
                mSerials[i].events |= gSMGR_Rx_c;
                (void)OSA_EventSet(&mSMTaskEvent, gSMGR_Rx_c);
            }
        }
        else
        {
            PORT_HAL_SetPinIntMode( port, pin, kPortIntLogicOne);
        }

        PORT_HAL_ClearPinIntFlag(port, pin);
    }
}
#endif /* #if (gSerialMgrUseIIC_c) */

/*! *********************************************************************************
* \brief   Configures the data available pin for master SPI/IIC modules.
*          Pin is configured as input, with intrrupt on rising edge.
*
* \param[in] pinName      KSDK pin name
* \param[in] pfISR        pointer to the ISR
*
* \return                 pointer to old ISR
*                         if the old ISR is the default one, returns NULL
*
********************************************************************************** */
#if (gSerialMgrUseSPI_c) || (gSerialMgrUseIIC_c)
static void Serial_ConfigureMasterDap(uint32_t pinName, void (*pfISR)(void))
{
    gpio_input_pin_user_config_t dapPinCfg = {
        .pinName = pinName,
        .config.isPullEnable = FALSE,
        .config.pullSelect = kPortPullDown,
        .config.isPassiveFilterEnabled = FALSE,
        .config.interrupt = kPortIntLogicOne
    };

    GpioInstallIsr(pfISR, gGpioIsrPrioNormal_c, mSMGR_DapIsrPrio_c, pinName);
    GPIO_DRV_InputPinInit(&dapPinCfg);
}
#endif

/*! *********************************************************************************
* \brief   Configures the data available pin for slave SPI/IIC modules.
*          Pin is configured as output.
*
* \param[in] pinName      KSDK pin name
*
********************************************************************************** */
#if (gSerialMgrUseSPI_c) || (gSerialMgrUseIIC_c)
static void Serial_ConfigureSlaveDap(uint32_t pinName)
{
    gpio_output_pin_user_config_t dapPinCfg = {
        .pinName = pinName,
        .config.outputLogic = 0,
        .config.slewRate = kPortFastSlewRate,
#if FSL_FEATURE_PORT_HAS_OPEN_DRAIN
        .config.isOpenDrainEnabled = FALSE,
#endif
        .config.driveStrength = kPortHighDriveStrength,
    };

    GPIO_DRV_OutputPinInit(&dapPinCfg);
}
#endif

#if (gSerialMgrUseUart_c)
#if FSL_FEATURE_SOC_UART_COUNT
/*! *********************************************************************************
* \brief   Common UART ISR
*
********************************************************************************** */
void UART_IRQHandler(void)
{
    uint32_t instance = 0;
#if (FSL_FEATURE_SOC_UART_COUNT > 1)
    uint32_t uartIrqOffset = g_uartRxTxIrqId[1] - g_uartRxTxIrqId[0];
    instance = (__get_IPSR() - 16 - g_uartRxTxIrqId[0])/uartIrqOffset;
#endif
    
    UART_DRV_IRQHandler(instance);
}

/*! *********************************************************************************
* \brief   UART Rx ISR callback.
*
* \param[in] instance  UART instance
* \param[in] state     pointer to the UART state structure
*
********************************************************************************** */
void Serial_UartRxCb(uint32_t instance, void* state)
{
    uart_state_t *pState = (uart_state_t*)state;
    uint32_t i = (uint32_t)pState->rxCallbackParam;

    SerialManager_RxNotify(i);
    /* Update rxBuff because rxIn was incremented by the RxNotify function */
    pState->rxBuff = &mSerials[i].rxBuffer[mSerials[i].rxIn];
}

/*! *********************************************************************************
* \brief   UART Tx ISR callback.
*
* \param[in] instance  UART instance
* \param[in] state     pointer to the UART state structure
*
********************************************************************************** */
void Serial_UartTxCb(uint32_t instance, void* state)
{
    uart_state_t *pState = (uart_state_t*)state;
    uint32_t i = (uint32_t)pState->txCallbackParam;

    /* will get here only if txSize > 0 */
    pState->txBuff++;
    pState->txSize--;
    
    if( pState->txSize == 0 )
    {
        /* Transmit complete. Notify SMGR */
        UART_DRV_AbortSendingData(instance);
        SerialManager_TxNotify(i);
    }
}
#endif /* #if FSL_FEATURE_SOC_UART_COUNT */

#if FSL_FEATURE_SOC_LPUART_COUNT
/*! *********************************************************************************
* \brief   Common LPUART ISR.
*
********************************************************************************** */
void LPUART_IRQHandler(void)
{
    uint32_t instance = 0;
#if (FSL_FEATURE_SOC_LPUART_COUNT > 1)
    uint32_t lpuartIrqOffset = g_lpuartRxTxIrqId[1] - g_lpuartRxTxIrqId[0];
    instance = (__get_IPSR() - 16 - g_lpuartRxTxIrqId[0])/lpuartIrqOffset;
#endif
    
    LPUART_DRV_IRQHandler(instance);
}

/*! *********************************************************************************
* \brief   LPUART Rx ISR callback.
*
* \param[in] instance  LPUART instance
* \param[in] state     pointer to the LPUART state structure
*
********************************************************************************** */
void Serial_LpuartRxCb(uint32_t instance, void* state)
{
    lpuart_state_t *pState = (lpuart_state_t*)state;
    uint32_t i = (uint32_t)pState->rxCallbackParam;

    SerialManager_RxNotify(i);
    /* Update rxBuff because rxIn was incremented by the RxNotify function */
    pState->rxBuff = &mSerials[i].rxBuffer[mSerials[i].rxIn];
}

/*! *********************************************************************************
* \brief   LPUART Tx ISR callback.
*
* \param[in] instance  LPUART instance
* \param[in] state     pointer to the LPUART state structure
*
********************************************************************************** */
void Serial_LpuartTxCb(uint32_t instance, void* state)
{
    lpuart_state_t *pState = (lpuart_state_t*)state;
    uint32_t i = (uint32_t)pState->txCallbackParam;

    /* will get here only if txSize > 0 */
    pState->txBuff++;
    pState->txSize--;
    
    if( pState->txSize == 0 )
    {
        /* Transmit complete. Notify SMGR */
        LPUART_DRV_AbortSendingData(instance);
        SerialManager_TxNotify(i);
    }
}
#endif /* #if FSL_FEATURE_SOC_LPUART_COUNT */

#if FSL_FEATURE_SOC_LPSCI_COUNT
/*! *********************************************************************************
* \brief   Common LPSCI ISR.
*
********************************************************************************** */
void LPSCI_IRQHandler(void)
{
    uint32_t instance = 0;
#if (FSL_FEATURE_SOC_LPSCI_COUNT > 1)
    uint32_t lpsciIrqOffset = g_lpsciRxTxIrqId[1] - g_lpsciRxTxIrqId[0];
    instance = (__get_IPSR() - 16 - g_lpsciRxTxIrqId[0])/lpsciIrqOffset;
#endif

    LPSCI_DRV_IRQHandler(instance);
}

/*! *********************************************************************************
* \brief   LPSCI Rx ISR callback.
*
* \param[in] instance  LPSCI instance
* \param[in] state     pointer to the LPSCI state structure
*
********************************************************************************** */
void Serial_LpsciRxCb(uint32_t instance, void* state)
{
    lpsci_state_t *pState = (lpsci_state_t*)state;
    uint32_t i = (uint32_t)pState->rxCallbackParam;

    SerialManager_RxNotify(i);
    /* Update rxBuff because rxIn was incremented by the RxNotify function */
    pState->rxBuff = &mSerials[i].rxBuffer[mSerials[i].rxIn];
}

/*! *********************************************************************************
* \brief   LPSCI Tx ISR callback.
*
* \param[in] instance  LPSCI instance
* \param[in] state     pointer to the LPSCI state structure
*
********************************************************************************** */
void Serial_LpsciTxCb(uint32_t instance, void* state)
{
    lpsci_state_t *pState = (lpsci_state_t*)state;
    uint32_t i = (uint32_t)pState->txCallbackParam;

    /* will get here only if txSize > 0 */
    pState->txBuff++;
    pState->txSize--;
    
    if( pState->txSize == 0 )
    {
        /* Transmit complete. Notify SMGR */
        LPSCI_DRV_AbortSendingData(instance);
        SerialManager_TxNotify(i);
    }
}
#endif /* #if FSL_FEATURE_SOC_LPSCI_COUNT */
#endif /* #if (gSerialMgrUseUart_c) */
#endif /* #if (gSerialManagerMaxInterfaces_c) */
