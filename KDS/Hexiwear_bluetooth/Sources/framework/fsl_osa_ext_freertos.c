/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file freertos_Adapter.c
* This is the source file for the OS Abstraction layer for freertos. 
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
#include "fsl_osa_ext.h"
#include "fsl_os_abstraction.h"
#include "fsl_interrupt_manager.h"
#include <string.h>
#include "GenericList.h"


/*! *********************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
********************************************************************************** */
#define millisecToTicks(millisec) ((millisec * configTICK_RATE_HZ + 999)/1000)

#ifdef DEBUG_ASSERT
#define OS_ASSERT(condition) if(!(condition))while(1);
#else
#define OS_ASSERT(condition) (void)(condition);
#endif

#if (osNumberOfTimers || osNumberOfEvents)
#define osObjectAlloc_c 1
#else
#define osObjectAlloc_c 0
#endif


/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/
typedef struct osTimerStruct_tag
{
    uint32_t inUse;
    osaTimerFctPtr_t userCallback;
    void *userArg;
    //uint32_t timer_type;
    TimerHandle_t tmr;
}osTimerStruct_t;

typedef struct osEventStruct_tag
{
    uint32_t inUse;
    event_t event;
}osEventStruct_t;

typedef struct osObjStruct_tag
{
    uint32_t inUse;
    uint32_t osObj;
}osObjStruct_t;

typedef struct osObjectInfo_tag
{
    void* pHeap;
    uint32_t objectStructSize;
    uint32_t objNo;
} osObjectInfo_t;


/*! *********************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
********************************************************************************** */
#if osObjectAlloc_c
static void* osObjectAlloc(const osObjectInfo_t* pOsObjectInfo);
static bool_t osObjectIsAllocated(const osObjectInfo_t* pOsObjectInfo, void* pObjectStruct);
static void osObjectFree(const osObjectInfo_t* pOsObjectInfo, void* pObjectStruct);
#endif
#if osNumberOfTimers
static void TimerCallback(TimerHandle_t t);
#endif
extern void main_task(void const *argument);
void startup_task(void const *argument);


/*! *********************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
********************************************************************************** */
const uint8_t gUseRtos_c = USE_RTOS;  // USE_RTOS = 0 for BareMetal and 1 for OS


/*! *********************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
********************************************************************************** */
#if osNumberOfTimers
osTimerStruct_t osTimersHeap[osNumberOfTimers];
const osObjectInfo_t osTimerInfo = {osTimersHeap, sizeof(osTimerStruct_t),osNumberOfTimers};
#endif

#if osNumberOfEvents
osEventStruct_t osEventHeap[osNumberOfEvents];
const osObjectInfo_t osEventInfo = {osEventHeap, sizeof(osEventStruct_t),osNumberOfEvents};
#endif


/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */

/*FUNCTION**********************************************************************
 *
 * Function Name : startup_task
 * Description   : Wrapper over main_task..
 *
 *END**************************************************************************/
void startup_task(void const *argument)
{
    main_task(argument);
    while(1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_TaskGetId
 * Description   : This function is used to get current active task's handler.
 *
 *END**************************************************************************/
osaTaskId_t OSA_EXT_TaskGetId(void)
{
    return (osaTaskId_t)OSA_TaskGetHandler();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_TaskYield
 * Description   : When a task calls this function, it will give up CPU and put
 * itself to the tail of ready list.
 *
 *END**************************************************************************/
osaStatus_t OSA_EXT_TaskYield(void)
{
    return (osaStatus_t)OSA_TaskYield();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_TaskGetPriority
 * Description   : This function returns task's priority by task handler.
 *
 *END**************************************************************************/
osaTaskPriority_t OSA_EXT_TaskGetPriority(osaTaskId_t taskId)
{
    return (osaTaskPriority_t)OSA_TaskGetPriority((task_handler_t)taskId);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_TaskSetPriority
 * Description   : This function sets task's priority by task handler.
 *
 *END**************************************************************************/
osaStatus_t OSA_EXT_TaskSetPriority(osaTaskId_t taskId, osaTaskPriority_t taskPriority)
{
    return (osaStatus_t)OSA_TaskSetPriority((task_handler_t)taskId,( uint16_t)taskPriority);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_TaskCreate
 * Description   : This function is used to create a task and make it ready.
 * Param[in]     :  threadDef  - Definition of the thread.
 *                  task_param - Parameter to pass to the new thread.
 * Return Thread handle of the new thread, or NULL if failed.
  *
 *END**************************************************************************/
osaTaskId_t OSA_EXT_TaskCreate(osaThreadDef_t *thread_def,osaTaskParam_t task_param)
{
    uint32_t *threadStackPtr;
    osaTaskId_t taskId = NULL;
    uint16_t oldPriority;
    osa_status_t status;
    task_handler_t task_handler;
    
    if(thread_def->instances == 0)
    {
        return NULL;
    }

    /*Change priority to avoid context switches*/
    oldPriority = OSA_TaskGetPriority(OSA_TaskGetHandler());
    (void)OSA_TaskSetPriority(OSA_TaskGetHandler(), OSA_PRIORITY_REAL_TIME);
    threadStackPtr = thread_def->tstack;
    
    status = OSA_TaskCreate((task_t)thread_def->pthread,
                            thread_def->tname,
                            thread_def->stacksize,
                            (task_stack_t*)threadStackPtr,
                            thread_def->tpriority,
                            (task_param_t)task_param,
                            thread_def->useFloat,
                            &task_handler);

    if(kStatus_OSA_Success == status)
    {
        taskId = (osaTaskId_t)task_handler;
        thread_def->instances--;  
    }

    (void)OSA_TaskSetPriority(OSA_TaskGetHandler(), oldPriority);

    return taskId;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_TaskDestroy
 * Description   : This function destroy a task. 
 * Param[in]     :taskId - Thread handle.
 * Return osaStatus_Success if the task is destroied, otherwise return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_EXT_TaskDestroy(osaTaskId_t taskId)
{
    osa_status_t status;
    uint16_t oldPriority;
    
    /*Change priority to avoid context switches*/
    oldPriority = OSA_TaskGetPriority(OSA_TaskGetHandler());

    (void)OSA_TaskSetPriority(OSA_TaskGetHandler(), OSA_PRIORITY_REAL_TIME);
    
    status = OSA_TaskDestroy((task_handler_t)taskId);

    (void)OSA_TaskSetPriority(OSA_TaskGetHandler(), oldPriority);
    
    return (osaStatus_t)status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_TimeDelay
 * Description   : This function is used to suspend the active thread for the given number of milliseconds.
 *
 *END**************************************************************************/
void OSA_EXT_TimeDelay(uint32_t millisec)
{
    vTaskDelay(millisecToTicks(millisec));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_SemaphoreCreate
 * Description   : This function is used to create a semaphore. 
 * Return         : Semaphore handle of the new semaphore, or NULL if failed. 
  *
 *END**************************************************************************/
osaSemaphoreId_t OSA_EXT_SemaphoreCreate(uint32_t initValue)
{
#if osNumberOfSemaphores
    semaphore_t sem;

    if( kStatus_OSA_Success == OSA_SemaCreate(&sem, (uint8_t)initValue) )
    {
        return (osaSemaphoreId_t)sem;
    }
#else 
    (void)initValue;
#endif
    return NULL;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_SemaphoreDestroy
 * Description   : This function is used to destroy a semaphore.
 * Return        : osaStatus_Success if the semaphore is destroyed successfully, otherwise return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_EXT_SemaphoreDestroy(osaSemaphoreId_t semId)
{
#if osNumberOfSemaphores
    osa_status_t osa_status;
    semaphore_t sem = (semaphore_t)semId; 
    
    osa_status = OSA_SemaDestroy(&sem);
    
    return(osaStatus_t)osa_status;  
#else
    (void)semId;
    return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_SemaphoreWait
 * Description   : This function checks the semaphore's counting value, if it is
 * positive, decreases it and returns osaStatus_Success, otherwise, timeout
 * will be used for wait. The parameter timeout indicates how long should wait
 * in milliseconds. Pass osaWaitForever_c to wait indefinitely, pass 0 will
 * return osaStatus_Timeout immediately if semaphore is not positive.
 * This function returns osaStatus_Success if the semaphore is received, returns
 * osaStatus_Timeout if the semaphore is not received within the specified
 * 'timeout', returns osaStatus_Error if any errors occur during waiting.
 *
 *END**************************************************************************/
osaStatus_t OSA_EXT_SemaphoreWait(osaSemaphoreId_t semId, uint32_t millisec)
{
#if osNumberOfSemaphores
    osa_status_t osa_status;
    semaphore_t sem = (semaphore_t)semId;
    
    osa_status = OSA_SemaWait(&sem, millisec);
    return  (osaStatus_t)osa_status;
#else
    (void)semId; 
    (void)millisec;
    return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_SemaphorePost
 * Description   : This function is used to wake up one task that wating on the
 * semaphore. If no task is waiting, increase the semaphore. The function returns
 * osaStatus_Success if the semaphre is post successfully, otherwise returns
 * osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_EXT_SemaphorePost(osaSemaphoreId_t semId)
{
#if osNumberOfSemaphores
    osa_status_t osa_status;
    semaphore_t sem = (semaphore_t)semId;
    
    osa_status = OSA_SemaPost(&sem);
    return (osaStatus_t)osa_status;
#else
    (void)semId;
    return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_MutexCreate
 * Description   : This function is used to create a mutex.
 * Return        : Mutex handle of the new mutex, or NULL if failed. 
 *
 *END**************************************************************************/
osaMutexId_t OSA_EXT_MutexCreate(void)
{
#if osNumberOfMutexes  
    mutex_t mutex;

    if( OSA_MutexCreate(&mutex) == kStatus_OSA_Success )
    {
        return (osaMutexId_t)mutex;
    }
#endif  
    return NULL;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_MutexLock
 * Description   : This function checks the mutex's status, if it is unlocked,
 * lock it and returns osaStatus_Success, otherwise, wait for the mutex.
 * This function returns osaStatus_Success if the mutex is obtained, returns
 * osaStatus_Error if any errors occur during waiting. If the mutex has been
 * locked, pass 0 as timeout will return osaStatus_Timeout immediately.
 *
 *END**************************************************************************/
osaStatus_t OSA_EXT_MutexLock(osaMutexId_t mutexId, uint32_t millisec)
{
#if osNumberOfMutexes    
    osa_status_t osa_status;
    mutex_t mutex = (mutex_t)mutexId;
    
    osa_status = OSA_MutexLock(&mutex, millisec);
    return  (osaStatus_t)osa_status; 
#else
    (void)mutexId;
    (void)millisec;  
    return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_MutexUnlock
 * Description   : This function is used to unlock a mutex.
 *
 *END**************************************************************************/
osaStatus_t OSA_EXT_MutexUnlock(osaMutexId_t mutexId)
{
#if osNumberOfMutexes  
    osa_status_t osa_status;
    mutex_t mutex = (mutex_t)mutexId;
    
    osa_status = OSA_MutexUnlock(&mutex);
    return (osaStatus_t)osa_status;
#else
    (void)mutexId;
    return osaStatus_Error;  
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_MutexDestroy
 * Description   : This function is used to destroy a mutex.
 * Return        : osaStatus_Success if the lock object is destroyed successfully, otherwise return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_EXT_MutexDestroy(osaMutexId_t mutexId)
{
#if osNumberOfMutexes    
    osa_status_t osa_status;
    mutex_t mutex = (mutex_t)mutexId;
    
    osa_status = OSA_MutexDestroy(&mutex);
    return(osaStatus_t)osa_status;  
#else
    (void)mutexId;
    return osaStatus_Error;    
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_EventCreate
 * Description   : This function is used to create a event object.
 * Return        : Event handle of the new event, or NULL if failed. 
 *
 *END**************************************************************************/
osaEventId_t OSA_EXT_EventCreate(bool_t autoClear)
{
#if osNumberOfEvents  
    osaEventId_t eventId;
    osEventStruct_t* pEventStruct; 
    osa_status_t osa_status;
    OSA_EXT_InterruptDisable();
    eventId = pEventStruct = osObjectAlloc(&osEventInfo);
    OSA_EXT_InterruptEnable();
    if(eventId == NULL)
    {
        return NULL;
    }
    osa_status = OSA_EventCreate(&pEventStruct->event, autoClear? (kEventAutoClear):(kEventManualClear));
    if(osa_status != kStatus_OSA_Success)
    {
        OSA_EXT_InterruptDisable();
        osObjectFree(&osEventInfo, eventId);
        OSA_EXT_InterruptEnable();
        eventId = NULL;
    }
    return eventId;
#else
    (void)autoClear;
    return NULL;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_EventSet
 * Description   : Set one or more event flags of an event object.
 * Return        : osaStatus_Success if set successfully, osaStatus_Error if failed.
 *
 *END**************************************************************************/
osaStatus_t OSA_EXT_EventSet(osaEventId_t eventId, osaEventFlags_t flagsToSet)
{
#if osNumberOfEvents    
    osa_status_t osa_status;
    osEventStruct_t* pEventStruct; 
    if(osObjectIsAllocated(&osEventInfo, eventId) == FALSE)
    {
        return osaStatus_Error;
    }
    pEventStruct = (osEventStruct_t*)eventId;  
    osa_status = OSA_EventSet(&pEventStruct->event, (event_flags_t)flagsToSet);
    return (osaStatus_t)osa_status;
#else
    (void)eventId;
    (void)flagsToSet;  
    return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_EventClear
 * Description   : Clear one or more event flags of an event object.
 * Return        :osaStatus_Success if clear successfully, osaStatus_Error if failed.
 *
 *END**************************************************************************/
osaStatus_t OSA_EXT_EventClear(osaEventId_t eventId, osaEventFlags_t flagsToClear)
{
#if osNumberOfEvents      
    osa_status_t osa_status;
    osEventStruct_t* pEventStruct; 
    if(osObjectIsAllocated(&osEventInfo, eventId) == FALSE)
    {
        return osaStatus_Error;
    }
    pEventStruct = (osEventStruct_t*)eventId;  
    osa_status = OSA_EventClear(&pEventStruct->event, (event_flags_t)flagsToClear);
    return (osaStatus_t)osa_status;
#else
    (void)eventId;
    (void)flagsToClear;  
    return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_EventWait
 * Description   : This function checks the event's status, if it meets the wait
 * condition, return osaStatus_Success, otherwise, timeout will be used for
 * wait. The parameter timeout indicates how long should wait in milliseconds.
 * Pass osaWaitForever_c to wait indefinitely, pass 0 will return the value
 * osaStatus_Timeout immediately if wait condition is not met. The event flags
 * will be cleared if the event is auto clear mode. Flags that wakeup waiting
 * task could be obtained from the parameter setFlags.
 * This function returns osaStatus_Success if wait condition is met, returns
 * osaStatus_Timeout if wait condition is not met within the specified
 * 'timeout', returns osaStatus_Error if any errors occur during waiting.
 *
 *END**************************************************************************/
osaStatus_t OSA_EXT_EventWait(osaEventId_t eventId, osaEventFlags_t flagsToWait, bool_t waitAll, uint32_t millisec, osaEventFlags_t *pSetFlags)
{
#if osNumberOfEvents  
    osa_status_t osa_status;
    osEventStruct_t* pEventStruct; 
    if(osObjectIsAllocated(&osEventInfo, eventId) == FALSE)
    {
        return osaStatus_Error;
    }
    
    /* Clean FreeRTOS cotrol flags */
    flagsToWait = flagsToWait & 0x00FFFFFF;
    
    pEventStruct = (osEventStruct_t*)eventId;  
    osa_status = OSA_EventWait(&pEventStruct->event, (event_flags_t)flagsToWait, waitAll, millisec, (event_flags_t*)pSetFlags);
    return (osaStatus_t)osa_status;
#else
    (void)eventId;
    (void)flagsToWait;  
    (void)waitAll;  
    (void)millisec;  
    (void)pSetFlags;  
    return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_EventDestroy
 * Description   : This function is used to destroy a event object. Return
 * osaStatus_Success if the event object is destroyed successfully, otherwise
 * return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_EXT_EventDestroy(osaEventId_t eventId)
{
#if osNumberOfEvents    
    osa_status_t osa_status;
    osEventStruct_t* pEventStruct; 
    if(osObjectIsAllocated(&osEventInfo, eventId) == FALSE)
    {
        return osaStatus_Error;
    }
    pEventStruct = (osEventStruct_t*)eventId;
    osa_status = OSA_EventDestroy(&pEventStruct->event);
    if(osa_status == kStatus_OSA_Success)
    {
        OSA_EXT_InterruptDisable();
        osObjectFree(&osEventInfo, eventId);
        OSA_EXT_InterruptEnable();
    }
    return(osaStatus_t)osa_status;    
#else
    (void)eventId;
    return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_MsgQCreate
 * Description   : This function is used to create a message queue.
 * Return        : the handle to the message queue if create successfully, otherwise
 * return NULL.
 *
 *END**************************************************************************/
osaMsgQId_t OSA_EXT_MsgQCreate( uint32_t  msgNo )
{
#if osNumberOfMessageQs
    msg_queue_handler_t msg_queue_handler; 

    /* Create the message queue where each element is a pointer to the message item. */
    msg_queue_handler = OSA_MsgQCreate(NULL, msgNo, (sizeof(osaMsg_t)+3)/4);

    return (osaMsgQId_t)msg_queue_handler;
#else
    (void)msgNo;
    return NULL;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_MsgQPut
 * Description   : This function is used to put a message to a message queue.
* Return         : osaStatus_Success if the message is put successfully, otherwise return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_EXT_MsgQPut(osaMsgQId_t msgQId, void* pMessage)
{
#if osNumberOfMessageQs  
    osa_status_t osa_status;

    osa_status = OSA_MsgQPut((msg_queue_handler_t)msgQId, pMessage);

    return (osaStatus_t)osa_status;  
#else
    (void)msgQId;
    (void)pMessage;
    return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_MsgQGet
 * Description   : This function checks the queue's status, if it is not empty,
 * get message from it and return osaStatus_Success, otherwise, timeout will
 * be used for wait. The parameter timeout indicates how long should wait in
 * milliseconds. Pass osaWaitForever_c to wait indefinitely, pass 0 will return
 * osaStatus_Timeout immediately if queue is empty.
 * This function returns osaStatus_Success if message is got successfully,
 * returns osaStatus_Timeout if message queue is empty within the specified
 * 'timeout', returns osaStatus_Error if any errors occur during waiting.
 *
 *END**************************************************************************/
osaStatus_t OSA_EXT_MsgQGet(osaMsgQId_t msgQId, void *pMessage, uint32_t millisec)
{
#if osNumberOfMessageQs  
    osa_status_t osa_status;

    osa_status = OSA_MsgQGet((msg_queue_handler_t)msgQId, pMessage ,millisec);

    return (osaStatus_t)osa_status;
#else
    (void)msgQId;
    (void)pMessage;
    (void)millisec;
    return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_MsgQDestroy
 * Description   : This function is used to destroy the message queue.
 * Return        : osaStatus_Success if the message queue is destroyed successfully, otherwise return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_EXT_MsgQDestroy(osaMsgQId_t msgQId)
{
#if osNumberOfMessageQs  
    osa_status_t osa_status;

    osa_status = OSA_MsgQDestroy((msg_queue_handler_t)msgQId);

    return(osaStatus_t)osa_status;    
#else
    (void)msgQId;
    return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_InterruptEnable
 * Description   : self explanatory.
 *
 *END**************************************************************************/
void OSA_EXT_InterruptEnable(void)
{
    OSA_ExitCritical(kCriticalDisableInt) ;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_InterruptDisable
 * Description   : self explanatory.
 *
 *END**************************************************************************/
void OSA_EXT_InterruptDisable(void)
{
    OSA_EnterCritical(kCriticalDisableInt);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EXT_InstallIntHandler
 * Description   : This function is used to install interrupt handler.
 *
 *END**************************************************************************/
void * OSA_EXT_InstallIntHandler(uint32_t IRQNumber, void (*handler)(void))
{
    return (void *)OSA_InstallIntHandler((int32_t)IRQNumber,handler);
}

/*! *********************************************************************************
* \brief   Creates a timer object.
*
* \param[in] timer_def - Timer definition which includes the callback function pointer.
*            type - One-shot or periodic timer.
*            param - Parameter to pass to callback.
*
* \return Timer handle if procedure is successful or NULL if failed.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
osaTimerId_t OSA_EXT_TimerCreate (osaTimerDef_t *timer_def, osaTimer_t type, void *argument)
{
#if osNumberOfTimers  
    osaTimerId_t timer_id;
    osTimerStruct_t* pTimerStruct;
    UBaseType_t autoReload = pdFALSE;
    
    OSA_EXT_InterruptDisable();
    timer_id = pTimerStruct = osObjectAlloc(&osTimerInfo);
    OSA_EXT_InterruptEnable();
    
    if(timer_id == NULL)
    {
        return NULL; /*Alloc error*/
    }
    
    if(type == osaTimer_Periodic)
    {
        autoReload = pdTRUE;
    }
    
    pTimerStruct->userCallback = timer_def->pfCallback;
    pTimerStruct->userArg = argument;
    pTimerStruct->tmr = xTimerCreate(NULL, 1, autoReload, pTimerStruct, TimerCallback);
    
    return timer_id;
#else
    (void)timer_def;
    (void)type;
    (void)argument;  
    return NULL;
#endif  
}

/*! *********************************************************************************
* \brief   Starts a timer.
*
* \param[in] timer_id - Timer handle of the timer to start.
*            millisec - Timer period in milliseconds.
*
* \return osOK: The specified timer has been started or restarted.
*         osErrorParameter: Timer ID is incorrect.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
osaStatus_t OSA_EXT_TimerStart (osaTimerId_t timer_id, uint32_t millisec)
{
#if osNumberOfTimers  
    BaseType_t status;
    portBASE_TYPE taskToWake = pdFALSE;
    osTimerStruct_t* pTimerStruct = (osTimerStruct_t*)timer_id;
    
    if(osObjectIsAllocated(&osTimerInfo, timer_id) == FALSE)
    {
        return osaStatus_Error;
    }
    
    if (__get_IPSR())
    {
        status = xTimerChangePeriodFromISR(pTimerStruct->tmr, millisecToTicks(millisec), &taskToWake);
        if(pdPASS ==  status)
        {
            status = xTimerStartFromISR(pTimerStruct->tmr, &taskToWake);
        }
        if (pdTRUE == taskToWake)
        {
            vPortYieldFromISR();
        }
    }
    else
    {
        status = xTimerChangePeriod(pTimerStruct->tmr, millisecToTicks(millisec), portMAX_DELAY);
        if(pdPASS ==  status)
        {
            status = xTimerStart(pTimerStruct->tmr, portMAX_DELAY);
        }
    }
    
    if(pdPASS ==  status)
    {
        return osaStatus_Success;
    }
    
    return osaStatus_Error;
#else
    (void)timer_id;
    (void)millisec;
    return osaStatus_Error;
#endif  
}

/*! *********************************************************************************
* \brief   Stops a timer.
*
* \param[in] timer_id - Timer handle of the timer to stop.
*
* \return osOK: The specified timer has been stopped.
*         osErrorParameter: Timer handle is incorrect.
*         osErrorResource: The timer was not started.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
osaStatus_t OSA_EXT_TimerStop (osaTimerId_t timer_id)
{
#if osNumberOfTimers  
    BaseType_t status;
    portBASE_TYPE taskToWake = pdFALSE;
    osTimerStruct_t* pTimerStruct = (osTimerStruct_t*)timer_id;
    
    if(osObjectIsAllocated(&osTimerInfo, timer_id) == FALSE)
    {
        return osaStatus_Error;
    }
    
    if (__get_IPSR())
    {
        status = xTimerStopFromISR(pTimerStruct->tmr, &taskToWake);
        if (pdTRUE == taskToWake)
        {
            vPortYieldFromISR();
        }
    }
    else
    {
        status = xTimerStop(pTimerStruct->tmr, portMAX_DELAY);
    }
    
    if(pdPASS ==  status)
    {
        return osaStatus_Success;
    }
    return osaStatus_Error;
#else
    (void)timer_id;
    return osaStatus_Error;
#endif  
}

/*! *********************************************************************************
* \brief   Unqueues the timer from the os and deallocates it from the timers heap.
*
* \param[in] timer_id - Timer handle of the timer to destroy.
*
* \return osOK: The specified timer has been destroyd.
*         osErrorParameter: Timer handle is incorrect.
*         osErrorResource: The timer was not started.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
osaStatus_t OSA_EXT_TimerDestroy (osaTimerId_t timer_id)
{
#if osNumberOfTimers  
    BaseType_t status;
    portBASE_TYPE taskToWake = pdFALSE;
    osTimerStruct_t* pTimerStruct = (osTimerStruct_t*)timer_id;
    
    if(osObjectIsAllocated(&osTimerInfo, timer_id) == FALSE)
    {
        return osaStatus_Error;
    }
    
    if (__get_IPSR())
    {
        status = xTimerStopFromISR(pTimerStruct->tmr, &taskToWake);
        if (pdTRUE == taskToWake)
        {
            vPortYieldFromISR();
        }
    }
    else
    {
        status = xTimerStop(pTimerStruct->tmr, portMAX_DELAY);
    }
    
    if(pdPASS ==  status)
    {
        status = xTimerDelete(pTimerStruct->tmr, portMAX_DELAY);
    }
    
    if(pdPASS ==  status)
    {
        OSA_EXT_InterruptDisable();
        osObjectFree(&osTimerInfo, timer_id);
        OSA_EXT_InterruptEnable();
        return osaStatus_Success;
    }
    
    return osaStatus_Error;
#else
    (void)timer_id;
    return osaStatus_Error;
#endif  
}

/*! *********************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
********************************************************************************** */

int main (void)
{
    task_handler_t handler;
    
    OSA_Init();

    OSA_TaskCreate((task_t)startup_task, 
                   "main",
                   gMainThreadStackSize_c,
                   NULL,
                   gMainThreadPriority_c,
                   0,
                   FALSE,
                   &handler);

    OSA_Start();

    return 0;
}

/*! *********************************************************************************
* \brief     Allocates a osObjectStruct_t block in the osObjectHeap array.
* \param[in] pointer to the object info struct.
* Object can be semaphore, mutex, osTimer, message Queue, event
* \return Pointer to the allocated osObjectStruct_t, NULL if failed.
*
* \pre 
*
* \post
*
* \remarks Function is unprotected from interrupts. 
*
********************************************************************************** */
#if osObjectAlloc_c
static void* osObjectAlloc(const osObjectInfo_t* pOsObjectInfo)
{
    uint32_t i;
    uint8_t* pObj = (uint8_t*)pOsObjectInfo->pHeap;
    for( i=0 ; i < pOsObjectInfo->objNo ; i++, pObj += pOsObjectInfo->objectStructSize)
    {
        if(((osObjStruct_t*)pObj)->inUse == 0)
        {
            ((osObjStruct_t*)pObj)->inUse = 1;
            return (void*)pObj;
        }
    }
    return NULL;
}
#endif

/*! *********************************************************************************
* \brief     Verifies the object is valid and allocated in the osObjectHeap array.
* \param[in] the pointer to the object info struct.
* \param[in] the pointer to the object struct.
* Object can be semaphore, mutex, osTimer, message Queue, event
* \return TRUE if the object is valid and allocated, FALSE otherwise
*
* \pre 
*
* \post
*
* \remarks Function is unprotected from interrupts. 
*
********************************************************************************** */
#if osObjectAlloc_c
static bool_t osObjectIsAllocated(const osObjectInfo_t* pOsObjectInfo, void* pObjectStruct)
{
    uint32_t i;
    uint8_t* pObj = (uint8_t*)pOsObjectInfo->pHeap;
    for( i=0 ; i < pOsObjectInfo->objNo ; i++ , pObj += pOsObjectInfo->objectStructSize)
    {
        if(pObj == pObjectStruct)
        {
            if(((osObjStruct_t*)pObj)->inUse)
            {
                return TRUE;
            }
            break;
        }
    }
    return FALSE;
}
#endif

/*! *********************************************************************************
* \brief     Frees an osObjectStruct_t block from the osObjectHeap array.
* \param[in] pointer to the object info struct.
* \param[in] Pointer to the allocated osObjectStruct_t to free.
* Object can be semaphore, mutex, osTimer, message Queue, event
* \return none.
*
* \pre 
*
* \post
*
* \remarks Function is unprotected from interrupts. 
*
********************************************************************************** */
#if osObjectAlloc_c
static void osObjectFree(const osObjectInfo_t* pOsObjectInfo, void* pObjectStruct)
{
    uint32_t i;
    uint8_t* pObj = (uint8_t*)pOsObjectInfo->pHeap;
    for( i=0; i < pOsObjectInfo->objNo; i++, pObj += pOsObjectInfo->objectStructSize )
    {
        if(pObj == pObjectStruct)
        {
            ((osObjStruct_t*)pObj)->inUse = 0;
            break;
        }
    }
}
#endif

/*! *********************************************************************************
* \brief   Re-entrant timer callback used for all timers to catch and stop one-shot timers.
*
* \param[in] param - Timer handle.
*
* \return void.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
#if osNumberOfTimers
static void TimerCallback(TimerHandle_t t)
{
    uint32_t i;
    void *userArg;
    osaTimerFctPtr_t userFunc = NULL;
    
    for( i=0; i<osNumberOfTimers; i++)
    {
        if( osTimersHeap[i].tmr == t )
        {
            userFunc = osTimersHeap[i].userCallback;
            userArg = osTimersHeap[i].userArg;
            break;
        }
    }

    if( userFunc )
    {
        userFunc(userArg);
    }
}
#endif