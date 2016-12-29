/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file Keyboard.c
* Description: Keyboard implementation file for ARM CORTEX-M4 processor.
*              The keyboard handling logic can understand one or more keys 
*              pressed simultaneous. 
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
#include "EmbeddedTypes.h"
#include "Keyboard.h"
#include "TimersManager.h"

#include "board.h"
#include "fsl_gpio_driver.h"
#include "fsl_os_abstraction.h"
#include "Gpio_IrqAdapter.h"


/******************************************************************************
*******************************************************************************
* Private macros
*******************************************************************************
******************************************************************************/

/*
* Hardware settings
*/

#define gKeyboard_IsrPrio_c (0x80)

#if gKeyBoardSupported_d
    /* gSWITCHx_MASK_c is used for detecting which key is pressed */
    #define gSWITCH1_MASK_c                 0x01
    #define gSWITCH2_MASK_c                 0x02
    #define gSWITCH3_MASK_c                 0x04
    #define gSWITCH4_MASK_c                 0x08
#else
    #define gSWITCH1_MASK_c                 0
    #define gSWITCH2_MASK_c                 0
    #define gSWITCH3_MASK_c                 0
    #define gSWITCH4_MASK_c                 0
#endif /* gKeyBoardSupported_d */

#if gKeyBoardSupported_d
/*
 * Name: mNoKey_c
 * Description: no key macro definition
 */
#define mNoKey_c 0xff
#endif /* gKeyBoardSupported_d */

/* Configuration check */

#if (gKeyBoardSupported_d) && (gKBD_KeysCount_c > 4)
#error "Cannot support more than 4 switches"
#endif

#if (gKeyBoardSupported_d) && (gKBD_KeysCount_c == 0)
#warning "KEYBOARD module is enabled but the pushbuttons count is ZERO"
#endif

#if (gKeyBoardSupported_d) && (!gTMR_Enabled_d)
#warning "Keyboard scan cannot operate without the TIMER platform component"
#endif


/******************************************************************************
*******************************************************************************
* Private type definitions
*******************************************************************************
******************************************************************************/
#if gKeyBoardSupported_d
/* 
 * Name: KeyState_t
 * Description: enumerated data type for key states
 */
#if (gKeyEventNotificationMode_d == gKbdEventShortLongPressMode_c)

typedef enum tag_KeyState{
  mStateKeyIdle,        /* coming in for first time */
  mStateKeyDetected,    /* got a key, waiting to see if it's a long key */
  mStateKeyWaitRelease  /* got the long key, waiting for the release to go back to idle */
}KeyState_t;

#elif (gKeyEventNotificationMode_d == gKbdEventPressHoldReleaseMode_c)

typedef enum tag_KeyState {
  mStateKeyDebounce,
  mStateKeyHoldDetection,
  mStateKeyHoldGen,
} KeyState_t;

#endif /* gKeyEventNotificationMode_d */
#endif /* gKeyBoardSupported_d*/

#if gKeyBoardSupported_d
/*
 * Name:
 * Description: switches scan result definition 
 */
typedef uint32_t    switchScan_t;
#endif /* gKeyBoardSupported_d */


/******************************************************************************
*******************************************************************************
* Private prototypes
*******************************************************************************
******************************************************************************/

#if gKeyBoardSupported_d
/******************************************************************************
 * Name: KbGpioInit
 * Description: Initialize the GPIOs used by the keyboard (switches)
 * Parameter(s): -
 * Return: -
 ******************************************************************************/
static void KbGpioInit
(
    void
);

#if gTMR_Enabled_d

/******************************************************************************
 * Name: KBD_KeySwitchPortGet
 * Description: Gets the switch port
 * Parameter(s): -
 * Return: switch port value (pressed / not pressed keys)
 ******************************************************************************/
static switchScan_t KBD_KeySwitchPortGet
(
        void
);

/******************************************************************************
 * Name: KBD_KeyCheck
 * Description: Called to check if a key is still pressed
 * Parameter(s): [IN] previousPressed - previously pressed key
 * Return: TRUE if the key passed as argument is still pressed, FALSE otherwise
 ******************************************************************************/
#if ((gKeyEventNotificationMode_d == gKbdEventShortLongPressMode_c) ||  \
    (gKeyEventNotificationMode_d == gKbdEventPressHoldReleaseMode_c))
static bool_t KBD_KeyCheck
(
    switchScan_t previousPressed
);
#endif /* #if ((gKeyEventNotificationMode_d == gKbdEventShortLongPressMode_c) ||    \
    (gKeyEventNotificationMode_d == gKbdEventPressHoldReleaseMode_c)) */

/******************************************************************************
 * Name: KeyScan
 * Description: scan the keyboard switches and detects key press/hold/release 
 *              or short/long press
 * Parameter(s): [IN]timerId - key scan timer ID
 * Return: -
 ******************************************************************************/
static void KeyScan
(
    uint8_t timerId
);
#endif /* #if gTMR_Enabled_d */

static bool_t Switch_CheckIRQ
(
    uint32_t idx
);
#endif /* gKeyBoardSupported_d */

/******************************************************************************
*******************************************************************************
* Private memory declarations
*******************************************************************************
******************************************************************************/

#if gKeyBoardSupported_d

#if (gKeyEventNotificationMode_d == gKbdEventPressOnly_c)
/*
 * Name: mKeyPressed
 * Description: Keys pressed mask; each bit represents a key / electrode; 
 *             (i.e. bit0 -> KEY0, bit1 -> KEY1, etc)  
 */
static volatile uint16_t mKeyPressed;
#endif /* gKeyEventNotificationMode_d == gKbdEventPressOnly_c */

#if (gKeyEventNotificationMode_d == gKbdEventPressHoldReleaseMode_c)

#if gKeyBoardSupported_d
#if gTMR_Enabled_d
/*
 * Name: mKbdHoldDectionKeyCount
 * Description: counter used for key hold detection
 */
static uint16_t       mKbdHoldDectionKeyCount;

/*
 * Name: mKbdHoldGenKeyCount
 * Description: counter used for key hold generation
 */
static uint16_t       mKbdHoldGenKeyCount;
#endif /* #if gTMR_Enabled_d */
#endif /* gKeyBoardSupported_d */
#endif /* gKeyEventNotificationMode_d == gKbdEventPressHoldReleaseMode_c */

#if ( (gKeyEventNotificationMode_d == gKbdEventShortLongPressMode_c) || (gKeyEventNotificationMode_d == gKbdEventPressHoldReleaseMode_c) ) 

#if gKeyBoardSupported_d
#if gTMR_Enabled_d
/*
 * Name: mSwitch_SCAN
 * Description: switch scan result
 */
static uint32_t mSwitch_SCAN;

#if (gKeyEventNotificationMode_d == gKbdEventShortLongPressMode_c)
/*
 * Name: mKeyState
 * Description: key scan state variable used in the scanning state machine
 */
static uint8_t mKeyState = mStateKeyIdle;

/*
 * Name: mLongKeyCount
 * Description: self explanatory
 */
static uint8_t mLongKeyCount;
#elif (gKeyEventNotificationMode_d == gKbdEventPressHoldReleaseMode_c)
/*
 * Name: mKeyState
 * Description: key scan state variable used in the scanning state machine
 */
static uint8_t mKeyState = mStateKeyDebounce;

#endif /* gKeyEventNotificationMode_d */
#endif /* #if gTMR_Enabled_d */
#endif /* gKeyBoardSupported_d */
#endif /* (gKeyEventNotificationMode_d == gKbdEventShortLongPressMode_c) || (gKeyEventNotificationMode_d == gKbdEventPressHoldReleaseMode_c) */

/*
 * Name: pressedKey
 * Description: pressed key number
 */
#if gKeyBoardSupported_d
#if gTMR_Enabled_d
static uint8_t pressedKey;
#endif
#endif

/*
 * Name: mKeyScanTimerID
 * Description: timer ID used for key scanning
 */
tmrTimerID_t mKeyScanTimerID = gTmrInvalidTimerID_c;

/*
 * Name: mpfKeyFunction
 * Description: pointer to the application callback function
 */
static KBDFunction_t mpfKeyFunction = NULL;

static const uint32_t mSwNames[] = {
#if gKBD_KeysCount_c > 0
    kGpioSW1,
#endif
#if gKBD_KeysCount_c > 1
    kGpioSW2,
#endif
#if gKBD_KeysCount_c > 2
    kGpioSW3,
#endif
#if gKBD_KeysCount_c > 3
    kGpioSW4,
#endif
};

#endif /* gKeyBoardSupported_d */

/******************************************************************************
*******************************************************************************
* Private functions
*******************************************************************************
******************************************************************************/
#if gKeyBoardSupported_d
/******************************************************************************
 * Name: KbGpioInit
 * Description: Initialize the GPIOs used by the keyboard (switches)
 * Parameter(s): -
 * Return: -
 ******************************************************************************/
static void KbGpioInit
(
    void
)
{
    uint32_t i;
 
    for( i=0; i<gKBD_KeysCount_c; i++ )
    {
        /* Initialize KBD pins. Function also sets pin MUX as GPIO */
        GPIO_DRV_InputPinInit(&switchPins[i]);
    }
}

void KBD_Deinit
(
      void
)
{
    uint8_t i;
    TMR_FreeTimer(mKeyScanTimerID);

    for( i=0; i<gKBD_KeysCount_c; i++ )
    {
        GpioUninstallIsr(switchPins[i].pinName);
    }
}

#if gTMR_Enabled_d

/******************************************************************************
 * Name: KBD_KeySwitchPortGet
 * Description: Gets the switch port
 * Parameter(s): -
 * Return: switch port value (pressed / not pressed keys)
 ******************************************************************************/
static switchScan_t KBD_KeySwitchPortGet
(
void
)
{
    uint32_t portScan, i;
    pressedKey = mNoKey_c;

    for( i=0, portScan=0; i<gKBD_KeysCount_c; i++ )
    {
        if( GPIO_DRV_ReadPinInput(mSwNames[i]) == 0 )
        {
            portScan |= (1 << i);
        }
    }

#if gSWITCH1_MASK_c 
    if(portScan & gSWITCH1_MASK_c)
        pressedKey = 0;
#if (gSWITCH2_MASK_c | gSWITCH3_MASK_c | gSWITCH4_MASK_c)
    else
#endif
#endif 
        
#if gSWITCH2_MASK_c
        if(portScan & gSWITCH2_MASK_c)
            pressedKey = 1;
#if(gSWITCH3_MASK_c | gSWITCH4_MASK_c)
        else
#endif 
#endif 
            
#if gSWITCH3_MASK_c
            if(portScan & gSWITCH3_MASK_c)
                pressedKey = 2;
#if gSWITCH4_MASK_c
            else
#endif
#endif 
                
#if gSWITCH4_MASK_c 
                if(portScan & gSWITCH4_MASK_c)
                    pressedKey = 3;
#endif
            
            return portScan;
}

/******************************************************************************
 * Name: KBD_KeyCheck
 * Description: Called to check if a key is still pressed
 * Parameter(s): [IN] previousPressed - previously pressed key
 * Return: TRUE if the key passed as argument is still pressed, FALSE otherwise
 ******************************************************************************/
#if ((gKeyEventNotificationMode_d == gKbdEventShortLongPressMode_c) ||  \
    (gKeyEventNotificationMode_d == gKbdEventPressHoldReleaseMode_c))
static bool_t KBD_KeyCheck
(
    switchScan_t previousPressed
)
{
    bool_t pressed = FALSE;
    uint32_t portScan, i;
    uint8_t key = mNoKey_c;

    (void)previousPressed; /* avoid compiler warnings */

    for( i=0, portScan=0; i<gKBD_KeysCount_c; i++ )
    {
        if( GPIO_DRV_ReadPinInput(mSwNames[i]) == 0 )
        {
            portScan |= (1 << i);
        }
    }
    
#if gSWITCH1_MASK_c
    if(portScan & gSWITCH1_MASK_c)
        key = 0;
#if(gSWITCH2_MASK_c | gSWITCH3_MASK_c | gSWITCH4_MASK_c) 
    else 
#endif
#endif 

#if gSWITCH2_MASK_c
        if(portScan & gSWITCH2_MASK_c)
            key = 1;
#if(gSWITCH3_MASK_c | gSWITCH4_MASK_c)
        else
#endif
#endif 

#if gSWITCH3_MASK_c
            if(portScan & gSWITCH3_MASK_c)
                key = 2;
#if gSWITCH4_MASK_c
            else
#endif
#endif  

#if gSWITCH4_MASK_c
                if(portScan & gSWITCH4_MASK_c)
                    key = 3;
#endif 
    /* Check if the switch is still pressed */
    if(pressedKey == key)
    {
        pressed = TRUE;
    }

    return pressed;
}
#endif /*#if ((gKeyEventNotificationMode_d == gKbdEventShortLongPressMode_c) || \
    (gKeyEventNotificationMode_d == gKbdEventPressHoldReleaseMode_c))*/


/******************************************************************************
 * Name: KeyScan
 * Description: scan the keyboard switches and detects key press/hold/release 
 *              or short/long press
 * Parameter(s): [IN]timerId - key scan timer ID
 * Return: -
 ******************************************************************************/
#if (gKeyEventNotificationMode_d == gKbdEventPressOnly_c)
static void KeyScan
(
    uint8_t timerId
)
{   
    if(KBD_KeySwitchPortGet())
    {
        TMR_StopTimer(timerId);

        mpfKeyFunction(1<<pressedKey); /* bits 0..3 are for keyboard */
    }   
}

#elif (gKeyEventNotificationMode_d == gKbdEventShortLongPressMode_c)
static void KeyScan
(
        uint8_t timerId
)
{
    uint8_t keyBase;
    uint32_t portScan, i;

    switch(mKeyState) 
    {

    /* got a fresh key */
    case mStateKeyIdle:      
        mSwitch_SCAN = KBD_KeySwitchPortGet();
        if(mSwitch_SCAN != 0)
        {        
            mKeyState = mStateKeyDetected;
            mLongKeyCount = 0;
        }
        else
        {
            TMR_StopTimer(timerId); 
        }
        break;

        /* a key was detected. Has it been released or still being pressed? */
    case mStateKeyDetected:
        keyBase = 0;  /* assume no key */

        /* Check to see if the key is still pressed. Ignore other pressed keys */
        if( KBD_KeyCheck(mSwitch_SCAN) ) 
        {
            mLongKeyCount++;

            if(mLongKeyCount >= gKbdLongKeyIterations_c) 
            {
                keyBase = gKBD_EventLongPB1_c;
            }
        }
        else 
        {     
            /* short key press */
            keyBase = gKBD_EventPB1_c;
        }

        if(keyBase) 
        {       
            /* if a key was pressed, send it */
            if(pressedKey != mNoKey_c)
            {
                mpfKeyFunction(keyBase + pressedKey);
            }

            /* whether we sent a key or not, wait to go back to keyboard  */
            mKeyState = mStateKeyWaitRelease;
        }
        break;

        /* got the long key, waiting for the release to go back to idle */
    case mStateKeyWaitRelease:      
        /* wait for the release before going back to idle */  
        for( i=0, portScan=0; i<gKBD_KeysCount_c; i++ )
        {
            if( GPIO_DRV_ReadPinInput(mSwNames[i]) == 0 )
            {
                portScan |= (1 << i);
            }
        }

        if((mSwitch_SCAN == 0) || (portScan == 0)) 
        {
            mKeyState = mStateKeyIdle;
            TMR_StopTimer(timerId);                
        }
        break;        
    default:
        break;
    }
}

#elif (gKeyEventNotificationMode_d == gKbdEventPressHoldReleaseMode_c)
static void KeyScan
(
        uint8_t timerId
)
{
    uint32_t portScan, i;
  
    switch(mKeyState) 
    {    
    case mStateKeyDebounce:
        mSwitch_SCAN = KBD_KeySwitchPortGet();
        if(mSwitch_SCAN != 0) 
        {
            mKeyState =  mStateKeyHoldDetection;
            mKbdHoldDectionKeyCount = 0; 
            //Generate press event indication     
            if(pressedKey != mNoKey_c) 
            {
                mpfKeyFunction(gKBD_EventPressPB1_c + pressedKey);           
            }
        } 
        else 
        {
            TMR_StopTimer(timerId);
        }      
        break;
    case mStateKeyHoldDetection:
        if( KBD_KeyCheck(mSwitch_SCAN) ) 
        {
            mKbdHoldDectionKeyCount++;        
            if (mKbdHoldDectionKeyCount >= gKbdFirstHoldDetectIterations_c) 
            {
                //first hold event detected - generate hold event
                if(pressedKey != mNoKey_c) 
                {
                    mpfKeyFunction(gKBD_EventHoldPB1_c + pressedKey);                       
                }                    
                mKbdHoldGenKeyCount = 0;
                mKeyState = mStateKeyHoldGen;
            }
        } 
        else 
        {
            for( i=0, portScan=0; i<gKBD_KeysCount_c; i++ )
            {
                if( GPIO_DRV_ReadPinInput(mSwNames[i]) == 0 )
                {
                    portScan |= (1 << i);
                }
            }

            if((mSwitch_SCAN == 0) || (portScan == 0)) 
            {
                if(pressedKey != mNoKey_c) 
                {
                    mpfKeyFunction(gKBD_EventReleasePB1_c + pressedKey);                       
                    mKeyState = mStateKeyDebounce;
                    TMR_StopTimer(timerId);            
                }                                       
            }                
        }
        break;
    case mStateKeyHoldGen:
        if( KBD_KeyCheck(mSwitch_SCAN) ) 
        {
            mKbdHoldGenKeyCount++;
            if(mKbdHoldGenKeyCount >= gKbdHoldDetectIterations_c) 
            {
                mKbdHoldGenKeyCount = 0;
                if(pressedKey != mNoKey_c) 
                {
                    mpfKeyFunction(gKBD_EventHoldPB1_c + pressedKey);
                }           
            }
        } 
        else 
        {
            for( i=0, portScan=0; i<gKBD_KeysCount_c; i++ )
            {
                if( GPIO_DRV_ReadPinInput(mSwNames[i]) == 0 )
                {
                    portScan |= (1 << i);
                }
            }

            if((mSwitch_SCAN == 0) || (portScan == 0)) 
            {
                if(pressedKey != mNoKey_c) 
                {
                    mpfKeyFunction(gKBD_EventReleasePB1_c + pressedKey);

                    mKeyState = mStateKeyDebounce;
                    TMR_StopTimer(timerId);            
                }                                       
            }                     
        }
        break;
    default:
        break;
    }
}
#endif /* gKeyEventNotificationMode_d */
#endif /* gTMR_Enabled_d */

static bool_t Switch_CheckIRQ(uint32_t idx)
{
    PORT_Type * baseAddr = g_portBase[GPIO_EXTRACT_PORT(mSwNames[idx])];
    uint32_t pin = GPIO_EXTRACT_PIN(mSwNames[idx]);

    if( PORT_HAL_GetPortIntFlag(baseAddr) )
    {
        if( PORT_HAL_IsPinIntPending(baseAddr, pin) )
        {
            /* set the local variable to mark that the interrupt is caused by one of the keyboard switches */
            PORT_HAL_ClearPinIntFlag(baseAddr, pin);
            return TRUE;
        }
    }

    return FALSE;
}
#endif /* gKeyBoardSupported_d */

/******************************************************************************
*******************************************************************************
* Public functions
*******************************************************************************/

#if gKeyBoardSupported_d

/******************************************************************************
 * Name: KBD_Init
 * Description: Initializes the keyboard module internal variables 
 * Parameter(s): [IN] pfCallBackAdr - pointer to application callback function
 * Return: -
 * Notes: It the TIMER platform component is enabled, TMR_Init() function MUST
 *        be called before KBD_Init() function
 ******************************************************************************/
void KBD_Init( KBDFunction_t pfCallBackAdr )
{
    uint32_t i;

    /* if no valid pointer provided, return */
    if(NULL == pfCallBackAdr) 
        return;

    /* store the pointer to callback function provided by the application */
    mpfKeyFunction = pfCallBackAdr;

#if gKeyBoardSupported_d        
#if gTMR_Enabled_d
    /* timer is used to determine short or long key press */
    mKeyScanTimerID = TMR_AllocateTimer();
#endif /* #if gTMR_Enabled_d */    

    /* initialize all the GPIO pins for keyboard */
    KbGpioInit();

    for( i=0; i<gKBD_KeysCount_c; i++ )
    {
        GpioInstallIsr(Switch_Press_ISR, gGpioIsrPrioLow_c, gKeyboard_IsrPrio_c, mSwNames[i]);
    }

#endif /* gKeyBoardSupported_d */                
}

/******************************************************************************
 * Name: KBD_IsWakeUpSource
 * Description: 
 * Parameter(s): -
 * Return: -
 ******************************************************************************/
#if gKeyBoardSupported_d
bool_t KBD_IsWakeUpSource
(
  void
)
{
    uint32_t i;

    for( i=0; i<gKBD_KeysCount_c; i++ )
    {
        if( PORT_HAL_IsPinIntPending(g_portBase[GPIO_EXTRACT_PORT(mSwNames[i])], GPIO_EXTRACT_PIN(mSwNames[i])) )
        {
            return TRUE;
        }
    }

    return FALSE;
}
#endif /* gKeyBoardSupported_d */

/******************************************************************************
 * Name: KBD_SwitchPressOnWakeUp
 * Description: 
 * Parameter(s): -
 * Return: -
 ******************************************************************************/
#if gKeyBoardSupported_d
void KBD_SwitchPressedOnWakeUp
(
  void
)
{
#if gTMR_Enabled_d
        TMR_StartIntervalTimer(mKeyScanTimerID, gKeyScanInterval_c, (pfTmrCallBack_t)KeyScan, (void*)mKeyScanTimerID);       
#endif
}
#endif /* gKeyBoardSupported_d */

/******************************************************************************
 * Name: Switch_Press_ISR
 * Description: Keyboard (switches) interrupt handler
 * Parameter(s): [IN] mask - mask corresponding to pin
 * Return: -
 ******************************************************************************/
#if gKeyBoardSupported_d
#if defined(__IAR_SYSTEMS_ICC__)
#pragma location = ".isr_handler"
#endif
void Switch_Press_ISR
(
void
)
{
    uint32_t i = 0;
    uint8_t kbi_irq = 0;

    for(i=0; i<gKBD_KeysCount_c; i++)
    {
        if( Switch_CheckIRQ(i) )
        {
            kbi_irq++;
        }
    }

    if(kbi_irq)
    {
#if gTMR_Enabled_d
        TMR_StartIntervalTimer(mKeyScanTimerID, gKeyScanInterval_c, (pfTmrCallBack_t)KeyScan, (void*)mKeyScanTimerID);       
#endif
    }
}
#endif /* gKeyBoardSupported_d */
#endif /* #if gKeyBoardSupported_d */
