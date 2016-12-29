/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file LED.h
* LED export interface file for ARM CORTEX-M4 processor
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

#ifndef _LED_INTERFACE_H_
#define _LED_INTERFACE_H_

#include "EmbeddedTypes.h"


#ifndef gLEDSupported_d
#define gLEDSupported_d                TRUE
#endif

/*
* Name: gLEDsOnTargetBoardDefault_c
* Description: Default value for LEDs count located on a
*              generic target board
*/
#ifndef gLEDsOnTargetBoardDefault_c
#define gLEDsOnTargetBoardDefault_c    4
#endif

/*
* Name: gLEDsOnBoardCnt
* Description: Specifies the number of physical LEDs on the target board
*/
#ifndef gLEDsOnTargetBoardCnt_c
#define gLEDsOnTargetBoardCnt_c        gLEDsOnTargetBoardDefault_c
#endif

/*
* Name: gLEDBlipEnabled_d
* Description: Specifies if blip (blink) is supported by the LED module
*/
#ifndef gLEDBlipEnabled_d
#define gLEDBlipEnabled_d              TRUE
#endif

/*
* Name: mLEDInterval_c
* Description: Interval time (milliseconds) for toggle LED which is used
*              for flashing LED
*/

#ifndef mLEDInterval_c
#define mLEDInterval_c                 100
#endif

/*
* LEDs mapping
*/
#define LED1                     0x01
#define LED2                     0x02
#define LED3                     0x04
#define LED4                     0x08
#define LED_ALL                  0x0F
#define LED1_FLASH               0x10
#define LED2_FLASH               0x20
#define LED3_FLASH               0x40
#define LED4_FLASH               0x80
#define LED_FLASH_ALL            0xF0  /* flash all LEDs */

#if (gLEDSupported_d)

#define Led1Off()                LED_Operate(LED1, gLedOff_c)
#define Led2Off()                LED_Operate(LED2, gLedOff_c)
#define Led3Off()                LED_Operate(LED3, gLedOff_c)
#define Led4Off()                LED_Operate(LED4, gLedOff_c)

#define Led1On()                 LED_Operate(LED1, gLedOn_c)
#define Led2On()                 LED_Operate(LED2, gLedOn_c)
#define Led3On()                 LED_Operate(LED3, gLedOn_c)
#define Led4On()                 LED_Operate(LED4, gLedOn_c)

#define Led1Toggle()             LED_Operate(LED1, gLedToggle_c)
#define Led2Toggle()             LED_Operate(LED2, gLedToggle_c)
#define Led3Toggle()             LED_Operate(LED3, gLedToggle_c)
#define Led4Toggle()             LED_Operate(LED4, gLedToggle_c)

/*
*  Turn OFF LEDs macro
*/
#define TurnOffLeds()            LED_TurnOffAllLeds()

/*
* Turn ON LEDs macro
*/
#define TurnOnLeds()             LED_TurnOnAllLeds()

/*
* Start Serial flashing macro
*/
#define SerialFlashing()         LED_StartSerialFlash()

/* Flashing LED macros */
#define Led1Flashing()           LED_StartFlash(LED1)
#define Led2Flashing()           LED_StartFlash(LED2)
#define Led3Flashing()           LED_StartFlash(LED3)
#define Led4Flashing()           LED_StartFlash(LED4)
#define StopLed1Flashing()       LED_StopFlash(LED1)
#define StopLed2Flashing()       LED_StopFlash(LED2)
#define StopLed3Flashing()       LED_StopFlash(LED3)
#define StopLed4Flashing()       LED_StopFlash(LED4)

#endif /* gLEDSupported_d == TRUE */

/******************************************************************************
*******************************************************************************
* Public type definitions
*******************************************************************************
******************************************************************************/

/*
* Name: LED_t
* Description: LED type definition
*/
typedef uint8_t LED_t;

/*
* Name: LED_OpMode_t
* Description: enumerated data type for all possible LED operation modes
*/
typedef enum LED_OpMode_tag{
    gLedFlashing_c,       /* flash at a fixed rate */
    gLedStopFlashing_c,   /* same as gLedOff_c */
    gLedBlip_c,           /* just like flashing, but blinks only once */
    gLedOn_c,             /* on solid */
    gLedOff_c,            /* off solid */
    gLedToggle_c          /* toggle state */
} LED_OpMode_t;

/*
* Name: LedState_t
* Description: possible LED states for LED_SetLed()
* Note: all LEDs can operate independently
*/
typedef uint8_t LedState_t;

/******************************************************************************
*******************************************************************************
* Public prototypes
*******************************************************************************
******************************************************************************/

#if (gLEDSupported_d)

/******************************************************************************
* Name: LED_Init
* Description: Initialize the LED module
* Parameters: -
* Return: -
******************************************************************************/
extern void LED_Init
(
void
);

/******************************************************************************
* Name: Un-initialize the LED module
* Description: turns off all the LEDs and disables clock gating for LED port
* Parameters: -
* Return: -
******************************************************************************/
extern void LED_UnInit
(
void
);

/******************************************************************************
* Name: LED_Operate
* Description: Basic LED operation: ON, OFF, TOGGLE.
* Param(s): -
* Return: -
******************************************************************************/
extern void LED_Operate
(
LED_t led,
LED_OpMode_t operation
);

/******************************************************************************
* Name: LED_TurnOnLed
* Description: Turns ON the specified LED(s)
* Parameters: LEDNr - LED number(s) to be turned ON
* Return:
******************************************************************************/
extern void LED_TurnOnLed
(
LED_t LEDNr
);

/******************************************************************************
* Name: LED_TurnOffLed
* Description: Turns OFF the specified LED(s)
* Parameters: LEDNr - LED number(s) to be turned OFF
* Return:
******************************************************************************/
extern void LED_TurnOffLed
(
LED_t LEDNr
);

/******************************************************************************
* Name: LED_ToggleLed
* Description: Toggles the specified LED(s)
* Parameters:  LEDNr - LED number(s) to be toggled
* Return: -
******************************************************************************/
extern void LED_ToggleLed
(
LED_t LEDNr
);

/******************************************************************************
* Name: LED_TurnOffAllLeds
* Description: Turns OFF all LEDs
* Parameters: -
* Return: -
******************************************************************************/
extern void LED_TurnOffAllLeds
(
void
);

/******************************************************************************
* Name: LED_TurnOnAllLeds
* Description: Turns ON all LEDs
* Parameters: -
* Return: -
******************************************************************************/
extern void LED_TurnOnAllLeds
(
void
);

/******************************************************************************
* Name: LED_StopFlashingAllLeds
* Description: Stops flashing and turns OFF all LEDs
* Parameters: -
* Return: -
******************************************************************************/
extern void LED_StopFlashingAllLeds
(
void
);

/******************************************************************************
* LED_StartFlash
*
*
*******************************************************************************/
/******************************************************************************
* Name: LED_StartFlash
* Description: Starts flashing one or more LEDs
* Parameters: [IN] LED_t LEDNr - LED Number (may be an OR of the list)
* Return: -
******************************************************************************/
void LED_StartFlash
(
LED_t LEDNr
);

#if gLEDBlipEnabled_d
/******************************************************************************
* Name: LED_StartBlip
* Description: Set up for blinking one or more LEDs once
* Parameters: [IN] LED_t LEDNr - LED Number (may be an OR of the list)
* Return: -
******************************************************************************/
extern void LED_StartBlip
(
LED_t LEDNr
);
#endif

/******************************************************************************
* Name: LED_StopFlash
* Description: Stop an LED from flashing.
* Parameters: [IN] LED_t LEDNr - LED Number (may be an OR of the list)
* Return: -
******************************************************************************/
extern void LED_StopFlash
(
LED_t LEDNr
);

/******************************************************************************
* Name: LED_StartSerialFlash
* Description: starts serial flashing LEDs
* Parameters: -
* Return: -
******************************************************************************/
extern void LED_StartSerialFlash
(
uint8_t LEDStartPosition
);

/******************************************************************************
* Name: LED_SetHex
* Description: Sets a specified hex value on the LEDs
* Parameters: [IN] hexValue - the value to be set on LEDs
* Return: -
******************************************************************************/
extern void LED_SetHex
(
uint8_t hexValue
);

/******************************************************************************
* Name: LED_SetLed
* Description: This function can set the specified LED(s) in one of the
*              following states: On, Off, Toggle, Flashing or StopFlash
* Parameters: [IN] LEDNr - LED(s) to
*                [IN] LedState_t state - one of the possible states listed above
* Return:
******************************************************************************/
extern void LED_SetLed
(
LED_t LEDNr,
LedState_t state
);

#else /* LEDs not supported */

/* define empty prototypes */
#define LED_Init()
#define LED_TurnOffLed(LEDNr)
#define LED_TurnOnLed(LEDNr)
#define LED_ToggleLed(LEDNr)
#define LED_StartFlash(LEDNr)
#define LED_StopFlash(LEDNr)
#define LED_StartSerialFlash(LEDStartPosition)
#define LED_TurnOffAllLeds()
#define LED_TurnOnAllLeds()
#define LED_StopFlashingAllLeds()
#define LED_SetLed(LEDNr,state)
#define LED_SetHex(hexValue)

#define Led1On()
#define Led1Off()
#define Led1Toggle()

#define Led2On()
#define Led2Off()
#define Led2Toggle()

#define Led3On()
#define Led3Off()
#define Led3Toggle()

#define Led4On()
#define Led4Off()
#define Led4Toggle()

#define Led1Flashing()
#define Led2Flashing()
#define Led3Flashing()
#define Led4Flashing()
#define StopLed1Flashing()
#define StopLed2Flashing()
#define StopLed3Flashing()
#define StopLed4Flashing()
#define TurnOffLeds()
#define TurnOnLeds()
#define SerialFlashing()

#endif /* gLEDSupported_d */

#endif /* _LED_INTERFACE_H_ */

