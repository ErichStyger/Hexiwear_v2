/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : LEDpin4.c
**     Project     : Hexiwear_PEx_2_SDKv2
**     Processor   : MK64FN1M0VDC12
**     Component   : SDK_BitIO
**     Version     : Component 01.017, Driver 01.00, CPU db: 3.00.000
**     Repository  : Legacy User Components
**     Compiler    : GNU C Compiler
**     Date/Time   : 2016-11-18, 19:44, # CodeGen: 22
**     Abstract    :
**
**     Settings    :
**          Component name                                 : LEDpin4
**          SDK                                            : KSDK1
**          GPIO Name                                      : GPIOA
**          PORT Name                                      : PORTA
**          Pin Number                                     : 12
**          Pin Symbol                                     : LED_RED1
**          Do Pin Muxing                                  : no
**     Contents    :
**         GetDir    - bool LEDpin4_GetDir(void);
**         SetDir    - void LEDpin4_SetDir(bool Dir);
**         SetInput  - void LEDpin4_SetInput(void);
**         SetOutput - void LEDpin4_SetOutput(void);
**         GetVal    - bool LEDpin4_GetVal(void);
**         PutVal    - void LEDpin4_PutVal(bool Val);
**         ClrVal    - void LEDpin4_ClrVal(void);
**         SetVal    - void LEDpin4_SetVal(void);
**         NegVal    - void LEDpin4_NegVal(void);
**         Init      - void LEDpin4_Init(void);
**         Deinit    - void LEDpin4_Deinit(void);
**
**     (c) Copyright Erich Styger, 2016
**     http      : www.mcuoneclipse.com
**     This is a free software and is opened for education,  research  and commercial developments under license policy of following terms:
**     This is a free software and there is NO WARRANTY.
**     No restriction on use. You can use, modify and redistribute it for personal, non-profit or commercial product UNDER YOUR RESPONSIBILITY.
**     Redistributions of source code must retain the above copyright notice.
** ###################################################################*/
/*!
** @file LEDpin4.c
** @version 01.00
** @brief
**
*/         
/*!
**  @addtogroup LEDpin4_module LEDpin4 module documentation
**  @{
*/         

/* MODULE LEDpin4. */

#include "LEDpin4.h"
#if KSDK1_SDK_VERSION_USED == KSDK1_SDK_VERSION_2_0
  #include "fsl_port.h" /* include SDK header file for port muxing */
  #include "fsl_gpio.h" /* include SDK header file for GPIO */
#elif KSDK1_SDK_VERSION_USED == KSDK1_SDK_VERSION_1_3
  #include "fsl_gpio_driver.h" /* include SDK header file for GPIO */
#elif KSDK1_SDK_VERSION_USED == KSDK1_SDK_VERSION_NONE
  #error "This component only works with the Kinetis SDK!"
#endif

#if KSDK1_SDK_VERSION_USED == KSDK1_SDK_VERSION_2_0
  #define LEDpin4_PORTName   PORTA /* name of PORT, is pointer to PORT_Type */
  #define LEDpin4_GPIOName   GPIOA /* name of GPIO, is pointer to GPIO_Type */
  #define LEDpin4_PinNumber  12u   /* number of pin, type unsigned integer */

  static const gpio_pin_config_t LEDpin4_configOutput = {
    kGPIO_DigitalOutput,  /* use as output pin */
    1,  /* initial value */
  };

  static const gpio_pin_config_t LEDpin4_configInput = {
    kGPIO_DigitalInput,  /* use as input pin */
    0,  /* initial value */
  };
#elif KSDK1_SDK_VERSION_USED == KSDK1_SDK_VERSION_1_3
  const gpio_output_pin_user_config_t LEDpin4_OutputConfig[] = {
    {
      .pinName = LED_RED1,
      .config.outputLogic = 0,
    #if FSL_FEATURE_PORT_HAS_SLEW_RATE
      .config.slewRate = kPortSlowSlewRate,
    #endif
    #if FSL_FEATURE_PORT_HAS_OPEN_DRAIN
      .config.isOpenDrainEnabled = true,
    #endif
    #if FSL_FEATURE_PORT_HAS_DRIVE_STRENGTH
      .config.driveStrength = kPortLowDriveStrength,
    #endif
    },
    {
      .pinName = GPIO_PINS_OUT_OF_RANGE,
    }
  };

  const gpio_input_pin_user_config_t LEDpin4_InputConfig[] = {
    {
      .pinName = LED_RED1,
    #if FSL_FEATURE_PORT_HAS_PULL_ENABLE
      .config.isPullEnable = true,
    #endif
    #if FSL_FEATURE_PORT_HAS_PULL_SELECTION
      .config.pullSelect = kPortPullDown,
    #endif
    #if FSL_FEATURE_PORT_HAS_PASSIVE_FILTER
      .config.isPassiveFilterEnabled = true,
    #endif
    #if FSL_FEATURE_PORT_HAS_DIGITAL_FILTER
      .config.isDigitalFilterEnabled = true,
    #endif
    #if FSL_FEATURE_GPIO_HAS_INTERRUPT_VECTOR
      .config.interrupt = kPortIntDisabled
    #endif
    },
    {
      .pinName = GPIO_PINS_OUT_OF_RANGE,
    }
  };

#endif

static bool LEDpin4_isOutput = false;
/*
** ===================================================================
**     Method      :  LEDpin4_ClrVal (component SDK_BitIO)
**     Description :
**         Clears the pin value (sets it to a low level)
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void LEDpin4_ClrVal(void)
{
#if KSDK1_SDK_VERSION_USED == KSDK1_SDK_VERSION_2_0
  GPIO_ClearPinsOutput(LEDpin4_GPIOName, 1<<LEDpin4_PinNumber);
#elif KSDK1_SDK_VERSION_USED == KSDK1_SDK_VERSION_1_3
  GPIO_DRV_ClearPinOutput(LED_RED1);
#endif
}

/*
** ===================================================================
**     Method      :  LEDpin4_SetVal (component SDK_BitIO)
**     Description :
**         Sets the pin value to a high value.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void LEDpin4_SetVal(void)
{
#if KSDK1_SDK_VERSION_USED == KSDK1_SDK_VERSION_2_0
  GPIO_SetPinsOutput(LEDpin4_GPIOName, 1<<LEDpin4_PinNumber);
#elif KSDK1_SDK_VERSION_USED == KSDK1_SDK_VERSION_1_3
  GPIO_DRV_SetPinOutput(LED_RED1);
#endif
}

/*
** ===================================================================
**     Method      :  LEDpin4_NegVal (component SDK_BitIO)
**     Description :
**         Toggles/negates the pin value
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void LEDpin4_NegVal(void)
{
#if KSDK1_SDK_VERSION_USED == KSDK1_SDK_VERSION_2_0
  GPIO_TogglePinsOutput(LEDpin4_GPIOName, 1<<LEDpin4_PinNumber);
#elif KSDK1_SDK_VERSION_USED == KSDK1_SDK_VERSION_1_3
  GPIO_DRV_TogglePinOutput(LED_RED1);
#endif
}

/*
** ===================================================================
**     Method      :  LEDpin4_GetVal (component SDK_BitIO)
**     Description :
**         Returns the pin value
**     Parameters  : None
**     Returns     :
**         ---             - Returns the value of the pin:
**                           FALSE/logical level '0' or TRUE/logical
**                           level '1'
** ===================================================================
*/
bool LEDpin4_GetVal(void)
{
#if KSDK1_SDK_VERSION_USED == KSDK1_SDK_VERSION_2_0
  return GPIO_ReadPinInput(LEDpin4_GPIOName, LEDpin4_PinNumber)!=0;
#elif KSDK1_SDK_VERSION_USED == KSDK1_SDK_VERSION_1_3
  return GPIO_DRV_ReadPinInput(LED_RED1)!=0;
#else
  return FALSE;
#endif
}

/*
** ===================================================================
**     Method      :  LEDpin4_GetDir (component SDK_BitIO)
**     Description :
**         Return the direction of the pin (input/output)
**     Parameters  : None
**     Returns     :
**         ---             - FALSE if port is input, TRUE if port is
**                           output
** ===================================================================
*/
bool LEDpin4_GetDir(void)
{
  return LEDpin4_isOutput;
}

/*
** ===================================================================
**     Method      :  LEDpin4_SetDir (component SDK_BitIO)
**     Description :
**         Sets the direction of the pin (input or output)
**     Parameters  :
**         NAME            - DESCRIPTION
**         Dir             - FALSE: input, TRUE: output
**     Returns     : Nothing
** ===================================================================
*/
void LEDpin4_SetDir(bool Dir)
{
  if (Dir) {
    LEDpin4_SetOutput();
  } else {
    LEDpin4_SetInput();
  }
}

/*
** ===================================================================
**     Method      :  LEDpin4_SetInput (component SDK_BitIO)
**     Description :
**         Sets the pin as input
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void LEDpin4_SetInput(void)
{
#if KSDK1_SDK_VERSION_USED == KSDK1_SDK_VERSION_2_0
  GPIO_PinInit(LEDpin4_GPIOName, LEDpin4_PinNumber, &LEDpin4_configInput);
#elif KSDK1_SDK_VERSION_USED == KSDK1_SDK_VERSION_1_3
  GPIO_DRV_SetPinDir(LED_RED1, kGpioDigitalInput);
#endif
  LEDpin4_isOutput = false;
}

/*
** ===================================================================
**     Method      :  LEDpin4_SetOutput (component SDK_BitIO)
**     Description :
**         Sets the pin as output
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void LEDpin4_SetOutput(void)
{
#if KSDK1_SDK_VERSION_USED == KSDK1_SDK_VERSION_2_0
  GPIO_PinInit(LEDpin4_GPIOName, LEDpin4_PinNumber, &LEDpin4_configOutput);
#elif KSDK1_SDK_VERSION_USED == KSDK1_SDK_VERSION_1_3
  GPIO_DRV_SetPinDir(LED_RED1, kGpioDigitalOutput);
#endif
  LEDpin4_isOutput = true;
}

/*
** ===================================================================
**     Method      :  LEDpin4_PutVal (component SDK_BitIO)
**     Description :
**         Sets the pin value
**     Parameters  :
**         NAME            - DESCRIPTION
**         Val             - Value to set. FALSE/logical '0' or
**                           TRUE/logical '1'
**     Returns     : Nothing
** ===================================================================
*/
void LEDpin4_PutVal(bool Val)
{
#if KSDK1_SDK_VERSION_USED == KSDK1_SDK_VERSION_2_0
  if (Val) {
    GPIO_SetPinsOutput(LEDpin4_GPIOName, 1<<LEDpin4_PinNumber);
  } else {
    GPIO_ClearPinsOutput(LEDpin4_GPIOName, 1<<LEDpin4_PinNumber);
  }
#elif KSDK1_SDK_VERSION_USED == KSDK1_SDK_VERSION_1_3
  GPIO_DRV_WritePinOutput(LED_RED1, Val);
#endif
}

/*
** ===================================================================
**     Method      :  LEDpin4_Init (component SDK_BitIO)
**     Description :
**         Driver initialization method
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void LEDpin4_Init(void)
{
#if KSDK1_SDK_VERSION_USED == KSDK1_SDK_VERSION_2_0
  #if LEDpin4_DO_PIN_MUXING
  PORT_SetPinMux(LEDpin4_PORTName, LEDpin4_PinNumber, kPORT_MuxAsGpio); /* mux as GPIO */
  #endif
  LEDpin4_SetOutput();
#elif KSDK1_SDK_VERSION_USED == KSDK1_SDK_VERSION_1_3
  /*! \todo Pin Muxing not implemented */
  GPIO_DRV_Init(LEDpin4_InputConfig, LEDpin4_OutputConfig);
#endif
}

/*
** ===================================================================
**     Method      :  LEDpin4_Deinit (component SDK_BitIO)
**     Description :
**         Driver de-initialization method
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void LEDpin4_Deinit(void)
{
  /* nothing needed */
}

/* END LEDpin4. */

/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
