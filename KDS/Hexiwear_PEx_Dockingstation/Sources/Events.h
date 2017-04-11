/* ###################################################################
**     Filename    : Events.h
**     Project     : Hexiwear_PEx_Dockingstation
**     Processor   : MK64FN1M0VDC12
**     Component   : Events
**     Version     : Driver 01.00
**     Compiler    : GNU C Compiler
**     Date/Time   : 2016-11-29, 10:52, # CodeGen: 0
**     Abstract    :
**         This is user's event module.
**         Put your event handler code here.
**     Contents    :
**         Cpu_OnNMI - void Cpu_OnNMI(void);
**
** ###################################################################*/
/*!
** @file Events.h
** @version 01.00
** @brief
**         This is user's event module.
**         Put your event handler code here.
*/         
/*!
**  @addtogroup Events_module Events module documentation
**  @{
*/         

#ifndef __Events_H
#define __Events_H
/* MODULE Events */

#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "Pins1.h"
#include "WAIT1.h"
#include "RGBR.h"
#include "LEDpin1.h"
#include "BitIoLdd1.h"
#include "RGBG.h"
#include "LEDpin2.h"
#include "BitIoLdd2.h"
#include "RGBB.h"
#include "LEDpin3.h"
#include "BitIoLdd3.h"
#include "UTIL1.h"
#include "MCUC1.h"
#include "CS1.h"
#include "CLS1.h"
#include "BLEUart.h"
#include "ASerialLdd2.h"
#include "Vibro.h"
#include "BitIoLdd12.h"
#include "GDisp1.h"
#include "LCD1.h"
#include "RESpin1.h"
#include "BitIoLdd13.h"
#include "D_Cpin1.h"
#include "BitIoLdd14.h"
#include "SCEpin1.h"
#include "BitIoLdd15.h"
#include "SM2.h"
#include "Clock1.h"
#include "BitIoLdd17.h"
#include "Output1.h"
#include "BitIoLdd19.h"
#include "OLEDPower.h"
#include "BitIoLdd20.h"
#include "UI1.h"
#include "UIScreen.h"
#include "UIWindow.h"
#include "UIText.h"
#include "UIIcon.h"
#include "UIHeader.h"
#include "FDisp1.h"
#include "Helv08n.h"
#include "TRG1.h"
#include "Helv12n.h"
#include "I2C1.h"
#include "GI2C1.h"
#include "FX1.h"
#include "I2CSPY1.h"
#include "TMOUT1.h"
#include "TmDt1.h"
#include "RTC1.h"
#include "PTRC1.h"
#include "AmblInt.h"
#include "ExtIntLdd4.h"
#include "GI2C0.h"
#include "I2C2.h"
#include "SDA1.h"
#include "BitIoLdd21.h"
#include "SCL1.h"
#include "BitIoLdd22.h"
#include "Vcc3V3B_EN.h"
#include "BitIoLdd23.h"
#include "XF1.h"
#include "HF1.h"
#include "RTT1.h"
#include "FRTOS1.h"
#include "KIN1.h"

#ifdef __cplusplus
extern "C" {
#endif 

/*
** ===================================================================
**     Event       :  Cpu_OnNMI (module Events)
**
**     Component   :  Cpu [MK64FN1M0LQ12]
*/
/*!
**     @brief
**         This event is called when the Non maskable interrupt had
**         occurred. This event is automatically enabled when the [NMI
**         interrupt] property is set to 'Enabled'.
*/
/* ===================================================================*/
void Cpu_OnNMI(void);


void FRTOS1_vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);
/*
** ===================================================================
**     Event       :  FRTOS1_vApplicationStackOverflowHook (module Events)
**
**     Component   :  FRTOS1 [FreeRTOS]
**     Description :
**         if enabled, this hook will be called in case of a stack
**         overflow.
**     Parameters  :
**         NAME            - DESCRIPTION
**         pxTask          - Task handle
**       * pcTaskName      - Pointer to task name
**     Returns     : Nothing
** ===================================================================
*/

void FRTOS1_vApplicationTickHook(void);
/*
** ===================================================================
**     Event       :  FRTOS1_vApplicationTickHook (module Events)
**
**     Component   :  FRTOS1 [FreeRTOS]
**     Description :
**         If enabled, this hook will be called by the RTOS for every
**         tick increment.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void FRTOS1_vApplicationIdleHook(void);
/*
** ===================================================================
**     Event       :  FRTOS1_vApplicationIdleHook (module Events)
**
**     Component   :  FRTOS1 [FreeRTOS]
**     Description :
**         If enabled, this hook will be called when the RTOS is idle.
**         This might be a good place to go into low power mode.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void FRTOS1_vApplicationMallocFailedHook(void);
/*
** ===================================================================
**     Event       :  FRTOS1_vApplicationMallocFailedHook (module Events)
**
**     Component   :  FRTOS1 [FreeRTOS]
**     Description :
**         If enabled, the RTOS will call this hook in case memory
**         allocation failed.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

/*
** ===================================================================
**     Event       :  BLEUart_OnError (module Events)
**
**     Component   :  BLEUart [AsynchroSerial]
**     Description :
**         This event is called when a channel error (not the error
**         returned by a given method) occurs. The errors can be read
**         using <GetError> method.
**         The event is available only when the <Interrupt
**         service/event> property is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void BLEUart_OnError(void);

/*
** ===================================================================
**     Event       :  BLEUart_OnRxChar (module Events)
**
**     Component   :  BLEUart [AsynchroSerial]
**     Description :
**         This event is called after a correct character is received.
**         The event is available only when the <Interrupt
**         service/event> property is enabled and either the <Receiver>
**         property is enabled or the <SCI output mode> property (if
**         supported) is set to Single-wire mode.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void BLEUart_OnRxChar(void);

/*
** ===================================================================
**     Event       :  BLEUart_OnTxChar (module Events)
**
**     Component   :  BLEUart [AsynchroSerial]
**     Description :
**         This event is called after a character is transmitted.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void BLEUart_OnTxChar(void);

/*
** ===================================================================
**     Event       :  BLEUart_OnFullRxBuf (module Events)
**
**     Component   :  BLEUart [AsynchroSerial]
**     Description :
**         This event is called when the input buffer is full;
**         i.e. after reception of the last character 
**         that was successfully placed into input buffer.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void BLEUart_OnFullRxBuf(void);

/*
** ===================================================================
**     Event       :  BLEUart_OnFreeTxBuf (module Events)
**
**     Component   :  BLEUart [AsynchroSerial]
**     Description :
**         This event is called after the last character in output
**         buffer is transmitted.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void BLEUart_OnFreeTxBuf(void);

void PTRC1_OnTraceWrap(void);
/*
** ===================================================================
**     Event       :  PTRC1_OnTraceWrap (module Events)
**
**     Component   :  PTRC1 [PercepioTrace]
**     Description :
**         Called for trace ring buffer wrap around. This gives the
**         application a chance to dump the trace buffer.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void AmblInt_OnInterrupt(void);
/*
** ===================================================================
**     Event       :  AmblInt_OnInterrupt (module Events)
**
**     Component   :  AmblInt [ExtInt]
**     Description :
**         This event is called when an active signal edge/level has
**         occurred.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

/* END Events */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

#endif 
/* ifndef __Events_H*/
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
