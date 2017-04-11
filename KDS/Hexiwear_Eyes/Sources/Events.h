/* ###################################################################
**     Filename    : Events.h
**     Project     : Hexiwear_Eyes
**     Processor   : MK64FN1M0VDC12
**     Component   : Events
**     Version     : Driver 01.00
**     Compiler    : GNU C Compiler
**     Date/Time   : 2017-04-01, 17:25, # CodeGen: 0
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
#include "MCUC1.h"
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
#include "CS1.h"
#include "HF1.h"
#include "KIN1.h"
#include "LCD1.h"
#include "SCEpin1.h"
#include "BitIoLdd5.h"
#include "RESpin1.h"
#include "BitIoLdd6.h"
#include "D_Cpin1.h"
#include "BitIoLdd7.h"
#include "SM1.h"
#include "TI1.h"
#include "TimerIntLdd1.h"
#include "TU1.h"
#include "GDisp1.h"
#include "OLEDPower.h"
#include "BitIoLdd8.h"

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

/*
** ===================================================================
**     Event       :  SM1_OnBlockSent (module Events)
**
**     Component   :  SM1 [SPIMaster_LDD]
*/
/*!
**     @brief
**         This event is called after the last character from the
**         output buffer is moved to the transmitter. This event is
**         available only if the SendBlock method is enabled.
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. The pointer is passed
**                           as the parameter of Init method. 
*/
/* ===================================================================*/
void SM1_OnBlockSent(LDD_TUserData *UserDataPtr);

/*
** ===================================================================
**     Event       :  TI1_OnInterrupt (module Events)
**
**     Component   :  TI1 [TimerInt]
**     Description :
**         When a timer interrupt occurs this event is called (only
**         when the component is enabled - <Enable> and the events are
**         enabled - <EnableEvent>). This event is enabled only if a
**         <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void TI1_OnInterrupt(void);

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
