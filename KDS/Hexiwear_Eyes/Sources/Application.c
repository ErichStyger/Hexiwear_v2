/*
 * Application.c
 *
 *  Created on: 01.04.2017
 *      Author: Erich Styger
 */

#include "Application.h"
#include "WAIT1.h"
#include "RGBR.h"
#include "RGBG.h"
#include "RGBB.h"
#include "LCD1.h"
#include "GDisp1.h"
#include "OLEDPower.h"
#include "uncannyEyes.h"

void APP_Run(void) {
  //OLEDPower_ClrVal(); /* turn on power for OLED (low active) */
  EYES_Init();
  LCD1_Init();
  LCD1_Clear();
  GDisp1_DrawFilledBox(0, 0, GDisp1_GetWidth()-1, GDisp1_GetHeight()-1, LCD1_COLOR_RED);
  GDisp1_UpdateFull();
  for(;;) {
    EYES_Run();
    RGBR_On();
    WAIT1_Waitms(5);
    RGBR_Off();
  }
}

