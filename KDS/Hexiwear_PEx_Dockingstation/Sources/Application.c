/*
 * Application.c
 *
 *  Created on: 29.11.2016
 *      Author: Erich Styger Local
 */

#include "Platform.h"
#include "Application.h"
#include "FRTOS1.h"
#include "Shell.h"
#include "RNet_App.h"
#include "RGBR.h"
#include "RGBB.h"
#include "RGBG.h"
#if PL_CONFIG_HAS_HOME_LEDS
  #include "HLED1.h"
  #include "HLED2.h"
  #include "HLED3.h"
  #include "HLED4.h"
  #include "HLED5.h"
  #include "HLED6.h"
#endif
#if PL_CONFIG_HAS_KW40_COMM
  #include "KW40Comm.h"
  #include "HostComm.h"
#endif
#if PL_CONFIG_HAS_QUIZZ
  #include "Quizz.h"
#endif
#include "Vibro.h"
#include "GDisp1.h"
#include "LCD1.h"
#include "Cube.h"
#include "UI1.h"

#if PL_CONFIG_HAS_CUBE_DEMO
  static CUBE_WindowDesc cubeWindow;
#endif

void APP_Event(APP_EventType kind) {
  switch(kind) {
  case APP_EVENT_BUTTON_LEFT:
  case APP_EVENT_BUTTON_UP:
    UI1_SelectNextElement(UI1_GetRoot(), FALSE);
    break;
  case APP_EVENT_BUTTON_RIGHT:
  case APP_EVENT_BUTTON_DOWN:
    UI1_SelectNextElement(UI1_GetRoot(), TRUE);
    break;
  } /* switch */
}

static void AppTask(void *param) {
  bool closeIt = FALSE;

  RGBR_On();
  FRTOS1_vTaskDelay(pdMS_TO_TICKS(100));
  RGBR_Off();
  RGBG_On();
  FRTOS1_vTaskDelay(pdMS_TO_TICKS(100));
  RGBG_Off();
  RGBB_On();
  FRTOS1_vTaskDelay(pdMS_TO_TICKS(100));
  RGBB_Off();
#if PL_CONFIG_HAS_HOME_LEDS
  HLED1_On();
  FRTOS1_vTaskDelay(pdMS_TO_TICKS(100));
  HLED1_Off();
  HLED2_On();
  FRTOS1_vTaskDelay(pdMS_TO_TICKS(100));
  HLED2_Off();
  HLED3_On();
  FRTOS1_vTaskDelay(pdMS_TO_TICKS(100));
  HLED3_Off();
  HLED4_On();
  FRTOS1_vTaskDelay(pdMS_TO_TICKS(100));
  HLED4_Off();
  HLED5_On();
  FRTOS1_vTaskDelay(pdMS_TO_TICKS(100));
  HLED5_Off();
  HLED6_On();
  FRTOS1_vTaskDelay(pdMS_TO_TICKS(100));
  HLED6_Off();
#endif
  //Vibro_SetVal();
  //Vibro_ClrVal();

  LCD1_Init();
  LCD1_Clear();

#if PL_CONFIG_HAS_CUBE_DEMO
  CUBE_CreateWindow(&cubeWindow);
#endif
#if PL_CONFIG_HAS_QUIZZ
  QUIZZ_CreateTask();
#endif
  for(;;) {
#if PL_CONFIG_HAS_QUIZZ
    if (closeIt) {
      QUIZZ_KillTask();
    }
#endif
    RGBG_On();
    FRTOS1_vTaskDelay(pdMS_TO_TICKS(50));
    RGBG_Off();
    FRTOS1_vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void APP_Run(void) {
  SHELL_Init();
#if PL_CONFIG_HAS_RADIO
  RNETA_Init();
#endif
#if PL_CONFIG_HAS_KW40_COMM
  KW40Comm_Init();
  HostComm_Init();
#endif
#if PL_CONFIG_HAS_QUIZZ
  QUIZZ_Init();
#endif
  if (xTaskCreate(AppTask, (uint8_t *)"App", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL) != pdPASS) {
    for(;;){} /* error case only, stay here! */
  }
  vTaskStartScheduler();
}

