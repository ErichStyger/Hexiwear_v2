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

static void AppTask(void *param) {
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
  for(;;) {
    RGBG_Neg();
    FRTOS1_vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void APP_Run(void) {
  SHELL_Init();
#if PL_CONFIG_HAS_RADIO
  RNETA_Init();
#endif
  if (FRTOS1_xTaskCreate(AppTask, (uint8_t *)"App", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL) != pdPASS) {
    for(;;){} /* error case only, stay here! */
  }

  vTaskStartScheduler();
}

