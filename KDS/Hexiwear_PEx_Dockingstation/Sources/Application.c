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
#if PL_CONFIG_HAS_UI
  #include "UI.h"
#endif
#if PL_CONFIG_HAS_WATCH
  #include "Watch.h"
#endif
#if PL_CONFIG_HAS_ACCELEROMETER
  #include "FX1.h"
  //#include "Accel_RST.h"
#endif
#if PL_CONFIG_HAS_IDENTIFY
  #include "Identify.h"
#endif
#include "Vibro.h"
#include "GDisp1.h"
#include "LCD1.h"
#include "Cube.h"
#include "UI1.h"
#if PL_CONFIG_HAS_PAIRING
  #include "Pairing.h"
#endif
#include "Bluetooth.h"
#if PL_CONFIG_HAS_TSL2561
  #include "TSL2561.h"
  #include "Vcc3V3B_EN.h"
#endif

#if PL_CONFIG_HAS_CUBE_DEMO
  static CUBE_WindowDesc cubeWindow;
#endif

static void AppTask(void *param) {
  bool closeIt = FALSE;

  RGBR_On(); vTaskDelay(pdMS_TO_TICKS(100)); RGBR_Off();
  RGBG_On(); vTaskDelay(pdMS_TO_TICKS(100)); RGBG_Off();
  RGBB_On(); vTaskDelay(pdMS_TO_TICKS(100)); RGBB_Off();
#if PL_CONFIG_HAS_HOME_LEDS
  HLED1_On(); vTaskDelay(pdMS_TO_TICKS(100)); HLED1_Off();
  HLED2_On(); vTaskDelay(pdMS_TO_TICKS(100)); HLED2_Off();
  HLED3_On(); vTaskDelay(pdMS_TO_TICKS(100)); HLED3_Off();
  HLED4_On(); vTaskDelay(pdMS_TO_TICKS(100)); HLED4_Off();
  HLED5_On(); vTaskDelay(pdMS_TO_TICKS(100)); HLED5_Off();
  HLED6_On(); vTaskDelay(pdMS_TO_TICKS(100)); HLED6_Off();
#endif
  //Vibro_SetVal();
  //Vibro_ClrVal();

#if PL_CONFIG_HAS_CUBE_DEMO
 // CUBE_CreateWindow(&cubeWindow);
#endif
#if PL_CONFIG_HAS_ACCELEROMETER
  CLS1_SendStr("Initializing accelerometer.\r\n", CLS1_GetStdio()->stdOut);
  FX1_Init(); /* init and enable device */
#endif

  vTaskDelay(pdMS_TO_TICKS(1000)); /* give it some time */
  CLS1_SendStr("Sending version request.\r\n", CLS1_GetStdio()->stdOut);
  BLUETOOTH_SendVersionReq(); /* get version number */
  vTaskDelay(pdMS_TO_TICKS(100));

  /* advertisement mode: flash might be erased, it gets properly set with 'toggle' first */
  CLS1_SendStr("Get current advertisement mode.\r\n", CLS1_GetStdio()->stdOut);
  BLUETOOTH_SendAdvModeGetReq();
  vTaskDelay(pdMS_TO_TICKS(100)); /* give it some time, it will respond with advModeSend so I will have the current mode */
  if (BLUETOOTH_GetAdvMode()!=bluetooth_advMode_enable) { /* check if we are in the right mode */
    CLS1_SendStr("Advertisement disabled, sending request to enable it.\r\n", CLS1_GetStdio()->stdOut);
    BLUETOOTH_SendToggleAdvModeReq(); /* no? request toggle */
  }

  /* active buttons: flash might be erased, it gets properly set with 'toggle' first */
  CLS1_SendStr("Get current active button mode.\r\n", CLS1_GetStdio()->stdOut);
  BLUETOOTH_SendActiveButtonsGetReq();
  vTaskDelay(pdMS_TO_TICKS(100)); /* give it some time, it will respond with buttonsGroupSendActive so I will have the current mode */
  if (BLUETOOTH_GetActiveButtons()!=buttonsGroup_right) { /* check if we are in the right mode */
    CLS1_SendStr("Turning on touch buttons on the right side.\r\n", CLS1_GetStdio()->stdOut);
    BLUETOOTH_SendToggleActiveButtonsReq(); /* no? request toggle */
  }

  /* request current link state */
  CLS1_SendStr("Getting current link state.\r\n", CLS1_GetStdio()->stdOut);
  BLUETOOTH_SendLinkStateGetReq();
  vTaskDelay(pdMS_TO_TICKS(100));

  CLS1_SendStr("Enabling TLS2561 sensor.\r\n", CLS1_GetStdio()->stdOut);
  {
    uint8_t res;
    uint8_t id;
    uint16_t broadband, infrared;
    uint32_t lux;

    /* 3V3B_EN:
     * HI-Z: Disabled
     * LOW: enabled (humidity, temperature, ambiLight
     */
    //Vcc3V3B_EN_SetInput(); /* disable */
    /* enable */
    Vcc3V3B_EN_SetOutput();
    Vcc3V3B_EN_ClrVal();
    vTaskDelay(pdMS_TO_TICKS(50));
    TSL2561_Init();

    res = TSL2561_Disable();
    if (res!=ERR_OK) {
      for(;;){}
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    res = TSL2561_Enable();
    if (res!=ERR_OK) {
      for(;;){}
    }

    res = TSL2561_SetIntegrationTime(TSL2561_INTEGRATION_TIME_13MS);
    if (res!=ERR_OK) {
      for(;;){}
    }
    res = TSL2561_SetGain(TSL2561_GAIN_16X);
    if (res!=ERR_OK) {
      for(;;){}
    }

    res = TSL2561_ReadRawDataFull(&broadband);
    if (res!=ERR_OK) {
      for(;;){}
    }
    res = TSL2561_ReadRawDataInfrared(&infrared);
    if (res!=ERR_OK) {
      for(;;){}
    }

    lux = TSL2561_CalculateLux(broadband, infrared);
  }
  CLS1_SendStr("Running application loop.\r\n", CLS1_GetStdio()->stdOut);
  for(;;) {
    if (BLUETOOTH_GetAdvMode()==bluetooth_advMode_enable) {
      RGBB_On(); /* BLE on, advertising, show with blue LED */
      if (BLUETOOTH_GetLinkState()==linkState_connected) {
        vTaskDelay(pdMS_TO_TICKS(500)); /* connected, show with longer blue LED */
      } else {
        vTaskDelay(pdMS_TO_TICKS(10));
      }
      RGBB_Off();
    } else {
      RGBG_On();
      vTaskDelay(pdMS_TO_TICKS(10));
      RGBG_Off();
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
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
#if PL_CONFIG_HAS_UI
  UI_Init();
#endif
#if PL_CONFIG_HAS_PAIRING
  PAIRING_Init();
#endif
#if PL_CONFIG_HAS_WATCH
  WATCH_Init();
#endif
#if PL_CONFIG_HAS_IDENTIFY
  ID_Init();
#endif
#if PL_CONFIG_HAS_TSL2561
  TSL2561_Init();
#endif
  BLUETOOTH_Init();
  if (xTaskCreate(AppTask, (uint8_t *)"App", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL) != pdPASS) {
    for(;;){} /* error case only, stay here! */
  }
  vTaskStartScheduler();
}

