/*
 * Application.c
 *
 *  Created on: 29.11.2016
 *      Author: Erich Styger Local
 */

#include "Application.h"
#include "FRTOS1.h"
#include "Shell.h"
#include "RNet_App.h"

void APP_Run(void) {
  SHELL_Init();
  RNETA_Init();
  vTaskStartScheduler();
}

