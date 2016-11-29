/**
 * \file
 * \brief Shell and console interface implementation.
 * \author Erich Styger
 *
 * This module implements the front to the console/shell functionality.
 */

#include "Shell.h"
#include "CLS1.h"
#include "Application.h"
#include "FRTOS1.h"
#include "KIN1.h"
#include "RNET1.h"
#include "RNet_App.h"

static const CLS1_ParseCommandCallback CmdParserTable[] =
{
  CLS1_ParseCommand, /* Processor Expert Shell component, is first in list */
  KIN1_ParseCommand,
  RNET1_ParseCommand,
  RNETA_ParseCommand,
  FRTOS1_ParseCommand,
  NULL /* Sentinel */
};

static void ShellTask(void *pvParameters) {
  (void)pvParameters; /* not used */
  CLS1_DefaultShellBuffer[0] = '\0';
  CLS1_SendStr("hello world!\r\n", CLS1_GetStdio()->stdOut);
  for(;;) {
    (void)CLS1_ReadAndParseWithCommandTable(CLS1_DefaultShellBuffer, sizeof(CLS1_DefaultShellBuffer), CLS1_GetStdio(), CmdParserTable);
    FRTOS1_vTaskDelay(10/portTICK_PERIOD_MS);
  } /* for */
}

void SHELL_Init(void) {
  if (FRTOS1_xTaskCreate(ShellTask, "Shell", configMINIMAL_STACK_SIZE+100, NULL, tskIDLE_PRIORITY+1, NULL) != pdPASS) {
    for(;;){} /* error */
  }
}

void SHELL_Deinit(void) {
}

