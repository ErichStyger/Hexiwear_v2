/**
 * \file
 * \brief Shell and console interface implementation.
 * \author Erich Styger
 *
 * This module implements the front to the console/shell functionality.
 */

#include "Platform.h"
#include "Shell.h"
#include "CLS1.h"
#include "Application.h"
#include "FRTOS1.h"
#include "KIN1.h"
#if PL_CONFIG_HAS_RADIO
  #include "RNET1.h"
  #include "RNetConf.h"
  #include "RNet_App.h"
#endif
#if PL_CONFIG_HAS_SHELL_RTT
  #include "RTT1.h"
#endif
#if RNET_CONFIG_REMOTE_STDIO
  #include "RStdIO.h"
#endif
#if PL_CONFIG_HAS_HOME_LEDS
  #include "HLED1.h"
  #include "HLED2.h"
  #include "HLED3.h"
  #include "HLED4.h"
  #include "HLED5.h"
  #include "HLED6.h"
#endif
#include "RGBR.h"
#include "RGBG.h"
#include "RGBB.h"

static const CLS1_ParseCommandCallback CmdParserTable[] =
{
  CLS1_ParseCommand, /* Processor Expert Shell component, is first in list */
  KIN1_ParseCommand,
#if PL_CONFIG_HAS_HOME_LEDS
  #if HLED1_PARSE_COMMAND_ENABLED
  HLED1_ParseCommand,
  #endif
  #if HLED2_PARSE_COMMAND_ENABLED
  HLED2_ParseCommand,
  #endif
  #if HLED3_PARSE_COMMAND_ENABLED
  HLED3_ParseCommand,
  #endif
  #if HLED4_PARSE_COMMAND_ENABLED
  HLED4_ParseCommand,
  #endif
  #if HLED5_PARSE_COMMAND_ENABLED
  HLED5_ParseCommand,
  #endif
  #if HLED6_PARSE_COMMAND_ENABLED
  HLED6_ParseCommand,
  #endif
#endif
#if RGBR_PARSE_COMMAND_ENABLED
  RGBR_ParseCommand,
#endif
#if RGBG_PARSE_COMMAND_ENABLED
  RGBG_ParseCommand,
#endif
#if RGBB_PARSE_COMMAND_ENABLED
  RGBB_ParseCommand,
#endif
#if PL_CONFIG_HAS_RADIO
  RNET1_ParseCommand,
  RNETA_ParseCommand,
#endif
  FRTOS1_ParseCommand,
  NULL /* Sentinel */
};

static void ShellTask(void *pvParameters) {
#if PL_CONFIG_HAS_RADIO && RNET_CONFIG_REMOTE_STDIO
  CLS1_ConstStdIOType *ioRemote = RSTDIO_GetStdio();
#endif
  (void)pvParameters; /* not used */
  CLS1_DefaultShellBuffer[0] = '\0';
#if PL_CONFIG_HAS_RADIO && RNET_CONFIG_REMOTE_STDIO
  RSTDIO_DefaultShellBuffer[0] = '\0';
#endif
  CLS1_SendStr("hello world!\r\n", CLS1_GetStdio()->stdOut);
  for(;;) {
    (void)CLS1_ReadAndParseWithCommandTable(CLS1_DefaultShellBuffer, sizeof(CLS1_DefaultShellBuffer), CLS1_GetStdio(), CmdParserTable);
#if PL_CONFIG_HAS_RADIO && RNET_CONFIG_REMOTE_STDIO
    RSTDIO_Print(CLS1_GetStdio()); /* dispatch incoming messages */
    (void)CLS1_ReadAndParseWithCommandTable(RSTDIO_DefaultShellBuffer, sizeof(RSTDIO_DefaultShellBuffer), ioRemote, CmdParserTable);
#endif
    FRTOS1_vTaskDelay(10/portTICK_PERIOD_MS);
  } /* for */
}

void SHELL_Init(void) {
#if PL_CONFIG_HAS_SHELL_RTT
  CLS1_SetStdio(RTT1_GetStdio());
#endif
  if (xTaskCreate(ShellTask, "Shell", configMINIMAL_STACK_SIZE+100, NULL, tskIDLE_PRIORITY+1, NULL) != pdPASS) {
    for(;;){} /* error */
  }
}

void SHELL_Deinit(void) {
}

