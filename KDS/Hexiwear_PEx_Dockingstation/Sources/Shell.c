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
#include "RNET1.h"
#include "RNetConf.h"
#include "RNet_App.h"
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
#if PL_CONFIG_HAS_RADIO
  RNET1_ParseCommand,
  RNETA_ParseCommand,
#endif
  FRTOS1_ParseCommand,
  NULL /* Sentinel */
};

static void ShellTask(void *pvParameters) {
#if RNET_CONFIG_REMOTE_STDIO
  static unsigned char radio_cmd_buf[CLS1_DEFAULT_SHELL_BUFFER_SIZE];
  CLS1_ConstStdIOType *ioRemote = RSTDIO_GetStdioRx();
#endif
  (void)pvParameters; /* not used */
  CLS1_DefaultShellBuffer[0] = '\0';
#if RNET_CONFIG_REMOTE_STDIO
  radio_cmd_buf[0] = '\0';
#endif
  CLS1_SendStr("hello world!\r\n", CLS1_GetStdio()->stdOut);
  for(;;) {
    (void)CLS1_ReadAndParseWithCommandTable(CLS1_DefaultShellBuffer, sizeof(CLS1_DefaultShellBuffer), CLS1_GetStdio(), CmdParserTable);
#if RNET_CONFIG_REMOTE_STDIO
    RSTDIO_Print(CLS1_GetStdio()); /* dispatch incoming messages */
    (void)CLS1_ReadAndParseWithCommandTable(radio_cmd_buf, sizeof(radio_cmd_buf), ioRemote, CmdParserTable);
#endif
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

