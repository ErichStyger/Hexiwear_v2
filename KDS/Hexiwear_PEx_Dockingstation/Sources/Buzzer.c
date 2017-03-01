/*
 * Buzzer.c
 *
 *  Created on: 25.02.2017
 *      Author: Erich Styger Local
 */


#include "Platform.h"
#if PL_CONFIG_HAS_BUZZER
#include "Buzzer.h"
#include "Vibro.h"
#include "TRG1.h"
#include "UTIL1.h"

uint8_t BUZ_ParseCommand(const unsigned char* cmd, bool *handled, const CLS1_StdIOType *io) {
  uint8_t res = ERR_OK;

  if (UTIL1_strcmp((char*)cmd, CLS1_CMD_HELP) == 0 || UTIL1_strcmp((char*)cmd, "buzzer help") == 0) {
    CLS1_SendHelpStr((unsigned char*)"buzzer", (unsigned char*)"Group of buzzer commands\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Print help or status information\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  buz <ms>", (unsigned char*)"buzzer for given milliseconds\r\n", io->stdOut);
    *handled = TRUE;
  } else if (UTIL1_strncmp((char*)cmd, "buzzer buz ", sizeof("buzzer buz ")-1) == 0) {
    uint32_t ms;
    const uint8_t *p;

    p = cmd + sizeof("buzzer buz ")-1;
    if (UTIL1_ScanDecimal32uNumber(&p, &ms)==ERR_OK) {
      BUZ_Buzzer(ms);
    } else {
      CLS1_SendStr("Failed reading millisecond number\r\n", io->stdErr);
    }
    *handled = TRUE;
  }
  return res;
}

static void BuzzerOff(void) {
  Vibro_ClrVal(); /* turn buzzer off */
}

void BUZ_Buzzer(uint32_t durationMs) {
  Vibro_SetVal(); /* turn buzzer on */
  TRG1_AddTrigger(TRG1_BUZZER, durationMs/TRG1_TICK_PERIOD_MS, BuzzerOff);
}


void BUZ_Init(void) {
  /* nothing */
}


#endif
