/*
 * Buzzer.h
 *
 *  Created on: 25.02.2017
 *      Author: Erich Styger Local
 */

#ifndef SOURCES_BUZZER_H_
#define SOURCES_BUZZER_H_

#include "Platform.h"

#if PL_CONFIG_HAS_BUZZER

#include "CLS1.h"

uint8_t BUZ_ParseCommand(const unsigned char* cmd, bool *handled, const CLS1_StdIOType *io);

void BUZ_Buzzer(uint32_t durationMs);

void BUZ_Init(void);

#endif /* PL_CONFIG_HAS_BUZZER */

#endif /* SOURCES_BUZZER_H_ */
