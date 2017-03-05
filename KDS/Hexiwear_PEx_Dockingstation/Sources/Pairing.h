/*
 * Pairing.h
 *
 *  Created on: 01.03.2017
 *      Author: Erich Styger Local
 */

#ifndef SOURCES_PAIRING_H_
#define SOURCES_PAIRING_H_

#include "Platform.h"
#if PL_CONFIG_HAS_PAIRING
#include "UI1.h"
#include "FreeRTOS.h"
#include "task.h"

void PAIRING_SetPairingCode(uint32_t code);

xTaskHandle PAIRING_CreateUITask(UI1_Element **root);

void PAIRING_KillTask(void);

void PAIRING_Init(void);

#endif /* PL_CONFIG_HAS_PAIRING */


#endif /* SOURCES_PAIRING_H_ */
