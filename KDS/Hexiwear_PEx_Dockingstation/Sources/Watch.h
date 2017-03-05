/*
 * Watch.h
 *
 *  Created on: 05.03.2017
 *      Author: Erich Styger Local
 */

#ifndef SOURCES_WATCH_H_
#define SOURCES_WATCH_H_

#include "Platform.h"
#if PL_CONFIG_HAS_WATCH
#include "FreeRTOS.h"
#include "task.h"
#include "UI1.h"

xTaskHandle WATCH_CreateUITask(UI1_Element **root);

void WATCH_KillTask(void);

void WATCH_Init(void);

#endif /* PL_CONFIG_HAS_WATCH */

#endif /* SOURCES_WATCH_H_ */
