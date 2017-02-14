/*
 * UI.h
 *
 *  Created on: 13.02.2017
 *      Author: Erich Styger Local
 */

#ifndef SOURCES_UI_H_
#define SOURCES_UI_H_

#include "Platform.h"

typedef enum {
  UI_EVENT_BUTTON_DOWN,
  UI_EVENT_BUTTON_UP,
  UI_EVENT_BUTTON_LEFT,
  UI_EVENT_BUTTON_RIGHT,
} UI_EventType;

void UI_Event(UI_EventType kind);

void UI_Init(void);


#endif /* SOURCES_UI_H_ */
