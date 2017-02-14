/*
 * UI.c
 *
 *  Created on: 13.02.2017
 *      Author: Erich Styger Local
 */

#include "Platform.h"
#if PL_CONFIG_HAS_UI
#include "UI.h"
#include "UI1.h"


void UI_Event(UI_EventType kind) {
  switch(kind) {
    case UI_EVENT_BUTTON_LEFT:
    case UI_EVENT_BUTTON_UP:
      UI1_SelectNextElement(UI1_GetRoot(), FALSE);
      UI1_MsgPaintAllElements(UI1_GetRoot());
      break;
    case UI_EVENT_BUTTON_RIGHT:
    case UI_EVENT_BUTTON_DOWN:
      UI1_SelectNextElement(UI1_GetRoot(), TRUE);
      UI1_MsgPaintAllElements(UI1_GetRoot());
      break;
    default:
      break;
  } /* switch */
}


void UI_Init(void) {

}

#endif /* PL_CONFIG_HAS_UI */


