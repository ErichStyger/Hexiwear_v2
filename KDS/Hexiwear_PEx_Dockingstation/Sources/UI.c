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
#include "Quizz.h"

static xTaskHandle UI_CurrentUserInterfacTask = NULL;
static bool isBlankScreen = FALSE;

void UI_SetCurrentUITask(xTaskHandle handle) {
  UI_CurrentUserInterfacTask = handle;
}

void UI_BlankScreen(void) {
  isBlankScreen = TRUE;
  LCD1_Clear(); /* blank display */
}

void UI_ShowScreen(void) {
  UI1_MsgPaintAllElements(UI1_GetRoot());
  isBlankScreen = FALSE;
}

void UI_Event(UI_EventType kind) {
  UI1_Element *element;

  if (UI_CurrentUserInterfacTask==NULL) {
    return; /* no active UI task */
#if 0 && PL_CONFIG_HAS_QUIZZ
    QUIZZ_CreateTask(); /* create task again */
#endif
    // return;
  }
  if (isBlankScreen) {
    UI_ShowScreen();
  }
  switch(kind) {
    case UI_EVENT_BUTTON_LEFT:
      break;
    case UI_EVENT_BUTTON_RIGHT: /* enter */
      if (UI1_GetSelectedElement(&element)) {
        UI1_SendMessage(element, UI1_MSG_CLICK, NULL);
      }
      break;
    case UI_EVENT_BUTTON_UP:
      UI1_SelectNextElement(UI1_GetRoot(), FALSE);
      break;
    case UI_EVENT_BUTTON_DOWN:
      UI1_SelectNextElement(UI1_GetRoot(), TRUE);
      break;
    default:
      break;
  } /* switch */
}


void UI_Init(void) {
  UI_CurrentUserInterfacTask = NULL;
}

#endif /* PL_CONFIG_HAS_UI */


