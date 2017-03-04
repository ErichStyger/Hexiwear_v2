/*
 * UI.c
 *
 *  Created on: 13.02.2017
 *      Author: Erich Styger Local
 */

#include "Platform.h"
#if PL_CONFIG_HAS_UI
#include "FreeRTOS.h"
#include "timers.h"
#include "UI.h"
#include "UI1.h"
#include "Buzzer.h"
#include "UI1.h"
#include "UIScreen.h"
#include "UIText.h"
#if PL_CONFIG_HAS_QUIZZ
  #include "Quizz.h"
#endif
#if PL_CONFIG_HAS_PAIRING
  #include "Pairing.h"
#endif

typedef enum {
  UI_PAGE_NONE,
  UI_PAGE_HOME,
  UI_PAGE_QUIZZ,
  UI_PAGE_PAIRING,
} UI_PageType;

#define UI_TASK_NOTIFY_SHOW_PAIRING           (1<<0)
#define UI_TASK_NOTIFY_SCREENSAVER_EXPIRED    (1<<1) /* timer for screensaver expired */
#define UI_TASK_NOTIFY_USER_BUTTON            (1<<2) /* user entered something */
#define UI_TASK_NOTIFY_UI_ACTIVATE            (1<<3) /* activate user interface */
#define UI_TASK_NOTIFY_PAGE_CLOSE             (1<<4) /* close application UI */
#define UI_TASK_NOTIFY_PAGE_SET_NEXT          (1<<5) /* Set next GUI page */

#define GUI_HAPTIC_TOUCH_MS         (30)
#define GUI_SCREEN_BLANK_TIMEOUT_MS (10*1000)

static struct {
  /* UI elements in home screen */
  struct {
    UIScreen_ScreenWidget screen;
    UI1_Element *currSelection;
    UIText_TextWidget textMenu1;
    UIText_TextWidget textMenu2;
  } home;
  /* UI page state */
  UI_PageType prevPage; /* used to switch back to the previous state */
  UI_PageType currentPage; /* current state/mode/UI */
  UI_PageType nextPage; /* next UI State */
  bool screenSaverOn; /* screen saving mode */
  bool hapticTouch; /* haptic feature turned on or off */
  /* information about the current GUI */
  UI1_Element *currentPageUIElement; /* element pointer of the current UI inside the screen */
  xTaskHandle currentUITask; /* handle of current UI task */
} UI_CurrState;

static xTimerHandle screenSaverTimerHndl = NULL;
static xTaskHandle UITaskHandle;

static uint32_t UI_pairingCode;

static void UI_BlankScreen(void) {
  if (!UI_CurrState.screenSaverOn) {
    UI_CurrState.screenSaverOn = TRUE;
    LCD1_Clear(); /* blank display */
  }
}

static void UI_ShowScreen(void) {
  xTimerReset(screenSaverTimerHndl, 0); /* reset timeout timer */
  UI1_MsgPaintAllElements(UI1_GetRoot());
  UI_CurrState.screenSaverOn = FALSE;
}

static void Haptic(void) {
  if (UI_CurrState.hapticTouch) {
    BUZ_Buzzer(GUI_HAPTIC_TOUCH_MS);
  }
}

/* UI events, created by KW40 (touch button messages, pairing, ...) */
void UI_Event(UI_EventType kind, void *data) {
  UI1_Element *element;

  switch(kind) {
    case UI_EVENT_BUTTON_LEFT:
    case UI_EVENT_BUTTON_RIGHT: /* enter */
    case UI_EVENT_BUTTON_UP:
    case UI_EVENT_BUTTON_DOWN:
      (void)xTaskNotify(UITaskHandle, UI_TASK_NOTIFY_USER_BUTTON, eSetBits);
      if (UI_CurrState.screenSaverOn) {
        return; /* ignore button press: get out of screensaver mode */
      }
      break;
    default:
      break;
  } /* switch */
  switch(kind) {
    case UI_EVENT_BUTTON_LEFT: /* lower left touch */
      element = UI1_GetSelectedElement();
      if (element!=NULL) {
        UI1_SendMessage(element, UI1_MSG_CLICK, NULL);
      } else { /* send message to root */
        UI1_SendMessage(UI1_GetRoot(), UI1_MSG_CLICK, NULL);
      }
      break;
    case UI_EVENT_BUTTON_RIGHT: /* lower right touch */
      element = UI1_GetSelectedElement();
      if (element!=NULL) {
        UI1_SendMessage(element, UI1_MSG_CANCEL, NULL);
      } else { /* send message to root */
        UI1_SendMessage(UI1_GetRoot(), UI1_MSG_CANCEL, NULL);
      }
      break;
    case UI_EVENT_BUTTON_UP:
      UI1_SelectNextElement(UI1_GetRoot(), FALSE);
      break;
    case UI_EVENT_BUTTON_DOWN:
      UI1_SelectNextElement(UI1_GetRoot(), TRUE);
      break;
    case UI_EVENT_PARING_CODE:
      UI_pairingCode = *((uint32_t*)data);
      (void)xTaskNotify(UITaskHandle, UI_TASK_NOTIFY_SHOW_PAIRING, eSetBits);
      break;
    default:
      break;
  } /* switch */
}

static void SwitchToUI(UI_PageType newUI) {
  /* switch to new UI */
  if (newUI==UI_PAGE_HOME) {
    if (UI_CurrState.currentUITask!=NULL) {
      (void)xTaskNotify(UI_CurrState.currentUITask, UI_NOTIFY_KILL_TASK, eSetBits);
      //UI1_RemoveElement(UI_CurrState.currentPageUIElement); /* remove current UI */
    }
    UI_CurrState.prevPage = UI_CurrState.currentPage;
    UI_CurrState.currentPage = newUI;
    UI1_SetRoot(&UI_CurrState.home.screen.element); /* set UI root */
    UI1_SetSelectedElement(UI_CurrState.home.currSelection); /* restore selection */
    UI1_SelectElement(UI_CurrState.home.currSelection);
    UI_ShowScreen(); /* show UI */
  }
  if (UI_CurrState.currentPage==UI_PAGE_HOME) {
    /* remember current selection */
    UI_CurrState.home.currSelection = UI1_GetSelectedElement();
  }
#if PL_CONFIG_HAS_QUIZZ
  if (newUI==UI_PAGE_QUIZZ) {
    if (UI_CurrState.currentUITask!=NULL) {
      (void)xTaskNotify(UI_CurrState.currentUITask, UI_NOTIFY_KILL_TASK, eSetBits);
      //UI1_RemoveElement(UI_CurrState.currentPageUIElement); /* remove current UI */
    }
    UI_CurrState.currentUITask = QUIZZ_CreateUITask(&UI_CurrState.currentPageUIElement);
    //(void)UI1_AddSubElement(&UI_CurrState.home.screen.element, UI_CurrState.currentPageUIElement); /* add new element */
    UI_CurrState.prevPage = UI_CurrState.currentPage;
    UI_CurrState.currentPage = newUI;
    UI1_SetRoot(UI_CurrState.currentPageUIElement); /* set UI root */
    UI_ShowScreen(); /* show UI */
  }
#endif
#if PL_CONFIG_HAS_PAIRING
  if (newUI==UI_PAGE_PAIRING) {
    if (UI_CurrState.currentUITask!=NULL) {
      (void)xTaskNotify(UI_CurrState.currentUITask, UI_NOTIFY_KILL_TASK, eSetBits);
      //UI1_RemoveElement(UI_CurrState.currentPageUIElement); /* remove current UI */
    }
    UI_CurrState.currentUITask = PAIRING_CreateUITask(&UI_CurrState.currentPageUIElement);
    //(void)UI1_AddSubElement(&UI_CurrState.home.screen.element, UI_CurrState.currentPageUIElement); /* add new element */
    UI_CurrState.prevPage = UI_CurrState.currentPage;
    UI_CurrState.currentPage = newUI;
    UI1_SetRoot(UI_CurrState.currentPageUIElement); /* set UI root */
    UI_ShowScreen(); /* show UI */
  }
#endif
}

static void guiCallback(UI1_Element *element, UI1_MsgKind kind, void *pData) {
  (void)pData; /* unused argument */
  if (kind==UI1_MSG_CLICK) {
    if (UI1_EqualElement(element, &UI_CurrState.home.textMenu1.element)) {
      UI_CurrState.nextPage = UI_PAGE_QUIZZ;
      (void)xTaskNotify(UITaskHandle, UI_TASK_NOTIFY_PAGE_SET_NEXT, eSetBits);
    } else if (UI1_EqualElement(element, &UI_CurrState.home.textMenu2.element)) {
      UI_CurrState.nextPage = UI_PAGE_PAIRING;
      (void)xTaskNotify(UITaskHandle, UI_TASK_NOTIFY_PAGE_SET_NEXT, eSetBits);
    }
  } /* if click */
}

static void CreateHomeScreen(void) {
  UI1_PixelDim x, y;

  /* screen */
  (void)UIScreen_Create(NULL, &UI_CurrState.home.screen, 0, 0, UI1_GetWidth(), UI1_GetHeight());
  UIScreen_SetBackgroundColor(&UI_CurrState.home.screen, GDisp1_COLOR_BRIGHT_GREY);
  UIScreen_SetUserMsgHandler(&UI_CurrState.home.screen, guiCallback); /* set custom message handler */

  /* menu text */
  x = 0; y = 0;
  UIText_Create((UI1_Element*)&UI_CurrState.home.screen, &UI_CurrState.home.textMenu1, x, y, 0, 0);
  UIText_SetText(&UI_CurrState.home.textMenu1, (unsigned char*)"Quizz");
  UIText_SetBackgroundColor(&UI_CurrState.home.textMenu1, UI_CurrState.home.screen.bgColor);
  UIText_Resize(&UI_CurrState.home.textMenu1);
  UI1_EnableElementSelection(&UI_CurrState.home.textMenu1.element);
  UIText_SetUserMsgHandler(&UI_CurrState.home.textMenu1, guiCallback);
  UI_CurrState.home.currSelection = &UI_CurrState.home.textMenu1.element; /* by default, have first element selected */

  x = 0; y = UI1_GetElementPosBottom(&UI_CurrState.home.textMenu1.element);
  UIText_Create((UI1_Element*)&UI_CurrState.home.screen, &UI_CurrState.home.textMenu2, x, y, 0, 0);
  UIText_SetText(&UI_CurrState.home.textMenu2, (unsigned char*)"Pairing Code");
  UIText_SetBackgroundColor(&UI_CurrState.home.textMenu2, UI_CurrState.home.screen.bgColor);
  UIText_Resize(&UI_CurrState.home.textMenu2);
  UI1_EnableElementSelection(&UI_CurrState.home.textMenu2.element);
  UIText_SetUserMsgHandler(&UI_CurrState.home.textMenu2, guiCallback);
}

static void UITask(void *pvParameters) {
  uint32_t notifcationValue;
  BaseType_t notified;

  LCD1_Init();
  LCD1_Clear();

  /* init */
  UI_pairingCode = 0;

  CreateHomeScreen();
  UI_CurrState.prevPage = UI_PAGE_NONE;
  UI_CurrState.currentPage = UI_PAGE_NONE;
  UI_CurrState.nextPage = UI_PAGE_NONE;
  UI_CurrState.currentUITask = NULL;
  UI_CurrState.hapticTouch = FALSE;
  UI_CurrState.screenSaverOn = FALSE;

  /* show initial UI */
  SwitchToUI(UI_PAGE_HOME);
  for(;;) {
    notified = xTaskNotifyWait(0UL,
        ( UI_TASK_NOTIFY_SCREENSAVER_EXPIRED
         |UI_TASK_NOTIFY_USER_BUTTON
         |UI_TASK_NOTIFY_UI_ACTIVATE
         |UI_TASK_NOTIFY_SHOW_PAIRING
         |UI_TASK_NOTIFY_PAGE_CLOSE
         |UI_TASK_NOTIFY_PAGE_SET_NEXT
         ),
        &notifcationValue, 1); /* check flags, need to wait for one tick */
    if (notified==pdTRUE) { /* received notification */
      if (notifcationValue&UI_TASK_NOTIFY_SCREENSAVER_EXPIRED) {
        UI_BlankScreen(); /* blank screen */
      }
      if (notifcationValue&UI_TASK_NOTIFY_UI_ACTIVATE) {
      }
      if (notifcationValue&UI_TASK_NOTIFY_USER_BUTTON) {
        Haptic();
        xTimerReset(screenSaverTimerHndl, 0); /* reset screensaver timeout timer */
        if (UI_CurrState.screenSaverOn) {
          UI_ShowScreen();
        }
      }
      if (notifcationValue&UI_TASK_NOTIFY_SHOW_PAIRING) {
        SwitchToUI(UI_PAGE_PAIRING);
      }
      if (notifcationValue&UI_TASK_NOTIFY_PAGE_CLOSE) {
        SwitchToUI(UI_PAGE_HOME);
      }
      if (notifcationValue&UI_TASK_NOTIFY_PAGE_SET_NEXT) {
        SwitchToUI(UI_CurrState.nextPage);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  } /* for */
}

static void vTimerCallback(xTimerHandle pxTimer) {
  /* GUI_SCREEN_BLANK_TIMEOUT_MS ms timer */
  (void)xTaskNotify(UITaskHandle, UI_TASK_NOTIFY_SCREENSAVER_EXPIRED, eSetBits);
}

void UI_CloseAppUI(void) {
  (void)xTaskNotify(UITaskHandle, UI_TASK_NOTIFY_PAGE_CLOSE, eSetBits);
}

void UI_Init(void) {
  if (xTaskCreate(UITask, "UI", configMINIMAL_STACK_SIZE+80, NULL, tskIDLE_PRIORITY, &UITaskHandle)!=pdPASS) {
    for(;;) {} /* out of memory? */
  }
  screenSaverTimerHndl = xTimerCreate("gui_timer", GUI_SCREEN_BLANK_TIMEOUT_MS/portTICK_RATE_MS, pdFALSE, (void *)0, vTimerCallback);
  if (screenSaverTimerHndl==NULL) {
    for(;;); /* failure! */
  }
  if (xTimerStart(screenSaverTimerHndl, 0)!=pdPASS) {
    for(;;); /* failure! */
  }
}

#endif /* PL_CONFIG_HAS_UI */


