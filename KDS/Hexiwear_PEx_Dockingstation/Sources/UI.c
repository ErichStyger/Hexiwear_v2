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
#include "UIIcon.h"
#if PL_CONFIG_HAS_QUIZZ
  #include "Quizz.h"
#endif
#if PL_CONFIG_HAS_PAIRING
  #include "Pairing.h"
#endif
#if PL_CONFIG_HAS_WATCH
  #include "Watch.h"
#endif
#include "Helv08n.h"

typedef enum {
  UI_PAGE_NONE,
  UI_PAGE_SCREENSAVER,
  UI_PAGE_HOME,
  UI_PAGE_QUIZZ,
  UI_PAGE_PAIRING,
  UI_PAGE_WATCH,
} UI_PageType;

#define UI_TASK_NOTIFY_SHOW_PAIRING           (1<<0)
#define UI_TASK_NOTIFY_SCREENSAVER_EXPIRED    (1<<1) /* timer for screensaver expired */
#define UI_TASK_NOTIFY_USER_BUTTON            (1<<2) /* user entered something */
#define UI_TASK_NOTIFY_UI_ACTIVATE            (1<<3) /* activate user interface */
#define UI_TASK_NOTIFY_PAGE_CLOSE             (1<<4) /* close application UI */
#define UI_TASK_NOTIFY_PAGE_SET_NEXT          (1<<5) /* Set next GUI page */

#define GUI_HAPTIC_TOUCH_MS         (30)
#define GUI_SCREEN_BLANK_TIMEOUT_MS (30*1000)

static struct {
  /* UI elements in home screen */
  struct {
    UIScreen_ScreenWidget screen;
    UI1_Element *currSelection;
    UIText_TextWidget textMenu1; /* Quizz */
    UIText_TextWidget textMenu2; /* Pairing */
    UIText_TextWidget textMenu3; /* watch */
    /* navigation */
    UIIcon_IconWidget iconNavigationUp;
    UIIcon_IconWidget iconNavigationDown;
    UIIcon_IconWidget iconNavigationEnter;
  } home;
  /* UI page state */
  UI_PageType prevPage; /* used to switch back to the previous state */
  UI_PageType currentPage; /* current state/mode/UI */
  UI_PageType nextPage; /* next UI State */
  UI_PageType prevScreenSaverPage; /* previous page before screensaver */
  bool screenSaverOn; /* screen saving mode */
  bool hapticTouch; /* haptic feature turned on or off */
  /* information about the current GUI */
  UI1_Element *currentPageUIElement; /* element pointer of the current UI inside the screen */
  xTaskHandle currentUITask; /* handle of current UI task */
} UI_CurrState;

static xTimerHandle screenSaverTimerHndl = NULL;
static xTaskHandle UITaskHandle;

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
      UI1_GetUI();
      element = UI1_GetSelectedElement();
      if (element!=NULL) {
        UI1_SendMessage(element, UI1_MSG_CLICK, NULL);
      } else { /* send message to root */
        UI1_SendMessage(UI1_GetRoot(), UI1_MSG_CLICK, NULL);
      }
      UI1_GiveUI();
      break;
    case UI_EVENT_BUTTON_RIGHT: /* lower right touch */
      UI1_GetUI();
      element = UI1_GetSelectedElement();
      if (element!=NULL) {
        UI1_SendMessage(element, UI1_MSG_CANCEL, NULL);
      } else { /* send message to root */
        UI1_SendMessage(UI1_GetRoot(), UI1_MSG_CANCEL, NULL);
      }
      UI1_GiveUI();
      break;
    case UI_EVENT_BUTTON_UP:
      UI1_GetUI();
      UI1_SelectNextElement(UI1_GetRoot(), FALSE);
      UI1_GiveUI();
      break;
    case UI_EVENT_BUTTON_DOWN:
      UI1_GetUI();
      UI1_SelectNextElement(UI1_GetRoot(), TRUE);
      UI1_GiveUI();
      break;
    case UI_EVENT_PARING_CODE:
      PAIRING_SetPairingCode(*((uint32_t*)data));
      (void)xTaskNotify(UITaskHandle, UI_TASK_NOTIFY_SHOW_PAIRING, eSetBits);
      break;
    default:
      break;
  } /* switch */
}

static void UI_ShowScreen(void) {
  xTimerReset(screenSaverTimerHndl, 0); /* reset timeout timer */
  UI1_GetUI();
  UI1_MsgPaintAllElements(UI1_GetRoot());
  UI1_GiveUI();
  UI_CurrState.screenSaverOn = FALSE;
}

static void SwitchToUI(UI_PageType newUI) {
  /* switch to new UI */
  if (newUI==UI_PAGE_SCREENSAVER) { /* going into screensaver mode */
    if (UI_CurrState.currentUITask!=NULL) {
      vTaskSuspend(UI_CurrState.currentUITask);
      //(void)xTaskNotify(UI_CurrState.currentUITask, UI_NOTIFY_SUSPEND_TASK, eSetBits);
    }
    if (!UI_CurrState.screenSaverOn) {
      UI_CurrState.screenSaverOn = TRUE;
      UI_CurrState.prevScreenSaverPage = UI_CurrState.currentPage;
      UI1_GetUI();
      LCD1_Clear(); /* blank display */
      UI1_GiveUI();
    }
    return;
  } else if (UI_CurrState.screenSaverOn) { /* exit screensaver mode */
    UI_ShowScreen(); /* show UI */
    UI_CurrState.prevScreenSaverPage = UI_PAGE_NONE;
    if (UI_CurrState.currentUITask!=NULL) {
      vTaskResume(UI_CurrState.currentUITask);
    }
    return;
  }

  if (newUI==UI_PAGE_HOME) {
    if (UI_CurrState.currentUITask!=NULL) {
      (void)xTaskNotify(UI_CurrState.currentUITask, UI_NOTIFY_KILL_TASK, eSetBits);
    }
    UI1_GetUI();
    UI_CurrState.prevPage = UI_CurrState.currentPage;
    UI_CurrState.currentPage = newUI;
    UI1_SetRoot(&UI_CurrState.home.screen.element); /* set UI root */
    UI1_SetSelectedElement(UI_CurrState.home.currSelection); /* restore selection */
    UI1_SelectElement(UI_CurrState.home.currSelection);
    UI1_GiveUI();
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
    }
    UI1_GetUI();
    UI_CurrState.currentUITask = QUIZZ_CreateUITask(&UI_CurrState.currentPageUIElement);
    UI_CurrState.prevPage = UI_CurrState.currentPage;
    UI_CurrState.currentPage = newUI;
    UI1_SetRoot(UI_CurrState.currentPageUIElement); /* set UI root */
    UI1_GiveUI();
    UI_ShowScreen(); /* show UI */
  }
#endif
#if PL_CONFIG_HAS_PAIRING
  if (newUI==UI_PAGE_PAIRING) {
    if (UI_CurrState.currentUITask!=NULL) {
      (void)xTaskNotify(UI_CurrState.currentUITask, UI_NOTIFY_KILL_TASK, eSetBits);
    }
    UI1_GetUI();
    UI_CurrState.currentUITask = PAIRING_CreateUITask(&UI_CurrState.currentPageUIElement);
    UI_CurrState.prevPage = UI_CurrState.currentPage;
    UI_CurrState.currentPage = newUI;
    UI1_SetRoot(UI_CurrState.currentPageUIElement); /* set UI root */
    UI1_GiveUI();
    UI_ShowScreen(); /* show UI */
  }
#endif
#if PL_CONFIG_HAS_WATCH
  if (newUI==UI_PAGE_WATCH) {
    if (UI_CurrState.currentUITask!=NULL) {
      (void)xTaskNotify(UI_CurrState.currentUITask, UI_NOTIFY_KILL_TASK, eSetBits);
    }
    UI1_GetUI();
    UI_CurrState.currentUITask = WATCH_CreateUITask(&UI_CurrState.currentPageUIElement);
    UI_CurrState.prevPage = UI_CurrState.currentPage;
    UI_CurrState.currentPage = newUI;
    UI1_SetRoot(UI_CurrState.currentPageUIElement); /* set UI root */
    UI1_GiveUI();
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
    } else if (UI1_EqualElement(element, &UI_CurrState.home.textMenu3.element)) {
      UI_CurrState.nextPage = UI_PAGE_WATCH;
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

  x = 0; y = UI1_GetElementPosBottom(&UI_CurrState.home.textMenu2.element);
  UIText_Create((UI1_Element*)&UI_CurrState.home.screen, &UI_CurrState.home.textMenu3, x, y, 0, 0);
  UIText_SetText(&UI_CurrState.home.textMenu3, (unsigned char*)"Watch");
  UIText_SetBackgroundColor(&UI_CurrState.home.textMenu3, UI_CurrState.home.screen.bgColor);
  UIText_Resize(&UI_CurrState.home.textMenu3);
  UI1_EnableElementSelection(&UI_CurrState.home.textMenu3.element);
  UIText_SetUserMsgHandler(&UI_CurrState.home.textMenu3, guiCallback);

  /* Navigation Icons */
  (void)UIIcon_Create(&UI_CurrState.home.screen.element, &UI_CurrState.home.iconNavigationEnter,
      15, UI1_GetElementHeight(&UI_CurrState.home.screen.element)-10, 10, 10);
  UIIcon_SetType(&UI_CurrState.home.iconNavigationEnter, UIIcon_ICON_CHECKMARK);
  UIIcon_SetForegroundColor(&UI_CurrState.home.iconNavigationEnter, UI1_COLOR_BLACK);
  UIIcon_SetBackgroundColor(&UI_CurrState.home.iconNavigationEnter, UI_CurrState.home.screen.bgColor);
  UIIcon_SetInsideColor(&UI_CurrState.home.iconNavigationEnter, UI1_COLOR_WHITE);

  (void)UIIcon_Create(&UI_CurrState.home.screen.element, &UI_CurrState.home.iconNavigationUp,
      UI1_GetElementWidth(&UI_CurrState.home.screen.element)-10, 20, 10, 10);
  UIIcon_SetType(&UI_CurrState.home.iconNavigationUp, UIIcon_ICON_ARROW_UP);
  UIIcon_SetForegroundColor(&UI_CurrState.home.iconNavigationUp, UI1_COLOR_BLACK);
  UIIcon_SetBackgroundColor(&UI_CurrState.home.iconNavigationUp, UI_CurrState.home.screen.bgColor);
  UIIcon_SetInsideColor(&UI_CurrState.home.iconNavigationUp, UI1_COLOR_WHITE);

  (void)UIIcon_Create(&UI_CurrState.home.screen.element, &UI_CurrState.home.iconNavigationDown,
      UI1_GetElementWidth(&UI_CurrState.home.screen.element)-10, 65, 10, 10);
  UIIcon_SetType(&UI_CurrState.home.iconNavigationDown, UIIcon_ICON_ARROW_DOWN);
  UIIcon_SetForegroundColor(&UI_CurrState.home.iconNavigationDown, UI1_COLOR_BLACK);
  UIIcon_SetBackgroundColor(&UI_CurrState.home.iconNavigationDown, UI_CurrState.home.screen.bgColor);
  UIIcon_SetInsideColor(&UI_CurrState.home.iconNavigationDown, UI1_COLOR_WHITE);
}

static void UITask(void *pvParameters) {
  uint32_t notifcationValue;
  BaseType_t notified;

  LCD1_Init(); /* initialize display */
  LCD1_Clear(); /* clear display */

#if 1 /* display test code */
  {
    GDisp1_PixelDim x, y;

    LCD1_SetDisplayOrientation(LCD1_ORIENTATION_PORTRAIT);
    GDisp1_DrawFilledBox(0, 0, 80, 20, LCD1_COLOR_YELLOW);
    x = 0; y = 0;
    FDisp1_WriteString("Portrait", LCD1_COLOR_RED, &x, &y, Helv08n_GetFont());

    LCD1_SetDisplayOrientation(LCD1_ORIENTATION_LANDSCAPE);
    GDisp1_DrawFilledBox(0, 0, 80, 20, LCD1_COLOR_ORANGE);
    x = 0; y = 0;
    FDisp1_WriteString("Landscape", LCD1_COLOR_GREEN, &x, &y, Helv08n_GetFont());

    LCD1_SetDisplayOrientation(LCD1_ORIENTATION_PORTRAIT180);
    GDisp1_DrawFilledBox(0, 0, 80, 20, LCD1_COLOR_RED);
    x = 0; y = 0;
    FDisp1_WriteString("Portrait180", LCD1_COLOR_BLUE, &x, &y, Helv08n_GetFont());

    LCD1_SetDisplayOrientation(LCD1_ORIENTATION_LANDSCAPE180);
    GDisp1_DrawFilledBox(0, 0, 80, 20, LCD1_COLOR_BRIGHT_GREEN);
    x = 0; y = 0;
    FDisp1_WriteString("Landscape180", LCD1_COLOR_WHITE, &x, &y, Helv08n_GetFont());

    LCD1_SetDisplayOrientation(LCD1_ORIENTATION_PORTRAIT);
}
#endif

  /* init */
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
        SwitchToUI(UI_PAGE_SCREENSAVER);
      }
      if (notifcationValue&UI_TASK_NOTIFY_UI_ACTIVATE) {
      }
      if (notifcationValue&UI_TASK_NOTIFY_USER_BUTTON) {
        Haptic();
        xTimerReset(screenSaverTimerHndl, 0); /* reset screensaver timeout timer */
        if (UI_CurrState.screenSaverOn) {
          SwitchToUI(UI_CurrState.prevScreenSaverPage);
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


