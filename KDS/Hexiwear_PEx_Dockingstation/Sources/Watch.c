/*
 * Watch.c
 *
 *  Created on: 05.03.2017
 *      Author: Erich Styger Local
 */

#include "Platform.h"
#if PL_CONFIG_HAS_WATCH
#include "Watch.h"
#include "UI.h"
#include "UI1.h"
#include "UIIcon.h"
#include "UIText.h"
#include "UIWindow.h"
#include "Helv12n.h"
#include "UTIL1.h"
#include "TmDt1.h"
#include "HostComm.h"

typedef struct {
  UIWindow_WindowWidget window;
  /* date */
  UIText_TextWidget textDate;
  uint8_t dateBuf[sizeof(TmDt1_DEFAULT_DATE_FORMAT_STR)];
  /* date */
  UIText_TextWidget textTime;
  uint8_t timeBuf[sizeof(TmDt1_DEFAULT_TIME_FORMAT_STR)];
  /* navigation */
  UIIcon_IconWidget iconNavigationEnter;
  UIIcon_IconWidget iconNavigationExit;
} WatchGUIDesc;

static WatchGUIDesc *WatchGui;
static xTaskHandle WatchTaskHandle;

static void guiCallback(UI1_Element *element, UI1_MsgKind kind, void *pData) {
  (void)pData; /* unused argument */
  if (kind==UI1_MSG_CLICK || kind==UI1_MSG_CANCEL) {
    UI_CloseAppUI();
  } /* if click */
}

static void WatchCreateGUI(WatchGUIDesc *gui) {
  UI1_PixelDim x, y, h;
  DATEREC date;
  TIMEREC time;

  /* window */
  (void)UIWindow_Create(NULL, &gui->window, 0, 0, UI1_GetWidth(), UI1_GetHeight());
  UIWindow_SetBackgroundColor(&gui->window, GDisp1_COLOR_RED);
  UIWindow_SetUserMsgHandler(&gui->window, guiCallback);

  (void)TmDt1_GetTimeDate(&time, &date);
  /* date */
  x = 0; y = 10;
  UIText_Create((UI1_Element*)&gui->window, &gui->textDate, x, y, 0, 0);
  gui->dateBuf[0] = '\0';
  TmDt1_AddDateString(gui->dateBuf, sizeof(gui->dateBuf), &date, TmDt1_DEFAULT_DATE_FORMAT_STR);
  UIText_SetText(&gui->textDate, gui->dateBuf);
  UIText_SetBackgroundColor(&gui->textDate, gui->window.bgColor);
  UIText_SetFont(&gui->textDate, Helv12n_GetFont());
  UIText_Resize(&gui->textDate);

  /* time */
  x = 0; y = UI1_GetElementPosBottom(&gui->textDate.element);
  UIText_Create((UI1_Element*)&gui->window, &gui->textTime, x, y, 0, 0);
  gui->timeBuf[0] = '\0';
  TmDt1_AddTimeString(gui->timeBuf, sizeof(gui->timeBuf), &time, TmDt1_DEFAULT_TIME_FORMAT_STR);
  UIText_SetText(&gui->textTime, gui->timeBuf);
  UIText_SetBackgroundColor(&gui->textTime, gui->window.bgColor);
  UIText_SetFont(&gui->textTime, Helv12n_GetFont());
  UIText_Resize(&gui->textTime);

  /* Navigation Icons */
  (void)UIIcon_Create(&gui->window.element, &gui->iconNavigationEnter,
      15, UI1_GetHeight()-10, 10, 10);
  UIIcon_SetType(&gui->iconNavigationEnter, UIIcon_ICON_CHECKMARK);
  UIIcon_SetForegroundColor(&gui->iconNavigationEnter, UI1_COLOR_BLACK);
  UIIcon_SetBackgroundColor(&gui->iconNavigationEnter, gui->window.bgColor);
  UIIcon_SetInsideColor(&gui->iconNavigationEnter, UI1_COLOR_WHITE);

  (void)UIIcon_Create(&gui->window.element, &gui->iconNavigationExit,
      UI1_GetWidth()-30, UI1_GetHeight()-10, 10, 10);
  UIIcon_SetType(&gui->iconNavigationExit, UIIcon_ICON_X);
  UIIcon_SetForegroundColor(&gui->iconNavigationExit, UI1_COLOR_BLACK);
  UIIcon_SetBackgroundColor(&gui->iconNavigationExit, gui->window.bgColor);
  UIIcon_SetInsideColor(&gui->iconNavigationExit, UI1_COLOR_WHITE);
}

static void WatchTask(void *pvParameters) {
  uint32_t notifcationValue;
  BaseType_t notified;
  TIMEREC time;
  DATEREC date;
  uint8_t newTimeText[sizeof(TmDt1_DEFAULT_TIME_FORMAT_STR)];

  //watch_SendGetLinkStateReq();
  for(;;) {
    notified = xTaskNotifyWait(0UL, UI_NOTIFY_KILL_TASK, &notifcationValue, 1); /* check flags, need to wait for one tick */
    if (notified==pdTRUE) { /* received notification */
      if (notifcationValue&UI_NOTIFY_KILL_TASK) {
        vPortFree(WatchGui);
        WatchGui = NULL;
        vTaskDelete(NULL); /* killing myself */
      }
    }
    if (TmDt1_GetTimeDate(&time, &date)==ERR_OK) {
      newTimeText[0] = '\0';
      (void)TmDt1_AddTimeString(newTimeText, sizeof(newTimeText), &time, TmDt1_DEFAULT_TIME_FORMAT_STR);
      UI1_GetUI();
      (void)UIText_ChangeText(&WatchGui->textTime, sizeof(WatchGui->timeBuf), newTimeText);
      UI1_GiveUI();
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  } /* for */
}


xTaskHandle WATCH_CreateUITask(UI1_Element **root) {
  WatchGui = pvPortMalloc(sizeof(WatchGUIDesc));
  if (WatchGui==NULL) {
    for(;;) {}
  }
  WatchCreateGUI(WatchGui);
  if (xTaskCreate(WatchTask, "Watch", configMINIMAL_STACK_SIZE+80, NULL, tskIDLE_PRIORITY+3, &WatchTaskHandle)!=pdPASS) {
    for(;;) {} /* out of memory? */
  }
  *root = &WatchGui->window.element;
  return WatchTaskHandle;
}

void WATCH_KillTask(void) {
  (void)xTaskNotify(WatchTaskHandle, UI_NOTIFY_KILL_TASK, eSetBits);
}

void WATCH_Init(void) {
  WatchGui = NULL;
  WatchTaskHandle = NULL;
}

#endif /* PL_CONFIG_HAS_WATCH */

