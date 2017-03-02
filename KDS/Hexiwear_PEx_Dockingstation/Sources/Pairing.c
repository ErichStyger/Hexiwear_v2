/*
 * Pairing.c
 *
 *  Created on: 01.03.2017
 *      Author: Erich Styger Local
 */

#include "Platform.h"

#if PL_CONFIG_HAS_PAIRING
#include "Pairing.h"
#include "UI.h"
#include "UI1.h"
#include "UIScreen.h"
#include "UIWindow.h"
#include "UIHeader.h"
#include "GDisp1.h"
#include "FRTOS1.h"
#include "Helv08n.h"

#define FONT   Helv08n_GetFont()

typedef struct {
  UIScreen_ScreenWidget screen;
  UIWindow_WindowWidget window;
  UIHeader_HeaderWidget header;

  UIText_TextWidget textParingCode;

  /* navigation */
  UIIcon_IconWidget iconNavigationEnter;
  UIIcon_IconWidget iconNavigationExit;
} PAIRING_GUIDesc;

static PAIRING_GUIDesc *PAIRING_Gui;

/* task notification bits */
#define PAIRING_KILL_TASK            (1<<0)  /* close window and kill task */
#define PAIRING_USER_BUTTON_PRESSED  (1<<1)  /* user has pressed button */
static xTaskHandle PairingTaskHandle;

static void guiCallback(UI1_Element *element, UI1_MsgKind kind, void *pData) {
  (void)pData; /* unused argument */
  if (kind==UI1_MSG_CLICK) {
    if (UI1_EqualElement(element, &PAIRING_Gui->header.iconWidget.element)) {
      PAIRING_KillTask();
      return;
    }
  } /* if click */
}

static void PAIRING_CreateGUI(PAIRING_GUIDesc *gui) {
  UI1_PixelDim x, y, h;

  /* screen */
  (void)UIScreen_Create(NULL, &gui->screen, 0, 0, UI1_GetWidth(), UI1_GetHeight());
  UIScreen_SetBackgroundColor(&gui->screen, GDisp1_COLOR_BRIGHT_YELLOW);

  /* Navigation Icons */
  (void)UIIcon_Create(&gui->screen.element, &gui->iconNavigationEnter,
      UI1_GetWidth()-30, UI1_GetHeight()-10, 10, 10);
  UIIcon_SetType(&gui->iconNavigationEnter, UIIcon_ICON_CHECKMARK);
  UIIcon_SetForegroundColor(&gui->iconNavigationEnter, UI1_COLOR_BLACK);
  UIIcon_SetBackgroundColor(&gui->iconNavigationEnter, gui->screen.bgColor);
  UIIcon_SetInsideColor(&gui->iconNavigationEnter, UI1_COLOR_WHITE);

  (void)UIIcon_Create(&gui->screen.element, &gui->iconNavigationExit,
      15, UI1_GetHeight()-10, 10, 10);
  UIIcon_SetType(&gui->iconNavigationExit, UIIcon_ICON_X);
  UIIcon_SetForegroundColor(&gui->iconNavigationExit, UI1_COLOR_BLACK);
  UIIcon_SetBackgroundColor(&gui->iconNavigationExit, gui->screen.bgColor);
  UIIcon_SetInsideColor(&gui->iconNavigationExit, UI1_COLOR_WHITE);

  /* window */
  (void)UIWindow_Create(&gui->screen.element, &gui->window,
      0, 0, GDisp1_GetWidth()-10, GDisp1_GetHeight()-10);
  UIWindow_SetBackgroundColor(&gui->window, UI1_COLOR_BRIGHT_GREEN);
  UIWindow_SetBorder(&gui->window);

  /* header */
  (void)UIHeader_Create(&gui->window.element, &gui->header,
      0, 0, /* for window border */
      0, 0 /* auto-size */
     );
  UIHeader_SetBackgroundColor(&gui->header, UI1_COLOR_BRIGHT_BLUE);
  UIHeader_SetForegroundColor(&gui->header, UI1_COLOR_BLACK);
  UIHeader_SetText(&gui->header, (uint8_t*)"Pairing Code");
  UIHeader_Resize(&gui->header); /* adjust size */
  UIIcon_SetUserMsgHandler(&gui->header.iconWidget, guiCallback);
  UI1_EnableElementSelection(&gui->header.iconWidget.element);

  h = 0;

  h += UI1_GetElementHeight(&gui->header.element);

  /* pairing code text */
  x = 0;
  y = UI1_GetElementHeight(&gui->header.element)+10;
  UIText_Create((UI1_Element*)&gui->window, &gui->textParingCode, x, y, 0, 0);
  UIText_SetText(&gui->textParingCode, (unsigned char*)"123456");
  UIText_SetBackgroundColor(&gui->textParingCode, gui->window.bgColor);
  UIText_Resize(&gui->textParingCode);


  /* update the screen */
  UI1_MsgPaintAllElements((UI1_Element*)&gui->screen);
  /* assign root element */
  UI1_SetRoot(&gui->screen.element);
}

static void PairingTask(void *pvParameters) {
  uint32_t notifcationValue;
  BaseType_t notified;
  TickType_t startTicks, ticks;
  const int timeoutSecs = 60;

  PAIRING_Gui = pvPortMalloc(sizeof(PAIRING_GUIDesc));
  if (PAIRING_Gui==NULL) {
    for(;;) {}
  }
  PAIRING_CreateGUI(PAIRING_Gui);
  startTicks = xTaskGetTickCount();
  for(;;) {
    ticks = xTaskGetTickCount();
    if (((ticks-startTicks)/portTICK_RATE_MS)>(timeoutSecs*1000)) {
      PAIRING_KillTask();
    }
    notified = xTaskNotifyWait(0UL, PAIRING_KILL_TASK, &notifcationValue, 1); /* check flags, need to wait for one tick */
    if (notified==pdTRUE) { /* received notification */
#if 1
      if (notifcationValue&PAIRING_KILL_TASK) {
        LCD1_Clear();//GDisp1_Clear();
        vPortFree(PAIRING_Gui);
        PAIRING_Gui = NULL;
        UI_SetCurrentUITask(NULL);
        vTaskDelete(NULL); /* killing myself */
      }
      if (notifcationValue&PAIRING_USER_BUTTON_PRESSED) {
        startTicks = xTaskGetTickCount(); /* restart timeout */
      }
#endif
    }
    vTaskDelay(pdMS_TO_TICKS(80)); /* give user a chance to see the cube rotating... */
  } /* for */
}

void PAIRING_CreateTask(void) {
  PAIRING_Gui = NULL;
  PairingTaskHandle = NULL;
  if (xTaskCreate(PairingTask, "Pairing", configMINIMAL_STACK_SIZE+80, NULL, tskIDLE_PRIORITY+3, &PairingTaskHandle)!=pdPASS) {
    for(;;) {} /* out of memory? */
  }
  UI_SetCurrentUITask(PairingTaskHandle);
}

void PAIRING_KillTask(void) {
  (void)xTaskNotify(PairingTaskHandle, PAIRING_KILL_TASK, eSetBits);
}

void PAIRING_Init(void) {
  PAIRING_Gui = NULL;
  PairingTaskHandle = NULL;
}

#endif /* PL_CONFIG_HAS_PAIRING */



