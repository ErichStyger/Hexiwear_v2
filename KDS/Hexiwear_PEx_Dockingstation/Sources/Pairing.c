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
#include "Helv12n.h"
#include "UTIL1.h"

typedef struct {
  UIScreen_ScreenWidget screen;
  UIWindow_WindowWidget window;
  UIHeader_HeaderWidget header;
  /* pairing code text */
  UIText_TextWidget textParingCode;
  /* navigation */
  UIIcon_IconWidget iconNavigationEnter;
  UIIcon_IconWidget iconNavigationExit;
} PAIRING_GUIDesc;

static PAIRING_GUIDesc *PAIRING_Gui;
static xTaskHandle PairingTaskHandle;
static uint8_t pairingTxt[sizeof("12345678")]; /* UI is using this text buffer */

void PAIRING_SetPairingCode(uint32_t code) {
  UTIL1_Num32uToStr(pairingTxt, sizeof(pairingTxt), code);
}

static void guiCallback(UI1_Element *element, UI1_MsgKind kind, void *pData) {
  (void)pData; /* unused argument */
  if (kind==UI1_MSG_CLICK || kind==UI1_MSG_CANCEL) {
    UI_CloseAppUI();
  } /* if click */
}

static void PAIRING_CreateGUI(PAIRING_GUIDesc *gui) {
  UI1_PixelDim x, y, h;

  /* screen */
  (void)UIScreen_Create(NULL, &gui->screen, 0, 0, UI1_GetWidth(), UI1_GetHeight());
  UIScreen_SetBackgroundColor(&gui->screen, GDisp1_COLOR_BRIGHT_YELLOW);
  UIScreen_SetUserMsgHandler(&gui->screen, guiCallback);

  /* Navigation Icons */
  (void)UIIcon_Create(&gui->screen.element, &gui->iconNavigationEnter,
      15, UI1_GetHeight()-10, 10, 10);
  UIIcon_SetType(&gui->iconNavigationEnter, UIIcon_ICON_CHECKMARK);
  UIIcon_SetForegroundColor(&gui->iconNavigationEnter, UI1_COLOR_BLACK);
  UIIcon_SetBackgroundColor(&gui->iconNavigationEnter, gui->screen.bgColor);
  UIIcon_SetInsideColor(&gui->iconNavigationEnter, UI1_COLOR_WHITE);

  (void)UIIcon_Create(&gui->screen.element, &gui->iconNavigationExit,
      UI1_GetWidth()-30, UI1_GetHeight()-10, 10, 10);
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
  h = UI1_GetElementHeight(&gui->header.element);

  /* pairing code text */
  x = 0;
  y = UI1_GetElementHeight(&gui->header.element)+10;
  UIText_Create((UI1_Element*)&gui->window, &gui->textParingCode, x, y, 0, 0);
  UIText_SetText(&gui->textParingCode, pairingTxt);
  UIText_SetBackgroundColor(&gui->textParingCode, gui->window.bgColor);
  UIText_SetFont(&gui->textParingCode, Helv12n_GetFont());
  UIText_Resize(&gui->textParingCode);
}

static void PairingTask(void *pvParameters) {
  uint32_t notifcationValue;
  BaseType_t notified;

  for(;;) {
    notified = xTaskNotifyWait(0UL, UI_NOTIFY_KILL_TASK, &notifcationValue, 1); /* check flags, need to wait for one tick */
    if (notified==pdTRUE) { /* received notification */
      if (notifcationValue&UI_NOTIFY_KILL_TASK) {
        vPortFree(PAIRING_Gui);
        PAIRING_Gui = NULL;
        vTaskDelete(NULL); /* killing myself */
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  } /* for */
}

xTaskHandle PAIRING_CreateUITask(UI1_Element **root) {
  PAIRING_Gui = pvPortMalloc(sizeof(PAIRING_GUIDesc));
  if (PAIRING_Gui==NULL) {
    for(;;) {}
  }
  PAIRING_CreateGUI(PAIRING_Gui);
  if (xTaskCreate(PairingTask, "Pairing", configMINIMAL_STACK_SIZE+80, NULL, tskIDLE_PRIORITY+3, &PairingTaskHandle)!=pdPASS) {
    for(;;) {} /* out of memory? */
  }
  *root = &PAIRING_Gui->screen.element;
  return PairingTaskHandle;
}

void PAIRING_KillTask(void) {
  (void)xTaskNotify(PairingTaskHandle, UI_NOTIFY_KILL_TASK, eSetBits);
}

void PAIRING_Init(void) {
  PAIRING_Gui = NULL;
  PairingTaskHandle = NULL;
  PAIRING_SetPairingCode(0);
}

#endif /* PL_CONFIG_HAS_PAIRING */



