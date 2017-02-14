/** ###################################################################
**     Filename  : Cube.C
**     Project   : Tower_LCD
**     Processor : MCF51JM128VLH
**     Compiler  : CodeWarrior ColdFireV1 C Compiler
**     Date/Time : 09.01.2010, 20:03
**     Contents  :
**         User source code
**
** ###################################################################*/

/* MODULE Cube */

#include "Platform.h"

#if PL_CONFIG_HAS_QUIZZ
#include "Quizz.h"
#include "UI1.h"
#include "UIScreen.h"
#include "UIWindow.h"
#include "GDisp1.h"
#include "FRTOS1.h"
#include "Helv08n.h"

#if PL_USE_SINGLE_FONT /* use only one font */
  #define FONT   PL_FONT()
#else
  #define FONT   Helv08n_GetFont()
#endif

typedef struct {
  UIScreen_ScreenWidget screen;
  UIWindow_WindowWidget window;
  UIHeader_HeaderWidget header;

  UIIcon_IconWidget iconRadioFast;
  UIText_TextWidget textFast;

  UIIcon_IconWidget iconRadioSlow;
  UIText_TextWidget textSlow;

  UIIcon_IconWidget iconCheckEnabled;
  UIText_TextWidget textEnabled;
} QUIZZ_GUIDesc;

static QUIZZ_GUIDesc *QUIZZ_Gui;

/* task notification bits */
#define QUIZZ_KILL_TASK   (1<<0)  /* close window and kill task */
static xTaskHandle QuizzTaskHandle;

void QUIZZ_CloseWindow(void) {
#if 0
  EVNT1_SetEvent(EVNT1_APP_MODE_CHANGE); /* request to close application */
  while(EVNT1_GetEvent(EVNT1_APP_MODE_CHANGE)) {
    /* wait until task has killed itself */
    FRTOS1_vTaskDelay(50/portTICK_RATE_MS);
  }
  (void)UI1_RemoveWindowPaintBackground(&params.descP->screen, &params.descP->quizzWindow);
  if (params.descP->screen.first==NULL) {
    /* were the last window: close screen */
    APP_SetApplicationMode(APP_MODE_MAIN_MENU);
  }
#endif
}

static void windowCallback(UI1_Element *element, UI1_MsgKind kind, void *pData) {
  (void)pData; /* unused argument */
  if (kind==UI1_MSG_CLICK) {
    if (UI1_EqualElement(element, &QUIZZ_Gui->header.iconWidget.element)) {
      QUIZZ_CloseWindow();
      return;
    }
  }
}

static void QUIZZ_CreateGUI(QUIZZ_GUIDesc *gui) {
  UI1_PixelDim x, y, h;

  /* screen */
  (void)UIScreen_Create(NULL, &gui->screen, 0, 0, UI1_GetWidth(), UI1_GetHeight());
  UIScreen_SetBackgroundColor(&gui->screen, UI1_COLOR_RED);

  /* window */
  (void)UIWindow_Create((UI1_Element*)&gui->screen, &gui->window,
      5, 5,
      GDisp1_GetWidth()-10, GDisp1_GetHeight()-30);
  UIWindow_SetBackgroundColor(&gui->window, UI1_COLOR_BRIGHT_GREEN);
  UIWindow_SetBorder(&gui->window);

  /* header */
  (void)UIHeader_Create((UI1_Element*)&gui->window, &gui->header,
      0, 0, /* for window border */
      0, 0 /* auto-size */
     );
  UIHeader_SetBackgroundColor(&gui->header, UI1_COLOR_BLUE);
  UIHeader_SetForegroundColor(&gui->header, UI1_COLOR_BLACK);
  UIHeader_SetText(&gui->header, "Options");
  UIHeader_SetUserMsgHandler(&gui->header, windowCallback);

  /* Icon: Radio button Fast */
  h = UI1_GetElementHeight(&gui->header.element);
  (void)UIIcon_Create((UI1_Element *)&gui->window, &gui->iconRadioFast,
      0, h, 10, 10);
  UIIcon_SetType(&gui->iconRadioFast, UIIcon_ICON_CIRCLE_DOT);
  UIIcon_SetForegroundColor(&gui->iconRadioFast, UI1_COLOR_BLACK);
  UIIcon_SetBackgroundColor(&gui->iconRadioFast, gui->window.bgColor);
//  UI1_EnableElementSelection(&gui->iconRadioFast.element);

  /* text */
  x = (UI1_PixelDim)(UI1_GetElementPosRight(&gui->iconRadioFast.element)+1);
  y = (UI1_PixelDim)(UI1_GetElementPosTop(&gui->iconRadioFast.element));
  UIText_Create((UI1_Element*)&gui->window, &gui->textFast, x, y, 0, 0);
  UIText_SetText(&gui->textFast, (unsigned char*)"Fast Mode");
  UIText_SetBackgroundColor(&gui->textFast, gui->window.bgColor);
  UIText_Resize(&gui->textFast);
  UI1_EnableElementSelection(&gui->textFast.element);

  /* Icon: Radio button Slow */
  h = UI1_GetElementPosBottom(&gui->iconRadioFast.element);
  (void)UIIcon_Create((UI1_Element *)&gui->window, &gui->iconRadioSlow,
      0, h, 10, 10);
  UIIcon_SetType(&gui->iconRadioSlow, UIIcon_ICON_CIRCLE);
  UIIcon_SetForegroundColor(&gui->iconRadioSlow, UI1_COLOR_BLACK);
  UIIcon_SetBackgroundColor(&gui->iconRadioSlow, gui->window.bgColor);
//  UI1_EnableElementSelection(&gui->iconRadioSlow.element);

  /* text */
  x = (UI1_PixelDim)(UI1_GetElementPosRight(&gui->iconRadioSlow.element)+1);
  y = (UI1_PixelDim)(UI1_GetElementPosTop(&gui->iconRadioSlow.element));
  UIText_Create((UI1_Element*)&gui->window, &gui->textSlow, x, y, 0, 0);
  UIText_SetText(&gui->textSlow, (unsigned char*)"Slow Mode");
  UIText_SetBackgroundColor(&gui->textSlow, gui->window.bgColor);
  UIText_Resize(&gui->textSlow);
  UI1_EnableElementSelection(&gui->textSlow.element);

  /* Icon: Check box */
  h = UI1_GetElementPosBottom(&gui->iconRadioSlow.element)+5;
  (void)UIIcon_Create((UI1_Element *)&gui->window, &gui->iconCheckEnabled,
      0, h, 10, 10);
  UIIcon_SetType(&gui->iconCheckEnabled, UIIcon_ICON_CHECKMARK_BOXED);
  UIIcon_SetForegroundColor(&gui->iconCheckEnabled, UI1_COLOR_BLACK);
  UIIcon_SetBackgroundColor(&gui->iconCheckEnabled, gui->window.bgColor);
//  UI1_EnableElementSelection(&gui->iconCheckEnabled.element);

  /* text */
  x = (UI1_PixelDim)(UI1_GetElementPosRight(&gui->iconCheckEnabled.element)+1);
  y = (UI1_PixelDim)(UI1_GetElementPosTop(&gui->iconCheckEnabled.element));
  UIText_Create((UI1_Element*)&gui->window, &gui->textEnabled, x, y, 0, 0);
  UIText_SetText(&gui->textEnabled, (unsigned char*)"Debug");
  UIText_SetBackgroundColor(&gui->textEnabled, gui->window.bgColor);
  UIText_Resize(&gui->textEnabled);
  UI1_EnableElementSelection(&gui->textEnabled.element);

  /* update the screen */
  UI1_MsgPaintAllElements((UI1_Element*)&gui->screen);
  /* assign root element */
  UI1_SetRoot(&gui->screen.element);

#if 0 /* test different icons */
  UIIcon_SetType(&gui->iconClose, UIIcon_ICON_BOX);
  UI1_MsgPaintAllElements((UI1_Element*)&gui->screen);

  UIIcon_SetType(&gui->iconClose, UIIcon_ICON_CIRCLE);
  UI1_MsgPaintAllElements((UI1_Element*)&gui->screen);

  UIIcon_SetType(&gui->iconClose, UIIcon_ICON_CIRCLE_DOT);
  UI1_MsgPaintAllElements((UI1_Element*)&gui->screen);

  UIIcon_SetType(&gui->iconClose, UIIcon_ICON_CHECKMARK);
  UI1_MsgPaintAllElements((UI1_Element*)&gui->screen);

  UIIcon_SetType(&gui->iconClose, UIIcon_ICON_CHECKMARK_BOXED);
  UI1_MsgPaintAllElements((UI1_Element*)&gui->screen);
#endif
}

static void QuizzTask(void *pvParameters) {
  uint32_t notifcationValue;
  BaseType_t notified;
  TickType_t startTicks, ticks;

  QUIZZ_Gui = pvPortMalloc(sizeof(QUIZZ_GUIDesc));
  if (QUIZZ_Gui==NULL) {
    for(;;) {}
  }
  QUIZZ_CreateGUI(QUIZZ_Gui);
  startTicks = xTaskGetTickCount();
  for(;;) {
    ticks = xTaskGetTickCount();
    if (((ticks-startTicks)/portTICK_RATE_MS)>10000) {
      QUIZZ_KillTask();
    }
    notified = xTaskNotifyWait(0UL, QUIZZ_KILL_TASK, &notifcationValue, 1); /* check flags, need to wait for one tick */
    if (notified==pdTRUE) { /* received notification */
#if 1
      if ((notifcationValue&QUIZZ_KILL_TASK)) {
        LCD1_Clear();//GDisp1_Clear();
        vPortFree(QUIZZ_Gui);
        QUIZZ_Gui = NULL;
        vTaskDelete(NULL); /* killing myself */
      }
#endif
    }
    vTaskDelay(pdMS_TO_TICKS(80)); /* give user a chance to see the cube rotating... */
  } /* for */
}

void QUIZZ_CreateTask(void) {
  QUIZZ_Gui = NULL;
  QuizzTaskHandle = NULL;
  if (xTaskCreate(QuizzTask, "Quizz", configMINIMAL_STACK_SIZE+80, NULL, tskIDLE_PRIORITY+3, &QuizzTaskHandle)!=pdPASS) {
    for(;;) {} /* out of memory? */
  }
}

void QUIZZ_KillTask(void) {
  (void)xTaskNotify(QuizzTaskHandle, QUIZZ_KILL_TASK, eSetBits);
}

#if 0
void QUIZZ_OnEvent(UI1_Screen *screen, UI1_Window *window, UI1_Element *element, UI1_EventCallbackKind event) {
  (void)screen;
  (void)window;
  (void)element;
  switch(event) {
    case UI1_EVENT_ORIENTATION_CHANGE:
     break;
  } /* switch */
}
#endif

void QUIZZ_Init(void) {
  QUIZZ_Gui = NULL;
  QuizzTaskHandle = NULL;
}

#endif /* PL_CONFIG_HAS_QUIZZ */

/* END Cube */
