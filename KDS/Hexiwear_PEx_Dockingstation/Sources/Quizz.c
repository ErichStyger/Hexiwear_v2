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

/* task notification bits */
#define QUIZZ_KILL_TASK   (1<<0)  /* close window and kill task */
static xTaskHandle QuizzTaskHandle;

/* structure to hold our screen and module settings */
typedef struct {
  QUIZZ_WindowDesc *descP;
} QUIZZ_WinParams;

static QUIZZ_WinParams params;

static void QuizzTask(void *pvParameters) {
  uint32_t notifcationValue;
  BaseType_t notified;
  TickType_t startTicks, ticks;

  startTicks = xTaskGetTickCount();
  for(;;) {
    ticks = xTaskGetTickCount();
    if (((ticks-startTicks)/portTICK_RATE_MS)>5000) {
      QUIZZ_KillTask();
    }
    notified = xTaskNotifyWait(0UL, QUIZZ_KILL_TASK, &notifcationValue, 1); /* check flags, need to wait for one tick */
    if (notified==pdTRUE) { /* received notification */
      if ((notifcationValue&QUIZZ_KILL_TASK)) {
        LCD1_Clear();//GDisp1_Clear();
        vTaskDelete(NULL); /* killing myself */
      }
    }
    vTaskDelay(pdMS_TO_TICKS(80)); /* give user a chance to see the cube rotating... */
  } /* for */
}

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
    if (UI1_EqualElement(element, &params.descP->header.iconWidget.element)) {
      QUIZZ_CloseWindow();
      return;
    }
  }
}

static void QUIZZ_CreateScreen(void) {
  UI1_PixelDim h;

  /* screen */
  (void)UIScreen_Create(NULL, &params.descP->screen, 0, 0, UI1_GetWidth(), UI1_GetHeight());
  UIScreen_SetBackgroundColor(&params.descP->screen, UI1_COLOR_RED);
  /* window */
  (void)UIWindow_Create((UI1_Element*)&params.descP->screen, &params.descP->window,
      5, 5,
      GDisp1_GetWidth()-10, GDisp1_GetHeight()-10);
  UIWindow_SetBackgroundColor(&params.descP->window, UI1_COLOR_BRIGHT_GREEN);
  UIWindow_SetBorder(&params.descP->window);

  /* header */
  (void)UIHeader_Create((UIHeader_Element*)&params.descP->window, &params.descP->header,
      1, 1,
      0, /* auto-size */
      10);
  UIHeader_SetBackgroundColor(&params.descP->header, UI1_COLOR_BLUE);
  UIHeader_SetTextForegroundColor(&params.descP->header, UI1_COLOR_BLACK);
  UIHeader_SetUserMsgHandler(&params.descP->header, windowCallback);

  /* Icon: Radio button */
  h = (UI1_PixelDim)(UI1_GetElementHeight(&params.descP->header.element));
  (void)UIIcon_Create((UI1_Element *)&params.descP->window, &params.descP->iconRadio,
      0, h,
      10, 10);
  UIIcon_SetType(&params.descP->iconRadio, UIIcon_ICON_CIRCLE_DOT);
  UIIcon_SetForegroundColor(&params.descP->iconRadio, UI1_COLOR_BLACK);
  UIIcon_SetBackgroundColor(&params.descP->iconRadio, UI1_COLOR_WHITE);
  //params.descP->iconClose.element.prop.flags |= UI1_FLAGS_ALIGN_RIGHT;
  //UI1_OnWindowResize(&params.descP->window); /* right align element(s) if needed */
  UI1_EnableElementSelection(&params.descP->iconClose);

#if 0
  /* icon */
  UIIcon_Create((UI1_Element*)&params.descP->window, &params.descP->iconClose,
      UI1_GetElementPosX(&params.descP->window.element)+3,
      UI1_GetElementPosY(&params.descP->window.element)+30, 11, 11);
  UIIcon_SetForegroundColor(&params.descP->iconClose, UI1_COLOR_BLACK);
  UIIcon_SetBackgroundColor(&params.descP->iconClose, UI1_COLOR_WHITE);
  UIIcon_SetType(&params.descP->iconClose, UIIcon_ICON_X_BOXED);
#endif
  /* text */
  UIText_Create((UI1_Element*)&params.descP->window, &params.descP->text,
      15, 30, 0, 0);
  UIText_SetText(&params.descP->text, (unsigned char*)"Fast Mode");
  UIText_Resize(&params.descP->text);
  /* update the screen */
  UI1_MsgPaintAllElements((UI1_Element*)&params.descP->screen);
#if 0
  UIIcon_SetType(&params.descP->iconClose, UIIcon_ICON_BOX);
  UI1_MsgPaintAllElements((UI1_Element*)&params.descP->screen);

  UIIcon_SetType(&params.descP->iconClose, UIIcon_ICON_CIRCLE);
  UI1_MsgPaintAllElements((UI1_Element*)&params.descP->screen);

  UIIcon_SetType(&params.descP->iconClose, UIIcon_ICON_CIRCLE_DOT);
  UI1_MsgPaintAllElements((UI1_Element*)&params.descP->screen);

  UIIcon_SetType(&params.descP->iconClose, UIIcon_ICON_CHECKMARK);
  UI1_MsgPaintAllElements((UI1_Element*)&params.descP->screen);

  UIIcon_SetType(&params.descP->iconClose, UIIcon_ICON_CHECKMARK_BOXED);
  UI1_MsgPaintAllElements((UI1_Element*)&params.descP->screen);
#endif
}

void QUIZZ_CreateTask(QUIZZ_WindowDesc *desc) {
  params.descP = desc; /* store pointer */
  
  QuizzTaskHandle = NULL;
  QUIZZ_CreateScreen();
  if (xTaskCreate(QuizzTask, "Quizz", configMINIMAL_STACK_SIZE+80, &params, tskIDLE_PRIORITY+3, &QuizzTaskHandle)!=pdPASS) {
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
}

#endif /* PL_CONFIG_HAS_QUIZZ */

/* END Cube */
