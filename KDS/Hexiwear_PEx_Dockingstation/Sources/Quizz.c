#include "Platform.h"

#if PL_CONFIG_HAS_QUIZZ
#include "Quizz.h"
#include "UI.h"
#include "UI1.h"
#include "UIScreen.h"
#include "UIWindow.h"
#include "UIHeader.h"
#include "GDisp1.h"
#include "FRTOS1.h"
#include "Helv08n.h"
#include "UIMultipleChoice.h"

#define FONT   Helv08n_GetFont()

typedef enum {
  ICON_RADIO_FAST,
  ICON_RADIO_SLOW
} ICON_RADIO_State;

typedef struct {
  UIIcon_IconWidget radio[5];
  UIIcon_IconWidget text[5];
  int choice; /* 0..4 */
} QUIZZ_MultipleChoice;

typedef struct {
  UIScreen_ScreenWidget screen;
  UIWindow_WindowWidget window;
  UIHeader_HeaderWidget header;

  UIMC_MultipleChoiceWidget quizz;

  UIIcon_IconWidget iconRadioFast;
  UIText_TextWidget textFast;
  UIIcon_IconWidget iconRadioSlow;
  UIText_TextWidget textSlow;
  ICON_RADIO_State iconRadioStatus;

  UIIcon_IconWidget iconCheckboxDebug;
  UIText_TextWidget textCheckboxDebug;
  bool isiconCheckboxDebugEnabled;

  /* navigation */
  UIIcon_IconWidget iconNavigationUp;
  UIIcon_IconWidget iconNavigationDown;
  UIIcon_IconWidget iconNavigationEnter;
  UIIcon_IconWidget iconNavigationExit;
} QUIZZ_GUIDesc;

static QUIZZ_GUIDesc *QUIZZ_Gui;

/* task notification bits */
#define QUIZZ_KILL_TASK            (1<<0)  /* close window and kill task */
#define QUIZZ_SUSPEND_TASK         (1<<1)  /* blank screen and suspend task */
#define QUIZZ_USER_BUTTON_PRESSED  (1<<2)  /* user has pressed button */
static xTaskHandle QuizzTaskHandle;

static void guiCallback(UI1_Element *element, UI1_MsgKind kind, void *pData) {
  (void)pData; /* unused argument */
  if (kind==UI1_MSG_CLICK) {
    if (UI1_EqualElement(element, &QUIZZ_Gui->header.iconWidget.element)) {
      QUIZZ_KillTask();
      return;
    } else if (UI1_EqualElement(element, &QUIZZ_Gui->textCheckboxDebug.element)) {
      QUIZZ_Gui->isiconCheckboxDebugEnabled = !QUIZZ_Gui->isiconCheckboxDebugEnabled; /* toggle */
      if (QUIZZ_Gui->isiconCheckboxDebugEnabled) {
        UIIcon_SetType(&QUIZZ_Gui->iconCheckboxDebug, UIIcon_ICON_CHECKMARK_BOXED);
      } else {
        UIIcon_SetType(&QUIZZ_Gui->iconCheckboxDebug, UIIcon_ICON_BOX);
      }
      /* update checkbox icon */
      UI1_SendMessage(&QUIZZ_Gui->iconCheckboxDebug.element, UI1_MSG_WIDGET_UPDATE, NULL);
    } else if (UI1_EqualElement(element, &QUIZZ_Gui->textSlow.element)) {
      if (QUIZZ_Gui->iconRadioStatus != ICON_RADIO_SLOW) {
        QUIZZ_Gui->iconRadioStatus = ICON_RADIO_SLOW;
        UIIcon_SetType(&QUIZZ_Gui->iconRadioFast, UIIcon_ICON_CIRCLE);
        UIIcon_SetType(&QUIZZ_Gui->iconRadioSlow, UIIcon_ICON_CIRCLE_DOT);
        /* update icons */
        UI1_SendMessage(&QUIZZ_Gui->iconRadioFast.element, UI1_MSG_WIDGET_UPDATE, NULL);
        UI1_SendMessage(&QUIZZ_Gui->iconRadioSlow.element, UI1_MSG_WIDGET_UPDATE, NULL);
      }
    } else if (UI1_EqualElement(element, &QUIZZ_Gui->textFast.element)) {
      if (QUIZZ_Gui->iconRadioStatus != ICON_RADIO_FAST) {
        QUIZZ_Gui->iconRadioStatus = ICON_RADIO_FAST;
        UIIcon_SetType(&QUIZZ_Gui->iconRadioFast, UIIcon_ICON_CIRCLE_DOT);
        UIIcon_SetType(&QUIZZ_Gui->iconRadioSlow, UIIcon_ICON_CIRCLE);
        /* update icons */
        UI1_SendMessage(&QUIZZ_Gui->iconRadioFast.element, UI1_MSG_WIDGET_UPDATE, NULL);
        UI1_SendMessage(&QUIZZ_Gui->iconRadioSlow.element, UI1_MSG_WIDGET_UPDATE, NULL);
      }
    }
  } /* if click */
}

static const UIMC_MultipleChoicQuestion questions[] = {
    {
        .question = "Disable Inter-\nrupts on ARM\nCortex-M4F:",
        .answer[0] = "cpsie i",
        .answer[1] = "cpsid i",
        .answer[2] = "cpsie primask",
        .answer[3] = "cpsie basepri",
        .answer[4] = "mov i, basepri",
    },
    {
        .question = "How are you\ndoing?",
        .answer[0] = "Excellent!",
        .answer[1] = "Doing fine",
        .answer[2] = "Could be better",
        .answer[3] = "Oh, well ...",
        .answer[4] = "@!#!?!!",
    },
};

static void QUIZZ_CreateGUI(QUIZZ_GUIDesc *gui) {
  UI1_PixelDim x, y, h;

  /* screen */
  (void)UIScreen_Create(NULL, &gui->screen, 0, 0, UI1_GetWidth(), UI1_GetHeight());
  UIScreen_SetBackgroundColor(&gui->screen, GDisp1_COLOR_BRIGHT_YELLOW);

  /* Navigation Icons */
  (void)UIIcon_Create(&gui->screen.element, &gui->iconNavigationUp,
      UI1_GetWidth()-10, 20, 10, 10);
  UIIcon_SetType(&gui->iconNavigationUp, UIIcon_ICON_ARROW_UP);
  UIIcon_SetForegroundColor(&gui->iconNavigationUp, UI1_COLOR_BLACK);
  UIIcon_SetBackgroundColor(&gui->iconNavigationUp, gui->screen.bgColor);
  UIIcon_SetInsideColor(&gui->iconNavigationUp, UI1_COLOR_WHITE);

  (void)UIIcon_Create(&gui->screen.element, &gui->iconNavigationDown,
      UI1_GetWidth()-10, 65, 10, 10);
  UIIcon_SetType(&gui->iconNavigationDown, UIIcon_ICON_ARROW_DOWN);
  UIIcon_SetForegroundColor(&gui->iconNavigationDown, UI1_COLOR_BLACK);
  UIIcon_SetBackgroundColor(&gui->iconNavigationDown, gui->screen.bgColor);
  UIIcon_SetInsideColor(&gui->iconNavigationDown, UI1_COLOR_WHITE);

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
  UIHeader_SetText(&gui->header, (uint8_t*)questions[0].question);
  UIHeader_Resize(&gui->header); /* adjust size */
  UIIcon_SetUserMsgHandler(&gui->header.iconWidget, guiCallback);
  UI1_EnableElementSelection(&gui->header.iconWidget.element);

  h = 0;

  h += UI1_GetElementHeight(&gui->header.element);
#if 1
  /* multiple choice questions */
  UIMC_Create(&gui->window.element, &gui->quizz, 0, h, 0, 0);
  UIMC_SetBackgroundColor(&gui->quizz, gui->window.bgColor);
  for(int i=0;i<UIMC_NOF_QUESTIONS;i++) {
    UIMC_SetChoiceText(&gui->quizz, i, (uint8_t*)questions[0].answer[i]);
  }

  h += UI1_GetElementHeight(&gui->quizz.element);
#else
  /* Icon: Radio button Fast */
  (void)UIIcon_Create(&gui->window.element, &gui->iconRadioFast,
      0, h, 10, 10);
  UIIcon_SetType(&gui->iconRadioFast, UIIcon_ICON_CIRCLE_DOT);
  UIIcon_SetForegroundColor(&gui->iconRadioFast, UI1_COLOR_BLACK);
  UIIcon_SetBackgroundColor(&gui->iconRadioFast, gui->window.bgColor);
  UIIcon_SetInsideColor(&gui->iconRadioFast, UI1_COLOR_WHITE);
  //UI1_EnableElementSelection(&gui->iconRadioFast.element);

  /* text */
  x = (UI1_PixelDim)(UI1_GetElementPosRight(&gui->iconRadioFast.element)+1);
  y = (UI1_PixelDim)(UI1_GetElementPosTop(&gui->iconRadioFast.element));
  UIText_Create((UI1_Element*)&gui->window, &gui->textFast, x, y, 0, 0);
  UIText_SetText(&gui->textFast, (unsigned char*)"Answer A");
  UIText_SetBackgroundColor(&gui->textFast, gui->window.bgColor);
  UIText_Resize(&gui->textFast);
  UI1_EnableElementSelection(&gui->textFast.element);
  UIText_SetUserMsgHandler(&gui->textFast, guiCallback);

  /* Icon: Radio button Slow */
  h = UI1_GetElementPosBottom(&gui->iconRadioFast.element);
  (void)UIIcon_Create((UI1_Element *)&gui->window, &gui->iconRadioSlow,
      0, h, 10, 10);
  UIIcon_SetType(&gui->iconRadioSlow, UIIcon_ICON_CIRCLE);
  UIIcon_SetForegroundColor(&gui->iconRadioSlow, UI1_COLOR_BLACK);
  UIIcon_SetBackgroundColor(&gui->iconRadioSlow, gui->window.bgColor);
  UIIcon_SetInsideColor(&gui->iconRadioSlow, UI1_COLOR_WHITE);
  //UI1_EnableElementSelection(&gui->iconRadioSlow.element);

  /* text */
  x = (UI1_PixelDim)(UI1_GetElementPosRight(&gui->iconRadioSlow.element)+1);
  y = (UI1_PixelDim)(UI1_GetElementPosTop(&gui->iconRadioSlow.element));
  UIText_Create((UI1_Element*)&gui->window, &gui->textSlow, x, y, 0, 0);
  UIText_SetText(&gui->textSlow, (unsigned char*)"Slow Mode");
  UIText_SetBackgroundColor(&gui->textSlow, gui->window.bgColor);
  UIText_Resize(&gui->textSlow);
  UI1_EnableElementSelection(&gui->textSlow.element);
  UIText_SetUserMsgHandler(&gui->textSlow, guiCallback);
  gui->iconRadioStatus = ICON_RADIO_FAST;

  /* Icon: Check box */
  h = UI1_GetElementPosBottom(&gui->iconRadioSlow.element)+5;
  (void)UIIcon_Create((UI1_Element *)&gui->window, &gui->iconCheckboxDebug,
      0, h, 10, 10);
  UIIcon_SetType(&gui->iconCheckboxDebug, UIIcon_ICON_CHECKMARK_BOXED);
  UIIcon_SetForegroundColor(&gui->iconCheckboxDebug, UI1_COLOR_BLACK);
  UIIcon_SetBackgroundColor(&gui->iconCheckboxDebug, UI1_COLOR_WHITE);
//  UIIcon_SetUserMsgHandler(&gui->iconCheckboxDebug, guiCallback);
//  UI1_EnableElementSelection(&gui->iconCheckboxDebug.element);

  /* text */
  x = (UI1_PixelDim)(UI1_GetElementPosRight(&gui->iconCheckboxDebug.element)+1);
  y = (UI1_PixelDim)(UI1_GetElementPosTop(&gui->iconCheckboxDebug.element));
  UIText_Create((UI1_Element*)&gui->window, &gui->textCheckboxDebug, x, y, 0, 0);
  UIText_SetText(&gui->textCheckboxDebug, (unsigned char*)"Debug");
  UIText_SetBackgroundColor(&gui->textCheckboxDebug, gui->window.bgColor);
  UIText_Resize(&gui->textCheckboxDebug);
  UI1_EnableElementSelection(&gui->textCheckboxDebug.element);
  UIText_SetUserMsgHandler(&gui->textCheckboxDebug, guiCallback);
  gui->isiconCheckboxDebugEnabled = TRUE;
#endif

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

  /* assign root element */
  UI1_SetRoot(&gui->screen.element);
  UI_ShowScreen();
}

static void QuizzTask(void *pvParameters) {
  uint32_t notifcationValue;
  BaseType_t notified;
  TickType_t startTicks, ticks;
  const int timeoutSecs = 60;

  QUIZZ_Gui = pvPortMalloc(sizeof(QUIZZ_GUIDesc));
  if (QUIZZ_Gui==NULL) {
    for(;;) {}
  }
  QUIZZ_CreateGUI(QUIZZ_Gui);
  startTicks = xTaskGetTickCount();
  for(;;) {
    ticks = xTaskGetTickCount();
    if (((ticks-startTicks)/portTICK_RATE_MS)>(timeoutSecs*1000)) {
      UI_BlankScreen();
    }
    notified = xTaskNotifyWait(0UL, QUIZZ_KILL_TASK, &notifcationValue, 1); /* check flags, need to wait for one tick */
    if (notified==pdTRUE) { /* received notification */
#if 1
      if (notifcationValue&QUIZZ_KILL_TASK) {
        LCD1_Clear();//GDisp1_Clear();
        vPortFree(QUIZZ_Gui);
        QUIZZ_Gui = NULL;
        UI_SetCurrentUITask(NULL);
        vTaskDelete(NULL); /* killing myself */
      }
      if (notifcationValue&QUIZZ_USER_BUTTON_PRESSED) {
        UI_ShowScreen();
        startTicks = xTaskGetTickCount(); /* restart timeout */
      }
      if (notifcationValue&QUIZZ_SUSPEND_TASK) {
        UI_BlankScreen();
        vTaskSuspend(NULL); /* suspend */
        /* here we are taken back from suspending */
        UI_ShowScreen(); /* show GUI again */
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
  UI_SetCurrentUITask(QuizzTaskHandle);
}

void QUIZZ_SuspendTask(void) {
  (void)xTaskNotify(QuizzTaskHandle, QUIZZ_SUSPEND_TASK, eSetBits);
}

void QUIZZ_ResumeTask(void) {
  vTaskResume(QuizzTaskHandle); /* resume task */
}


void QUIZZ_KillTask(void) {
  (void)xTaskNotify(QuizzTaskHandle, QUIZZ_KILL_TASK, eSetBits);
}

void QUIZZ_Init(void) {
  QUIZZ_Gui = NULL;
  QuizzTaskHandle = NULL;
}

#endif /* PL_CONFIG_HAS_QUIZZ */
