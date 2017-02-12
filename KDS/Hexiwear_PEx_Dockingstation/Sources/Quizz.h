#ifndef __QUIZZ_H
#define __QUIZZ_H

#include "Platform.h"
#if PL_CONFIG_HAS_QUIZZ
#include "UI1.h"
#include "UIScreen.h"
#include "UIWindow.h"
#include "UIText.h"
#include "UIIcon.h"
//#include "UIHeader.h"
//#include "UIIcon.h"
//#include "UIHeader.h"

typedef struct {
  UIScreen_ScreenWidget screen;
  UIWindow_WindowWidget window;
  //UIHeader_HeaderWidget header;
  UIIcon_IconWidget iconClose;
  UIText_TextWidget text;
} QUIZZ_WindowDesc;

void QUIZZ_CreateTask(QUIZZ_WindowDesc *desc);

void QUIZZ_KillTask(void);

//void QUIZZ_OnEvent(UI_Screen *screen, UIWindow_WindowWidget *window, UI1_Element *element, UI_EventCallbackKind event);
  /* UI event handler */

void QUIZZ_Init(void);

#endif /* PL_CONFIG_HAS_QUIZZ */

#endif /* __QUIZZ_H */

