/*
 * UIMultipleChoice.h
 *
 *  Created on: 14.02.2017
 *      Author: Erich Styger
 */

#ifndef SOURCES_UIMULTIPLECHOICE_H_
#define SOURCES_UIMULTIPLECHOICE_H_

#include "UI1.h"
#include "UIIcon.h"
#include "UIText.h"

#define UI1_WIDGET_MULTIPLE_CHOICE   (UI1_WIDGET_LAST_PREDEFINED+1)

/* config */
#ifndef UIMC_CONFIG_MULTIPLECHOICE_DEFAULT_BACKGROUND_COLOR
  #define UIMC_CONFIG_MULTIPLECHOICE_DEFAULT_BACKGROUND_COLOR  UI1_COLOR_BRIGHT_GREY
    /*!< Default background color */
#endif

#ifndef UIMC_CONFIG_MULTIPLECHOICE_DEFAULT_FOREGROUND_COLOR
  #define UIMC_CONFIG_MULTIPLECHOICE_DEFAULT_FOREGROUND_COLOR    UI1_COLOR_BLACK
    /*!< Default foreground color */
#endif

#ifndef UIMC_CONFIG_MULTIPLECHOICE_DEFAULT_ICON_INSIDE_COLOR
  #define UIMC_CONFIG_MULTIPLECHOICE_DEFAULT_ICON_INSIDE_COLOR  UI1_COLOR_WHITE
    /*!< Default icon inside color */
#endif

#ifndef UIMC_CONFIG_MULTIPLECHOICE_DEFAULT_TEXT
  #define UIMC_CONFIG_MULTIPLECHOICE_DEFAULT_TEXT                "Answer"
  /*!< Default multiple choice text */
#endif

#ifndef UIMC_CONFIG_MULTIPLECHOICE_HAS_USER_MSG_HANDLER
  #define UIMC_CONFIG_MULTIPLECHOICE_HAS_USER_MSG_HANDLER  (1)
    /*!< 1: have extra user message handler; 0: no extra message handler*/
#endif

#define UIMC_MAX_NOF_ANSWERS 5  /* maximum possible number of answers for question */

typedef struct {
  const unsigned char *question; /* The question itself */
  const uint8_t nofAnswers; /* how many answer possibilities */
  const unsigned char *answer[UIMC_MAX_NOF_ANSWERS];
} UIMC_MultipleChoicQuestion;

typedef struct {
  UIIcon_IconWidget icon;
  UIText_TextWidget text;
} UIMC_Question;

typedef struct {
  UI1_Element element;                 /* the base element, always first in structure */
  int nofAnswers;                    /* nof answers in answer list */
  UIMC_Question questions[UIMC_MAX_NOF_ANSWERS];
  int choice; /* 0...nofAnswer-1 */
  UI1_PixelColor fgColor;              /* foreground color */
  UI1_PixelColor bgColor;              /* foreground color */
  UI1_PixelColor insideColor;          /* inside color */
#if UIMC_CONFIG_MULTIPLECHOICE_HAS_USER_MSG_HANDLER
  UI1_MsgHandler userMsgHandler;       /* optional user handler */
#endif
} UIMC_MultipleChoiceWidget;

void UIMC_SetBackgroundColor(UIMC_MultipleChoiceWidget *widget, UI1_PixelColor color);
void UIMC_SetForegroundColor(UIMC_MultipleChoiceWidget *widget, UI1_PixelColor color);
void UIMC_SetInsideColor(UIMC_MultipleChoiceWidget *widget, UI1_PixelColor color);

void UIMC_SetChoiceText(UIMC_MultipleChoiceWidget *widget, int choice, uint8_t *text);

uint8_t UIMC_Create(UI1_Element *parent, UIMC_MultipleChoiceWidget *widget, UI1_PixelDim x, UI1_PixelDim y, UI1_PixelDim width, UI1_PixelDim height, uint8_t nofAnswers);

void UIMC_Init(void);

#endif /* SOURCES_UIMULTIPLECHOICE_H_ */
