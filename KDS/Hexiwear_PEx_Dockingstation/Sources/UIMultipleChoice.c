/*
 * UIMultipleChoice.c
 *
 *  Created on: 14.02.2017
 *      Author: Erich Styger Local
 */

#include "Platform.h"
#include "UIMultipleChoice.h"

#include "UI1.h"
#include "UIIcon.h"
#include "UIText.h"

void UIMC_SetBackgroundColor(UIMC_MultipleChoiceWidget *widget, UI1_PixelColor color) {
  int i;

  widget->bgColor = color;
  for(i=0;i<UIMC_NOF_QUESTIONS;i++) {
    UIIcon_SetBackgroundColor(&widget->questions[i].icon, color);
    UIText_SetBackgroundColor(&widget->questions[i].text, color);
  }
}

void UIMC_SetForegroundColor(UIMC_MultipleChoiceWidget *widget, UI1_PixelColor color) {
  int i;

  widget->fgColor = color;
  for(i=0;i<UIMC_NOF_QUESTIONS;i++) {
    UIIcon_SetForegroundColor(&widget->questions[i].icon, color);
    UIText_SetForegroundColor(&widget->questions[i].text, color);
  }
}

void UIMC_SetInsideColor(UIMC_MultipleChoiceWidget *widget, UI1_PixelColor color) {
  int i;

  widget->insideColor = color;
  for(i=0;i<UIMC_NOF_QUESTIONS;i++) {
    UIIcon_SetInsideColor(&widget->questions[i].icon, color);
  }
}

void UIMC_SetChoiceText(UIMC_MultipleChoiceWidget *widget, int choice, uint8_t *text) {
  if (choice>=UIMC_NOF_QUESTIONS) {
    return; /* selection out of bounds! */
  }
  UIText_SetText(&widget->questions[choice].text, text);
  UIText_Resize(&widget->questions[choice].text); /* resize widget */
}

void UIMC_Select(UIMC_MultipleChoiceWidget *widget, int choice) {
  int i;

  if (choice>=UIMC_NOF_QUESTIONS) {
    return; /* selection out of bounds! */
  }
  if (choice== widget->choice) {
    return; /* already selected, nothing to do */
  }
  UIIcon_SetType(&widget->questions[widget->choice].icon, UIIcon_ICON_CIRCLE); /* deselect select old item */
  UI1_SendMessage(&widget->questions[widget->choice].icon.element, UI1_MSG_WIDGET_UPDATE, NULL); /* update widget */
  widget->choice = choice; /* store new choice */
  UIIcon_SetType(&widget->questions[choice].icon, UIIcon_ICON_CIRCLE_DOT); /* select new item */
  UI1_SendMessage(&widget->questions[choice].icon.element, UI1_MSG_WIDGET_UPDATE, NULL); /* update widget */
}

void UIMC_Paint(UIMC_MultipleChoiceWidget *widget) {
  if (widget->element.prop.type != UI1_WIDGET_MULTIPLE_CHOICE) {
    return;
  }
  UI1_DrawFilledBox(&widget->element,
      widget->element.prop.x, widget->element.prop.y,
      widget->element.prop.width, widget->element.prop.height,
      widget->bgColor);
}

uint8_t UIMC_OnClick(UIMC_MultipleChoiceWidget *widget, UIText_TextWidget *textw) {
  int i;

  for(i=0;i<UIMC_NOF_QUESTIONS;i++) {
    if (UI1_EqualElement(&textw->element, &widget->questions[i].text.element)) {
      break; /* found it */
    }
  }
  UIMC_Select(widget, i); /* make selection */
}

void UIMC_MessageHandler(UI1_Element *element, UI1_MsgKind kind, void *pData) {
  UIMC_MultipleChoiceWidget *widget = (UIMC_MultipleChoiceWidget*)element;

  if (widget==NULL || widget->element.prop.type!=UI1_WIDGET_MULTIPLE_CHOICE) {
    return; /* not the correct widget */
  }
  switch(kind) {
    case UI1_MSG_WIDGET_SELECTED:
    case UI1_MSG_WIDGET_UPDATE:
    case UI1_MSG_WIDGET_PAINT:
      UIMC_Paint(widget);
      break;
    default:
      break;
  } /* switch */
#if UIMC_CONFIG_MULTIPLECHOICE_HAS_USER_MSG_HANDLER
  if (widget->userMsgHandler != NULL){
    widget->userMsgHandler(element, kind, pData); /* call user event  */
  }
#else
  (void)pData; /* avoid warning about unused argument */
#endif
}

void UIMC_SelectionHandler(UI1_Element *element, UI1_MsgKind kind, void *pData) {
  UIMC_MultipleChoiceWidget *widget = (UIMC_MultipleChoiceWidget*)element;

  (void)pData; /* avoid warning about unused argument */
  if (widget==NULL || widget->element.prop.type!=UI1_WIDGET_TEXT) {
    return; /* not a text widget */
  }
  switch(kind) {
    case UI1_MSG_CLICK:
      UIMC_OnClick((UIMC_MultipleChoiceWidget*)element->parent, (UIText_TextWidget*)element);
      break;
    default:
      break;
  } /* switch */
}


uint8_t UIMC_Create(UI1_Element *parent, UIMC_MultipleChoiceWidget *widget, UI1_PixelDim x, UI1_PixelDim y, UI1_PixelDim width, UI1_PixelDim height) {
  FDisp1_Font *font;
  int i;
  UI1_PixelDim xpos, ypos;

  if (widget == NULL) {
    return ERR_FAILED;
  }
  font = Helv08n_GetFont(); /* font defined in user properties */
  if (width==0) {                      /* auto size */
    width = FDisp1_GetStringWidth((unsigned char*)UIMC_CONFIG_MULTIPLECHOICE_DEFAULT_TEXT, font, NULL);
    width += 10+1; /* size for icon */
  }
  if (height==0) {                     /* auto size */
    height = UIMC_NOF_QUESTIONS*FDisp1_GetStringHeight((unsigned char*)UIMC_CONFIG_MULTIPLECHOICE_DEFAULT_TEXT, font, NULL);
  }
  UI1_ElementInitCommon(&widget->element, UI1_WIDGET_MULTIPLE_CHOICE, x, y, width, height, UIMC_MessageHandler);
  widget->choice = 0; /* initialize selected item */
  xpos = 0;
  ypos = 0;
  for(i=0;i<UIMC_NOF_QUESTIONS;i++) {
    /* icon items */
    UIIcon_Create(&widget->element, &widget->questions[i].icon, xpos, ypos, 10, 10);
    UIIcon_SetBackgroundColor(&widget->questions[i].icon, UIMC_CONFIG_MULTIPLECHOICE_DEFAULT_BACKGROUND_COLOR);
    UIIcon_SetForegroundColor(&widget->questions[i].icon, UIMC_CONFIG_MULTIPLECHOICE_DEFAULT_FOREGROUND_COLOR);
    UIIcon_SetInsideColor(&widget->questions[i].icon, UIMC_CONFIG_MULTIPLECHOICE_DEFAULT_ICON_INSIDE_COLOR);
    if (i==widget->choice) {
      UIIcon_SetType(&widget->questions[i].icon, UIIcon_ICON_CIRCLE_DOT);
    } else {
      UIIcon_SetType(&widget->questions[i].icon, UIIcon_ICON_CIRCLE);
    }

    /* text items */
    UIText_Create(&widget->element, &widget->questions[i].text, xpos+11, ypos,
        FDisp1_GetStringWidth((unsigned char*)UIMC_CONFIG_MULTIPLECHOICE_DEFAULT_TEXT, font, NULL),
        FDisp1_GetStringHeight((unsigned char*)UIMC_CONFIG_MULTIPLECHOICE_DEFAULT_TEXT, font, NULL));
    UIText_SetBackgroundColor(&widget->questions[i].text, UIMC_CONFIG_MULTIPLECHOICE_DEFAULT_BACKGROUND_COLOR);
    UIText_SetForegroundColor(&widget->questions[i].text, UIMC_CONFIG_MULTIPLECHOICE_DEFAULT_FOREGROUND_COLOR);
    UIText_SetText(&widget->questions[i].text, UIMC_CONFIG_MULTIPLECHOICE_DEFAULT_TEXT);
    UIText_SetFont(&widget->questions[i].text, font);
    UI1_EnableElementSelection(&widget->questions[i].text.element); /* make text selectable */
    UIText_SetUserMsgHandler(&widget->questions[i].text, UIMC_SelectionHandler);

    ypos += FDisp1_GetStringHeight((unsigned char*)UIMC_CONFIG_MULTIPLECHOICE_DEFAULT_TEXT, font, NULL);
  }
#if UIMC_CONFIG_MULTIPLECHOICE_HAS_USER_MSG_HANDLER
  widget->userMsgHandler = NULL;         /* needs to be set by user */
#endif
  if (parent!=NULL) {
    return UI1_AddSubElement(parent, &widget->element);
  }
  return ERR_OK;
}

void UIMC_Init(void) {
}


