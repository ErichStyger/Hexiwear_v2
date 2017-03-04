/*
 * UI.h
 *
 *  Created on: 13.02.2017
 *      Author: Erich Styger Local
 */

#ifndef SOURCES_UI_H_
#define SOURCES_UI_H_

/* common task notification bits */
#define UI_NOTIFY_KILL_TASK            (1<<0)  /* close window and kill task */
#define UI_NOTIFY_SUSPEND_TASK         (1<<1)  /* blank screen and suspend task */
#define UI_NOTIFY_LAST                 UI_NOTIFY_SUSPEND_TASK /* last bit */

/* called by app to close the UI */
void UI_CloseAppUI(void);

/* touch events from KW40 */
typedef enum {
  UI_EVENT_BUTTON_DOWN,
  UI_EVENT_BUTTON_UP,
  UI_EVENT_BUTTON_LEFT,
  UI_EVENT_BUTTON_RIGHT,
  UI_EVENT_PARING_CODE, /* show pairing code */
} UI_EventType;

void UI_Event(UI_EventType kind, void *data);

void UI_Init(void);

#endif /* SOURCES_UI_H_ */
