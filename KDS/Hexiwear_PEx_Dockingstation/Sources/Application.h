/*
 * Application.h
 *
 *  Created on: 29.11.2016
 *      Author: Erich Styger Local
 */

#ifndef SOURCES_APPLICATION_H_
#define SOURCES_APPLICATION_H_


typedef enum {
  APP_EVENT_BUTTON_DOWN,
  APP_EVENT_BUTTON_UP,
  APP_EVENT_BUTTON_LEFT,
  APP_EVENT_BUTTON_RIGHT,
} APP_EventType;

void APP_Event(APP_EventType kind);

void APP_Run(void);


#endif /* SOURCES_APPLICATION_H_ */
