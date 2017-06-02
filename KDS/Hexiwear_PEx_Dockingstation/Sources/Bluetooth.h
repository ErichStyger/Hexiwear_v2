/*
 * Bluetooth.h
 *
 *  Created on: 05.03.2017
 *      Author: Erich Styger Local
 */

#ifndef SOURCES_BLUETOOTH_H_
#define SOURCES_BLUETOOTH_H_

#include "Platform.h"
#include "CLS1.h"

typedef enum {
  GUI_CURRENT_APP_IDLE                = 0,
  GUI_CURRENT_APP_SENSOR_TAG          = 2, // All sensors.
  GUI_CURRENT_APP_HEART_RATE          = 5, // heart rate
  GUI_CURRENT_APP_PEDOMETER           = 6 // Pedometer
} gui_current_app_t;

/* ------------------------------------------ */
/** Link state values */
typedef enum {
  linkState_disconnected = 0,                  /*!< Device is disconnected. */
  linkState_connected    = 1,                  /*!< Device is connected. */
} linkState_t;

void BLUETOOTH_SendLinkStateGetReq(void);
void BLUETOOTH_SetLinkState(linkState_t state);
linkState_t BLUETOOTH_GetLinkState(void);

/* ------------------------------------------ */
typedef enum {
  bluetooth_advMode_disable  = 0,
  bluetooth_advMode_enable   = 1,
} bluetooth_advMode_t;

void BLUETOOTH_SetAdvMode(bluetooth_advMode_t mode);
bluetooth_advMode_t BLUETOOTH_GetAdvMode(void);

void BLUETOOTH_SendAdvModeGetReq(void);
void BLUETOOTH_SendToggleAdvModeReq(void);

/* ------------------------------------------ */
typedef enum {
  buttonsGroup_left   = 0,
  buttonsGroup_right  = 1,
} buttonsGroup_t;

buttonsGroup_t BLUETOOTH_GetActiveButtons(void);
void BLUETOOTH_SetActiveButtons(buttonsGroup_t active);
void BLUETOOTH_SendToggleActiveButtonsReq(void);
void BLUETOOTH_SendActiveButtonsGetReq(void);

/* ------------------------------------------ */
void BLUETOOTH_SetVersionInformation(uint8_t major, uint8_t minor, uint8_t patch);
void BLUETOOTH_SendVersionReq(void);

/* ------------------------------------------ */
gui_current_app_t BLUETOOTH_GetCurrentAppMode(void);
void BLUETOOTH_SetCurrentAppMode(gui_current_app_t mode);

/* ------------------------------------------ */
uint8_t BLUETOOTH_ParseCommand(const uint8_t *cmd, bool *handled, CLS1_ConstStdIOType *io);

void BLUETOOTH_Init(void);

#endif /* SOURCES_BLUETOOTH_H_ */
