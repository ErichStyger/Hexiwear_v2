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
uint8_t BLUETOOTH_ParseCommand(const uint8_t *cmd, bool *handled, CLS1_ConstStdIOType *io);

void BLUETOOTH_Init(void);

#endif /* SOURCES_BLUETOOTH_H_ */
