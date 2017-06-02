/*
 * Bluetooth.c
 *
 *  Created on: 05.03.2017
 *      Author: Erich Styger Local
 */

#include "Platform.h"
#include "Bluetooth.h"
#include "HostComm.h"
#include "CLS1.h"
#include "UTIL1.h"

typedef struct {
  uint8_t major, minor, patch;
} versionInfo;

static versionInfo versionKW40;  /* version number of KW40 */
static const versionInfo versionK64F = {10,0,1}; /* K64F version */

static bluetooth_advMode_t currAdvMode = bluetooth_advMode_disable;
static buttonsGroup_t buttonsGroupCurrentActive = buttonsGroup_right;
static linkState_t currLinkState = linkState_disconnected;

static gui_current_app_t appMode = GUI_CURRENT_APP_IDLE;

gui_current_app_t BLUETOOTH_GetCurrentAppMode(void) {
  return appMode;
}

void BLUETOOTH_SetCurrentAppMode(gui_current_app_t mode) {
  appMode = mode;
  HostComm_SendSetApplicationMode(mode);
}
const unsigned char *GetCurrentAppModeString(void) {
  switch(appMode) {
    case GUI_CURRENT_APP_IDLE: return "Idle";
    case GUI_CURRENT_APP_SENSOR_TAG: return "SensorTag";
    case GUI_CURRENT_APP_HEART_RATE: return "HeartRate";
    case GUI_CURRENT_APP_PEDOMETER: return "Pedometer";
    default: break;
  }
  return "UNKNOWN";
}

void BLUETOOTH_SendLinkStateGetReq(void) {
  hostInterface_packet_t dataPacket;

  dataPacket.type    = packetType_linkStateGet;
  dataPacket.length  = 0;
  HostComm_SendMessage(&dataPacket, TRUE);
}

void BLUETOOTH_SetLinkState(linkState_t state) {
  if (state==linkState_disconnected || state==linkState_connected) {
    currLinkState = state;
  } else {
    CLS1_SendStr("Wrong link state: ", CLS1_GetStdio()->stdErr);
    CLS1_SendNum8u(state, CLS1_GetStdio()->stdErr);
    CLS1_SendStr("\r\n", CLS1_GetStdio()->stdErr);
  }
}

linkState_t BLUETOOTH_GetLinkState(void) {
  return currLinkState;
}


buttonsGroup_t BLUETOOTH_GetActiveButtons(void) {
  return buttonsGroupCurrentActive;
}

void BLUETOOTH_SetActiveButtons(buttonsGroup_t active) {
  if (active==buttonsGroup_left || active==buttonsGroup_right) {
    buttonsGroupCurrentActive = active;
  } else {
    CLS1_SendStr("Wrong active buttons group: ", CLS1_GetStdio()->stdErr);
    CLS1_SendNum8u(active, CLS1_GetStdio()->stdErr);
    CLS1_SendStr("\r\n", CLS1_GetStdio()->stdErr);
  }
}

void BLUETOOTH_SendToggleActiveButtonsReq(void) {
  hostInterface_packet_t dataPacket;

  dataPacket.type    = packetType_buttonsGroupToggleActive;
  dataPacket.length  = 0;
  HostComm_SendMessage(&dataPacket, TRUE);
}

void BLUETOOTH_SendActiveButtonsGetReq(void) {
  hostInterface_packet_t dataPacket;

  dataPacket.type    = packetType_buttonsGroupGetActive;
  dataPacket.length  = 0;
  HostComm_SendMessage(&dataPacket, TRUE);
}

void BLUETOOTH_SetAdvMode(bluetooth_advMode_t mode) {
  if (mode==bluetooth_advMode_disable || mode==bluetooth_advMode_enable) {
    currAdvMode = mode;
  } else {
    CLS1_SendStr("Wrong Advertisement mode: ", CLS1_GetStdio()->stdErr);
    CLS1_SendNum8u(mode, CLS1_GetStdio()->stdErr);
    CLS1_SendStr("\r\n", CLS1_GetStdio()->stdErr);
  }
}

bluetooth_advMode_t BLUETOOTH_GetAdvMode(void) {
  return currAdvMode;
}

void BLUETOOTH_SendAdvModeGetReq(void) {
  hostInterface_packet_t dataPacket;

  dataPacket.type    = packetType_advModeGet;
  dataPacket.length  = 0;
  HostComm_SendMessage(&dataPacket, TRUE);
}

void BLUETOOTH_SendToggleAdvModeReq(void) {
  hostInterface_packet_t dataPacket;

  dataPacket.type    = packetType_advModeToggle;
  dataPacket.length  = 0;
  HostComm_SendMessage(&dataPacket, TRUE);
}

void BLUETOOTH_SendVersionReq(void) {
  hostInterface_packet_t dataPacket;

  dataPacket.type    = packetType_buildVersion;
  dataPacket.length  = 3;
  dataPacket.data[0] = versionK64F.major;
  dataPacket.data[1] = versionK64F.minor;
  dataPacket.data[2] = versionK64F.patch;

  HostComm_SendMessage(&dataPacket, TRUE);
}

void BLUETOOTH_SetVersionInformation(uint8_t major, uint8_t minor, uint8_t patch) {
  versionKW40.major = major;
  versionKW40.minor = minor;
  versionKW40.patch = patch;
}

static uint8_t PrintStatus(const CLS1_StdIOType *io) {
  uint8_t buf[32];

  CLS1_SendStatusStr((unsigned char*)"bluetooth", (unsigned char*)"\r\n", io->stdOut);

  CLS1_SendStatusStr((unsigned char*)"  version", (const unsigned char*)"KW40 V", io->stdOut);
  UTIL1_Num8uToStr(buf, sizeof(buf), versionKW40.major);
  UTIL1_chcat(buf, sizeof(buf), '.');
  UTIL1_strcatNum8u(buf, sizeof(buf), versionKW40.minor);
  UTIL1_chcat(buf, sizeof(buf), '.');
  UTIL1_strcatNum8u(buf, sizeof(buf), versionKW40.patch);
  CLS1_SendStr(buf, io->stdOut);

  UTIL1_strcpy(buf, sizeof(buf), (unsigned char*)", K64F V");
  UTIL1_strcatNum8u(buf, sizeof(buf), versionK64F.major);
  UTIL1_chcat(buf, sizeof(buf), '.');
  UTIL1_strcatNum8u(buf, sizeof(buf), versionK64F.minor);
  UTIL1_chcat(buf, sizeof(buf), '.');
  UTIL1_strcatNum8u(buf, sizeof(buf), versionK64F.patch);
  UTIL1_strcat(buf, sizeof(buf), (uint8_t*)"\r\n");
  CLS1_SendStr(buf, io->stdOut);

  if (currAdvMode==bluetooth_advMode_disable) {
    UTIL1_strcpy(buf, sizeof(buf), "disabled\r\n");
  } else if (currAdvMode==bluetooth_advMode_enable) {
      UTIL1_strcpy(buf, sizeof(buf), "enabled\r\n");
  } else {
    UTIL1_strcpy(buf, sizeof(buf), "UNKNOWN\r\n");
  }
  CLS1_SendStatusStr((unsigned char*)"  adv mode", (const unsigned char*)buf, io->stdOut);

  if (buttonsGroupCurrentActive==buttonsGroup_right) {
    UTIL1_strcpy(buf, sizeof(buf), "right\r\n");
  } else if (buttonsGroupCurrentActive==buttonsGroup_left) {
      UTIL1_strcpy(buf, sizeof(buf), "left\r\n");
  } else {
    UTIL1_strcpy(buf, sizeof(buf), "UNKNOWN\r\n");
  }
  CLS1_SendStatusStr((unsigned char*)"  button grp", (const unsigned char*)buf, io->stdOut);

  if (currLinkState==linkState_disconnected) {
    UTIL1_strcpy(buf, sizeof(buf), "disconnected\r\n");
  } else if (currLinkState==linkState_connected) {
      UTIL1_strcpy(buf, sizeof(buf), "connected\r\n");
  } else {
    UTIL1_strcpy(buf, sizeof(buf), "UNKNOWN\r\n");
  }
  CLS1_SendStatusStr((unsigned char*)"  link state", (const unsigned char*)buf, io->stdOut);

  CLS1_SendStatusStr((unsigned char*)"  appmode", GetCurrentAppModeString(), io->stdOut);
  CLS1_SendStr((unsigned char*)"\r\n", io->stdOut);

  return ERR_OK;
}

uint8_t BLUETOOTH_ParseCommand(const uint8_t *cmd, bool *handled, CLS1_ConstStdIOType *io) {
  const char *p;

  if (UTIL1_strcmp((char*)cmd, CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, "bluetooth help")==0) {
    CLS1_SendHelpStr((unsigned char*)"bluetooth", (const unsigned char*)"Group of bluetooth commands\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  help|status", (const unsigned char*)"Print help or status information\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  req version", (const unsigned char*)"Request version information\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  req advmode", (const unsigned char*)"Request advertisement mode information\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  toggle advmode", (const unsigned char*)"Send request toggling advertisement mode information\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  req buttongrp", (const unsigned char*)"Request touch button group\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  toggle buttongrp", (const unsigned char*)"Send request toggling button group\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  req linkstate", (const unsigned char*)"Request link state\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  appmode <mode>", (const unsigned char*)"Set application mode: idle, sensor, heartrate, pedometer\r\n", io->stdOut);
    *handled = TRUE;
    return ERR_OK;
  } else if ((UTIL1_strcmp((char*)cmd, CLS1_CMD_STATUS)==0) || (UTIL1_strcmp((char*)cmd, "bluetooth status")==0)) {
    *handled = TRUE;
    return PrintStatus(io);
  } else if (UTIL1_strcmp((char*)cmd, "bluetooth req advmode")==0) {
    BLUETOOTH_SendAdvModeGetReq();
    *handled = TRUE;
  } else if (UTIL1_strcmp((char*)cmd, "bluetooth toggle advmode")==0) {
    BLUETOOTH_SendToggleAdvModeReq();
    *handled = TRUE;
  } else if (UTIL1_strcmp((char*)cmd, "bluetooth req version")==0) {
    BLUETOOTH_SendVersionReq();
    *handled = TRUE;
  } else if (UTIL1_strcmp((char*)cmd, "bluetooth req buttongrp")==0) {
    BLUETOOTH_SendActiveButtonsGetReq();
    *handled = TRUE;
  } else if (UTIL1_strcmp((char*)cmd, "bluetooth toggle buttongrp")==0) {
    BLUETOOTH_SendToggleActiveButtonsReq();
    *handled = TRUE;
  } else if (UTIL1_strcmp((char*)cmd, "bluetooth req linkstate")==0) {
    BLUETOOTH_SendLinkStateGetReq();
    *handled = TRUE;
  } else if (UTIL1_strncmp((char*)cmd, "bluetooth appmode ", sizeof("bluetooth appmode ")-1)==0) {
    p = cmd + sizeof("bluetooth appmode ")-1;
    if (UTIL1_strcmp(p, "idle")==0) {
      BLUETOOTH_SetCurrentAppMode(GUI_CURRENT_APP_IDLE);
      *handled = TRUE;
    } else if (UTIL1_strcmp(p, "sensor")==0) {
      BLUETOOTH_SetCurrentAppMode(GUI_CURRENT_APP_SENSOR_TAG);
      *handled = TRUE;
    } else if (UTIL1_strcmp(p, "heartrate")==0) {
      BLUETOOTH_SetCurrentAppMode(GUI_CURRENT_APP_HEART_RATE);
      *handled = TRUE;
    } else if (UTIL1_strcmp(p, "pedometer")==0) {
      BLUETOOTH_SetCurrentAppMode(GUI_CURRENT_APP_PEDOMETER);
      *handled = TRUE;
    } else {
      return ERR_FAILED;
    }
  }
  return ERR_OK; /* no error */
}

void BLUETOOTH_Init(void) {
  BLUETOOTH_SetAdvMode(bluetooth_advMode_disable);
  BLUETOOTH_SetVersionInformation(0,0,0);
  BLUETOOTH_SetLinkState(linkState_disconnected);
  appMode = GUI_CURRENT_APP_IDLE;
}
