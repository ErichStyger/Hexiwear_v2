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

static bluetooth_advMode_t currAdvMode = bluetooth_advMode_disable;

typedef struct {
  uint8_t major, minor, patch;
} versionInfo;

static versionInfo versionKW40;  /* version number of KW40 */
static const versionInfo versionK64F = {10,0,1}; /* K64F version */

void BLUETOOTH_SetAdvMode(bluetooth_advMode_t mode) {
  currAdvMode = mode;
}

bluetooth_advMode_t BLUETOOTH_GetAdvMode(void) {
  return currAdvMode;
}

void BLUETOOTH_SendAdvModeGetReq(void) {
  static hostInterface_packet_t dataPacket =
  {
    .start1 = gHostInterface_startByte1,
    .start2 = gHostInterface_startByte2,
    .length = 0,
    .data[0] = gHostInterface_trailerByte
  };

  dataPacket.type = packetType_advModeGet;
  HostComm_SendMessage(&dataPacket, TRUE);
}

void BLUETOOTH_SendToggleAdvModeReq(void) {
  static hostInterface_packet_t dataPacket =
  {
    .start1 = gHostInterface_startByte1,
    .start2 = gHostInterface_startByte2,
    .length = 0,
    .data[0] = gHostInterface_trailerByte
  };

  dataPacket.type = packetType_advModeToggle;
  HostComm_SendMessage(&dataPacket, TRUE);
}

void BLUETOOTH_SendVersionReq(void) {
  static hostInterface_packet_t dataPacket;

  dataPacket.length  = 3;
  dataPacket.type    = packetType_buildVersion;

  dataPacket.data[0] = versionK64F.major;
  dataPacket.data[1] = versionK64F.minor;
  dataPacket.data[2] = versionK64F.patch;

  dataPacket.data[3] = gHostInterface_trailerByte;

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

  return ERR_OK;
}

uint8_t BLUETOOTH_ParseCommand(const uint8_t *cmd, bool *handled, CLS1_ConstStdIOType *io) {
  if (UTIL1_strcmp((char*)cmd, CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, "bluetooth help")==0) {
    CLS1_SendHelpStr((unsigned char*)"bluetooth", (const unsigned char*)"Group of bluetooth commands\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  help|status", (const unsigned char*)"Print help or status information\r\n", io->stdOut);
    *handled = TRUE;
    return ERR_OK;
  } else if ((UTIL1_strcmp((char*)cmd, CLS1_CMD_STATUS)==0) || (UTIL1_strcmp((char*)cmd, "bluetooth status")==0)) {
    *handled = TRUE;
    return PrintStatus(io);
  }
  return ERR_OK; /* no error */
}


void BLUETOOTH_Init(void) {
  BLUETOOTH_SetAdvMode(bluetooth_advMode_disable);
  BLUETOOTH_SetVersionInformation(0,0,0);
}
