/*
 * Bluetooth.c
 *
 *  Created on: 05.03.2017
 *      Author: Erich Styger Local
 */

#include "Platform.h"
#include "Bluetooth.h"
#include "HostComm.h"

static bluetooth_advMode_t currAdvMode = bluetooth_advMode_disable;

static struct {
  uint8_t major, minor, patch;
} version; /* version number of KW40 */

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

  dataPacket.data[0] = 10; /* dummy */
  dataPacket.data[1] = 11;
  dataPacket.data[2] = 12;

  dataPacket.data[3] = gHostInterface_trailerByte;

  HostComm_SendMessage(&dataPacket, TRUE);
}

void BLUETOOTH_SetVersionInformation(uint8_t major, uint8_t minor, uint8_t patch) {
  version.major = major;
  version.minor = minor;
  version.patch = patch;
}

void BLUETOOTH_Init(void) {
  BLUETOOTH_SetAdvMode(bluetooth_advMode_disable);
  BLUETOOTH_SetVersionInformation(0,0,0);
}
