/*
 * HostComm.c
 *
 *  Created on: 09.02.2017
 *      Author: Erich Styger
 */

#include "Platform.h"
#include "HostComm.h"
#include "KW40Comm.h"
#include <string.h> /* for memcpy() */

void HostComm_SendOK(void) {
  static const hostInterface_packet_t okPacket = {
    .start1 = gHostInterface_startByte1,
    .start2 = gHostInterface_startByte2,
    .type   = packetType_OK,
    .length = 0,
    .data[0] = gHostInterface_trailerByte
  };

  KW40SendPacket(&okPacket);
}

void HostComm_SendBatteryLevel(uint8_t percentage) {
  hostInterface_packet_t txPacket;

  txPacket.start1     = gHostInterface_startByte1;
  txPacket.start2     = gHostInterface_startByte2;
  txPacket.type       = packetType_batteryLevel;
  txPacket.length     = 1;
  txPacket.data[0]    = percentage;
  txPacket.data[1]    = gHostInterface_trailerByte;

  KW40SendPacket(&txPacket);
}

void HostComm_SendAccel(int16_t x, int16_t y, int16_t z) {
  hostInterface_packet_t txPacket;

  txPacket.start1     = gHostInterface_startByte1;
  txPacket.start2     = gHostInterface_startByte2;
  txPacket.type       = packetType_accel;
  txPacket.length     = 6;

  txPacket.data[0]    = (uint8_t) ((x >> 8)&0xFF);
  txPacket.data[1]    = (uint8_t) x;
  txPacket.data[2]    = (uint8_t) ((y >> 8)&0xFF);
  txPacket.data[3]    = (uint8_t) y;
  txPacket.data[4]    = (uint8_t) ((z >> 8)&0xFF);
  txPacket.data[5]    = (uint8_t) z;
  txPacket.data[6]    = gHostInterface_trailerByte;
  KW40SendPacket(&txPacket);
}

void HostComm_SendGyro(int16_t x, int16_t y, int16_t z) {
  hostInterface_packet_t txPacket;

  txPacket.start1     = gHostInterface_startByte1;
  txPacket.start2     = gHostInterface_startByte2;
  txPacket.type       = packetType_gyro;
  txPacket.length     = 6;
  txPacket.data[0]    = (uint8_t) ((x >> 8)&0xFF);
  txPacket.data[1]    = (uint8_t) x;
  txPacket.data[2]    = (uint8_t) ((y >> 8)&0xFF);
  txPacket.data[3]    = (uint8_t) y;
  txPacket.data[4]    = (uint8_t) ((z >> 8)&0xFF);
  txPacket.data[5]    = (uint8_t) z;
  txPacket.data[6]    = gHostInterface_trailerByte;
  KW40SendPacket(&txPacket);
}

void HostComm_SendMag(int16_t x, int16_t y, int16_t z) {
  hostInterface_packet_t txPacket;

  txPacket.start1     = gHostInterface_startByte1;
  txPacket.start2     = gHostInterface_startByte2;
  txPacket.type       = packetType_magnet;
  txPacket.length     = 6;
  txPacket.data[0]    = (uint8_t) ((x >> 8)&0xFF);
  txPacket.data[1]    = (uint8_t) x;
  txPacket.data[2]    = (uint8_t) ((y >> 8)&0xFF);
  txPacket.data[3]    = (uint8_t) y;
  txPacket.data[4]    = (uint8_t) ((z >> 8)&0xFF);
  txPacket.data[5]    = (uint8_t) z;
  txPacket.data[6]    = gHostInterface_trailerByte;
  KW40SendPacket(&txPacket);
}

void HostComm_SendAmbientLight(uint8_t percentage) {
  hostInterface_packet_t txPacket;

  txPacket.start1     = gHostInterface_startByte1;
  txPacket.start2     = gHostInterface_startByte2;
  txPacket.type       = packetType_ambiLight;
  txPacket.length     = 1;
  txPacket.data[0]    = percentage;
  txPacket.data[1]    = gHostInterface_trailerByte;
  KW40SendPacket(&txPacket);
}

void HostComm_SendTemperature(uint16_t celsius) {
  hostInterface_packet_t txPacket;

  txPacket.start1     = gHostInterface_startByte1;
  txPacket.start2     = gHostInterface_startByte2;
  txPacket.type       = packetType_temperature;
  txPacket.length     = 2;
  memcpy(&txPacket.data[0],(uint8_t*)&celsius,txPacket.length);
  txPacket.data[2]    = gHostInterface_trailerByte;
  KW40SendPacket(&txPacket);
}

void HostComm_SendHumidity(uint16_t percentage) {
  hostInterface_packet_t txPacket;

  txPacket.start1     = gHostInterface_startByte1;
  txPacket.start2     = gHostInterface_startByte2;
  txPacket.type       = packetType_humidity;
  txPacket.length     = 2;
  memcpy(&txPacket.data[0],(uint8_t*)&percentage,txPacket.length);
  txPacket.data[2]    = gHostInterface_trailerByte;
  KW40SendPacket(&txPacket);
}

void HostComm_SendPressure(uint16_t pascal) {
  hostInterface_packet_t txPacket;

  txPacket.start1     = gHostInterface_startByte1;
  txPacket.start2     = gHostInterface_startByte2;
  txPacket.type       = packetType_pressure;
  txPacket.length     = 2;
  memcpy(&txPacket.data[0],(uint8_t*)&pascal,txPacket.length);
  txPacket.data[2]    = gHostInterface_trailerByte;
  KW40SendPacket(&txPacket);
}

void HostComm_SendHeartRate(uint8_t rate) {
  hostInterface_packet_t txPacket;

  txPacket.start1     = gHostInterface_startByte1;
  txPacket.start2     = gHostInterface_startByte2;
  txPacket.type       = packetType_steps;
  txPacket.length     = 1;
  txPacket.data[0]    = rate;
  txPacket.data[1]    = gHostInterface_trailerByte;
  KW40SendPacket(&txPacket);
}

void HostComm_SendSteps(uint16_t steps) {
  hostInterface_packet_t txPacket;

  txPacket.start1     = gHostInterface_startByte1;
  txPacket.start2     = gHostInterface_startByte2;
  txPacket.type       = packetType_steps;
  txPacket.length     = 2;
  memcpy(&txPacket.data[0],(uint8_t*)&steps,txPacket.length);
  txPacket.data[2]    = gHostInterface_trailerByte;
  KW40SendPacket(&txPacket);
}

void HostComm_SendCalories(uint16_t calories) {
  hostInterface_packet_t txPacket;

  txPacket.start1     = gHostInterface_startByte1;
  txPacket.start2     = gHostInterface_startByte2;
  txPacket.type       = packetType_calories;
  txPacket.length     = 2;
  memcpy(&txPacket.data[0],(uint8_t*)&calories,txPacket.length);
  txPacket.data[2]    = gHostInterface_trailerByte;
  KW40SendPacket(&txPacket);
}

void HostComm_SendAlert(uint8_t *pData, uint8_t length) {
  hostInterface_packet_t txPacket;

  txPacket.start1     = gHostInterface_startByte1;
  txPacket.start2     = gHostInterface_startByte2;
  txPacket.type       = packetType_alertOut;
  txPacket.length     = length;
  memcpy(&txPacket.data[0],pData,length);
  txPacket.data[length] = gHostInterface_trailerByte;
  KW40SendPacket(&txPacket);
}

void HostComm_ToggleTsiGroup(void) {
  hostInterface_packet_t txPacket;

  txPacket.start1     = gHostInterface_startByte1;
  txPacket.start2     = gHostInterface_startByte2;
  txPacket.type       = packetType_buttonsGroupToggleActive;
  txPacket.length     = 0;
  txPacket.data[0]    = gHostInterface_trailerByte;
  KW40SendPacket(&txPacket);
}

void HostComm_ToggleAdvertisementMode(void) {
  hostInterface_packet_t txPacket;

  txPacket.start1     = gHostInterface_startByte1;
  txPacket.start2     = gHostInterface_startByte2;
  txPacket.type       = packetType_advModeToggle;
  txPacket.length     = 0;
  txPacket.data[0]    = gHostInterface_trailerByte;
  KW40SendPacket(&txPacket);
}

void HostComm_SendSetApplicationMode(uint8_t mode) {
  hostInterface_packet_t txPacket;

  txPacket.start1     = gHostInterface_startByte1;
  txPacket.start2     = gHostInterface_startByte2;
  txPacket.type       = packetType_appMode;
  txPacket.length     = 1;
  txPacket.data[0]    = (uint8_t)mode;
  txPacket.data[1]    = gHostInterface_trailerByte;
  KW40SendPacket(&txPacket);
}

void HostComm_SendGetActiveTsiGroup(void) {
  hostInterface_packet_t txPacket;

  txPacket.start1     = gHostInterface_startByte1;
  txPacket.start2     = gHostInterface_startByte2;
  txPacket.type       = packetType_buttonsGroupGetActive;
  txPacket.length     = 0;
  txPacket.data[0]    = gHostInterface_trailerByte;
  KW40SendPacket(&txPacket);
}

void HostComm_SendGetAdvertisementMode(void) {
  hostInterface_packet_t txPacket;

  txPacket.start1     = gHostInterface_startByte1;
  txPacket.start2     = gHostInterface_startByte2;
  txPacket.type       = packetType_advModeGet;
  txPacket.length     = 0;
  txPacket.data[0]    = gHostInterface_trailerByte;
  KW40SendPacket(&txPacket);
}

void HostComm_SendGetLinkState(void) {
  hostInterface_packet_t txPacket;

  txPacket.start1     = gHostInterface_startByte1;
  txPacket.start2     = gHostInterface_startByte2;
  txPacket.type       = packetType_linkStateGet;
  txPacket.length     = 0;
  txPacket.data[0]    = gHostInterface_trailerByte;
  KW40SendPacket(&txPacket);
}

void HostComm_SendGetVersion(uint8_t major, uint8_t minor, uint8_t patch) {
  hostInterface_packet_t txPacket;

  txPacket.start1     = gHostInterface_startByte1;
  txPacket.start2     = gHostInterface_startByte2;
  txPacket.type       = packetType_buildVersion;
  txPacket.length     = 3;
  txPacket.data[0]    = major;
  txPacket.data[1]    = minor;
  txPacket.data[2]    = patch;
  txPacket.data[3]    = gHostInterface_trailerByte;
  KW40SendPacket(&txPacket);
}

void HostComm_Init(void) {
}
