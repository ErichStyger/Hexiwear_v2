/*
 * HostComm.h
 *
 *  Created on: 09.02.2017
 *      Author: Erich Styger Local
 */

#ifndef SOURCES_HOSTCOMM_H_
#define SOURCES_HOSTCOMM_H_

/* communication packet types */
typedef enum {
  packetType_pressUp          = 0, /**< touch press up */
  packetType_pressDown        = 1, /**< touch press down */
  packetType_pressLeft        = 2, /**< touch press left */
  packetType_pressRight       = 3, /**< touch press right */
  packetType_slide            = 4, /**< touch slide */

  packetType_batteryLevel     = 5, /**< battery Service */

  packetType_accel            = 6, /**< motion service */
  packetType_ambiLight        = 7, /**< weather service */
  packetType_pressure         = 8, /**< weather service */

  packetType_gyro             = 9,  /**< motion service */
  packetType_temperature      = 10, /**< weather service */
  packetType_humidity         = 11, /**< weather service */
  packetType_magnet           = 12, /**< motion service */

  packetType_heartRate        = 13, /**< health service */
  packetType_steps            = 14, /**< health service */
  packetType_calories         = 15, /**< health service */

  // Alert Service
  packetType_alertIn          = 16, /**<  incoming alerts */
  packetType_alertOut         = 17, /**<  outcoming alerts */

  packetType_passDisplay      = 18, /**< key display type */

  // OTAP procedure types
  packetType_otapKW40Started  = 19,
  packetType_otapMK64Started  = 20,
  packetType_otapCompleted    = 21,
  packetType_otapFailed       = 22,

  // active buttons types
  packetType_buttonsGroupToggleActive = 23,
  packetType_buttonsGroupGetActive    = 24,
  packetType_buttonsGroupSendActive   = 25,

    // Turn off/on bluetooth advertising
  packetType_advModeGet    = 26,
  packetType_advModeSend   = 27,
  packetType_advModeToggle = 28,

  packetType_appMode       = 29, /**< app mode service */

  // Link State
  packetType_linkStateGet  = 30, /**< connected */
  packetType_linkStateSend = 31, /**< disconnected */

  packetType_notification  = 32, /* notifications */

  packetType_buildVersion = 33, /**< build version */

  packetType_sleepON = 34,  /**< sleep ON */

  packetType_sleepOFF = 35, /**< sleep OFF */

  packetType_OK = 255       /**< OK packet */
} hostInterface_packetType_t;

/* different packetType_alertIn types */
typedef enum  {
  alertIn_type_notification      = 1,
  alertIn_type_settings          = 2,
  alertIn_type_timeUpdate        = 3,
} hostInterface_alertIn_type_t;

/*! Category of iOS notification */
typedef enum {
    ancCategoryId_other           = 0,
    ancCategoryId_incomingCall    = 1,
    ancCategoryId_missedCall      = 2,
    ancCategoryId_voiceMail       = 3,
    ancCategoryId_social          = 4,
    ancCategoryId_schedule        = 5,
    ancCategoryId_email           = 6,
    ancCategoryId_news            = 7,
    ancCategoryId_healthFitness   = 8,
    ancCategoryId_businessFinance = 9,
    ancCategoryId_location        = 10,
    ancCategoryId_entertainment   = 11
} ancCategoryId_t;



/* special constants */
#define gHostInterface_startByte1               0x55
#define gHostInterface_startByte2               0xAA
#define gHostInterface_startByte2_ack           (gHostInterface_startByte2|0x1)
#define gHostInterface_trailerByte              0x45

#define gHostInterface_dataSize     23

typedef struct {
  uint8_t start1; /* gHostInterface_startByte1 */
  uint8_t start2; /* gHostInterface_startByte2 */
  hostInterface_packetType_t type;
  uint8_t length;
  uint8_t data[gHostInterface_dataSize+1];
} hostInterface_packet_t;


void HostComm_SendMessage(hostInterface_packet_t *msg, bool confirmationReq);
void HostComm_SendOK(void);
void HostComm_SendBatteryLevel(uint8_t percentage);
void HostComm_SendAccel(int16_t x, int16_t y, int16_t z);
void HostComm_SendGyro(int16_t x, int16_t y, int16_t z);
void HostComm_SendMag(int16_t x, int16_t y, int16_t z);
void HostComm_SendAmbientLight(uint8_t percentage);
void HostComm_SendTemperature(uint16_t celsius);
void HostComm_SendHumidity(uint16_t percentage);
void HostComm_SendPressure(uint16_t pascal);
void HostComm_SendHeartRate(uint8_t rate);
void HostComm_SendSteps(uint16_t steps);
void HostComm_SendCalories(uint16_t calories);
void HostComm_SendAlert(uint8_t *pData, uint8_t length);
void HostComm_ToggleTsiGroup(void);
void HostComm_ToggleAdvertisementMode(void);
void HostComm_SendSetApplicationMode(uint8_t mode);
void HostComm_SendGetActiveTsiGroup(void);
void HostComm_SendGetAdvertisementMode(void);
void HostComm_SendGetLinkState(void);
void HostComm_SendGetVersion(uint8_t major, uint8_t minor, uint8_t patch);

void HostComm_Init(void);

#endif /* SOURCES_HOSTCOMM_H_ */
