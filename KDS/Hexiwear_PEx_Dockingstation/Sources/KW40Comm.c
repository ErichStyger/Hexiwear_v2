/*
 * KW40Comm.c
 *
 *  Created on: 09.02.2017
 *      Author: Erich Styger
 */

#include "Platform.h"
#if PL_CONFIG_HAS_KW40_COMM
#include "FRTOS1.h"
#include "BLEUart.h"
#include "HostComm.h"
#include "UI.h"
#include "Bluetooth.h"
#include "UART_PDD.h"
#include "CLS1.h"
#include "TmDt1.h"
#include "RStdIO.h"

#define PRING_DEBUG_PACKET_MSG    (1)

#if PRING_DEBUG_PACKET_MSG
  #define DEBUG_PRINT_MSG(msg)    CLS1_SendStr(msg, CLS1_GetStdio()->stdErr)
  #define DEBUG_PRINT_NUM8U(num)  CLS1_SendNum8u(num, CLS1_GetStdio()->stdErr)
  #define DEBUG_PRINT_NUM32S(num) CLS1_SendNum32(num, CLS1_GetStdio()->stdErr)
#else
  #define DEBUG_PRINT_MSG(msg)
  #define DEBUG_PRINT_NUM8U(num)
  #define DEBUG_PRINT_NUM32S(num)
#endif

#define NOF_RX_QUEUE_ITEMS   5
#define NOF_TX_QUEUE_ITEMS   5
static QueueHandle_t rxQueueHandle, txQueueHandle; /* handle for rx and tx messages */

static void PollChar(uint8_t *chp) {
  uint8_t res;

  for(;;) {
    res = BLEUart_RecvChar(chp);
    if (res==ERR_OK) {
      return;
    }
    if (res==ERR_RXEMPTY) {
      vTaskDelay(pdMS_TO_TICKS(10));
    } else { /* error case */
      BLEUart_ClearRxBuf();
    }
  } /* for */
}

static void KW40RxTask(void *param) {
  hostInterface_packet_t packet;
  int i;

  for(;;) {
    PollChar(&packet.start1);
    if (packet.start1==gHostInterface_startByte1) {
      PollChar(&packet.start2);
      if (packet.start2==gHostInterface_startByte2 || packet.start2==gHostInterface_startByte2_ack) {
        PollChar(&packet.type);
        PollChar(&packet.length);
        for(i=0; i<packet.length+1 && i<sizeof(packet.data); i++) {
          PollChar(&packet.data[i]);
        }
        if (packet.data[packet.length]==gHostInterface_trailerByte) { /* valid packet */
          (void)xQueueSendToBack(rxQueueHandle, &packet, portMAX_DELAY); /* put item into queue */
        }
      }
    } else {
      /* wrong start byte, throw it away */
    }
  }
}

#if 0
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

#endif

static void DebugPrintMsg(unsigned char *preStr, hostInterface_packet_t *packet) {
#if PRING_DEBUG_PACKET_MSG
  char *str;
  uint8_t buf[16], databuf[48];
  int i;

  databuf[0] = '\0';
  CLS1_SendStr(preStr, CLS1_GetStdio()->stdOut);
  switch(packet->type) {
    case packetType_pressUp:                  str = "pressUp"; break;
    case packetType_pressDown:                str = "pressDown"; break;
    case packetType_pressRight:               str = "pressRight"; break;
    case packetType_pressLeft:                str = "pressLeft"; break;
    case packetType_slide:                    str = "slide"; break;
    case packetType_passDisplay:              str = "passDisplay";
                                              UTIL1_Num32uToStr(databuf, sizeof(databuf), (((int)packet->data[2])<<16)+(((int)packet->data[1])<<8)+packet->data[0]);
                                              break;
    case packetType_buildVersion:             str = "buildVersion";
                                              UTIL1_Num8uToStr(databuf, sizeof(databuf), packet->data[0]);
                                              UTIL1_chcat(databuf, sizeof(databuf), '.');
                                              UTIL1_strcatNum8u(databuf, sizeof(databuf), packet->data[1]);
                                              UTIL1_chcat(databuf, sizeof(databuf), '.');
                                              UTIL1_strcatNum8u(databuf, sizeof(databuf), packet->data[2]);
                                              break;

    case packetType_advModeSend:              str = "advModeSend";
                                              UTIL1_Num8uToStr(databuf, sizeof(databuf), packet->data[0]);
                                              if (packet->data[0]==0) {
                                                UTIL1_strcat(databuf, sizeof(databuf), (uint8_t*)" disabled");
                                              } else if (packet->data[0]==1) {
                                                UTIL1_strcat(databuf, sizeof(databuf), (uint8_t*)" advertising");
                                              } else {
                                                UTIL1_strcat(databuf, sizeof(databuf), (uint8_t*)" wrong data!");
                                              }
                                              break;
    case packetType_advModeGet:               str = "advModeGet"; break;
    case packetType_advModeToggle:            str = "advModeToggle"; break;

    case packetType_batteryLevel:             str = "battyLevel"; break;
    case packetType_accel:                    str = "accel";
                                              databuf[0] = '\0';
                                              UTIL1_strcatNum8Hex(databuf, sizeof(databuf), packet->data[0]);
                                              UTIL1_strcatNum8Hex(databuf, sizeof(databuf), packet->data[1]);
                                              UTIL1_chcat(databuf, sizeof(databuf), ' ');
                                              UTIL1_strcatNum8Hex(databuf, sizeof(databuf), packet->data[2]);
                                              UTIL1_strcatNum8Hex(databuf, sizeof(databuf), packet->data[3]);
                                              UTIL1_chcat(databuf, sizeof(databuf), ' ');
                                              UTIL1_strcatNum8Hex(databuf, sizeof(databuf), packet->data[4]);
                                              UTIL1_strcatNum8Hex(databuf, sizeof(databuf), packet->data[5]);
                                              break;
    case packetType_ambiLight:                str = "ambilight";
                                              UTIL1_Num8uToStr(databuf, sizeof(databuf), packet->data[0]);
                                              UTIL1_chcat(databuf, sizeof(databuf), '%');
                                              break;
    case packetType_pressure:                 str = "pressure"; break;
    case packetType_gyro:                     str = "gyro"; break;
    case packetType_temperature:              str = "temperature"; break;
    case packetType_humidity:                 str = "humidity"; break;
    case packetType_magnet:                   str = "magnet";
                                              databuf[0] = '\0';
                                              UTIL1_strcatNum8Hex(databuf, sizeof(databuf), packet->data[0]);
                                              UTIL1_strcatNum8Hex(databuf, sizeof(databuf), packet->data[1]);
                                              UTIL1_chcat(databuf, sizeof(databuf), ' ');
                                              UTIL1_strcatNum8Hex(databuf, sizeof(databuf), packet->data[2]);
                                              UTIL1_strcatNum8Hex(databuf, sizeof(databuf), packet->data[3]);
                                              UTIL1_chcat(databuf, sizeof(databuf), ' ');
                                              UTIL1_strcatNum8Hex(databuf, sizeof(databuf), packet->data[4]);
                                              UTIL1_strcatNum8Hex(databuf, sizeof(databuf), packet->data[5]);
                                              break;
    case packetType_heartRate:                str = "heartrate"; break;
    case packetType_steps:                    str = "steps"; break;
    case packetType_calories:                 str = "calories"; break;
    case packetType_alertIn:                  str = "alertin";
                                              switch(packet->data[0]) {
                                                case alertIn_type_notification:
                                                  UTIL1_strcpy(databuf, sizeof(databuf), (uint8_t*)"notification ");
                                                  break;
                                                case alertIn_type_settings:
                                                  /* data[0] 03 (alertIn_type_settings)
                                                   * data[1] size
                                                   * data[2] id
                                                   * data[3+] data
                                                   * */
                                                  UTIL1_strcpy(databuf, sizeof(databuf), (uint8_t*)"settings ");
                                                  UTIL1_strcatNum8Hex(databuf, sizeof(databuf), packet->data[1]); /* size */
                                                  UTIL1_chcat(databuf, sizeof(databuf), ' ');
                                                  UTIL1_strcatNum8Hex(databuf, sizeof(databuf), packet->data[2]); /* id */
                                                  for(i=1; i<packet->data[1]; i++) { /* print all other data bytes */
                                                    UTIL1_chcat(databuf, sizeof(databuf), ' ');
                                                    UTIL1_strcatNum8Hex(databuf, sizeof(databuf), packet->data[2+i]); /* data */
                                                  }
                                                  break;
                                                case alertIn_type_timeUpdate:
                                                  UTIL1_strcpy(databuf, sizeof(databuf), (uint8_t*)"timeUpdate ");
                                                  /* packet->data[1] is size of data, must be 4! */
                                                  if (packet->data[1]==4) {
                                                    uint32_t secs;
                                                    TIMEREC time;
                                                    DATEREC date;

                                                    secs = (((int)packet->data[5])<<24)+(((int)packet->data[4])<<16)+(packet->data[3]<<8)+packet->data[2];
                                                    TmDt1_UnixSecondsToTimeDate(secs, 0, &time, &date);
                                                    TmDt1_AddDateString(databuf, sizeof(databuf), &date, TmDt1_DEFAULT_DATE_FORMAT_STR);
                                                    UTIL1_strcat(databuf, sizeof(databuf), (uint8_t*)", ");
                                                    TmDt1_AddTimeString(databuf, sizeof(databuf), &time, TmDt1_DEFAULT_TIME_FORMAT_STR);
                                                  } else {
                                                    UTIL1_strcat(databuf, sizeof(databuf), (uint8_t*)"wrong size!");
                                                  }
                                                  break;
                                              }
                                              break;
    case packetType_alertOut:                 str = "alertout"; break;
    case packetType_appMode:                  str = "appMode";
                                              UTIL1_Num8uToStr(databuf, sizeof(databuf), packet->data[0]);
                                              break;
    case packetType_linkStateGet:             str = "linkStateGet";  break;
    case packetType_linkStateSend:            str = "linkStateSend";
                                              UTIL1_Num8uToStr(databuf, sizeof(databuf), packet->data[0]);
                                              if (packet->data[0]==0) {
                                                UTIL1_strcat(databuf, sizeof(databuf), (uint8_t*)" disconnected");
                                              } else if (packet->data[0]==1) {
                                                UTIL1_strcat(databuf, sizeof(databuf), (uint8_t*)" connected");
                                              } else {
                                                UTIL1_strcat(databuf, sizeof(databuf), (uint8_t*)" wrong data!");
                                              }
                                              break;
    case packetType_otapKW40Started:          str = "otapKW40Started"; break;
    case packetType_otapMK64Started:          str = "otap;MK64Started"; break;
    case packetType_otapCompleted:            str = "otapCompleted"; break;
    case packetType_otapFailed:               str = "otapFailed"; break;
    case packetType_buttonsGroupToggleActive: str = "buttonsGroupToggleActive"; break;
    case packetType_buttonsGroupGetActive:    str = "buttonsGroupGetActive"; break;
    case packetType_buttonsGroupSendActive:   str = "buttonsGroupSendActive";
                                              UTIL1_Num8uToStr(databuf, sizeof(databuf), packet->data[0]);
                                              if (packet->data[0]==0) {
                                                UTIL1_strcat(databuf, sizeof(databuf), (uint8_t*)" left");
                                              } else if (packet->data[0]==1) {
                                                UTIL1_strcat(databuf, sizeof(databuf), (uint8_t*)" right");
                                              } else {
                                                UTIL1_strcat(databuf, sizeof(databuf), (uint8_t*)" wrong?");
                                              }
                                              break;
    case packetType_notification:             str = "notification"; break;
    case packetType_sleepON:                  str = "sleepON"; break;
    case packetType_sleepOFF:                 str = "sleepOFF"; break;
    case packetType_OK:                       str = "OK"; break;
    default:
      UTIL1_Num8uToStr(buf, sizeof(buf), packet->type);
      str = buf;
      break;
  }
  CLS1_SendStr(str, CLS1_GetStdio()->stdOut);
  if (databuf[0]!='\0') {
    CLS1_SendStr(" ", CLS1_GetStdio()->stdOut);
    CLS1_SendStr(databuf, CLS1_GetStdio()->stdOut);
  }
  CLS1_SendStr("\r\n", CLS1_GetStdio()->stdOut);
#else
  (void)packet;
#endif
}

static void KW40TxTask(void *param) {
  hostInterface_packet_t packet;
  BaseType_t res;
  uint16_t snt;

  for(;;) {
    res = xQueueReceive(txQueueHandle, &packet, portMAX_DELAY); /* get Tx packet */
    if (res==pdPASS) { /* received packet */
      DebugPrintMsg("Tx:", &packet);
      BLEUart_SendBlock((uint8_t*)&packet, sizeof(packet.start1)+sizeof(packet.start2)+sizeof(packet.type)+sizeof(packet.length)+packet.length+1/*trailer*/, &snt);
    }
  }
}

static void HandleRxPacket(hostInterface_packet_t *packet) {
  switch(packet->type) {
    case packetType_pressUp:
      UI_Event(UI_EVENT_BUTTON_UP, NULL);
      break;
    case packetType_pressDown:
      UI_Event(UI_EVENT_BUTTON_DOWN, NULL);
      break;
    case packetType_pressRight:
      UI_Event(UI_EVENT_BUTTON_RIGHT, NULL);
      break;
    case packetType_pressLeft:
      UI_Event(UI_EVENT_BUTTON_LEFT, NULL);
      break;
    case packetType_passDisplay:
      {
        uint32_t passkey = 0;
        passkey = (((int)packet->data[2])<<16)+(((int)packet->data[1])<<8)+packet->data[0];
        UI_Event(UI_EVENT_PARING_CODE, &passkey);
      }
      break;
    case packetType_advModeSend: /* response to packetType_advModeGet */
      BLUETOOTH_SetAdvMode(packet->data[0]);
      break;
    case packetType_buttonsGroupSendActive: /* response to packetType_buttonsGroupGetActive */
      BLUETOOTH_SetActiveButtons(packet->data[0]);
      break;
    case packetType_buildVersion:
      BLUETOOTH_SetVersionInformation(packet->data[0], packet->data[1], packet->data[2]);
      break;

    case packetType_linkStateSend: /* response to packetType_linkStateGet */
      BLUETOOTH_SetLinkState(packet->data[0]);
      break;

    case packetType_alertIn:
      switch(packet->data[0]) {
        case alertIn_type_notification:
          break;
        case alertIn_type_settings:
          {
          /* data[0] 03 (alertIn_type_settings)
           * data[1] size
           * data[2] id
           * data[3+] data
           * */
            uint8_t size;
            uint8_t id;

            size = packet->data[1];
            if (size>=1) { /* safety check */
              id = packet->data[2];
              if (id==alertIn_type_setting_stdin) {
                (void)RSTDIO_AddToQueue(RSTDIO_QUEUE_RX_IN, &packet->data[3], size-1);
              } else if (id==alertIn_type_setting_stdout) {
                (void)RSTDIO_AddToQueue(RSTDIO_QUEUE_RX_OUT, &packet->data[3], size-1);
              } else if (id==alertIn_type_setting_stderr) {
                (void)RSTDIO_AddToQueue(RSTDIO_QUEUE_RX_ERR, &packet->data[3], size-1);
              }
            }
          }
          break;
        case alertIn_type_timeUpdate:
          /* packet->data[1] is size of data, must be 4! */
          if (packet->data[1]==4) {
            uint32_t secs;
            TIMEREC time;
            DATEREC date;

            secs = (((int)packet->data[5])<<24)+(((int)packet->data[4])<<16)+(packet->data[3]<<8)+packet->data[2];
            TmDt1_UnixSecondsToTimeDate(secs, 0, &time, &date);
            TmDt1_SetTimeDate(&time, &date);
          }
          break;
      }
      break;

    default:
      break;
  } /* switch */
}

static void KW40CommTask(void *param) {
  hostInterface_packet_t packet;
  BaseType_t res;

  for(;;) {
    res = xQueueReceive(rxQueueHandle, &packet, portMAX_DELAY); /* get Rx packet */
    if (res==pdPASS) { /* received packet */
      if (packet.start2==gHostInterface_startByte2_ack) {
        HostComm_SendOK(); /* put item into queue */
      }
      DebugPrintMsg("Rx:", &packet);
      HandleRxPacket(&packet);
    }
  }
}

void KW40SendPacket(const hostInterface_packet_t *packet) {
  (void)xQueueSendToBack(txQueueHandle, packet, portMAX_DELAY); /* put item into queue */
}

void KW40Comm_Init(void) {
  /* communication is with 230400 baud on UART4: PTE25 (RxD) and PTE24 (RxD).
   * 8 data bits
   * 2 (!!!) stop bits  ==> need to set this manually as the AsynchroSerial component does not support this setting!
   * no parity
   */
  UART_PDD_SetStopBitLength(UART4_BASE_PTR, UART_PDD_STOP_BIT_LEN_2); /* two stop bits */

  rxQueueHandle = xQueueCreate(NOF_RX_QUEUE_ITEMS, sizeof(hostInterface_packet_t));
  if (rxQueueHandle==NULL) {
    for(;;){} /* error case only, stay here! */
  }
  vQueueAddToRegistry(rxQueueHandle, "KW40Rx");

  txQueueHandle = xQueueCreate(NOF_TX_QUEUE_ITEMS, sizeof(hostInterface_packet_t));
  if (txQueueHandle==NULL) {
    for(;;){} /* error case only, stay here! */
  }
  vQueueAddToRegistry(txQueueHandle, "KW40Tx");

  if (xTaskCreate(KW40RxTask, (uint8_t *)"KW40Rx", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+2, NULL) != pdPASS) {
    for(;;){} /* error case only, stay here! */
  }
  if (xTaskCreate(KW40TxTask, (uint8_t *)"KW40Tx", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+2, NULL) != pdPASS) {
    for(;;){} /* error case only, stay here! */
  }
  if (xTaskCreate(KW40CommTask, (uint8_t *)"KW40Comm", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+2, NULL) != pdPASS) {
    for(;;){} /* error case only, stay here! */
  }
}

#endif /* PL_CONFIG_HAS_KW40_COMM */
