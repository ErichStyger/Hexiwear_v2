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

static void KW40TxTask(void *param) {
  hostInterface_packet_t packet;
  BaseType_t res;
  uint16_t snt;

  for(;;) {
    res = xQueueReceive(txQueueHandle, &packet, portMAX_DELAY); /* get Tx packet */
    if (res==pdPASS) { /* received packet */
      BLEUart_SendBlock((uint8_t*)&packet, sizeof(packet.start1)+sizeof(packet.start2)+sizeof(packet.type)+sizeof(packet.length)+packet.length, &snt);
    }
  }
}

static void HandlePacket(hostInterface_packet_t *packet) {
  switch(packet->type) {
    case packetType_pressUp:
      UI_Event(UI_EVENT_BUTTON_UP);
      break;
    case packetType_pressDown:
      UI_Event(UI_EVENT_BUTTON_DOWN);
      break;
    case packetType_pressRight:
      UI_Event(UI_EVENT_BUTTON_RIGHT);
      break;
    case packetType_pressLeft:
      UI_Event(UI_EVENT_BUTTON_LEFT);
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
      HandlePacket(&packet);
    }
  }
}

void KW40SendPacket(const hostInterface_packet_t *packet) {
  (void)xQueueSendToBack(txQueueHandle, packet, portMAX_DELAY); /* put item into queue */
}

void KW40Comm_Init(void) {
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
