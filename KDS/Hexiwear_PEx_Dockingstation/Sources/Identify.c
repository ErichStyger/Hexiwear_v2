/*
 * Identify.c
 *
 *  Created on: 26.11.2015
 *      Author: tastyger
 */

#include "Platform.h"
#if PL_CONFIG_HAS_IDENTIFY
#include "Identify.h"
#include "KIN1.h"

static ID_Hexis currHexi = ID_HEXI_NONE;

static const KIN1_UID idTable[ID_HEXI_NOF_IDS] =
  { /* order has to match enumeration in ID_Robots! */
    /* ID_HEXI_01  */ {{0x00,0x2F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x4E,0x45,0x30,0x51,0x40,0x01,0x00,0x10}},
  };

static ID_Hexis Identify(void) {
  uint8_t res;
  KIN1_UID id;
  ID_Hexis i, device;

  device = ID_HEXI_UNKNOWN;
  res = KIN1_UIDGet(&id);
  if (res==ERR_OK) {
    for(i=(ID_Hexis)0; i<ID_HEXI_NOF_IDS; i++) {
      if (KIN1_UIDSame(&id, &idTable[i])) {
        device = i; /* found it */
        break; /* get out of for() loop */
      }
    }
  }
  return device;
}

ID_Hexis ID_WhichHexi(void) {
  if (currHexi==ID_HEXI_NONE) {
    /* not checked ID, try to find matching hexi */
    currHexi = Identify();
  }
  return currHexi;
}

void ID_Deinit(void) {
  currHexi = ID_HEXI_NONE;
}

void ID_Init(void) {
  currHexi = ID_HEXI_NONE;
}
#endif /* PL_CONFIG_HAS_IDENTIFY */
