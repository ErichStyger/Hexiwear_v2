/**
 * \file
 * \brief Module to identify different devices based on their unique ID.
 * \author Erich Styger, erich.styger@hslu.ch
 *
 * With this module individual devices are identified based on their unique ID.
 */

#include "Platform.h"
#if PL_CONFIG_HAS_IDENTIFY
#include "Identify.h"
#include "KIN1.h"
#include "UTIL1.h"

static ID_Devices currDevice = ID_DEVICE_NONE;

typedef struct {
  ID_Devices id;
  const unsigned char *name;
  KIN1_UID uuid;
} ID_Device;

static const ID_Device idTable[] =
{
  {.id=ID_DEVICE_HEXI_01,  .name=(const unsigned char*)"Hexi_01 ??:??:??:??:??:??", .uuid={{0x00,0x2F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x4E,0x45,0x30,0x51,0x40,0x01,0x00,0x10}}},
  {.id=ID_DEVICE_HEXI_24,  .name=(const unsigned char*)"Hexi_24 00:29:40:08:00:01", .uuid={{0x00,0x25,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x4E,0x45,0x30,0x51,0x40,0x01,0x00,0x11}}},
};

static ID_Devices IdentifyDevice(void) {
  uint8_t res;
  KIN1_UID id;
  unsigned int i;

  res = KIN1_UIDGet(&id);
  if (res==ERR_OK) {
    for(i=0; i<sizeof(idTable)/sizeof(idTable[0]); i++) {
      if (KIN1_UIDSame(&id, &idTable[i].uuid)) {
        return idTable[i].id; /* found it */
      }
    }
  }
  return ID_DEVICE_UNKNOWN;
}

static const ID_Device *GetDeviceDesc(ID_Devices id) {
  unsigned int i;

  for(i=0; i<sizeof(idTable)/sizeof(idTable[0]); i++) {
    if (idTable[i].id == id) {
      return &idTable[i]; /* found it */
    }
  }
  return NULL;
}


ID_Devices ID_WhichDevice(void) {
  if (currDevice==ID_DEVICE_NONE) {
    /* not checked ID, try to find matching robot */
    currDevice = IdentifyDevice();
  }
  return currDevice;
}

static uint8_t PrintStatus(const CLS1_StdIOType *io) {
  uint8_t buf[32];
  const ID_Device *device;

  CLS1_SendStatusStr((unsigned char*)"id", (unsigned char*)"\r\n", io->stdOut);
  device = GetDeviceDesc(ID_WhichDevice());
  if (device!=NULL) {
    UTIL1_strcpy(buf, sizeof(buf), device->name);
    UTIL1_strcat(buf, sizeof(buf), (uint8_t*)"\r\n");
    CLS1_SendStatusStr((unsigned char*)"  device", buf, io->stdOut);
  } else {
    CLS1_SendStatusStr((unsigned char*)"  device", (uint8_t*)"UNKNOWN\r\n", io->stdOut);
  }

  return ERR_OK;
}

uint8_t ID_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
  if (UTIL1_strcmp((char*)cmd, CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, "id help")==0) {
    CLS1_SendHelpStr((unsigned char*)"id", (const unsigned char*)"Group of id commands\r\n", io->stdOut);
    CLS1_SendHelpStr((unsigned char*)"  help|status", (const unsigned char*)"Print help or status information\r\n", io->stdOut);
    *handled = TRUE;
    return ERR_OK;
  } else if ((UTIL1_strcmp((char*)cmd, CLS1_CMD_STATUS)==0) || (UTIL1_strcmp((char*)cmd, "id status")==0)) {
    *handled = TRUE;
    return PrintStatus(io);
  }
  return ERR_OK;
}

void ID_Deinit(void) {
  currDevice = ID_DEVICE_NONE;
}

void ID_Init(void) {
  currDevice = ID_DEVICE_NONE;
}
#endif /* PL_CONFIG_HAS_IDENTIFY */
