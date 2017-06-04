/**
 * \file
 * \brief Module to identify different devices based on their unique ID.
 * \author Erich Styger, erich.styger@hslu.ch
 *
 * With this module individual devices are identified based on their unique ID.
 */

#ifndef __IDENTIFY_H_
#define __IDENTIFY_H_

#include "Platform.h"

#if PL_CONFIG_HAS_IDENTIFY
  #if PL_CONFIG_HAS_SHELL
    #include "CLS1.h"
    /*!
     * \brief Parses a command
     * \param cmd Command string to be parsed
     * \param handled Sets this variable to TRUE if command was handled
     * \param io I/O stream to be used for input/output
     * \return Error code, ERR_OK if everything was fine
     */
    uint8_t ID_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);
  #endif /* PL_CONFIG_HAS_SHELL */

  typedef enum {
    ID_DEVICE_HEXI_01,
    ID_DEVICE_HEXI_20,
    ID_DEVICE_HEXI_21,
    ID_DEVICE_HEXI_22,
    ID_DEVICE_HEXI_23,
    ID_DEVICE_HEXI_24,
    ID_DEVICE_HEXI_25,
    ID_DEVICE_UNKNOWN, /* unknown device */
    ID_DEVICE_NONE /* initialization value, used internally */
  } ID_Devices;

  ID_Devices ID_WhichDevice(void);
  void ID_Deinit(void);
  void ID_Init(void);
#endif /* PL_CONFIG_HAS_IDENTIFY */

#endif /* __IDENTIFY_H_ */
