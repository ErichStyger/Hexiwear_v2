/*
 * Identify.h
 *
 *  Created on: 26.11.2015
 *      Author: tastyger
 */

#ifndef SOURCES_INTRO_COMMON_MASTER_IDENTIFY_H_
#define SOURCES_INTRO_COMMON_MASTER_IDENTIFY_H_

#include "Platform.h"

#if PL_CONFIG_HAS_IDENTIFY

typedef enum { /* do *not* change order of enumeration, they are used internally for a table index */
  ID_HEXI_01,  /* damaged OLED? */
  /* special values at the end: */
  ID_HEXI_NOF_IDS, /* sentinel to count number of entries */
  ID_HEXI_UNKNOWN, /* unknown hexi, unknown ID */
  ID_HEXI_NONE /* initialization value, used internally */
} ID_Hexis;

ID_Hexis ID_WhichHexi(void);
void ID_Deinit(void);
void ID_Init(void);

#endif /* PL_CONFIG_HAS_IDENTIFY */

#endif /* SOURCES_INTRO_COMMON_MASTER_IDENTIFY_H_ */
