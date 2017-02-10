/*
 * KW40Comm.h
 *
 *  Created on: 09.02.2017
 *      Author: Erich Styger
 */

#ifndef SOURCES_KW40COMM_H_
#define SOURCES_KW40COMM_H_

#include "HostComm.h"

void KW40SendPacket(const hostInterface_packet_t *packet);

void KW40Comm_Init(void);

#endif /* SOURCES_KW40COMM_H_ */
