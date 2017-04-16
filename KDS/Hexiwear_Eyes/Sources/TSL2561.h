/*
 * TSL2561.h
 *  Ambient Light Sensor
 *  Created on: 10.04.2017
 *      Author: Erich Styger
 */

#ifndef SOURCES_TSL2561_H_
#define SOURCES_TSL2561_H_

#include <stdint.h>

#define TSL2561_GAIN_0X                 0x00  // no gain
#define TSL2561_GAIN_16X                0x10  // 16x gain
#define TSL2561_INTEGRATION_TIME_13MS   0  // 13.7ms
#define TSL2561_INTEGRATION_TIME_101MS  1  // 101ms
#define TSL2561_INTEGRATION_TIME_402MS  2  // 402ms

uint8_t TSL2561_SetIntegrationTime(uint8_t timing);
uint8_t TSL2561_SetGain(uint8_t gain);

uint8_t TSL2561_Disable(void);
uint8_t TSL2561_Enable(void);
uint8_t TSL2561_ReadID(uint8_t *id) ;

uint8_t TSL2561_ReadRawDataFull(uint16_t *data);
uint8_t TSL2561_ReadRawDataInfrared(uint16_t *data);
uint32_t TSL2561_CalculateLux(uint16_t broadband, uint16_t ir);

void TSL2561_Init(void);

#endif /* SOURCES_TSL2561_H_ */
