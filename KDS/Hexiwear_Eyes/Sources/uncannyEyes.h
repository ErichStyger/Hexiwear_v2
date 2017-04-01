/*
 * uncannyEyes.h
 *
 *  Created on: 31.03.2017
 *      Author: Erich Styger
 */

#ifndef SOURCES_UNCANNYEYES_H_
#define SOURCES_UNCANNYEYES_H_

#define EYES_US_CNTR_PERIOD   10
extern volatile uint32_t EYES_usCntr;

void EYES_Run(void);
void EYES_Init(void);

#endif /* SOURCES_UNCANNYEYES_H_ */
