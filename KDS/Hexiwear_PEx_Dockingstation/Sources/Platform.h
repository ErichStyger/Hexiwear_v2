/*
 * Platform.h
 *
 *  Created on: 06.12.2016
 *      Author: Erich Styger Local
 */

#ifndef SOURCES_PLATFORM_H_
#define SOURCES_PLATFORM_H_

#include "Cpu.h" /* for configuration macros */

#define PL_CONFIG_HAS_RADIO       (0 && defined(PEcfg_PAIND))  /* if we have a nRF24L01+ on the docking station */
#define PL_CONFIG_HAS_HOME_LEDS   (0 && defined(PEcfg_PAIND))  /* if we have the 'home' extension board attached to the docking station with 6 LEDs */
#define PL_CONFIG_HAS_SHELL_UART  (1 && defined(PEcfg_PAIND))  /* if it has uart for shell, e.g. over tinyK20 */
#define PL_CONFIG_HAS_SHELL_RTT   (1) /* using shell over SEGGER RTT */

#define PL_CONFIG_HAS_KW40_COMM   (0)  /* Communication with KW40, including touch buttons */
#define PL_CONFIG_HAS_OLED        (1)  /* support for OLED display */

#define PL_CONFIG_HAS_CUBE_DEMO   (0)
#define PL_CONFIG_HAS_SHIP_DEMO   (0 && PL_CONFIG_HAS_CUBE_DEMO)

#define PL_CONFIG_HAS_QUIZZ       (1)

#endif /* SOURCES_PLATFORM_H_ */
