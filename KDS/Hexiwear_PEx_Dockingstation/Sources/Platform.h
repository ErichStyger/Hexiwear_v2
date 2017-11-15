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
#define PL_CONFIG_HAS_SHELL       (1)
#define PL_CONFIG_HAS_SHELL_UART  (1 && PL_CONFIG_HAS_SHELL && defined(PEcfg_PAIND))  /* if it has uart for shell, e.g. over tinyK20 */
#define PL_CONFIG_HAS_SHELL_RTT   (1 && PL_CONFIG_HAS_SHELL) /* using shell over SEGGER RTT */

#define PL_CONFIG_HAS_KW40_COMM   (1)  /* Communication with KW40, including touch buttons */
#define PL_CONFIG_HAS_OLED        (1)  /* support for OLED display */

#define PL_CONFIG_HAS_UI          (1 && PL_CONFIG_HAS_OLED)
#define PL_CONFIG_HAS_CUBE_DEMO   (0)
#define PL_CONFIG_HAS_SHIP_DEMO   (0 && PL_CONFIG_HAS_CUBE_DEMO)

#define PL_CONFIG_HAS_QUIZZ       (1 && PL_CONFIG_HAS_OLED)
#define PL_CONFIG_HAS_PAIRING     (1 && PL_CONFIG_HAS_OLED)
#define PL_CONFIG_HAS_WATCH       (1 && PL_CONFIG_HAS_OLED)

#define PL_CONFIG_HAS_BUZZER          (1)
#define PL_CONFIG_HAS_ACCELEROMETER   (0)
#define PL_CONFIG_HAS_I2C_SPY         (0)
#define PL_CONFIG_HAS_RTC             (1)
#define PL_CONFIG_HAS_IDENTIFY        (1)
#define PL_CONFIG_HAS_TSL2561         (1) /* ambient light sensor */
#define PL_CONFIG_HAS_HTU21D          (1) /* temperature/humidity */

#endif /* SOURCES_PLATFORM_H_ */
