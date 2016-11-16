/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Direction type  */
typedef enum _pin_mux_direction
{
  kPIN_MUX_DirectionInput = 0U,         /* Input direction */
  kPIN_MUX_DirectionOutput = 1U,        /* Output direction */
  kPIN_MUX_DirectionInputOrOutput = 2U  /* Input or output direction */
} pin_mux_direction_t;

/*!
 * @addtogroup pin_mux
 * @{
 */

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/* PORTC8 (coord A7), RGB_R */
#define BOARD_INITPINS_RGBR_GPIO                                           GPIOC   /*!< GPIO device name: GPIOC */
#define BOARD_INITPINS_RGBR_PORT                                           PORTC   /*!< PORT device name: PORTC */
#define BOARD_INITPINS_RGBR_GPIO_PIN                                          8U   /*!< PORTC pin index: 8 */
#define BOARD_INITPINS_RGBR_PIN_NAME                                        PTC8   /*!< Pin name */
#define BOARD_INITPINS_RGBR_LABEL                                        "RGB_R"   /*!< Label */
#define BOARD_INITPINS_RGBR_NAME                                          "RGBR"   /*!< Identifier name */
#define BOARD_INITPINS_RGBR_DIRECTION                   kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTC9 (coord D6), RGB_B */
#define BOARD_INITPINS_RGBB_GPIO                                           GPIOC   /*!< GPIO device name: GPIOC */
#define BOARD_INITPINS_RGBB_PORT                                           PORTC   /*!< PORT device name: PORTC */
#define BOARD_INITPINS_RGBB_GPIO_PIN                                          9U   /*!< PORTC pin index: 9 */
#define BOARD_INITPINS_RGBB_PIN_NAME                                        PTC9   /*!< Pin name */
#define BOARD_INITPINS_RGBB_LABEL                                        "RGB_B"   /*!< Label */
#define BOARD_INITPINS_RGBB_NAME                                          "RGBB"   /*!< Identifier name */
#define BOARD_INITPINS_RGBB_DIRECTION                   kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTD0 (coord D4), RGB_G */
#define BOARD_INITPINS_RGBG_GPIO                                           GPIOD   /*!< GPIO device name: GPIOD */
#define BOARD_INITPINS_RGBG_PORT                                           PORTD   /*!< PORT device name: PORTD */
#define BOARD_INITPINS_RGBG_GPIO_PIN                                          0U   /*!< PORTD pin index: 0 */
#define BOARD_INITPINS_RGBG_PIN_NAME                                        PTD0   /*!< Pin name */
#define BOARD_INITPINS_RGBG_LABEL                                        "RGB_G"   /*!< Label */
#define BOARD_INITPINS_RGBG_NAME                                          "RGBG"   /*!< Identifier name */
#define BOARD_INITPINS_RGBG_DIRECTION                   kPIN_MUX_DirectionOutput   /*!< Direction */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
