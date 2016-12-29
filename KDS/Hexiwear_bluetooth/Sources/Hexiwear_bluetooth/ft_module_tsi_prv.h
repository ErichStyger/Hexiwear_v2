/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
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
 */
#ifndef FT_MODULE_TSI_PRV_H
#define FT_MODULE_TSI_PRV_H

/**
 * \defgroup tsi_private TSI module
 * \ingroup modules_private
 *
 * The TSI module describes the hardware configuration and control of elementary functionality
 * of the TSI peripheral, it covers all versions of the TSI peripheral by a generic
 * low-level driver API.
 *
 * The TSI module is designed for processors that have a hardware TSI module
 * with version 1, 2, or 4 (for example Kinetis L).
 *
 * The module also handles the NOISE mode supported by TSI v4 (Kinetis L).
 * \{
 */
#include "fsl_tsi_driver.h"
#include "ft_electrodes_prv.h"
#include "ft_modules.h"


#include "ft_types.h"
#include "ft_electrodes.h"
#include "ft_filters_prv.h"

   
/**
 * The TSI module noise mode initial touch threshold value.
 */   
#define FT_TSI_NOISE_INITIAL_TOUCH_THRESHOLD   2u 

/**
 * The TSI module noise mode touch range value.
 */   
#define FT_TSI_NOISE_TOUCH_RANGE               4u


/**
 * Noise data structure; This structure is used for internal
 * algorithms to store data while evaluating the noise.
 * Contains data of calculating the result and auxiliary variables.
 *
 * This structure manages and uses internal methods only.
 */
struct ft_module_tsi_noise_data {
    enum ft_filter_state filter_state;          /**< Noise filter state. */
    uint8_t              noise;                 /**< Noise current value. */
    uint8_t              touch_threshold;       /**< Noise touch threshold run-time value. */  
};

struct ft_module_tsi_data {
  tsi_state_t tsi_state;       /**< main FT driver data structure with state variables */ 
  uint32_t    noise_timestamp; /**< Noise mode switch event timestamp     */ 
};

/**
 * The TSI module's noise mode flags definition.
 */
enum ft_module_tsi_flags {
    FT_MODULE_IN_NOISE_MODE_FLAG    = 1 << FT_FLAGS_SPECIFIC_SHIFT(0),  /**< This flag signalises that the module is currently in the noise mode. */
    FT_MODULE_HAS_NOISE_MODE_FLAG   = 1 << FT_FLAGS_SPECIFIC_SHIFT(1),  /**< This flag signalises that the module can be switched to the noise mode (TSI v4). */
    FT_MODULE_NOISE_MODE_REQ_FLAG   = 1 << FT_FLAGS_SPECIFIC_SHIFT(2),  /**< This flag signalises that the module wants to switch to the noise mode. */
};

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

/** \} */ // end of tsi_private group

#endif
