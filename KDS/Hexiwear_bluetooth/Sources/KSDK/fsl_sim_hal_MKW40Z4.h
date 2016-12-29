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

#if !defined(__FSL_SIM_HAL_KW40Z4_H__)
#define __FSL_SIM_HAL_KW40Z4_H__


/*!
 * @addtogroup sim_hal_kw40z4
 * @{
 */
/*! @file*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

 /*! @brief COP clock source select */
typedef enum _clock_cop_src_t
{
    kClockCopSrcLpoClk,       /*!< LPO                                          */
    kClockCopSrcAltClk,       /*!< Alternative clock, for KW40Z4 it is Bus clock. */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} clock_cop_src_kw40z4_t;
#else
} clock_cop_src_t;
#endif

/*! @brief TPM clock source select */
typedef enum _clock_tpm_src
{
    kClockTpmSrcNone,             /*!< clock disabled */
    kClockTpmSrcMcgFllClk,        /*!< FLL clock */
    kClockTpmSrcOsc0erClk,        /*!< OSCERCLK clock */
    kClockTpmSrcMcgIrClk          /*!< MCGIR clock */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} clock_tpm_src_kw40z4_t;
#else
} clock_tpm_src_t;
#endif

/*! @brief LPTMR clock source select */
typedef enum _clock_lptmr_src_t
{
    kClockLptmrSrcMcgIrClk,      /*!< MCG out clock  */
    kClockLptmrSrcLpoClk,        /*!< LPO clock      */
    kClockLptmrSrcEr32kClk,      /*!< ERCLK32K clock */
    kClockLptmrSrcOsc0erClk,     /*!< OSCERCLK clock */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} clock_lptmr_src_kw40z4_t;
#else
} clock_lptmr_src_t;
#endif

/*! @brief LPUART0 clock source select */
typedef enum _clock_lpuart_src
{
    kClockLpuartSrcNone,          /*!< clock disabled */
    kClockLpuartSrcMcgFllClk,     /*!< FLL clock. */
    kClockLpuartSrcOsc0erClk,     /*!< OSCERCLK clock */
    kClockLpuartSrcMcgIrClk       /*!< MCGIR clock */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} clock_lpuart_src_kw40z4_t;
#else
} clock_lpuart_src_t;
#endif

/*! @brief SIM PLLFLLSEL clock source select */
typedef enum _clock_pllfll_sel
{
    kClockPllFllSelFll,       /*!< Fll clock */
    kClockPllFllSelPll        /*!< Pll0 clock */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} clock_pllfll_sel_kw40z4_t;
#else
} clock_pllfll_sel_t;
#endif

/*! @brief SIM external reference clock source select (OSC32KSEL) */
typedef enum _clock_er32k_src
{
    kClockEr32kSrcOsc0     = 0U,     /*!< OSC 32k clock */
    kClockEr32kSrcReserved = 1U,     /*!< Reserved */
    kClockEr32kSrcRtc      = 2U,     /*!< RTC 32k clock */
    kClockEr32kSrcLpo      = 3U      /*!< LPO clock */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} clock_er32k_src_kw40z4_t;
#else
} clock_er32k_src_t;
#endif

/*! @brief SIM CLKOUT_SEL clock source select */
typedef enum _clock_clkout_src
{
    kClockClkoutOsc0erClkDiv2 = 0U,   /*!< Reserved */
    kClockClkoutOsc0erClkDiv4 = 1U,   /*!< Reserved */
    kClockClkoutBusClk        = 2U,   /*!< Bus clock */
    kClockClkoutLpoClk        = 3U,   /*!< LPO clock */
    kClockClkoutMcgIrClk      = 4U,   /*!< MCG ir clock */
    kClockClkoutOsc0erClkDiv8 = 5U,   /*!< Reserved */
    kClockClkoutOsc0erClk     = 6U,   /*!< OSC0ER clock */
    kClockClkoutReserved3     = 7U    /*!< Reserved */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} clock_clkout_src_kw40z4_t;
#else
} clock_clkout_src_t;
#endif

/*! @brief SIM RTCCLKOUTSEL clock source select */
typedef enum _clock_rtcout_src
{
    kClockRtcoutSrc1Hz,         /*!< 1Hz clock */
    kClockRtcoutSrc32kHz        /*!< 32KHz clock */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} clock_rtcout_src_kw40z4_t;
#else
} clock_rtcout_src_t;
#endif

/*! @brief SIM ADCx pre-trigger select */
typedef enum _sim_adc_pretrg_sel
{
    kSimAdcPretrgselA,              /*!< Pre-trigger A selected for ADCx */
    kSimAdcPretrgselB               /*!< Pre-trigger B selected for ADCx */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} sim_adc_pretrg_sel_kw40z4_t;
#else
} sim_adc_pretrg_sel_t;
#endif

/*! @brief SIM ADCx trigger select */
typedef enum _sim_adc_trg_sel
{
    kSimAdcTrgselExt       = 0U,         /*!< External trigger */
    kSimAdcTrgSelComp0     = 1U,         /*!< CMP0 output */
    kSimAdcTrgSelReserved  = 2U,         /*!< Reserved */
    kSimAdcTrgSelReserved1 = 3U,         /*!< Reserved */
    kSimAdcTrgSelPit0      = 4U,         /*!< PIT trigger 0 */
    kSimAdcTrgSelPit1      = 5U,         /*!< PIT trigger 1 */
    kSimAdcTrgSelReserved2 = 6U,         /*!< Reserved */
    kSimAdcTrgSelReserved3 = 7U,         /*!< Reserved */
    kSimAdcTrgSelTpm0      = 8U,         /*!< TPM0 overflow */
    kSimAdcTrgSelTpm1      = 9U,         /*!< TPM1 overflow */
    kSimAdcTrgSelTpm2      = 10U,        /*!< TPM2 overflow */
    kSimAdcTrgSelReserved4 = 11U,        /*!< Reserved */
    kSimAdcTrgSelRtcAlarm  = 12U,        /*!< RTC alarm */
    kSimAdcTrgSelRtcSec    = 13U,        /*!< RTC seconds */
    kSimAdcTrgSelLptimer   = 14U,        /*!< Low-power timer trigger */
    kSimAdcTrgSelRadioTsm  = 15U         /*!< Radio TSM */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} sim_adc_trg_sel_kw40z4_t;
#else
} sim_adc_trg_sel_t;
#endif

/*! @brief SIM LPUART receive data source select */
typedef enum _sim_lpuart_rxsrc
{
    kSimLpuartRxsrcPin,               /*!< UARTx_RX Pin */
    kSimLpuartRxsrcCmp0,              /*!< CMP0 */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} sim_lpuart_rxsrc_kw40z4_t;
#else
} sim_lpuart_rxsrc_t;
#endif

/*! @brief SIM LPUART transmit data source select */
typedef enum _sim_lpuart_txsrc
{
    kSimLpuartTxsrcPin,               /*!< LPUARTx_TX Pin */
    kSimLpuartTxsrcTpm1,              /*!< LPUARTx_TX pin modulated with TPM1 channel 0 output */
    kSimLpuartTxsrcTpm2,              /*!< LPUARTx_TX pin modulated with TPM2 channel 0 output */
    kSimLpuartTxsrcReserved           /*!< Reserved */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} sim_lpuart_txsrc_kw40z4_t;
#else
} sim_lpuart_txsrc_t;
#endif

/*! @brief SIM Timer/PWM external clock select */
typedef enum _sim_tpm_clk_sel
{
    kSimTpmClkSel0,                 /*!< Timer/PWM TPM_CLKIN0 pin. */
    kSimTpmClkSel1                  /*!< Timer/PWM TPM_CLKIN1 pin. */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} sim_tpm_clk_sel_kw40z4_t;
#else
} sim_tpm_clk_sel_t;
#endif

/*! @brief SIM Timer/PWM x channel y input capture source select */
typedef enum _sim_tpm_ch_src
{
    kSimTpmChSrc0,                  /*!< TPMx_CH0 signal */
    kSimTpmChSrc1                   /*!< CMP0 output */
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} sim_tpm_ch_src_kw40z4_t;
#else
} sim_tpm_ch_src_t;
#endif

/*! @brief SIM SCGC bit index. */
#define FSL_SIM_SCGC_BIT(SCGCx, n) (((SCGCx-1U)<<5U) + n)

/*! @brief Clock gate name used for SIM_HAL_EnableClock/SIM_HAL_DisableClock. */
typedef enum _sim_clock_gate_name
{
/* SCGC4 */    
    kSimClockGateI2c0      = FSL_SIM_SCGC_BIT(4U, 6U),
    kSimClockGateI2c1      = FSL_SIM_SCGC_BIT(4U, 7U),
    kSimClockGateCmt0      = FSL_SIM_SCGC_BIT(4U, 2U),
    kSimClockGateCmp0      = FSL_SIM_SCGC_BIT(4U, 19U),
/* SCGC5 */    
    kSimClockGateLptmr0    = FSL_SIM_SCGC_BIT(5U, 0U),
    kSimClockGateTsi0      = FSL_SIM_SCGC_BIT(5U, 5U),
    kSimClockGatePortA     = FSL_SIM_SCGC_BIT(5U, 9U),
    kSimClockGatePortB     = FSL_SIM_SCGC_BIT(5U, 10U),
    kSimClockGatePortC     = FSL_SIM_SCGC_BIT(5U, 11U),
    kSimClockGateLpuart0   = FSL_SIM_SCGC_BIT(5U, 20U),
    kSimClockGateLtc       = FSL_SIM_SCGC_BIT(5U, 24U),
    kSimClockGateRsim      = FSL_SIM_SCGC_BIT(5U, 25U),
    kSimClockGateDcdc      = FSL_SIM_SCGC_BIT(5U, 26U),
    kSimClockGateBtll      = FSL_SIM_SCGC_BIT(5U, 27U),
    kSimClockGatePhydig    = FSL_SIM_SCGC_BIT(5U, 28U),
    kSimClockGateZigbee    = FSL_SIM_SCGC_BIT(5U, 29U),

/* SCGC6 */    
    kSimClockGateFtf0      = FSL_SIM_SCGC_BIT(6U, 0U),
    kSimClockGateDmamux0   = FSL_SIM_SCGC_BIT(6U, 1U),
    kSimClockGateTrng0     = FSL_SIM_SCGC_BIT(6U, 9U),
    kSimClockGateSpi0      = FSL_SIM_SCGC_BIT(6U, 12U),
    kSimClockGateSpi1      = FSL_SIM_SCGC_BIT(6U, 13U),
    kSimClockGatePit0      = FSL_SIM_SCGC_BIT(6U, 23U),
    kSimClockGateTpm0      = FSL_SIM_SCGC_BIT(6U, 24U),
    kSimClockGateTpm1      = FSL_SIM_SCGC_BIT(6U, 25U),
    kSimClockGateTpm2      = FSL_SIM_SCGC_BIT(6U, 26U),
    kSimClockGateAdc0      = FSL_SIM_SCGC_BIT(6U, 27U),
    kSimClockGateRtc0      = FSL_SIM_SCGC_BIT(6U, 29U),
    kSimClockGateDac0      = FSL_SIM_SCGC_BIT(6U, 31U),
/* SCGC7 */    
    kSimClockGateDma0      = FSL_SIM_SCGC_BIT(7U, 8U)
#if (defined(DOXYGEN_OUTPUT) && (DOXYGEN_OUTPUT))
} sim_clock_gate_name_kw40z4_t;
#else
} sim_clock_gate_name_t;
#endif

/*@}*/

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 * @addtogroup sim_hal
 * @{
 */

/*!
 * @brief Enable the clock for specific module.
 *
 * This function enables the clock for specific module.
 *
 * @param base Base address for current SIM instance.
 * @param name Name of the module to enable.
 */
static inline void SIM_HAL_EnableClock(SIM_Type * base, sim_clock_gate_name_t name)
{
    SIM_BWR_SCGC_BIT(base, name, 1U);
}

/*!
 * @brief Disable the clock for specific module.
 *
 * This function disables the clock for specific module.
 *
 * @param base Base address for current SIM instance.
 * @param name Name of the module to disable.
 */
static inline void SIM_HAL_DisableClock(SIM_Type * base, sim_clock_gate_name_t name)
{
    SIM_BWR_SCGC_BIT(base, name, 0U);
}

/*!
 * @brief Get the the clock gate state for specific module.
 *
 * This function will get the clock gate state for specific module.
 *
 * @param base Base address for current SIM instance.
 * @param name Name of the module to get.
 * @return state true - ungated(Enabled), false - gated (Disabled)
 */
static inline bool SIM_HAL_GetGateCmd(SIM_Type * base, sim_clock_gate_name_t name)
{
    return (bool)SIM_BRD_SCGC_BIT(base, name);
}

/*!
 * @brief Set the TPM clock source selection.
 *
 * This function sets the TPM clock source selection.
 *
 * @param base Base address for current SIM instance.
 * @param instance IP instance.
 * @param setting  The value to set.
 */
static inline void CLOCK_HAL_SetTpmSrc(SIM_Type * base,
                                       uint32_t  instance,
                                       clock_tpm_src_t setting)
{
    SIM_BWR_SOPT2_TPMSRC(base, setting);
}

/*!
 * @brief Get the TPM clock source selection.
 *
 * This function gets the TPM clock source selection.
 *
 * @param base Base address for current SIM instance.
 * @param instance IP instance.
 * @return Current selection.
 */
static inline clock_tpm_src_t CLOCK_HAL_GetTpmSrc(SIM_Type * base,
                                                  uint32_t instance)
{
    return (clock_tpm_src_t)SIM_BRD_SOPT2_TPMSRC(base);
}

/*!
 * @brief Set the LPUART clock source selection.
 *
 * This function sets the LPUART clock source selection.
 *
 * @param base Base address for current SIM instance.
 * @param instance IP instance.
 * @param setting  The value to set.
 */
static inline void CLOCK_HAL_SetLpuartSrc(SIM_Type * base,
                                         uint32_t instance,
                                         clock_lpuart_src_t setting)
{
    SIM_BWR_SOPT2_LPUART0SRC(base, setting);
}

/*!
 * @brief Get the LPUART clock source selection.
 *
 * This function gets the LPUART clock source selection.
 *
 * @param base Base address for current SIM instance.
 * @param instance IP instance.
 * @return Current selection.
 */
static inline clock_lpuart_src_t CLOCK_HAL_GetLpuartSrc(SIM_Type * base,
                                                      uint32_t instance)
{
    return (clock_lpuart_src_t)SIM_BRD_SOPT2_LPUART0SRC(base);
}
/*!
 * @brief Set the clock selection of ERCLK32K.
 *
 * This function sets the clock selection of ERCLK32K.
 *
 * @param base Base address for current SIM instance.
 * @param setting  The value to set.
 */
static inline void CLOCK_HAL_SetExternalRefClock32kSrc(SIM_Type * base,
                                                       clock_er32k_src_t setting)
{
    SIM_BWR_SOPT1_OSC32KSEL(base, setting);
}

/*!
 * @brief Get the clock selection of ERCLK32K.
 *
 * This function gets the clock selection of ERCLK32K.
 *
 * @param base Base address for current SIM instance.
 * @return Current selection.
 */
static inline clock_er32k_src_t CLOCK_HAL_GetExternalRefClock32kSrc(SIM_Type * base)
{
    return (clock_er32k_src_t)SIM_BRD_SOPT1_OSC32KSEL(base);
}

/*!
 * @brief Set CLKOUTSEL selection.
 *
 * This function sets the selection of the clock to output on the CLKOUT pin.
 *
 * @param base Base address for current SIM instance.
 * @param setting  The value to set.
 */
static inline void CLOCK_HAL_SetClkOutSel(SIM_Type * base, clock_clkout_src_t setting)
{
    SIM_BWR_SOPT2_CLKOUTSEL(base, setting);
}

/*!
 * @brief Get CLKOUTSEL selection.
 *
 * This function gets the selection of the clock to output on the CLKOUT pin.
 *
 * @param base Base address for current SIM instance.
 * @return Current selection.
 */
static inline clock_clkout_src_t CLOCK_HAL_GetClkOutSel(SIM_Type * base)
{
    return (clock_clkout_src_t)SIM_BRD_SOPT2_CLKOUTSEL(base);
}

/*!
 * @brief Set OUTDIV1.
 *
 * This function sets divide value OUTDIV1.
 *
 * @param base Base address for current SIM instance.
 * @param setting  The value to set.
 */
static inline void CLOCK_HAL_SetOutDiv1(SIM_Type * base, uint8_t setting)
{
    SIM_BWR_CLKDIV1_OUTDIV1(base, setting);
}

/*!
 * @brief Get OUTDIV1.
 *
 * This function gets divide value OUTDIV1.
 *
 * @param base Base address for current SIM instance.
 * @return Current divide value.
 */
static inline uint8_t CLOCK_HAL_GetOutDiv1(SIM_Type * base)
{
    return SIM_BRD_CLKDIV1_OUTDIV1(base);
}

/*!
 * @brief Set OUTDIV4.
 *
 * This function sets divide value OUTDIV4.
 *
 * @param base Base address for current SIM instance.
 * @param setting  The value to set.
 */
static inline void CLOCK_HAL_SetOutDiv4(SIM_Type * base, uint8_t setting)
{
    SIM_BWR_CLKDIV1_OUTDIV4(base, setting);
}

/*!
 * @brief Get OUTDIV4.
 *
 * This function gets divide value OUTDIV4.
 *
 * @param base Base address for current SIM instance.
 * @return Current divide value.
 */
static inline uint8_t CLOCK_HAL_GetOutDiv4(SIM_Type * base)
{
    return SIM_BRD_CLKDIV1_OUTDIV4(base);
}

/*!
 * @brief Sets the clock out dividers setting.
 *
 * This function sets the setting for all clock out dividers at the same time.
 *
 * @param base     Base address for current SIM instance.
 * @param outdiv1      Outdivider1 setting
 * @param outdiv2      Outdivider2 setting
 * @param outdiv3      Outdivider3 setting
 * @param outdiv4      Outdivider4 setting
 */
void CLOCK_HAL_SetOutDiv(SIM_Type * base,
                         uint8_t outdiv1,
                         uint8_t outdiv2, 
                         uint8_t outdiv3,
					     uint8_t outdiv4);

/*!
 * @brief Gets the clock out dividers setting.
 *
 * This function gets the setting for all clock out dividers at the same time.
 *
 * @param base     Base address for current SIM instance.
 * @param outdiv1      Outdivider1 setting
 * @param outdiv2      Outdivider2 setting
 * @param outdiv3      Outdivider3 setting
 * @param outdiv4      Outdivider4 setting
 */
void CLOCK_HAL_GetOutDiv(SIM_Type * base,
                         uint8_t *outdiv1,
                         uint8_t *outdiv2,
                         uint8_t *outdiv3,
                         uint8_t *outdiv4);

/*!
 * @brief Sets the ADCx alternate trigger enable setting.
 *
 * This function enables/disables the alternative conversion triggers for ADCx.
 *
 * @param base     Base address for current SIM instance.
 * @param instance     device instance.
 * @param enable Enable alternative conversion triggers for ADCx
 *               - true:  Select alternative conversion trigger.
 *               - false: Select PDB trigger.
 */
void SIM_HAL_SetAdcAlternativeTriggerCmd(SIM_Type * base,
                                         uint32_t instance,
                                         bool enable);

/*!
 * @brief Gets the ADCx alternate trigger enable setting.
 *
 * This function gets the ADCx alternate trigger enable setting.
 *
 * @param base     Base address for current SIM instance.
 * @param instance     device instance.
 * @return enabled True if  ADCx alternate trigger is enabled
 */
bool SIM_HAL_GetAdcAlternativeTriggerCmd(SIM_Type * base, uint32_t instance);

/*!
 * @brief Sets the ADCx pre-trigger select setting.
 *
 * This function selects the ADCx pre-trigger source when the alternative
 * triggers are enabled through ADCxALTTRGEN.
 *
 * @param base     Base address for current SIM instance.
 * @param instance     device instance.
 * @param select pre-trigger select setting for ADCx
 */
void SIM_HAL_SetAdcPreTriggerMode(SIM_Type * base,
                                  uint32_t instance,
                                  sim_adc_pretrg_sel_t select);

/*!
 * @brief Gets the ADCx pre-trigger select setting.
 *
 * This function  gets the ADCx pre-trigger select setting.
 *
 * @param base     Base address for current SIM instance.
 * @param instance     device instance.
 * @return select ADCx pre-trigger select setting
 */
sim_adc_pretrg_sel_t SIM_HAL_GetAdcPreTriggerMode(SIM_Type * base,
                                                  uint32_t instance);

/*!
 * @brief Sets the ADCx trigger select setting.
 *
 * This function  selects the ADCx trigger source when alternative triggers
 * are enabled through ADCxALTTRGEN.
 *
 * @param base     Base address for current SIM instance.
 * @param instance     device instance.
 * @param select       trigger select setting for ADCx
*/
void SIM_HAL_SetAdcTriggerMode(SIM_Type * base,
                               uint32_t instance,
                               sim_adc_trg_sel_t select);

/*!
 * @brief Gets the ADCx trigger select setting.
 *
 * This function  gets the ADCx trigger select setting.
 *
 * @param base     Base address for current SIM instance.
 * @param instance     device instance.
 * @return ADCx trigger select setting
 */
sim_adc_trg_sel_t SIM_HAL_GetAdcTriggerMode(SIM_Type * base,
                                            uint32_t instance);

/*!
 * @brief Sets the ADCx trigger select setting in one function.
 *
 * This function sets ADC alternate trigger, pre-trigger mode and trigger mode.
 *
 * @param base     Base address for current SIM instance.
 * @param instance     device instance.
 * @param altTrigEn    Alternative trigger enable or not.
 * @param preTrigSel   Pre-trigger mode.
 * @param trigSel      Trigger mode.
*/
void SIM_HAL_SetAdcTriggerModeOneStep(SIM_Type * base,
                                      uint32_t instance,
                                      bool    altTrigEn,
                                      sim_adc_pretrg_sel_t preTrigSel,
                                      sim_adc_trg_sel_t trigSel);

/*!
 * @brief Sets the LPUARTx receive data source select setting.
 *
 * This function  selects the source for the LPUARTx receive data.
 *
 * @param base     Base address for current SIM instance.
 * @param instance     device instance.
 * @param select the source for the LPUARTx receive data
 */
void SIM_HAL_SetLpuartRxSrcMode(SIM_Type * base,
                              uint32_t instance,
                              sim_lpuart_rxsrc_t select);

/*!
 * @brief Gets the LPUARTx receive data source select setting.
 *
 * This function  gets the LPUARTx receive data source select setting.
 *
 * @param base     Base address for current SIM instance.
 * @param instance     device instance.
 * @return select LPUARTx receive data source select setting
 */
sim_lpuart_rxsrc_t SIM_HAL_GetLpuartRxSrcMode(SIM_Type * base, uint32_t instance);

/*!
 * @brief Sets the LPUARTx transmit data source select setting.
 *
 * This function  selects the source for the LPUARTx transmit data.
 *
 * @param base     Base address for current SIM instance.
 * @param instance     device instance.
 * @param select the source for the LPUARTx transmit data
 */
void SIM_HAL_SetLpuartTxSrcMode(SIM_Type * base,
                              uint32_t instance,
                              sim_lpuart_txsrc_t select);

/*!
 * @brief Gets the LPUARTx transmit data source select setting.
 *
 * This function  gets the LPUARTx transmit data source select setting.
 *
 * @param base     Base address for current SIM instance.
 * @param instance     device instance.
 * @return select LPUARTx transmit data source select setting
 */
sim_lpuart_txsrc_t SIM_HAL_GetLpuartTxSrcMode(SIM_Type * base, uint32_t instance);

#if FSL_FEATURE_SIM_OPT_HAS_ODE
/*!
* @brief Sets the LPUARTx Open Drain Enable setting.
*
* This function  enables/disables the LPUARTx Open Drain.
*
* @param base Register base address of SIM.
* @param instance LPUART instance.
* @param enable Enable/disable LPUARTx Open Drain
*				 - True: Enable LPUARTx Open Drain
*				 - False: Disable LPUARTx Open Drain
*/
void SIM_HAL_SetLpuartOpenDrainCmd(SIM_Type * base, uint32_t  instance, bool enable);

/*!
* @brief Gets the LPUARTx Open Drain Enable setting.
*
* This function  gets the LPUARTx Open Drain Enable setting.
*
* @param base Register base address of SIM.
* @param instance LPUART instance.
* @return enabled True if LPUARTx Open Drain is enabled.
*/
bool SIM_HAL_GetLpuartOpenDrainCmd(SIM_Type * base, uint32_t  instance);

#endif

#if FSL_FEATURE_SIM_OPT_HAS_TPM
/*!
* @brief Sets the Timer/PWM x external clock pin select setting.
*
* This function selects the source of the Timer/PWM x external clock pin select.
*
* @param base     Base address for current SIM instance.
* @param instance     device instance.
* @param select Timer/PWM x external clock pin select
*/
void SIM_HAL_SetTpmExternalClkPinSelMode(SIM_Type * base,
                                         uint32_t instance,
                                         sim_tpm_clk_sel_t select);

/*!
* @brief Gets the Timer/PWM x external clock pin select setting.
*
* This function  gets the Timer/PWM x external clock pin select setting.
*
* @param base     Base address for current SIM instance.
* @param instance     device instance.
* @return Timer/PWM x external clock pin select setting
*/
sim_tpm_clk_sel_t SIM_HAL_GetTpmExternalClkPinSelMode(SIM_Type * base, uint32_t instance);

/*!
* @brief Sets the Timer/PWM x channel y input capture source select setting.
*
* This function  selects the Timer/PWM x channel y input capture source.
*
* @param base     Base address for current SIM instance.
* @param instance     device instance.
* @param channel      TPM channel y
* @param select Timer/PWM x channel y input capture source
*/
void SIM_HAL_SetTpmChSrcMode(SIM_Type * base,
                             uint32_t instance,
                             uint8_t channel,
                             sim_tpm_ch_src_t select);

/*!
* @brief Gets the Timer/PWM x channel y input capture source select setting.
*
* This function gets the Timer/PWM x channel y input capture source.
*
* @param base     Base address for current SIM instance.
* @param instance     device instance.
* @param channel      TPM channel y
* @return select Timer/PWM x channel y input capture source
*/
sim_tpm_ch_src_t SIM_HAL_GetTpmChSrcMode(SIM_Type * base,
                                         uint32_t instance,
                                         uint8_t channel);
#endif

/*!
 * @brief Gets the Kinetis Family ID in the System Device ID register (SIM_SDID).
 *
 * This function  gets the Kinetis Family ID in the System Device ID register.
 *
 * @param base     Base address for current SIM instance.
 * @return id Kinetis Family ID
 */
static inline uint32_t SIM_HAL_GetFamilyId(SIM_Type * base)
{
    return SIM_BRD_SDID_FAMID(base);
}

/*!
 * @brief Gets the Kinetis Sub-Family ID in the System Device ID register (SIM_SDID).
 *
 * This function  gets the Kinetis Sub-Family ID in System Device ID register.
 *
 * @param base     Base address for current SIM instance.
 * @return id Kinetis Sub-Family ID
 */
static inline uint32_t SIM_HAL_GetSubFamilyId(SIM_Type * base)
{
    return SIM_BRD_SDID_SUBFAMID(base);
}

/*!
 * @brief Gets the Kinetis SeriesID in the System Device ID register (SIM_SDID).
 *
 * This function  gets the Kinetis Series ID in System Device ID register.
 *
 * @param base     Base address for current SIM instance.
 * @return id Kinetis Series ID
 */
static inline uint32_t SIM_HAL_GetSeriesId(SIM_Type * base)
{
    return SIM_BRD_SDID_SERIESID(base);
}

/*!
 * @brief Gets the Kinetis SramSize in the System Device ID register (SIM_SDID).
 *
 * This function  gets the Kinetis SramSize in System Device ID register.
 *
 * @param base     Base address for current SIM instance.
 * @return id Kinetis SramSize
 */
static inline uint32_t SIM_HAL_GetSramSize(SIM_Type * base)
{
    return SIM_BRD_SDID_SRAMSIZE(base);
}

/*!
 * @brief Gets the Kinetis Pincount ID in System Device ID register (SIM_SDID).
 *
 * This function  gets the Kinetis Pincount ID in System Device ID register.
 *
 * @param base     Base address for current SIM instance.
 * @return id Kinetis Pincount ID
 */
static inline uint32_t SIM_HAL_GetPinCntId(SIM_Type * base)
{
    return SIM_BRD_SDID_PINID(base);
}

/*!
 * @brief Gets the Kinetis Revision ID in the System Device ID register (SIM_SDID).
 *
 * This function  gets the Kinetis Revision ID in System Device ID register.
 *
 * @param base     Base address for current SIM instance.
 * @return id Kinetis Revision ID
 */
static inline uint32_t SIM_HAL_GetRevId(SIM_Type * base)
{
    return SIM_BRD_SDID_REVID(base);
}

/*!
 * @brief Gets the Kinetis Die ID in the System Device ID register (SIM_SDID).
 *
 * This function  gets the Kinetis Die ID in System Device ID register.
 *
 * @param base     Base address for current SIM instance.
 * @return id Kinetis Die ID
 */
static inline uint32_t SIM_HAL_GetDieId(SIM_Type * base)
{
    return SIM_BRD_SDID_DIEID(base);
}	

/*!
 * @brief Gets the program flash size in the Flash Configuration Register 1 (SIM_FCFG).
 *
 * This function  gets the program flash size in the Flash Configuration Register 1.
 *
 * @param base     Base address for current SIM instance.
 * @return size Program flash Size
 */
static inline uint32_t SIM_HAL_GetProgramFlashSize(SIM_Type * base)
{
    return SIM_BRD_FCFG1_PFSIZE(base);
}		

/*!
 * @brief Sets the Flash Doze in the Flash Configuration Register 1  (SIM_FCFG).
 *
 * This function  sets the Flash Doze in the Flash Configuration Register 1.
 *
 * @param base     Base address for current SIM instance.
 * @param setting Flash Doze setting
 */
static inline void SIM_HAL_SetFlashDoze(SIM_Type * base, uint32_t setting)
{
    SIM_BWR_FCFG1_FLASHDOZE(base, setting);
}

/*!
 * @brief Gets the Flash Doze in the Flash Configuration Register 1  (SIM_FCFG).
 *
 * This function  gets the Flash Doze in the Flash Configuration Register 1.
 *
 * @param base     Base address for current SIM instance.
 * @return setting Flash Doze setting
 */
static inline uint32_t SIM_HAL_GetFlashDoze(SIM_Type * base)
{
    return SIM_BRD_FCFG1_FLASHDOZE(base);
}

/*!
 * @brief Sets the Flash disable setting.
 *
 * This function  sets the Flash disable setting in the
 * Flash Configuration Register 1.
 *
 * @param base     Base address for current SIM instance.
 * @param disable      Flash disable setting
 */
static inline void SIM_HAL_SetFlashDisableCmd(SIM_Type * base, bool disable)
{
    SIM_BWR_FCFG1_FLASHDIS(base, disable);
}

/*!
 * @brief Gets the Flash disable setting.
 *
 * This function  gets the Flash disable setting in the
 * Flash Configuration Register 1.
 *
 * @param base     Base address for current SIM instance.
 * @return setting Flash disable setting
 */
static inline bool SIM_HAL_GetFlashDisableCmd(SIM_Type * base)
{
    return (bool)SIM_BRD_FCFG1_FLASHDIS(base);
}

/*!
 * @brief Gets the Flash maximum address block 0 in the Flash Configuration Register 1  (SIM_FCFG).
 *
 * This function gets the Flash maximum block 0 in Flash Configuration Register 2.
 *
 * @param base     Base address for current SIM instance.
 * @return address Flash maximum block 0 address
 */
static inline uint32_t SIM_HAL_GetFlashMaxAddrBlock0(SIM_Type * base)
{
    return SIM_BRD_FCFG2_MAXADDR0(base);
}

/*!
 * @brief Gets the Flash maximum address block 1 in Flash Configuration Register 2.
 *
 * This function  gets the Flash maximum block 1 in Flash Configuration Register 1.
 *
 * @param base     Base address for current SIM instance.
 * @return address Flash maximum block 0 address
 */
static inline uint32_t SIM_HAL_GetFlashMaxAddrBlock1(SIM_Type * base)
{
    return SIM_BRD_FCFG2_MAXADDR1(base);
}							 

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* __FSL_SIM_HAL_KW40Z4_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

