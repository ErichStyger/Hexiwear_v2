/*
 * Copyright (c) 2013-2015, Freescale Semiconductor, Inc.
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
/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
#include "EmbeddedTypes.h"
#include "board.h"
#include "fsl_clock_manager.h"
#include "fsl_smc_hal.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "fsl_adc16_driver.h"
#include "fsl_pmc_hal.h"

#if cPWR_UsePowerDownMode
#include "PWR_Interface.h"
#endif

#if gDCDC_Enabled_d
#include "DCDC.h"
#endif

/************************************************************************************
*************************************************************************************
* Private type definitions and macros
*************************************************************************************
************************************************************************************/
#define ADC16_INSTANCE                (0)   /* ADC instance */
#define ADC16_CHN_GROUP               (0)   /* ADC group configuration selection */
#define ADC16_POTENTIOMETER_CHN       (kAdc16Chn0) /* Potentiometer channel */

#define ADC16_BATLVL_CHN              (kAdc16Chn23) /* Potentiometer channel */
#define ADC16_BL_LOWER_LIMIT          (0) /* min percentage of battery charge */
#define ADC16_BL_UPPER_LIMIT          (100) /* max percentage of battery charge */
#define ADC16_BL_DYNAMIC_RANGE        (ADC16_BL_UPPER_LIMIT - ADC16_BL_LOWER_LIMIT) /* Range = [ADC16_HB_LOWER_LIMIT .. ADC16_HB_LOWER_LIMIT + ADC16_HB_DYNAMIC_RANGE] */

#define ADC16_BANDGAP_CHN             (kAdc16Chn27) /* ADC channel of BANDGAP Voltage reference*/

#define MIN_VOLT_BUCK 180
#define MAX_VOLT_BUCK 310
#define FULL_BAT      100
#define EMPTY_BAT     0

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
uint32_t offsetVdd = 0;               
static adc16_converter_config_t adcUserConfig;   // structure for user config

static uint32_t adcValue = 0; /* ADC value */
static adc16_converter_config_t adcUserConfig; /* structure for user config */

/* Configuration for enter VLPR mode. Core clock = 4MHz. */
const clock_manager_user_config_t g_defaultClockConfigVlpr =
{
    .mcgConfig =
    {
        .mcg_mode           = kMcgModeBLPI,   // Work in BLPI mode.
        .irclkEnable        = true,  // MCGIRCLK enable.
        .irclkEnableInStop  = false, // MCGIRCLK disable in STOP mode.
        .ircs               = kMcgIrcFast, // Select IRC4M.
        .fcrdiv             = 0U,    // FCRDIV is 0.

        .frdiv   = 5U,
        .drs     = kMcgDcoRangeSelLow,  // Low frequency range
        .dmx32   = kMcgDmx32Default,    // DCO has a default range of 25%
        .oscsel  = kMcgOscselOsc,       // Select OSC

    },
    .simConfig =
    {
        .er32kSrc  = kClockEr32kSrcOsc0,     // ERCLK32K selection, use OSC0.
        .outdiv1   = 0U,
        .outdiv4   = 4U,
    }
};

/* Configuration for enter RUN mode. Core clock = 16MHz / 32MHz. */
const clock_manager_user_config_t g_defaultClockConfigRun =
{
    .mcgConfig =
    {
        .mcg_mode           = kMcgModeBLPE, // Work in BLPE mode.
        .irclkEnable        = true,  // MCGIRCLK enable.
        .irclkEnableInStop  = false, // MCGIRCLK disable in STOP mode.
        .ircs               = kMcgIrcSlow, // Select IRC32k.
        .fcrdiv             = 0U,    // FCRDIV is 0.

        .frdiv   = 5U,
        .drs     = kMcgDcoRangeSelLow,  // Low frequency range
        .dmx32   = kMcgDmx32Default,    // DCO has a default range of 25%
        .oscsel  = kMcgOscselOsc,       // Select 
    },
    .simConfig =
    {
        .pllFllSel = kClockPllFllSelFll,    // PLLFLLSEL select FLL.
        .er32kSrc  = kClockEr32kSrcOsc0,     // ERCLK32K selection, use OSC0.
#if CLOCK_INIT_CONFIG == CLOCK_RUN_16        
        .outdiv1   = 1U,
        .outdiv4   = 0U,
#else
        .outdiv1   = 0U,
        .outdiv4   = 1U,
#endif        
    }
};

#if gDCDC_Enabled_d == 1
const dcdcConfig_t mDcdcDefaultConfig = 
{
#if APP_DCDC_MODE == gDCDC_Mode_Buck_c
  .vbatMin = 1800,
  .vbatMax = 4200,
#elif APP_DCDC_MODE == gDCDC_Mode_Boost_c
  .vbatMin = 900,
  .vbatMax = 1800,
#endif  
  .dcdcMode = APP_DCDC_MODE,
  .vBatMonitorIntervalMs = APP_DCDC_VBAT_MONITOR_INTERVAL,
  .pfDCDCAppCallback = NULL, /* .pfDCDCAppCallback = DCDCCallback, */
  .dcdc1P45OutputTargetVal = gDCDC_1P45OutputTargetVal_1_450_c,
  .dcdc1P8OutputTargetVal = gDCDC_1P8OutputTargetVal_1_800_c
};
#endif

/************************************************************************************
*************************************************************************************
* Private functions prototypes
*************************************************************************************
************************************************************************************/
static void ADC16_CalibrateParams(void);
static inline uint32_t ADC16_Measure(void);
static inline uint32_t ADC16_BatLvl(void);
static inline uint32_t ADC16_BgLvl(void);
static uint16_t ADC16_ReadValue(adc16_chn_t chnIdx, uint8_t diffMode);
static void DCDC_AdjustVbatDiv4();
static void CLOCK_SetBootConfig(clock_manager_user_config_t const* config);
/************************************************************************************
*************************************************************************************
* Public functions prototypes
*************************************************************************************
************************************************************************************/
void BOARD_InstallLowPowerCallbacks(void);
void BOARD_EnterLowPowerCb(void);
void BOARD_ExitLowPowerCb(void);
/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/* Function to initialize OSC0 base on board configuration. */
void BOARD_InitOsc0(void)
{
    // OSC0 has not configuration register, only set frequency
    MCG_WR_C2_RANGE(MCG,kOscRangeHigh);
    g_xtal0ClkFreq = 32000000U;
}

/* Function to initialize RTC external clock base on board configuration. */
void BOARD_InitRtcOsc(void)
{
    rtc_osc_user_config_t rtcOscConfig =
    {
        .freq                = RTC_XTAL_FREQ,
        .enableCapacitor2p   = RTC_SC2P_ENABLE_CONFIG,
        .enableCapacitor4p   = RTC_SC4P_ENABLE_CONFIG,
        .enableCapacitor8p   = RTC_SC8P_ENABLE_CONFIG,
        .enableCapacitor16p  = RTC_SC16P_ENABLE_CONFIG,
        .enableOsc           = RTC_OSC_ENABLE_CONFIG,
    };

    CLOCK_SYS_RtcOscInit(0U, &rtcOscConfig);
}

void BOARD_InitAdc(void)
{
#if gDCDC_Enabled_d == 0
        SIM_HAL_EnableClock(SIM, kSimClockGateDcdc);
        CLOCK_SYS_EnableAdcClock(0);
        ADC16_DRV_StructInitUserConfigDefault(&adcUserConfig);
        adcUserConfig.resolution = kAdc16ResolutionBitOfDiffModeAs13;
        adcUserConfig.refVoltSrc = kAdc16RefVoltSrcOfVref;
        ADC16_DRV_Init(ADC16_INSTANCE, &adcUserConfig);
        ADC16_CalibrateParams();
#endif     
}

uint8_t BOARD_GetBatteryLevel(void)
{
    uint16_t batVal, bgVal, batLvl, batVolt, bgVolt = 100; /*cV*/
    
    bgVal = ADC16_BgLvl();
    DCDC_AdjustVbatDiv4(); /* Bat voltage  divided by 4 */
    batVal = ADC16_BatLvl() * 4; /* Need to multiply the value by 4 because the measured voltage is divided by 4*/
    
    batVolt = bgVolt * batVal / bgVal;
    
    batLvl = (batVolt - MIN_VOLT_BUCK) * (FULL_BAT - EMPTY_BAT) / (MAX_VOLT_BUCK - MIN_VOLT_BUCK);
    return ((batLvl <= 100) ? batLvl:100);    
}

uint16_t BOARD_GetPotentiometerLevel(void)
{
    uint16_t value;
	    
    value = ADC16_Measure();
	
	value = (0x8000 & value) ? 0 : value;
        
    return value;
}

/* Initialize clock. */
void BOARD_ClockInit(void)
{
    /* Set allowed power mode, allow all. */
    SMC_HAL_SetProtection(SMC, kAllowPowerModeAll);

    /* Setup board clock source. */
    // Setup OSC0 if used.
    // Configure OSC0 pin mux.
    PORT_HAL_SetMuxMode(EXTAL0_PORT, EXTAL0_PIN, EXTAL0_PINMUX);
    PORT_HAL_SetMuxMode(XTAL0_PORT, XTAL0_PIN, XTAL0_PINMUX);

    BOARD_InitOsc0();
    BOARD_InitRtcOsc();

    /* Set system clock configuration. */
#if (CLOCK_INIT_CONFIG == CLOCK_VLPR)
    CLOCK_SetBootConfig(&g_defaultClockConfigVlpr);
#else
    CLOCK_SetBootConfig(&g_defaultClockConfigRun);
#endif
    
    CLOCK_SYS_SetTpmSrc(0, kClockTpmSrcOsc0erClk);
}

/* Initialize DCDC. */
void BOARD_DCDCInit(void)
{
#if gDCDC_Enabled_d == 1
    /* Initialize DCDC module */
    DCDC_Init(&mDcdcDefaultConfig); 
#endif
}

void dbg_uart_init(void)
{
    configure_lpuart_pins(BOARD_DEBUG_UART_INSTANCE);

    // Select different clock source for LPSCI. */
#if (CLOCK_INIT_CONFIG == CLOCK_VLPR)
    CLOCK_SYS_SetLpuartSrc(BOARD_DEBUG_UART_INSTANCE, kClockLpuartSrcMcgIrClk);
#else
    CLOCK_SYS_SetLpuartSrc(BOARD_DEBUG_UART_INSTANCE, kClockLpuartSrcMcgFllClk);
#endif

//    DbgConsole_Init(BOARD_DEBUG_UART_INSTANCE, BOARD_DEBUG_UART_BAUD, kDebugConsoleLPUART);
}

int debug_printf( char const * s, ... )
{
    return 0;
}

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

/*!
 * @brief Parameters calibration: VDD and ADCR_TEMP25
 *
 * This function used BANDGAP as reference voltage to measure vdd and
 * calibrate V_TEMP25 with that vdd value.
 */
#if gDCDC_Enabled_d == 0
static const adc16_hw_average_config_t adcHwAverageConfig =
{
  .hwAverageEnable = true, /*!< Enable the hardware average function. */
  .hwAverageCountMode = kAdc16HwAverageCountOf16 /*!< Select the count of conversion result for accumulator. */
} ;
#endif

void ADC16_CalibrateParams(void)
{
    adc16_calibration_param_t adcCalibraitionParam;   

#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    ADC16_DRV_GetAutoCalibrationParam(ADC16_INSTANCE, &adcCalibraitionParam);
    ADC16_DRV_SetCalibrationParam(ADC16_INSTANCE, &adcCalibraitionParam);
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */
  
#if gDCDC_Enabled_d == 0    
  ADC16_DRV_ConfigHwAverage(0, &adcHwAverageConfig);
#endif
  
    pmc_bandgap_buffer_config_t pmcBandgapConfig = {
        .enable = true,
#if FSL_FEATURE_PMC_HAS_BGEN
        .enableInLowPower = false,
#endif
#if FSL_FEATURE_PMC_HAS_BGBDS
        .drive = kPmcBandgapBufferDriveLow,
#endif
    };
    
    // Enable BANDGAP reference voltage
    PMC_HAL_BandgapBufferConfig(PMC_BASE_PTR, &pmcBandgapConfig);
}


/*!
 * @brief Gets the current voltage of divider (potentiometer)
 *
 * This function measure the ADC channel corresponding to external potentiometer
 */
static inline uint32_t ADC16_Measure(void)
{
    adcValue = ADC16_ReadValue((adc16_chn_t)ADC16_POTENTIOMETER_CHN, true);
    return adcValue;
}


/*!
 * @brief Gets the current voltage of the battery
 *
 * This function measure the ADC channel corresponding to the battery
 */
static inline uint32_t ADC16_BatLvl(void)
{
    adcValue = ADC16_ReadValue((adc16_chn_t)ADC16_BATLVL_CHN, false);
    return adcValue;
}

/*!
 * @brief Gets the current bandgap voltage
 *
 * This function measure the ADC channel corresponding to the bandgap
 */
static inline uint32_t ADC16_BgLvl(void)
{
    adcValue = ADC16_ReadValue((adc16_chn_t)ADC16_BANDGAP_CHN, false);
    return adcValue;
}


/*!
 * @brief Reads the ADC value from the channel given as input
 *
 * This function measure the ADC channel given as input
 */
static uint16_t ADC16_ReadValue(adc16_chn_t chnIdx, uint8_t diffMode)
{
  adc16_chn_config_t chnConfig;

    /* Configure the conversion channel */
    chnConfig.chnIdx     = chnIdx;
#if FSL_FEATURE_ADC16_HAS_DIFF_MODE
    chnConfig.diffConvEnable = diffMode;
#endif
    chnConfig.convCompletedIntEnable  = false;

    /* Software trigger the conversion */
    ADC16_DRV_ConfigConvChn(ADC16_INSTANCE, ADC16_CHN_GROUP, &chnConfig);

    /* Wait for the conversion to be done */
    ADC16_DRV_WaitConvDone(ADC16_INSTANCE, ADC16_CHN_GROUP);

    /* Fetch the conversion value */
    adcValue = (diffMode) ? ADC16_DRV_GetConvValueSigned(ADC16_INSTANCE, ADC16_CHN_GROUP) : ADC16_DRV_GetConvValueRAW(ADC16_INSTANCE, ADC16_CHN_GROUP);

    /* Calculates adcValue in 16bit resolution from 12bit resolution 
    in order to convert to reading */
#if (FSL_FEATURE_ADC16_MAX_RESOLUTION < 16)
    adcValue = adcValue << 4;
#endif
    /* Pause the conversion */
    ADC16_DRV_PauseConv(ADC16_INSTANCE, ADC16_CHN_GROUP);
    
    return adcValue;
}

static void DCDC_AdjustVbatDiv4()
{
  const uint8_t vBatDiv = 3;
  DCDC_BWR_REG0_DCDC_VBAT_DIV_CTRL(DCDC_BASE_PTR, vBatDiv);  
}

static void CLOCK_SetBootConfig(clock_manager_user_config_t const* config)
{
    CLOCK_SYS_SetSimConfigration(&config->simConfig);

    CLOCK_SYS_SetMcgMode(&config->mcgConfig);

    SystemCoreClock = CORE_CLOCK_FREQ;
}

void BOARD_InstallLowPowerCallbacks()
{
#if cPWR_UsePowerDownMode
  PWR_RegisterLowPowerEnterCallback((pfPWRCallBack_t)BOARD_EnterLowPowerCb);
  PWR_RegisterLowPowerExitCallback((pfPWRCallBack_t)BOARD_ExitLowPowerCb); 
#endif
}

void BOARD_TogglePins(bool isLowPower)
{
    if(isLowPower)
    {
        PORT_HAL_SetMuxMode(PORTA,16u,kPortPinDisabled);
        PORT_HAL_SetMuxMode(PORTA,17u,kPortPinDisabled);

        PORT_HAL_SetMuxMode(PORTB,1u,kPortPinDisabled);
        PORT_HAL_SetMuxMode(PORTB,2u,kPortPinDisabled);
        PORT_HAL_SetMuxMode(PORTB,3u,kPortPinDisabled);
        PORT_HAL_SetMuxMode(PORTB,16u,kPortPinDisabled);
        PORT_HAL_SetMuxMode(PORTB,17u,kPortPinDisabled);

        PORT_HAL_SetMuxMode(PORTC,2u,kPortPinDisabled);
        PORT_HAL_SetMuxMode(PORTC,3u,kPortPinDisabled);
        PORT_HAL_SetMuxMode(PORTC,6u,kPortPinDisabled);
        PORT_HAL_SetMuxMode(PORTC,7u,kPortPinDisabled);

        /* LEDs */
        PORT_HAL_SetMuxMode(PORTC,0u,kPortPinDisabled);
        PORT_HAL_SetMuxMode(PORTC,1u,kPortPinDisabled);
        PORT_HAL_SetMuxMode(PORTC,4u,kPortPinDisabled);
        PORT_HAL_SetMuxMode(PORTC,5u,kPortPinDisabled);

        PORT_HAL_SetMuxMode(PORTC,16u,kPortPinDisabled);
        PORT_HAL_SetMuxMode(PORTC,17u,kPortPinDisabled);
        PORT_HAL_SetMuxMode(PORTC,18u,kPortPinDisabled);
        PORT_HAL_SetMuxMode(PORTC,19u,kPortPinDisabled);

        //NMI
        PORT_HAL_SetMuxMode(PORTB,18u,kPortPinDisabled);

        //SWD pins
        PORT_HAL_SetMuxMode(PORTA,0u,kPortPinDisabled);
        PORT_HAL_SetMuxMode(PORTA,1u,kPortPinDisabled);
    }
    else
    {
        //SWD pins
        PORT_HAL_SetMuxMode(PORTA,0u,kPortMuxAlt7);
        PORT_HAL_SetPullMode(PORTA,0u,kPortPullUp);
        PORT_HAL_SetPullCmd(PORTA,0u, true);
        
        PORT_HAL_SetMuxMode(PORTA,1u,kPortMuxAlt7);
        PORT_HAL_SetSlewRateMode(PORTA,1u,kPortSlowSlewRate);
        PORT_HAL_SetPullMode(PORTA,1u,kPortPullDown);
        PORT_HAL_SetPullCmd(PORTA,1u, true);

        /* LEDs */
        PORT_HAL_SetMuxMode(PORTC,0u,kPortMuxAsGpio);
        PORT_HAL_SetMuxMode(PORTC,1u,kPortMuxAsGpio);
        PORT_HAL_SetMuxMode(PORTC,4u,kPortMuxAsGpio);
        PORT_HAL_SetMuxMode(PORTC,5u,kPortMuxAsGpio);
        
        configure_lpuart_pins(0);
    }
}

void BOARD_EnterLowPowerCb()
{
#if APP_DISABLE_PINS_IN_LOW_POWER  
    BOARD_TogglePins(TRUE);
#endif
    
#if gDCDC_Enabled_d
    DCDC_BWR_REG0_DCDC_VBAT_DIV_CTRL(DCDC_BASE_PTR, 0);
    DCDC_PrepareForPulsedMode();
#endif
}

void BOARD_ExitLowPowerCb()
{  
#if APP_DISABLE_PINS_IN_LOW_POWER  
  BOARD_TogglePins(FALSE);
#endif
  
#if gDCDC_Enabled_d
    DCDC_PrepareForContinuousMode();
#endif
}
/*******************************************************************************
 * EOF
 ******************************************************************************/

 
