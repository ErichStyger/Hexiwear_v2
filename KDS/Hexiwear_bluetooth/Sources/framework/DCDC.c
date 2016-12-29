/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file TimersManager.c
* TIMER implementation file for the ARM CORTEX-M4 processor
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

#include "EmbeddedTypes.h"
#include "DCDC.h"
#include "fsl_clock_manager.h"
#include "fsl_adc16_driver.h"
#include "TimersManager.h"
#include "fsl_os_abstraction.h"

/*****************************************************************************
******************************************************************************
* Private type definitions
******************************************************************************
*****************************************************************************/

typedef struct dcdcInputs_tag
{
  dcdc1P45OutputTargetVal_t outputTarget_1P45;
  dcdc1P8OutputTargetVal_t  outputTarget_1P8;
  uint16_t vbatVal_mV;
  bool_t outputTargetsToUpdate;
}dcdcInputs_t;

/*****************************************************************************
******************************************************************************
* Private macros
******************************************************************************
*****************************************************************************/

#define mDCDC_VBatMinBuck_c  1800
#define mDCDC_VBatMaxBuck_c  4200
#define mDCDC_VBatMinBoost_c  900
#define mDCDC_VBatMaxBoost_c 1800
#define mDCDC_1P45TrgMaxBuck_c   gDCDC_1P45OutputTargetVal_1_650_c
#define mDCDC_1P45TrgMaxBoost_c  gDCDC_1P45OutputTargetVal_1_800_c
#define mDCDC_BoostVOutToVBatMin_c 50
#define mDCDC_BuckVBatToVOutMin_c  0
#define mDCDC_PosLimitBoostIn_c 0x12
#define mDCDC_BGAPVal_mV_c           999
#define mDCDC_DutyCycleMax_c         127
//#define mDCDC_BGAPVal_mV_c         1037
//#define mDCDC_BGAPVal_mV_c         1038
/*****************************************************************************
 *****************************************************************************
 * Private prototypes
 *****************************************************************************
 *****************************************************************************/

/*****************************************************************************
 *****************************************************************************
 * Private memory definitions
 *****************************************************************************
 *****************************************************************************/

#if gDCDC_Enabled_d

static const adc16_converter_config_t adcConfig = {
  .lowPowerEnable = false,
  .clkDividerMode = kAdc16ClkDividerOf8, 
  .longSampleTimeEnable = false,
  .resolution = kAdc16ResolutionBitOfSingleEndAs12,
  .clkSrc = kAdc16ClkSrcOfBusClk,
  .asyncClkEnable = false,
  .highSpeedEnable = false,
  .hwTriggerEnable = false,
  .refVoltSrc = kAdc16RefVoltSrcOfVref,
  .continuousConvEnable = false,
#if FSL_FEATURE_ADC16_HAS_DMA
  .dmaEnable = false
#endif /* FSL_FEATURE_ADC16_HAS_DMA */
};
const adc16_chn_config_t vbatChanConfig = {
  .chnIdx = kAdc16Chn23, /*!< Select the sample channel index. */
  .convCompletedIntEnable = 0, /*!< Enable the conversion complete interrupt. */
  .diffConvEnable = 0 /*!< Enable the differential conversion. */
};
const adc16_chn_config_t bgapChanConfig = {
  .chnIdx = kAdc16Chn27, /*!< Select the sample channel index. */
  .convCompletedIntEnable = 0, /*!< Enable the conversion complete interrupt. */
  .diffConvEnable = 0 /*!< Enable the differential conversion. */
};

const adc16_hw_average_config_t adcHwAverageConfig =
{
  .hwAverageEnable = false, /*!< Enable the hardware average function. */
  .hwAverageCountMode = kAdc16HwAverageCountOf4 /*!< Select the count of conversion result for accumulator. */
} ;
const adc16_hw_cmp_config_t adcHwCmpConfig =
{
  .hwCmpEnable = false, /*!< Enable the hardware compare function. @internal gui name="Hardware compare" */
  .hwCmpGreaterThanEnable = false, /*!< Configure the compare function. @internal gui name="Compare function greater than" */
  /*
  false - Configures less than the threshold. The outside and inside range are not inclusive.
  The functionality is based on the values
  placed in CV1 and CV2.
  true  - Configures greater than or equal to the threshold. The outside and inside
  ranges are inclusive. The functionality is based on the values placed in
  CV1 and CV2.
  */
  .hwCmpRangeEnable = false, /*!< Configure the comparator function. @internal gui name="Compare function range" */
  /*
  Configures the comparator function to check if the conversion result of the
  input being monitored is either between or outside the range formed by
  CV1 and CV2 and determined by the value of hwCmpGreaterThanEnable.
  
  false - Range function disabled. Only CV1 is compared.
  true  - Range function enabled. Both CV1 and CV2 are compared.
  */
  .cmpValue1 = 0, /*!< Setting value for CV1. @internal gui name="Compare value 1" */
  .cmpValue2 = 0 /*!< Setting value for CV2. @internal gui name="Compare value 2" */
} ;

static tmrTimerID_t mVBatMonitorTimerId;
static const dcdcConfig_t * mpDCDCConfig;
static dcdcCallbackParam_t mDCDCCallbackParam;
static dcdcInputs_t mDCDCInputs;
static pfDCDCPSwitchCallback_t mpfDCDCPSwitchCallback;
#endif /*gDCDC_Enabled_d*/





/*****************************************************************************
******************************************************************************
* Private functions
******************************************************************************
*****************************************************************************/

#if gDCDC_Enabled_d
///*---------------------------------------------------------------------------
//* NAME: DCDC_AdjustVbatDiv
//* DESCRIPTION: 
//* PARAMETERS:  
//* RETURN: 
//* NOTES: none
//*---------------------------------------------------------------------------*/
static void DCDC_AdjustVbatDiv()
{
  uint16_t batVal;
  uint8_t vBatDiv = 3;
  do
  {
    DCDC_BWR_REG0_DCDC_VBAT_DIV_CTRL(DCDC_BASE_PTR, vBatDiv);  
    ADC16_DRV_ConfigConvChn(0, 0, &vbatChanConfig);
    ADC16_DRV_WaitConvDone(0,0);
    batVal = ADC16_DRV_GetConvValueRAW(0,0);   
    if(batVal > 0x7FF)
    {
      break;
    }
  }
  while(vBatDiv-- != 1);
}
///*---------------------------------------------------------------------------
//* NAME: DCDC_Get1P8VOutputTargetAndVBatInmV
//* DESCRIPTION: 
//* PARAMETERS:  
//* RETURN: 
//* NOTES: none
//*---------------------------------------------------------------------------*/
static void DCDC_Get1P8VOutputTargetAndVBatInmV(uint16_t* p1P8VOutputInMv, uint16_t* pVBatInMv )
{
  uint16_t batVal;
  uint16_t bgapVal;
  uint32_t mvVal;
  uint8_t vBatDiv;
  ADC16_DRV_ConfigConvChn(0, 0, &vbatChanConfig);
  ADC16_DRV_WaitConvDone(0,0);
  batVal = ADC16_DRV_GetConvValueRAW(0,0);
  ADC16_DRV_ConfigConvChn(0, 0, &bgapChanConfig);
  ADC16_DRV_WaitConvDone(0,0);
  bgapVal = ADC16_DRV_GetConvValueRAW(0,0);
  mvVal = 4095;
  mvVal *= mDCDC_BGAPVal_mV_c;
  mvVal += (bgapVal>>1);
  mvVal /= bgapVal;
  *p1P8VOutputInMv = mvVal;
  vBatDiv = DCDC_BRD_REG0_DCDC_VBAT_DIV_CTRL(DCDC_BASE_PTR);
  if(vBatDiv)
  {
    vBatDiv--;
  }
  mvVal = (batVal<<vBatDiv);
  mvVal *= mDCDC_BGAPVal_mV_c;
  mvVal += (bgapVal>>1);
  mvVal /= bgapVal;  
  *pVBatInMv = mvVal;
}

///*---------------------------------------------------------------------------
//* NAME: DCDC_mVTo1P8OutputTargetBoost
//* DESCRIPTION: 
//* PARAMETERS:  
//* RETURN: 
//* NOTES: none
//*---------------------------------------------------------------------------*/
static dcdc1P8OutputTargetVal_t DCDC_mVTo1P8OutputTargetBoost(uint16_t mV)
{
  if(mV < 1650)
  {
    return gDCDC_1P8OutputTargetVal_1_650_c;
  }
  if(mV <= 2050)
  {
    mV -= 1650;
    if(mV%25)
    {
      mV = mV/25 +1;
    }
    else
    {
      mV /= 25;
    }
    return (dcdc1P8OutputTargetVal_t)mV;
  }
  if (mV < 2800)
  {
    return gDCDC_1P8OutputTargetVal_2_800_c;
  }
  mV -= 2800;
  if(mV%25)
  {
    mV = mV/25 +1;
  }
  else
  {
    mV /= 25;
  }
  mV += gDCDC_1P8OutputTargetVal_2_800_c;
  if(mV > gDCDC_1P8OutputTargetVal_3_575_c)
  {
    mV = gDCDC_1P8OutputTargetVal_3_575_c;
  }  
  return (dcdc1P8OutputTargetVal_t)mV;
}

///*---------------------------------------------------------------------------
//* NAME: DCDC_mVTo1P8OutputTargetBuck
//* DESCRIPTION: 
//* PARAMETERS:  
//* RETURN: 
//* NOTES: none
//*---------------------------------------------------------------------------*/
static dcdc1P8OutputTargetVal_t DCDC_mVTo1P8OutputTargetBuck(uint16_t mV)
{
  
  dcdc1P8OutputTargetVal_t base;
  if(mV < 1650)
  {
    return gDCDC_1P8OutputTargetVal_1_650_c;
  }
  if(mV <= 2050)
  {
    mV -= 1650;
    mV /= 25;
    return (dcdc1P8OutputTargetVal_t)mV;
  }
  if (mV < 2800)
  {
    return gDCDC_1P8OutputTargetVal_2_050_c;
  }
  mV -= 2800;
  mV /= 25;
  mV += gDCDC_1P8OutputTargetVal_2_800_c;
  if(mV > gDCDC_1P8OutputTargetVal_3_575_c)
  {
    mV = gDCDC_1P8OutputTargetVal_3_575_c;
  }  
  return (dcdc1P8OutputTargetVal_t)mV;
}


///*---------------------------------------------------------------------------
//* NAME: DCDC_mVTo1P45OutputTargetBoost
//* DESCRIPTION: 
//* PARAMETERS:  
//* RETURN: 
//* NOTES: none
//*---------------------------------------------------------------------------*/
dcdc1P45OutputTargetVal_t DCDC_mVTo1P45OutputTargetBoost(uint16_t mV)
{
  
  if (mV <= 1275)
  {
    return gDCDC_1P45OutputTargetVal_1_275_c;
  }
  mV -= 1275;
  
  if(mV%25)
  {
    mV = mV/25 +1;
  }
  else
  {
    mV /= 25;
  }
  if(mV > gDCDC_1P45OutputTargetVal_1_800_c)
  {
    mV = gDCDC_1P45OutputTargetVal_1_800_c;
  }
  return (dcdc1P45OutputTargetVal_t)mV;
}
///*---------------------------------------------------------------------------
//* NAME: DCDC_mVTo1P45OutputTargetBuck
//* DESCRIPTION: 
//* PARAMETERS:  
//* RETURN: 
//* NOTES: none
//*---------------------------------------------------------------------------*/
dcdc1P45OutputTargetVal_t DCDC_mVTo1P45OutputTargetBuck(uint16_t mV)
{
  if (mV <= 1275)
  {
    return gDCDC_1P45OutputTargetVal_1_275_c;
  }
  mV -= 1275;
  mV /= 25;
  
  if(mV > gDCDC_1P45OutputTargetVal_1_800_c)
  {
    mV = gDCDC_1P45OutputTargetVal_1_800_c;
  }
  return (dcdc1P45OutputTargetVal_t)mV;
}

///*---------------------------------------------------------------------------
//* NAME: DCDC_GetOutputTargetsBoost
//* DESCRIPTION: 
//* PARAMETERS:  
//* RETURN: 
//* NOTES: none
//*---------------------------------------------------------------------------*/
static void DCDC_GetOutputTargetsBoost(dcdc1P45OutputTargetVal_t* p1P45OutputTarget, dcdc1P8OutputTargetVal_t* p1P8OutputTarget )
{
  dcdc1P45OutputTargetVal_t dcdc1P45MinOutputTarget,dcdc1P45OutputTarget,dcdc1P45Wanted ;
  dcdc1P8OutputTargetVal_t dcdc1P8MinOutputTarget,dcdc1P8OutputTarget, dcdc1P8Wanted;
  uint16_t vBATmV;
  OSA_EnterCritical(kCriticalDisableInt); 
  {
    dcdc1P45Wanted = mDCDCInputs.outputTarget_1P45;
    dcdc1P8Wanted = mDCDCInputs.outputTarget_1P8;
    vBATmV = mDCDCInputs.vbatVal_mV;
  }
  OSA_ExitCritical(kCriticalDisableInt);    
  dcdc1P45MinOutputTarget = DCDC_mVTo1P45OutputTargetBoost(vBATmV + mDCDC_BoostVOutToVBatMin_c);
  dcdc1P8MinOutputTarget = DCDC_mVTo1P8OutputTargetBoost(vBATmV + mDCDC_BoostVOutToVBatMin_c);
  if(dcdc1P45Wanted >= dcdc1P45MinOutputTarget)
  {
    dcdc1P45OutputTarget = dcdc1P45Wanted;
  }
  else
  {
    dcdc1P45OutputTarget = dcdc1P45MinOutputTarget;
  }
  if(dcdc1P8Wanted >= dcdc1P8MinOutputTarget)
  {
    dcdc1P8OutputTarget = dcdc1P8Wanted;
  }
  else
  {
    dcdc1P8OutputTarget = dcdc1P8MinOutputTarget;
  }
  *p1P45OutputTarget = dcdc1P45OutputTarget;
  *p1P8OutputTarget = dcdc1P8OutputTarget;
}


///*---------------------------------------------------------------------------
//* NAME: DCDC_GetOutputTargetsBuck
//* DESCRIPTION: 
//* PARAMETERS:  
//* RETURN: 
//* NOTES: none
//*---------------------------------------------------------------------------*/
static void DCDC_GetOutputTargetsBuck(dcdc1P45OutputTargetVal_t* p1P45OutputTarget, dcdc1P8OutputTargetVal_t* p1P8OutputTarget )
{
  dcdc1P45OutputTargetVal_t dcdc1P45Wanted;
  dcdc1P8OutputTargetVal_t dcdc1P8MaxOutputTarget,dcdc1P8OutputTarget,dcdc1P8Wanted;
  uint16_t vBATmV;
  OSA_EnterCritical(kCriticalDisableInt); 
  {
    dcdc1P45Wanted = mDCDCInputs.outputTarget_1P45;
    dcdc1P8Wanted = mDCDCInputs.outputTarget_1P8;
    vBATmV = mDCDCInputs.vbatVal_mV;
  }
  OSA_ExitCritical(kCriticalDisableInt);    
  dcdc1P8MaxOutputTarget = DCDC_mVTo1P8OutputTargetBuck(vBATmV - mDCDC_BuckVBatToVOutMin_c);
  if(dcdc1P8Wanted <= dcdc1P8MaxOutputTarget)
  {
    dcdc1P8OutputTarget = dcdc1P8Wanted;
  }
  else
  {
    dcdc1P8OutputTarget = dcdc1P8MaxOutputTarget;
  }
  *p1P45OutputTarget = dcdc1P45Wanted;
  *p1P8OutputTarget = dcdc1P8OutputTarget;
}
///*---------------------------------------------------------------------------
//* NAME: DCDC_VBatMonitorBoost
//* DESCRIPTION: 
//* PARAMETERS:  
//* RETURN: 
//* NOTES: none
//*---------------------------------------------------------------------------*/
static void DCDC_VBatMonitorBoost(void* param)
{
  uint16_t mV_VBat;
  uint16_t mV_1P8V;
  dcdc1P45OutputTargetVal_t dcdc1P45OutputTarget;
  dcdc1P8OutputTargetVal_t dcdc1P8OutputTarget; 
  mDCDCCallbackParam.dcdcEvent = gDCDC_Event_NoEvent_c;
  DCDC_Get1P8VOutputTargetAndVBatInmV(&mV_1P8V ,&mV_VBat);
  OSA_EnterCritical(kCriticalDisableInt);  
  {
    mDCDCInputs.vbatVal_mV  = mV_VBat;
    mDCDCInputs.outputTargetsToUpdate = 1;
  }
  OSA_ExitCritical(kCriticalDisableInt);
  
  DCDC_BWR_REG2_DCDC_BATTMONITOR_EN_BATADJ(DCDC_BASE_PTR,0);
  DCDC_BWR_REG2_DCDC_BATTMONITOR_BATT_VAL(DCDC_BASE_PTR, mV_VBat >> 3);
  DCDC_BWR_REG2_DCDC_BATTMONITOR_EN_BATADJ(DCDC_BASE_PTR,1);  
  mDCDCCallbackParam.dcdcVbatMeasuredVal = mV_VBat;
  mDCDCCallbackParam.dcdc1P8OutputMeasuredVal = mV_1P8V;
  DCDC_GetOutputTargetsBoost(&dcdc1P45OutputTarget, &dcdc1P8OutputTarget );
  OSA_EnterCritical(kCriticalDisableInt);  
  {
    if(mDCDCInputs.outputTargetsToUpdate == 1)
    {
      DCDC_BWR_REG3_DCDC_VDD1P45CTRL_TRG_BOOST(DCDC_BASE_PTR, dcdc1P45OutputTarget);
      DCDC_BWR_REG3_DCDC_VDD1P8CTRL_TRG(DCDC_BASE_PTR, dcdc1P8OutputTarget);
      mDCDCInputs.outputTargetsToUpdate = 0;
    }
  }
  OSA_ExitCritical(kCriticalDisableInt);
  
  if(param == NULL)  
  {
    if(mpDCDCConfig->pfDCDCAppCallback)
    {
      if((mDCDCInputs.vbatVal_mV < mpDCDCConfig->vbatMin) || (mDCDCInputs.vbatVal_mV > mpDCDCConfig->vbatMax))
      {
        mDCDCCallbackParam.dcdcEvent |= gDCDC_Event_VBatOutOfRange_c;
      }
      OSA_EnterCritical(kCriticalDisableInt);
      if( mDCDCCallbackParam.dcdc1P45OutputTargetVal != (dcdc1P45OutputTargetVal_t)DCDC_BRD_REG3_DCDC_VDD1P45CTRL_TRG_BOOST(DCDC_BASE_PTR) )
      {
        mDCDCCallbackParam.dcdc1P45OutputTargetVal = (dcdc1P45OutputTargetVal_t)DCDC_BRD_REG3_DCDC_VDD1P45CTRL_TRG_BOOST(DCDC_BASE_PTR);
        mDCDCCallbackParam.dcdcEvent |= gDCDC_Event_1P45OutputTargetChange_c;
      }
      if(mDCDCCallbackParam.dcdc1P8OutputTargetVal  != (dcdc1P8OutputTargetVal_t)DCDC_BRD_REG3_DCDC_VDD1P8CTRL_TRG(DCDC_BASE_PTR))
      {
        mDCDCCallbackParam.dcdc1P8OutputTargetVal  = (dcdc1P8OutputTargetVal_t)DCDC_BRD_REG3_DCDC_VDD1P8CTRL_TRG(DCDC_BASE_PTR);
        mDCDCCallbackParam.dcdcEvent |= gDCDC_Event_1P8OutputTargetChange_c;
      }      
      OSA_ExitCritical(kCriticalDisableInt);
      mpDCDCConfig->pfDCDCAppCallback(&mDCDCCallbackParam);
    }
  }
  
}

///*---------------------------------------------------------------------------
//* NAME: DCDC_VBatMonitorBuck
//* DESCRIPTION: 
//* PARAMETERS:  
//* RETURN: 
//* NOTES: none
//*---------------------------------------------------------------------------*/
static void DCDC_VBatMonitorBuck(void* param)
{
  uint16_t mV_VBat;
  uint16_t mV_1P8V;
  dcdc1P45OutputTargetVal_t dcdc1P45OutputTarget;
  dcdc1P8OutputTargetVal_t dcdc1P8OutputTarget; 
  mDCDCCallbackParam.dcdcEvent = gDCDC_Event_NoEvent_c;
  DCDC_AdjustVbatDiv();
  DCDC_Get1P8VOutputTargetAndVBatInmV(&mV_1P8V ,&mV_VBat);
  OSA_EnterCritical(kCriticalDisableInt);  
  {
    mDCDCInputs.vbatVal_mV  = mV_VBat;
    mDCDCInputs.outputTargetsToUpdate = 1;
  }
  OSA_ExitCritical(kCriticalDisableInt);
  DCDC_BWR_REG2_DCDC_BATTMONITOR_EN_BATADJ(DCDC_BASE_PTR,0);
  DCDC_BWR_REG2_DCDC_BATTMONITOR_BATT_VAL(DCDC_BASE_PTR, mV_VBat >> 3);
  DCDC_BWR_REG2_DCDC_BATTMONITOR_EN_BATADJ(DCDC_BASE_PTR,1);
  mDCDCCallbackParam.dcdcVbatMeasuredVal = mV_VBat;
  mDCDCCallbackParam.dcdc1P8OutputMeasuredVal = mV_1P8V;
  DCDC_GetOutputTargetsBuck(&dcdc1P45OutputTarget, &dcdc1P8OutputTarget );
  
  OSA_EnterCritical(kCriticalDisableInt);  
  {
    if(mDCDCInputs.outputTargetsToUpdate == 1)
    {
      DCDC_BWR_REG3_DCDC_VDD1P45CTRL_TRG_BUCK(DCDC_BASE_PTR, dcdc1P45OutputTarget);
      DCDC_BWR_REG3_DCDC_VDD1P8CTRL_TRG(DCDC_BASE_PTR, dcdc1P8OutputTarget);
      mDCDCInputs.outputTargetsToUpdate = 0;
    }
  }
  OSA_ExitCritical(kCriticalDisableInt);
  
  if(param == NULL)
  {
    if(mpDCDCConfig->pfDCDCAppCallback)
    {
      if((mDCDCInputs.vbatVal_mV < mpDCDCConfig->vbatMin) || (mDCDCInputs.vbatVal_mV > mpDCDCConfig->vbatMax))
      {
        mDCDCCallbackParam.dcdcEvent |= gDCDC_Event_VBatOutOfRange_c;
      }
      OSA_EnterCritical(kCriticalDisableInt);
      if(mDCDCCallbackParam.dcdc1P45OutputTargetVal != (dcdc1P45OutputTargetVal_t)DCDC_BRD_REG3_DCDC_VDD1P45CTRL_TRG_BUCK(DCDC_BASE_PTR))
      {
        mDCDCCallbackParam.dcdc1P45OutputTargetVal = (dcdc1P45OutputTargetVal_t)DCDC_BRD_REG3_DCDC_VDD1P45CTRL_TRG_BUCK(DCDC_BASE_PTR);
        mDCDCCallbackParam.dcdcEvent |= gDCDC_Event_1P45OutputTargetChange_c;
      }
      if(mDCDCCallbackParam.dcdc1P8OutputTargetVal != (dcdc1P8OutputTargetVal_t)DCDC_BRD_REG3_DCDC_VDD1P8CTRL_TRG(DCDC_BASE_PTR))
      {
        mDCDCCallbackParam.dcdc1P8OutputTargetVal = (dcdc1P8OutputTargetVal_t)DCDC_BRD_REG3_DCDC_VDD1P8CTRL_TRG(DCDC_BASE_PTR);
        mDCDCCallbackParam.dcdcEvent |= gDCDC_Event_1P8OutputTargetChange_c;
      } 
      OSA_ExitCritical(kCriticalDisableInt);
      mpDCDCConfig->pfDCDCAppCallback(&mDCDCCallbackParam);
    }    
  }
}

///*---------------------------------------------------------------------------
//* NAME: DCDC_PSwitchIsr
//* DESCRIPTION: 
//* PARAMETERS:  
//* RETURN: 
//* NOTES: none
//*---------------------------------------------------------------------------*/
static void DCDC_PSwitchIsr(void)
{
    dcdcPSwStatus_t dcdcPSwStatus;
    DCDC_BWR_REG6_PSWITCH_INT_CLEAR(DCDC_BASE_PTR, 1);
    DCDC_BWR_REG6_PSWITCH_INT_CLEAR(DCDC_BASE_PTR, 0);
    if( DCDC_BRD_REG0_PSWITCH_STATUS(DCDC_BASE_PTR) == 0)
  {
    dcdcPSwStatus = gDCDC_PSwStatus_Low_c;
  }
  else
  {
    dcdcPSwStatus = gDCDC_PSwStatus_High_c;
  }
    mpfDCDCPSwitchCallback(dcdcPSwStatus);
}

#endif /*gDCDC_Enabled_d*/



/*****************************************************************************
******************************************************************************
* Public functions
******************************************************************************
*****************************************************************************/




/*---------------------------------------------------------------------------
 * NAME: DCDC_Init
 * DESCRIPTION: initialize 
 * PARAMETERS: -
 * RETURN: -
 *---------------------------------------------------------------------------*/
bool_t DCDC_Init
(
 const dcdcConfig_t * pDCDCConfig
)
{
#if gDCDC_Enabled_d == 0
(void) pDCDCConfig;
return TRUE;
#else  
  adc16_calibration_param_t adcCalibParam;  
  adc16_status_t adcStatus;
  if(pDCDCConfig == NULL)
  {
    return FALSE;
  }
  SIM_HAL_EnableClock(SIM, kSimClockGateDcdc);
  if(pDCDCConfig->dcdcMode == gDCDC_Mode_Bypass_c)
  {
    mpDCDCConfig = pDCDCConfig;
    return TRUE;
  }
  if(pDCDCConfig->vbatMin > pDCDCConfig->vbatMax)
  {
    return FALSE;
  }  
  if(pDCDCConfig->dcdcMode == gDCDC_Mode_Buck_c)
  {
    if((pDCDCConfig->vbatMin < mDCDC_VBatMinBuck_c)||(pDCDCConfig->vbatMax > mDCDC_VBatMaxBuck_c))
    {
      return FALSE;
    }
    if(mDCDC_1P45TrgMaxBuck_c < pDCDCConfig->dcdc1P45OutputTargetVal)
    {
      return FALSE;
    }
  }
  else if(pDCDCConfig->dcdcMode == gDCDC_Mode_Boost_c)
  {
    if((pDCDCConfig->vbatMin < mDCDC_VBatMinBoost_c)||(pDCDCConfig->vbatMax > mDCDC_VBatMaxBoost_c))
    {
      return FALSE;
    }
    if(mDCDC_1P45TrgMaxBoost_c < pDCDCConfig->dcdc1P45OutputTargetVal)
    {
      return FALSE;
    }
  }
  else
  {
    return FALSE;
  }
   if((gDCDC_1P8OutputTargetVal_2_050_c < pDCDCConfig->dcdc1P8OutputTargetVal) && (pDCDCConfig->dcdc1P8OutputTargetVal < gDCDC_1P8OutputTargetVal_2_800_c))
  {
    return FALSE;
  }
  if(gDCDC_1P8OutputTargetVal_3_575_c < pDCDCConfig->dcdc1P8OutputTargetVal)
  {
    return FALSE;
  }
  TMR_Init();
  mVBatMonitorTimerId = TMR_AllocateTimer();
  if(gTmrInvalidTimerID_c == mVBatMonitorTimerId)
  {
    return FALSE;
  }
  CLOCK_SYS_EnableAdcClock(0);
  adcStatus = ADC16_DRV_GetAutoCalibrationParam(0, &adcCalibParam);
  if (kStatus_ADC16_Success == adcStatus)
  {
    adcStatus = ADC16_DRV_SetCalibrationParam(0 , &adcCalibParam);
    if (kStatus_ADC16_Success == adcStatus)
    {
      adcStatus = ADC16_DRV_Init(0, &adcConfig);
    }
  }
  if(kStatus_ADC16_Success != adcStatus)
  {
    return FALSE;
  }
  ADC16_DRV_SetChnMux(0,kAdc16ChnMuxOfA);
  ADC16_DRV_ConfigHwAverage(0, &adcHwAverageConfig);
  ADC16_DRV_ConfigHwCompare(0, &adcHwCmpConfig);
  mDCDCInputs.outputTarget_1P45 = pDCDCConfig->dcdc1P45OutputTargetVal;
  mDCDCInputs.outputTarget_1P8 = pDCDCConfig->dcdc1P8OutputTargetVal; 
  mpDCDCConfig = pDCDCConfig;
  DCDC_BWR_REG1_DCDC_LOOPCTRL_EN_DF_HYST(DCDC_BASE_PTR,1);
  DCDC_BWR_REG1_DCDC_LOOPCTRL_EN_CM_HYST(DCDC_BASE_PTR,1);
  DCDC_BWR_REG2_DCDC_LOOPCTRL_HYST_SIGN(DCDC_BASE_PTR,1);
  DCDC_BWR_REG3_DCDC_VDD1P8CTRL_DISABLE_STEP(DCDC_BASE_PTR,0);  
  DCDC_BWR_REG3_DCDC_VDD1P45CTRL_DISABLE_STEP(DCDC_BASE_PTR,0);  
  PMC_BWR_REGSC_BGBE(PMC_BASE_PTR,1);
  
  mDCDCCallbackParam.dcdc1P8OutputTargetVal = gDCDC_1P8OutputTargetVal_1_800_c;
  if(pDCDCConfig->dcdcMode == gDCDC_Mode_Boost_c)
  {
    DCDC_BWR_REG0_DCDC_VBAT_DIV_CTRL(DCDC_BASE_PTR, 0x1);
    DCDC_BWR_REG1_POSLIMIT_BOOST_IN(DCDC_BASE_PTR,mDCDC_PosLimitBoostIn_c);
    mDCDCCallbackParam.dcdc1P45OutputTargetVal = gDCDC_1P45OutputTargetVal_1_800_c;
    DCDC_VBatMonitorBoost((void*)1);
    TMR_StartLowPowerTimer(mVBatMonitorTimerId, gTmrIntervalTimer_c ,pDCDCConfig->vBatMonitorIntervalMs, DCDC_VBatMonitorBoost, NULL);    
  }
  else
  {
    mDCDCCallbackParam.dcdc1P45OutputTargetVal = gDCDC_1P45OutputTargetVal_1_450_c;
    DCDC_AdjustVbatDiv();
    DCDC_VBatMonitorBuck((void*)1);
    TMR_StartLowPowerTimer(mVBatMonitorTimerId, gTmrIntervalTimer_c ,pDCDCConfig->vBatMonitorIntervalMs, DCDC_VBatMonitorBuck, NULL);    
  }
  
  
  return TRUE; 
#endif  
}

/*---------------------------------------------------------------------------
 * NAME: DCDC_SetOutputVoltageTargets
 * DESCRIPTION: initialize the timer module
 * PARAMETERS: -
 * RETURN: -
 *---------------------------------------------------------------------------*/
bool_t DCDC_SetOutputVoltageTargets
(
dcdc1P45OutputTargetVal_t dcdc1P45OutputTargetVal,
dcdc1P8OutputTargetVal_t  dcdc1P8OutputTargetVal
)
{
#if gDCDC_Enabled_d == 0
  (void)dcdc1P45OutputTargetVal;
  (void)dcdc1P8OutputTargetVal;
  return TRUE;
#else  
  dcdc1P45OutputTargetVal_t dcdc1P45OutputTarget;
  dcdc1P8OutputTargetVal_t dcdc1P8OutputTarget; 
  
  if(mpDCDCConfig == NULL)
  {
    // DCDC_Init has to be called prior to this function
    return FALSE;
  }
  if(mpDCDCConfig->dcdcMode == gDCDC_Mode_Bypass_c)
  {
    return TRUE;
  }
  if(mpDCDCConfig->dcdcMode == gDCDC_Mode_Boost_c)
  {
    if(mDCDC_1P45TrgMaxBoost_c < dcdc1P45OutputTargetVal)
    {
      return FALSE;
    } 
  }
  else
  {
    if(mDCDC_1P45TrgMaxBuck_c < dcdc1P45OutputTargetVal)
    {
      return FALSE;
    }
  }
  if((gDCDC_1P8OutputTargetVal_2_050_c < dcdc1P8OutputTargetVal) && (dcdc1P8OutputTargetVal < gDCDC_1P8OutputTargetVal_2_800_c))
  {
    return FALSE;
  }
  if(gDCDC_1P8OutputTargetVal_3_575_c < dcdc1P8OutputTargetVal)
  {
    return FALSE;
  }
  
  OSA_EnterCritical(kCriticalDisableInt);  
  {
    mDCDCInputs.outputTarget_1P45 = dcdc1P45OutputTargetVal;
    mDCDCInputs.outputTarget_1P8 = dcdc1P8OutputTargetVal; 
    mDCDCInputs.outputTargetsToUpdate = 1;
  }
  OSA_ExitCritical(kCriticalDisableInt);
  
  if(mpDCDCConfig->dcdcMode == gDCDC_Mode_Boost_c)
  {
    DCDC_GetOutputTargetsBoost(&dcdc1P45OutputTarget, &dcdc1P8OutputTarget );
  }
  else
  {
    DCDC_GetOutputTargetsBuck(&dcdc1P45OutputTarget, &dcdc1P8OutputTarget );
  }
  
  OSA_EnterCritical(kCriticalDisableInt);  
  {
    if(mDCDCInputs.outputTargetsToUpdate == 1)
    {
      if(mpDCDCConfig->dcdcMode == gDCDC_Mode_Boost_c)
      {
        DCDC_BWR_REG3_DCDC_VDD1P45CTRL_TRG_BOOST(DCDC_BASE_PTR, dcdc1P45OutputTarget);
        DCDC_BWR_REG3_DCDC_VDD1P8CTRL_TRG(DCDC_BASE_PTR, dcdc1P8OutputTarget);
      }
      else
      {
        DCDC_BWR_REG3_DCDC_VDD1P45CTRL_TRG_BUCK(DCDC_BASE_PTR, dcdc1P45OutputTarget);
        DCDC_BWR_REG3_DCDC_VDD1P8CTRL_TRG(DCDC_BASE_PTR, dcdc1P8OutputTarget);
      }
      mDCDCInputs.outputTargetsToUpdate = 0;
    }
  }
  OSA_ExitCritical(kCriticalDisableInt);
  return TRUE;
#endif
}
///*---------------------------------------------------------------------------
//* NAME: DCDC_1P45OutputTargetTomV
//* DESCRIPTION: 
//* PARAMETERS:  
//* RETURN: 
//* NOTES: none
//*---------------------------------------------------------------------------*/
uint16_t DCDC_1P45OutputTargetTomV(dcdc1P45OutputTargetVal_t dcdc1P45OutputTarget)
{
  uint16_t mV;
  mV = (uint16_t)(dcdc1P45OutputTarget-gDCDC_1P45OutputTargetVal_1_275_c) * 25 + 1275;
  return mV;
}
///*---------------------------------------------------------------------------
//* NAME: DCDC_1P8OutputTargetTomV
//* DESCRIPTION: 
//* PARAMETERS:  
//* RETURN: 
//* NOTES: none
//*---------------------------------------------------------------------------*/
uint16_t DCDC_1P8OutputTargetTomV(dcdc1P8OutputTargetVal_t dcdc1P8OutputTarget)
{
  uint16_t mV;
  if( dcdc1P8OutputTarget <= gDCDC_1P8OutputTargetVal_2_050_c)
  {
    mV = (uint16_t)(dcdc1P8OutputTarget - gDCDC_1P8OutputTargetVal_1_650_c)*25 + 1650;
  }
  else if(dcdc1P8OutputTarget >= gDCDC_1P8OutputTargetVal_2_800_c )
  {
    mV = (uint16_t)(dcdc1P8OutputTarget - gDCDC_1P8OutputTargetVal_2_800_c)*25 + 2800;
  }
  else  
  {
    mV = 1800;
  }
  return mV;
}

///*---------------------------------------------------------------------------
//* NAME: DCDC_PrepareForPulsedMode
//* DESCRIPTION: 
//* PARAMETERS:  
//* RETURN: 
//* NOTES: none
//*---------------------------------------------------------------------------*/
bool_t DCDC_PrepareForPulsedMode(void)
{
#if gDCDC_Enabled_d == 0
    return TRUE;
#else
   if(mpDCDCConfig == NULL)
  {
    // DCDC_Init has to be called prior to this function
    return FALSE;
  }
  if(mpDCDCConfig->dcdcMode == gDCDC_Mode_Bypass_c)
  {
    return TRUE;
  }
  DCDC_BWR_REG3_DCDC_VDD1P8CTRL_DISABLE_STEP(DCDC_BASE_PTR,1);  
  DCDC_BWR_REG3_DCDC_VDD1P45CTRL_DISABLE_STEP(DCDC_BASE_PTR,1);    
  DCDC_BWR_REG1_DCDC_LOOPCTRL_EN_DF_HYST(DCDC_BASE_PTR,1);
  DCDC_BWR_REG1_DCDC_LOOPCTRL_EN_CM_HYST(DCDC_BASE_PTR,1);
  DCDC_BWR_REG2_DCDC_LOOPCTRL_HYST_SIGN(DCDC_BASE_PTR,1);
  DCDC_BWR_REG0_DCDC_LP_DF_CMP_ENABLE(DCDC_BASE_PTR,1);
  return TRUE;
#endif
}

///*---------------------------------------------------------------------------
//* NAME: DCDC_PrepareForContinuousMode
//* DESCRIPTION: 
//* PARAMETERS:  
//* RETURN: 
//* NOTES: none
//*---------------------------------------------------------------------------*/
bool_t DCDC_PrepareForContinuousMode(void)
{
#if gDCDC_Enabled_d == 0
    return TRUE;
#else
   if(mpDCDCConfig == NULL)
  {
    // DCDC_Init has to be called prior to this function
    return FALSE;
  }
  if(mpDCDCConfig->dcdcMode == gDCDC_Mode_Bypass_c)
  {
    return TRUE;
  }
  DCDC_BWR_REG3_DCDC_VDD1P8CTRL_DISABLE_STEP(DCDC_BASE_PTR,0);  
  DCDC_BWR_REG3_DCDC_VDD1P45CTRL_DISABLE_STEP(DCDC_BASE_PTR,0);    
  DCDC_BWR_REG1_DCDC_LOOPCTRL_EN_DF_HYST(DCDC_BASE_PTR,1);
  DCDC_BWR_REG1_DCDC_LOOPCTRL_EN_CM_HYST(DCDC_BASE_PTR,1);
  DCDC_BWR_REG2_DCDC_LOOPCTRL_HYST_SIGN(DCDC_BASE_PTR,1);
  DCDC_BWR_REG0_DCDC_LP_DF_CMP_ENABLE(DCDC_BASE_PTR,0);
  return TRUE;
#endif
}

///*---------------------------------------------------------------------------
//* NAME: DCDC_SetUpperLimitDutyCycle
//* DESCRIPTION: 
//* PARAMETERS:  
//* RETURN: 
//* NOTES: none
//*---------------------------------------------------------------------------*/
bool_t DCDC_SetUpperLimitDutyCycle(uint8_t upperLimitDutyCycle)
{
#if gDCDC_Enabled_d == 0
    return TRUE;
#else
   if(mpDCDCConfig == NULL)
  {
    // DCDC_Init has to be called prior to this function
    return FALSE;
  }
  if(mpDCDCConfig->dcdcMode == gDCDC_Mode_Bypass_c)
  {
    return TRUE;
  }
  if(mDCDC_DutyCycleMax_c < upperLimitDutyCycle)
  {
    return FALSE;
  }
  if(mpDCDCConfig->dcdcMode == gDCDC_Mode_Boost_c)
  {
    DCDC_BWR_REG1_POSLIMIT_BOOST_IN(DCDC_BASE_PTR,upperLimitDutyCycle);
  }
  else
  {
    DCDC_BWR_REG1_POSLIMIT_BUCK_IN(DCDC_BASE_PTR,upperLimitDutyCycle);
  }
  return TRUE;
#endif
}

///*---------------------------------------------------------------------------
//* NAME: DCDC_GetPowerSwitchStatus
//* DESCRIPTION: 
//* PARAMETERS:  
//* RETURN: 
//* NOTES: none
//*---------------------------------------------------------------------------*/
bool_t DCDC_GetPowerSwitchStatus(dcdcPSwStatus_t* pDCDCPSwStatus)
{
#if gDCDC_Enabled_d == 0
  *pDCDCPSwStatus = gDCDC_PSwStatus_High_c;
  return TRUE;
#else
  if(mpDCDCConfig == NULL)
  {
    // DCDC_Init has to be called prior to this function
    *pDCDCPSwStatus = gDCDC_PSwStatus_High_c;
    return FALSE;
  }
  if( DCDC_BRD_REG0_PSWITCH_STATUS(DCDC_BASE_PTR) == 0)
  {
    *pDCDCPSwStatus = gDCDC_PSwStatus_Low_c;
  }
  else
  {
    *pDCDCPSwStatus = gDCDC_PSwStatus_High_c;
  }
  return TRUE;
#endif
}

///*---------------------------------------------------------------------------
//* NAME: DCDC_ShutDown
//* DESCRIPTION: 
//* PARAMETERS:  
//* RETURN: 
//* NOTES: none
//*---------------------------------------------------------------------------*/
void DCDC_ShutDown(void)
{
#if gDCDC_Enabled_d == 0
  return ;
#else
  uint32_t reg4; 
  if(mpDCDCConfig == NULL)
  {
    // DCDC_Init has to be called prior to this function
    return;
  }
  if(mpDCDCConfig->dcdcMode == gDCDC_Mode_Buck_c)
  {
    if( DCDC_BRD_REG0_PSWITCH_STATUS(DCDC_BASE_PTR) == 0)
    {
      DCDC_WR_REG4(DCDC_BASE_PTR, (uint32_t)0x3E770001);
      DCDC_WR_REG4(DCDC_BASE_PTR, (uint32_t)0x3E770001);      
    }
  }
#endif
}
///*---------------------------------------------------------------------------
//* NAME: DCDC_InstallPSwitchCallback
//* DESCRIPTION: 
//* PARAMETERS:  
//* RETURN: 
//* NOTES: none
//*---------------------------------------------------------------------------*/
bool_t DCDC_InstallPSwitchCallback(pfDCDCPSwitchCallback_t pfPSwCallback, dcdcPSwIntEdge_t pSwIntEdge)
{
#if gDCDC_Enabled_d == 0
   return FALSE;
#else
   uint32_t reg4; 
  if(mpDCDCConfig == NULL)
  {
    // DCDC_Init has to be called prior to this function
    return FALSE;
  }
  NVIC_DisableIRQ(LVD_LVW_DCDC_IRQn);
  mpfDCDCPSwitchCallback = pfPSwCallback;
  
  if(pfPSwCallback != NULL)
  {
    if(gDCDC_PSwIntEdge_Rising_c & pSwIntEdge )
    {
      DCDC_BWR_REG6_PSWITCH_INT_RISE_EN(DCDC_BASE_PTR, 1);
    }
    if(gDCDC_PSwIntEdge_Falling_c & pSwIntEdge )
    {
      DCDC_BWR_REG6_PSWITCH_INT_FALL_EN(DCDC_BASE_PTR, 1);  
    }
    DCDC_BWR_REG6_PSWITCH_INT_MUTE(DCDC_BASE_PTR, 0);
    NVIC_SetPriority(LVD_LVW_DCDC_IRQn, 0x80);
    OSA_InstallIntHandler(LVD_LVW_DCDC_IRQn, DCDC_PSwitchIsr);
    NVIC_EnableIRQ(LVD_LVW_DCDC_IRQn);
  }
  else
  {
    DCDC_BWR_REG6_PSWITCH_INT_RISE_EN(DCDC_BASE_PTR, 0); 
    DCDC_BWR_REG6_PSWITCH_INT_FALL_EN(DCDC_BASE_PTR, 0);   
  }
  return TRUE;
#endif
}
/*****************************************************************************
 *                               <<< EOF >>>                                 *
 *****************************************************************************/
