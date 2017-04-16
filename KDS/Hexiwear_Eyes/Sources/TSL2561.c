/*
 * TSL2561.c
 *  Ambient light sensor
 *  Created on: 10.04.2017
 *      Author: Erich Styger
 */

#include "TSL2561.h"
#include "GI2C0.h"

//#define TSL2561_PACKAGE_CS  /* define this for the CS package */

#define TSL2561_I2C_DEVICE_ADDRESS  0b0101001  /* device address with ADDR pin set to GND */
//#define TSL2561_I2C_DEVICE_ADDRESS  0b0111001  /* device address with ADDR pin floating */
//#define TSL2561_I2C_DEVICE_ADDRESS  0b1001001  /* device address with ADDR pin set to VDD */

#define TSL2561_LUX_SATURATED_VALUE  0x10000  /* special lux value in case sensor is saturated */

#define TSL2561_COMMAND_BIT (0x80)    // Must be 1
#define TSL2561_CLEAR_BIT   (0x40)    // Clears any pending interrupt (write 1 to clear)
#define TSL2561_WORD_BIT    (0x20)    // read/write word (rather than byte)
#define TSL2561_BLOCK_BIT   (0x10)    // using block read/write

#define TSL2561_CONTROL_POWERON  (0x03)
#define TSL2561_CONTROL_POWEROFF (0x00)

#define TSL2561_LUX_SCALE     (14)      // Scale by 2^14
#define TSL2561_RATIO_SCALE   (9)       // Scale ratio by 2^9
#define TSL2561_CHSCALE       (10)      // Scale channel values by 2^10
#define TSL2561_CHSCALE_TINT0 (0x7517)  // 322/11 * 2^TSL2561_LUX_CHSCALE
#define TSL2561_CHSCALE_TINT1 (0x0FE7)  // 322/81 * 2^TSL2561_LUX_CHSCALE

// T, FN and CL package values
#define TSL2561_LUX_K1T           (0x0040)  // 0.125 * 2^RATIO_SCALE
#define TSL2561_LUX_B1T           (0x01f2)  // 0.0304 * 2^LUX_SCALE
#define TSL2561_LUX_M1T           (0x01be)  // 0.0272 * 2^LUX_SCALE
#define TSL2561_LUX_K2T           (0x0080)  // 0.250 * 2^RATIO_SCALE
#define TSL2561_LUX_B2T           (0x0214)  // 0.0325 * 2^LUX_SCALE
#define TSL2561_LUX_M2T           (0x02d1)  // 0.0440 * 2^LUX_SCALE
#define TSL2561_LUX_K3T           (0x00c0)  // 0.375 * 2^RATIO_SCALE
#define TSL2561_LUX_B3T           (0x023f)  // 0.0351 * 2^LUX_SCALE
#define TSL2561_LUX_M3T           (0x037b)  // 0.0544 * 2^LUX_SCALE
#define TSL2561_LUX_K4T           (0x0100)  // 0.50 * 2^RATIO_SCALE
#define TSL2561_LUX_B4T           (0x0270)  // 0.0381 * 2^LUX_SCALE
#define TSL2561_LUX_M4T           (0x03fe)  // 0.0624 * 2^LUX_SCALE
#define TSL2561_LUX_K5T           (0x0138)  // 0.61 * 2^RATIO_SCALE
#define TSL2561_LUX_B5T           (0x016f)  // 0.0224 * 2^LUX_SCALE
#define TSL2561_LUX_M5T           (0x01fc)  // 0.0310 * 2^LUX_SCALE
#define TSL2561_LUX_K6T           (0x019a)  // 0.80 * 2^RATIO_SCALE
#define TSL2561_LUX_B6T           (0x00d2)  // 0.0128 * 2^LUX_SCALE
#define TSL2561_LUX_M6T           (0x00fb)  // 0.0153 * 2^LUX_SCALE
#define TSL2561_LUX_K7T           (0x029a)  // 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B7T           (0x0018)  // 0.00146 * 2^LUX_SCALE
#define TSL2561_LUX_M7T           (0x0012)  // 0.00112 * 2^LUX_SCALE
#define TSL2561_LUX_K8T           (0x029a)  // 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B8T           (0x0000)  // 0.000 * 2^LUX_SCALE
#define TSL2561_LUX_M8T           (0x0000)  // 0.000 * 2^LUX_SCALE

// CS package values
#define TSL2561_LUX_K1C           (0x0043)  // 0.130 * 2^RATIO_SCALE
#define TSL2561_LUX_B1C           (0x0204)  // 0.0315 * 2^LUX_SCALE
#define TSL2561_LUX_M1C           (0x01ad)  // 0.0262 * 2^LUX_SCALE
#define TSL2561_LUX_K2C           (0x0085)  // 0.260 * 2^RATIO_SCALE
#define TSL2561_LUX_B2C           (0x0228)  // 0.0337 * 2^LUX_SCALE
#define TSL2561_LUX_M2C           (0x02c1)  // 0.0430 * 2^LUX_SCALE
#define TSL2561_LUX_K3C           (0x00c8)  // 0.390 * 2^RATIO_SCALE
#define TSL2561_LUX_B3C           (0x0253)  // 0.0363 * 2^LUX_SCALE
#define TSL2561_LUX_M3C           (0x0363)  // 0.0529 * 2^LUX_SCALE
#define TSL2561_LUX_K4C           (0x010a)  // 0.520 * 2^RATIO_SCALE
#define TSL2561_LUX_B4C           (0x0282)  // 0.0392 * 2^LUX_SCALE
#define TSL2561_LUX_M4C           (0x03df)  // 0.0605 * 2^LUX_SCALE
#define TSL2561_LUX_K5C           (0x014d)  // 0.65 * 2^RATIO_SCALE
#define TSL2561_LUX_B5C           (0x0177)  // 0.0229 * 2^LUX_SCALE
#define TSL2561_LUX_M5C           (0x01dd)  // 0.0291 * 2^LUX_SCALE
#define TSL2561_LUX_K6C           (0x019a)  // 0.80 * 2^RATIO_SCALE
#define TSL2561_LUX_B6C           (0x0101)  // 0.0157 * 2^LUX_SCALE
#define TSL2561_LUX_M6C           (0x0127)  // 0.0180 * 2^LUX_SCALE
#define TSL2561_LUX_K7C           (0x029a)  // 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B7C           (0x0037)  // 0.00338 * 2^LUX_SCALE
#define TSL2561_LUX_M7C           (0x002b)  // 0.00260 * 2^LUX_SCALE
#define TSL2561_LUX_K8C           (0x029a)  // 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B8C           (0x0000)  // 0.000 * 2^LUX_SCALE
#define TSL2561_LUX_M8C           (0x0000)  // 0.000 * 2^LUX_SCALE

#define TSL2561_REG_CONTROL          (0x00)
#define TSL2561_REG_TIMING           (0x01)
#define TSL2561_REG_THRESHHOLDL_LOW  (0x02)
#define TSL2561_REG_THRESHHOLDL_HIGH (0x03)
#define TSL2561_REG_THRESHHOLDH_LOW  (0x04)
#define TSL2561_REG_THRESHHOLDH_HIGH (0x05)
#define TSL2561_REG_INTERRUPT        (0x06)
#define TSL2561_REG_CRC              (0x08)
#define TSL2561_REG_ID               (0x0A)
#define TSL2561_REG_CHAN0_LOW        (0x0C)
#define TSL2561_REG_CHAN0_HIGH       (0x0D)
#define TSL2561_REG_CHAN1_LOW        (0x0E)
#define TSL2561_REG_CHAN1_HIGH       (0x0F)

// Clipping thresholds
#define TSL2561_CLIPPING_13MS     (4900)
#define TSL2561_CLIPPING_101MS    (37000)
#define TSL2561_CLIPPING_402MS    (65000)

#define TSL2561_LUX_LUXSCALE      (14)      // Scale by 2^14
#define TSL2561_LUX_RATIOSCALE    (9)       // Scale ratio by 2^9
#define TSL2561_LUX_CHSCALE       (10)      // Scale channel values by 2^10
#define TSL2561_LUX_CHSCALE_TINT0 (0x7517)  // 322/11 * 2^TSL2561_LUX_CHSCALE
#define TSL2561_LUX_CHSCALE_TINT1 (0x0FE7)  // 322/81 * 2^TSL2561_LUX_CHSCALE

static uint8_t _tsl2561IntegrationTime = TSL2561_INTEGRATION_TIME_13MS;
static uint8_t _tsl2561Gain = TSL2561_GAIN_16X;

uint8_t TSL2561_Disable(void) {
  return GI2C0_WriteByteAddress8(TSL2561_I2C_DEVICE_ADDRESS, TSL2561_COMMAND_BIT|TSL2561_REG_CONTROL, TSL2561_CONTROL_POWEROFF);
}

uint8_t TSL2561_Enable(void) {
  return GI2C0_WriteByteAddress8(TSL2561_I2C_DEVICE_ADDRESS, TSL2561_COMMAND_BIT|TSL2561_REG_CONTROL, TSL2561_CONTROL_POWERON);
}

uint8_t TSL2561_ReadID(uint8_t *id) {
  return GI2C0_ReadByteAddress8(TSL2561_I2C_DEVICE_ADDRESS, TSL2561_REG_ID, id);
}

uint8_t TSL2561_SetIntegrationTime(uint8_t timing) {
  _tsl2561IntegrationTime = timing;
  return GI2C0_WriteByteAddress8(TSL2561_I2C_DEVICE_ADDRESS, TSL2561_COMMAND_BIT|TSL2561_REG_TIMING, timing|_tsl2561Gain);
}

uint8_t TSL2561_SetGain(uint8_t gain) {
  _tsl2561Gain = gain;
  return GI2C0_WriteByteAddress8(TSL2561_I2C_DEVICE_ADDRESS, TSL2561_COMMAND_BIT|TSL2561_REG_TIMING, gain|_tsl2561IntegrationTime);
}

uint8_t TSL2561_ReadRawDataFull(uint16_t *data) {
  /* read IR and visible light channel */
  uint8_t res;

  *data = 0;
  res = GI2C0_ReadWordAddress8(TSL2561_I2C_DEVICE_ADDRESS, TSL2561_COMMAND_BIT|TSL2561_REG_CHAN0_LOW, data);
  if (res!=ERR_OK) {
    return res;
  }
  return ERR_OK;
}

uint8_t TSL2561_ReadRawDataInfrared(uint16_t *data) {
  uint8_t res;

  *data = 0;
  res = GI2C0_ReadWordAddress8(TSL2561_I2C_DEVICE_ADDRESS, TSL2561_COMMAND_BIT|TSL2561_REG_CHAN1_LOW, data);
  if (res!=ERR_OK) {
    return res;
  }
  return ERR_OK;
}

/**************************************************************************/
/*!
    Converts the raw sensor values to the standard SI lux equivalent.
    Returns 0 if the sensor is saturated and the values are unreliable.
    This is taken from https://github.com/adafruit/Adafruit_TSL2561/blob/master/Adafruit_TSL2561_U.cpp
*/
/**************************************************************************/
uint32_t TSL2561_CalculateLux(uint16_t broadband, uint16_t ir) {
  unsigned long chScale;
  unsigned long channel1;
  unsigned long channel0;

  /* Make sure the sensor isn't saturated! */
  uint16_t clipThreshold;

  switch (_tsl2561IntegrationTime) {
    case TSL2561_INTEGRATION_TIME_13MS:
      clipThreshold = TSL2561_CLIPPING_13MS;
      break;
    case TSL2561_INTEGRATION_TIME_101MS:
      clipThreshold = TSL2561_CLIPPING_101MS;
      break;
    default:
      clipThreshold = TSL2561_CLIPPING_402MS;
      break;
  }

  /* Return 65536 lux if the sensor is saturated */
  if ((broadband > clipThreshold) || (ir > clipThreshold)) {
    return TSL2561_LUX_SATURATED_VALUE;
  }

  /* Get the correct scale depending on the integration time */
  switch (_tsl2561IntegrationTime) {
    case TSL2561_INTEGRATION_TIME_13MS:
      chScale = TSL2561_LUX_CHSCALE_TINT0;
      break;
    case TSL2561_INTEGRATION_TIME_101MS:
      chScale = TSL2561_LUX_CHSCALE_TINT1;
      break;
    default: /* No scaling ... integration time = 402ms */
      chScale = (1 << TSL2561_LUX_CHSCALE);
      break;
  }

  /* Scale for gain (1x or 16x) */
  if (!_tsl2561Gain) {
    chScale = chScale << 4;
  }

  /* Scale the channel values */
  channel0 = (broadband * chScale) >> TSL2561_LUX_CHSCALE;
  channel1 = (ir * chScale) >> TSL2561_LUX_CHSCALE;

  /* Find the ratio of the channel values (Channel1/Channel0) */
  unsigned long ratio1 = 0;
  if (channel0 != 0) ratio1 = (channel1 << (TSL2561_LUX_RATIOSCALE+1)) / channel0;

  /* round the ratio value */
  unsigned long ratio = (ratio1 + 1) >> 1;

  unsigned int b, m;

#ifdef TSL2561_PACKAGE_CS
  if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1C))
    {b=TSL2561_LUX_B1C; m=TSL2561_LUX_M1C;}
  else if (ratio <= TSL2561_LUX_K2C)
    {b=TSL2561_LUX_B2C; m=TSL2561_LUX_M2C;}
  else if (ratio <= TSL2561_LUX_K3C)
    {b=TSL2561_LUX_B3C; m=TSL2561_LUX_M3C;}
  else if (ratio <= TSL2561_LUX_K4C)
    {b=TSL2561_LUX_B4C; m=TSL2561_LUX_M4C;}
  else if (ratio <= TSL2561_LUX_K5C)
    {b=TSL2561_LUX_B5C; m=TSL2561_LUX_M5C;}
  else if (ratio <= TSL2561_LUX_K6C)
    {b=TSL2561_LUX_B6C; m=TSL2561_LUX_M6C;}
  else if (ratio <= TSL2561_LUX_K7C)
    {b=TSL2561_LUX_B7C; m=TSL2561_LUX_M7C;}
  else if (ratio > TSL2561_LUX_K8C)
    {b=TSL2561_LUX_B8C; m=TSL2561_LUX_M8C;}
#else
  if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1T))
    {b=TSL2561_LUX_B1T; m=TSL2561_LUX_M1T;}
  else if (ratio <= TSL2561_LUX_K2T)
    {b=TSL2561_LUX_B2T; m=TSL2561_LUX_M2T;}
  else if (ratio <= TSL2561_LUX_K3T)
    {b=TSL2561_LUX_B3T; m=TSL2561_LUX_M3T;}
  else if (ratio <= TSL2561_LUX_K4T)
    {b=TSL2561_LUX_B4T; m=TSL2561_LUX_M4T;}
  else if (ratio <= TSL2561_LUX_K5T)
    {b=TSL2561_LUX_B5T; m=TSL2561_LUX_M5T;}
  else if (ratio <= TSL2561_LUX_K6T)
    {b=TSL2561_LUX_B6T; m=TSL2561_LUX_M6T;}
  else if (ratio <= TSL2561_LUX_K7T)
    {b=TSL2561_LUX_B7T; m=TSL2561_LUX_M7T;}
  else if (ratio > TSL2561_LUX_K8T)
    {b=TSL2561_LUX_B8T; m=TSL2561_LUX_M8T;}
#endif

  unsigned long temp;
  temp = ((channel0 * b) - (channel1 * m));

  /* Do not allow negative lux value */
  if (temp < 0) {
    temp = 0;
  }

  /* Round lsb (2^(LUX_SCALE-1)) */
  temp += (1 << (TSL2561_LUX_LUXSCALE-1));

  /* Strip off fractional portion */
  uint32_t lux = temp >> TSL2561_LUX_LUXSCALE;

  return lux;
}

void TSL2561_Init(void) {
}

