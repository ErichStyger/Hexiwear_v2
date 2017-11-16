/*
 * hack.c
 *
 *  Created on: 16.11.2017
 *      Author: Erich Styger Local
 */


/* hack to enable power for sensors */

#if 1
#include <stdint.h>
#define __IO volatile

/** I2C - Register Layout Typedef */
typedef struct {
  __IO uint8_t A1;                                 /**< I2C Address Register 1, offset: 0x0 */
  __IO uint8_t F;                                  /**< I2C Frequency Divider register, offset: 0x1 */
  __IO uint8_t C1;                                 /**< I2C Control Register 1, offset: 0x2 */
  __IO uint8_t S;                                  /**< I2C Status register, offset: 0x3 */
  __IO uint8_t D;                                  /**< I2C Data I/O register, offset: 0x4 */
  __IO uint8_t C2;                                 /**< I2C Control Register 2, offset: 0x5 */
  __IO uint8_t FLT;                                /**< I2C Programmable Input Glitch Filter register, offset: 0x6 */
  __IO uint8_t RA;                                 /**< I2C Range Address register, offset: 0x7 */
  __IO uint8_t SMB;                                /**< I2C SMBus Control and Status register, offset: 0x8 */
  __IO uint8_t A2;                                 /**< I2C Address Register 2, offset: 0x9 */
  __IO uint8_t SLTH;                               /**< I2C SCL Low Timeout Register High, offset: 0xA */
  __IO uint8_t SLTL;                               /**< I2C SCL Low Timeout Register Low, offset: 0xB */
} I2C_Type, *I2C_MemMapPtr;
/**
 * @brief Macro to access a single bit of an 8-bit peripheral register (bit band region
 *        0x40000000 to 0x400FFFFF) using the bit-band alias region access.
 * @param Reg Register to access.
 * @param Bit Bit number to access.
 * @return Value of the targeted bit in the bit band region.
 */
#define BITBAND_ACCESS8(Reg,Bit) (*((uint8_t volatile*)(0x42000000u + (32u*((uintptr_t)(Reg) - (uintptr_t)0x40000000u)) + (4u*((uintptr_t)(Bit))))))

#define I2C_C1_REG(base)                         ((base)->C1)

#define I2C_C1_IICEN_SHIFT                       7
#define I2C_BWR_C1_IICEN(base, value) (BITBAND_ACCESS8(&I2C_C1_REG(base), I2C_C1_IICEN_SHIFT) = (value))


static void I2C_HAL_Enable(I2C_Type * base)
{
    I2C_BWR_C1_IICEN(base, 0x1U);
}

#define I2C0   ((I2C_Type*)0x40066000)

void hack(void) {
  I2C_HAL_Enable(I2C0);
}

#endif
