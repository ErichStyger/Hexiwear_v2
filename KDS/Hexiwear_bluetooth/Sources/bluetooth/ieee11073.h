/*! *********************************************************************************
 * \addtogroup IEEE 11073
 * @{
 ********************************************************************************** */
/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file ieee_11073_defines.h
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

#ifndef _IEEE_11073_DEFINES_H_
#define _IEEE_11073_DEFINES_H_

/************************************************************************************
*************************************************************************************
* Public constants & macros
*************************************************************************************
************************************************************************************/

/*! IEEE 11073-20601 - Not a number 16 bit float */
#define gIeee11073_16Bit_NaN_c      0x07FF

/*! IEEE 11073-20601 - Not a number 32 bit float */
#define gIeee11073_32Bit_NaN_c      0x07FFFFFF

/*! IEEE 11073-20601 - Not at this resolution 16 bit float*/
#define gIeee11073_16Bit_Nres_c     0x0800

/*! IEEE 11073-20601 - Not at this resolution 32 bit float*/
#define gIeee11073_32Bit_Nres_c     0x08000000

/*! IEEE 11073-20601 - Plus Infinity 16 bit float*/
#define gIeee11073_16Bit_PlusInf   0x07FE

/*! IEEE 11073-20601 - Plus Infinity 32 bit float*/
#define gIeee11073_32Bit_PlusInf   0x07FFFFFE

/*! IEEE 11073-20601 - Minus Infinity 16 bit float*/
#define gIeee11073_16Bit_MinusInf  0x0802

/*! IEEE 11073-20601 - Minus Infinity 32 bit float*/
#define gIeee11073_32Bit_MinusInf  0x08000002

/*! IEEE 11073-20601 - Reserved for future use 16 bit float*/
#define gIeee11073_16Bit_Reserved  0x0801

/*! IEEE 11073-20601 - Reserved for future use 32 bit float*/
#define gIeee11073_32Bit_Reserved  0x08000001

/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/

/*! IEEE 11073-20601 - 16 bit float*/
typedef uint16_t ieee11073_16BitFloat_t;

typedef enum
{
    gExponent16_c = 0x000F,
    gMantissa16_c = 0xFFF0
} ieee11073_16BitFloat_tag;

/*! IEEE 11073-20601 - 32 bit float*/
typedef uint32_t ieee11073_32BitFloat_t;

typedef enum
{
    gExponent32_c = 0x000000FF,
    gMantissa32_c = 0xFFFFFF00
} ieee11073_32BitFloat_tag;

#endif /* _IEEE_11073_DEFINES_H_ */

/*! *********************************************************************************
* @}
********************************************************************************** */
