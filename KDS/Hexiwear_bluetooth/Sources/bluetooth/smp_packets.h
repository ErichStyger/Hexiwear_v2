/*! *********************************************************************************
* \defgroup BLESecurityManagerProtocol BLE Secuirty Manager Protocol
* @{
********************************************************************************** */
/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file smp_packets.c
* This header file contains BLE Security Manager Protocol packet structures
* defined by the Bluetooth specification.
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
********************************************************************************** */

#ifndef _SMP_PACKETS_H_
#define _SMP_PACKETS_H_

#include "smp_types.h"

typedef PACKED_STRUCT smpAuthReq_tag
{
    uint8_t         bondingFlags   :2; /*< \ref smpAuthReqBondingFlags_t */
    uint8_t         mitm           :1; /*< \ref smpAuthReqMitm_t */
    uint8_t         reserved       :5;
} smpAuthReq_t;

#define smpAuthReq_Mask_c       0x07

typedef PACKED_STRUCT smpPairingKeyDistribution_tag
{
    uint8_t         encKey         :1;  /* LTK */
    uint8_t         idKey          :1;  /* IRK */
    uint8_t         sign           :1;  /* CSRK */
    uint8_t         reserved       :5;
} smpKeyDistribution_t;

#define smpPairingKeyDistribution_Mask_c    0x07


/******************************************************
** SMP Pairing Methods
******************************************************/

/*! Pairing Request : Code : 0x01 */
typedef PACKED_STRUCT smpPairingRequestResponse_tag
{
    uint8_t                     ioCapability;               /*!< \ref smpPairingIoCapability_t */
    uint8_t                     oobDataFlag;                /*!< \ref smpPairingOobDataFlag_t */
    smpAuthReq_t                authReq;                    /*!< Structure of size uint8_t */
    uint8_t                     maximumEncryptionkeySize;
    smpKeyDistribution_t        initatorKeyDistribution;    /*!< Structure of size uint8_t */
    smpKeyDistribution_t        responderKeyDistribution;   /*!< Structure of size uint8_t */
} smpPairingRequestResponse_t;

/*! Pairing Response : Code : 0x02 */
/*! Same parameter structure as the SMP Pairing Request Packet */

/*! Pairing Confirm : Code : 0x03 */
typedef PACKED_STRUCT   smpPairingConfirm_tag
{
    uint8_t         confirmValue[gSmpConfirmValueSize_c]; /*!< Mconfirm or Sconfirm */
} smpPairingConfirm_t;

/*! Pairing Random : Code : 0x04 */
typedef PACKED_STRUCT   smpPairingRandom_tag
{
    uint8_t         randomValue[gSmpConfirmValueSize_c]; /*!< Mrand or Srand */
} smpPairingRandom_t;

/*! Pairing Failed : Code : 0x05 */
typedef PACKED_STRUCT   smpPairingFailed_tag
{
    uint8_t         reason; /*!< \ref smpPairingFailedReason_t */
} smpPairingFailed_t;


/******************************************************
** SMP Key Distribution Commands
******************************************************/

/*! Encryption Information (Long Term Key) : Code : 0x06 */
typedef PACKED_STRUCT   smpEncryptionInformation_tag
{
    uint8_t         longTermKey[gSmpLtkSize_c];
} smpEncryptionInformation_t;

/*! Master Identification (EDIV and Rand) : Code : 0x07 */
typedef PACKED_STRUCT   smpMasterIdentification_tag
{
    uint8_t         ediv[gSmpLlEncryptionEdivSize_c];
    uint8_t         rand[gSmpLlEncryptionRandSize_c];
} smpMasterIdentification_t;

/*! Identity Information (Identity Resolving Key) : Code : 0x08 */
typedef PACKED_STRUCT   smpIdentityInformation_tag
{
    uint8_t         identityResolvingKey[gSmpIrkSize_c];
} smpIdentityInformation_t;

/*! Identity Address Information (BD_ADDR and it's type) : Code : 0x09 */
typedef PACKED_STRUCT   smpIdentityAddressInformation_tag
{
    uint8_t         addrType; /*!< \ref bleAddressType_t */
    uint8_t         bdAddr[6];
} smpIdentityAddressInformation_t;

/*! Signing Information (CSRK - Connection Signature Resolving Key) : Code : 0x0A */
typedef PACKED_STRUCT   smpSigningInformation_tag
{
    uint8_t         signatureKey[gSmpCsrkSize_c];
} smpSigningInformation_t;

/*! Security Request : Code : 0x0B */
typedef PACKED_STRUCT   smpSecurityRequestd_tag
{
    smpAuthReq_t    authReq;
} smpSecurityRequest_t;


/******************************************************
** SMP General Command Packet Structure
******************************************************/
typedef PACKED_STRUCT smpPacket_tag
{
    uint8_t            code; /*< \ref smpOpcode_t */
    PACKED_UNION
    {
        /* Pairing Methods */
        smpPairingRequestResponse_t         smpPairingRequest;
        smpPairingRequestResponse_t         smpPairingResponse;
        smpPairingConfirm_t                 smpPairingConfirm;
        smpPairingRandom_t                  smpPairingRandom;
        smpPairingFailed_t                  smpPairingFailed;
        /* Key Distribution */
        smpEncryptionInformation_t          smpEncryptionInformation;       /*!< LTK - Long Term Key */
        smpMasterIdentification_t           smpMasterIdentification;        /*!< EDIV and Rand */
        smpIdentityInformation_t            smpIdentityInformation;         /*!< IRK - Identity Resolving Key */
        smpIdentityAddressInformation_t     smpIdentityAddressInformation;  /*!< BD_ADDR and it's type */
        smpSigningInformation_t             smpSigningInformation;          /*!< CSRK - Connection Signature Resolvin Key */
        smpSecurityRequest_t                smpSecurityRequest;
    } data;
} smpPacket_t;

#endif /* _SMP_PACKETS_H_ */

/*! *********************************************************************************
* @}
********************************************************************************** */
