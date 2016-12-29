/*! *********************************************************************************
* \addtogroup BLESecurityManagerProtocol
* @{
********************************************************************************** */
/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file smp_types.c
* This header file contains BLE Security Manager Protocol types and values defined
* by the Bluetooth specification
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

#ifndef _SMP_TYPES_H_
#define _SMP_TYPES_H_

#define gSmpDefaultMtu_c        (23)
#define gSmpMaxMtu_c            (23)

#define gSmpL2capChannel_c      (0x0006)    /*!< L2CAP Channel reserved for the Security Manager Protocol */

#define gSmpTkSize_c            (16)        /*!< SM Temporary Key size in octets */
#define gSmpStkSize_c           (16)        /*!< SM Short Term Key size in octets */

#define gSmpLtkSize_c           (16)        /*!< SM Long Term Key size in octets */
#define gSmpIrkSize_c           (16)        /*!< SM Identity Resolving Key size in octets */
#define gSmpCsrkSize_c          (16)        /*!< SM Connection Signature Resolving Key size in octets */

#define gSmpMinKeySize_c        (7)         /*!< Minimum encryption key size accepted by the BLE SMP */
#define gSmpMaxKeySize_c        (16)        /*!< Maximum encryption key size accepted by the BLE SMP */

#define gSmpConfirmValueSize_c  (16)        /*!< Confirm value size - used during the SMP Pairing procedure.  */
#define gSmpPairingRandSize_c   (16)        /*!< Random number size - used during the SMP Pairing procedure for the calculation of the SMP Pairing Confirm value by the Initiator and Responder. */

#define gSmpMaxPasskeyValue_c   (999999U)

#define gSmpLlEncryptionRandSize_c  (8)     /*!< 8 byte random number used during the Link Layer Connection Encryption procedure */
#define gSmpLlEncryptionEdivSize_c  (2)     /*!< 2 byte encryption diversified used during the Link Layer Connection Encryption procedure */
#define gSmpLlEncryptBlockSize_c    (16)    /*!< AES block size used by the BLE Link Layer data encrypt command. */ 
#define gSmpLlRandSize_c            (8)     /*!< Link Layer random number size - returned by LL random number generation procedure. */

#define gSmpAuthSignatureLength_c   (8)

#define gSmpTimeoutSeconds_c        (30)

/*! This is the enumeration for the BLE SMP command codes defined by the specification. */
typedef enum
{
    gSmpNoOpcode_c                          = 0x00,
    /* Pairing Methods */
    gSmpOpcodePairingRequest_c              = 0x01,
    gSmpOpcodePairingResponse_c             = 0x02,
    gSmpOpcodePairingConfirm_c              = 0x03,
    gSmpOpcodePairingRandom_c               = 0x04,
    gSmpOpcodePairingFailed_c               = 0x05,
    /* Key Distribution */
    gSmpOpcodeEncryptionInformation_c       = 0x06,
    gSmpOpcodeMasterIdentification_c        = 0x07,
    gSmpOpcodeIdentityInformation_c         = 0x08,
    gSmpOpcodeIdentityAddressInformation_c  = 0x09,
    gSmpOpcodeSigningInformation_c          = 0x0A,
    /* Slave Security Request */
    gSmpOpcodeSecurityRequest_c             = 0x0B,
} smpOpcode_t;


/* Group of constants for SMP packet lengths in bytes */
/* Pairing Methods */
#define gSmpPairingRequestPacketLength_c                (7U)  /*!< Code[1] + IoCap[1] + OOB[1] + AuthReq[1] + MaxEncKeyS[1] + IKeyDist[1] + RKeyDist[1] */
#define gSmpPairingResponsePacketLength_c               (7U)  /*!< Code[1] + IoCap[1] + OOB[1] + AuthReq[1] + MaxEncKeyS[1] + IKeyDist[1] + RKeyDist[1] */
#define gSmpPairingConfirmPacketLength_c                (17U) /*!< Code[1] + ConfirmValue[16] */
#define gSmpPairingRandomPacketLength_c                 (17U) /*!< Code[1] + RandomValue[16] */
#define gSmpPairingFailedPacketLength_c                 (2U)  /*!< Code[1] + Reason[1] */
/* Key Distribution */
#define gSmpEncryptionInformationPacketLength_c         (17U) /*!< Code[1] + LTK[16] */
#define gSmpMasterIdentificationPacketLength_c          (11U) /*!< Code[1] + EDIV[2] + Rand[8] */
#define gSmpIdentityInformationPacketLength_c           (17U) /*!< Code[1] + IRK[16] */
#define gSmpIdentityAddressInformationPacketLength_c    (8U)  /*!< Code[1] + AddrType[1] + BD_ADDR[6] */
#define gSmpSigningInformationPacketLength_c            (17U) /*!< Code[1] + CSRK[16] */
/* Slave Security Request */
#define gSmpSecurityRequestPacketLength_c               (2U)  /*!< Code[1] + AuthReq[1] */

/*! This is the enumeration for the BLE SMP Pairing Failed - Reason field. */
typedef enum
{
    gSmpPairingFailedReasonReserved_c                   = 0x00,
    gSmpPairingFailedReasonPasskeyEntryFailed_c         = 0x01,
    gSmpPairingFailedReasonOobNotAvailable_c            = 0x02,
    gSmpPairingFailedReasonAuthenticationRequirements_c = 0x03,
    gSmpPairingFailedReasonConfirmValueFailed_c         = 0x04,
    gSmpPairingFailedReasonPairingNotSupported_c        = 0x05,
    gSmpPairingFailedReasonEncryptionKeySize_c          = 0x06,
    gSmpPairingFailedReasonCommandNotSupported_c        = 0x07,
    gSmpPairingFailedReasonUnspecifiedReason_c          = 0x08,
    gSmpPairingFailedReasonRepeatedAttempts_c           = 0x09,
    gSmpPairingFailedReasonInvalidParameters_c          = 0x0A,
    /* BLE 4.1 0x0B-0xFF Reserved */
} smpPairingFailedReason_t;

typedef enum
{
    gSmpPairingIoCapabDisplayOnly_c         = 0x00,
    gSmpPairingIoCapabDisplayyesNo_c        = 0x01,
    gSmpPairingIoCapabKeyboardOnly_c        = 0x02,
    gSmpPairingIoCapabNoInputNoOutput_c     = 0x03,
    gSmpPairingIoCapabKeyboardDisplay_c     = 0x04,
} smpPairingIoCapability_t;

typedef enum
{
    gSmpPairingOobDataFlagAuthDataNotPresent_c      = 0x00,
    gSmpPairingOobDataFlagAuthDataPresent_c         = 0x01,
} smpPairingOobDataFlag_t;

/*! Authentication request field, bonding flags subfield. */
typedef enum
{
    gSmpAuthReqBondingFlagsNoBonding_c      = 0x00,
    gSmpAuthReqBondingFlagsBonding_c        = 0x01,
    gSmpAuthReqBondingFlagsReserved10_c     = 0x02,
    gSmpAuthReqBondingFlagsReserved11_c     = 0x03,
} smpAuthReqBondingFlags_t;

/*! Authentication request field, Man In The Middle Protection flag subfield. */
typedef enum
{
    gSmpAuthReqMitmReqOff_c          = 0x00,
    gSmpAuthReqMitmReqOn_c           = 0x01,
} smpAuthReqMitm_t;


#endif /* _SMP_TYPES_H_ */

/*! *********************************************************************************
* @}
********************************************************************************** */
