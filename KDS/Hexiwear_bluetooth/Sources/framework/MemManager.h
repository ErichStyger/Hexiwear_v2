/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file MemManager.h
* This is the header file for the Memory Manager interface.
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

#ifndef _MEM_MANAGER_H_
#define _MEM_MANAGER_H_


/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include "EmbeddedTypes.h"
#include "GenericList.h"

/*! *********************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
********************************************************************************** */
/*Defines pools by block size and number of blocks. Must be alligned to 4 bytes.*/     
#ifndef PoolsDetails_c
#define PoolsDetails_c \
         _block_size_  64  _number_of_blocks_    8 _eol_  \
         _block_size_ 128  _number_of_blocks_    2 _eol_  \
         _block_size_ 256  _number_of_blocks_   10 _eol_
#endif


/*! *********************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
********************************************************************************** */
/*Defines statuses used in MEM_BufferAlloc and MEM_BufferFree*/
typedef enum
{
  MEM_SUCCESS_c = 0,                    /* No error occurred */
  MEM_INIT_ERROR_c,                     /* Memory initialization error */
  MEM_ALLOC_ERROR_c,                    /* Memory allocation error */
  MEM_FREE_ERROR_c,                     /* Memory free error */
  MEM_UNKNOWN_ERROR_c                   /* something bad has happened... */
}memStatus_t;


/*! *********************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
********************************************************************************** */
/*Initialises the Memory Manager.*/
memStatus_t MEM_Init(void);
/*Returns the number of available blocks that fit the given size.*/
uint32_t MEM_GetAvailableBlocks(uint32_t size);
/*Frees the givem buffer.*/
memStatus_t MEM_BufferFree(void* buffer);
/*Returns the allocated buffer of the given size.*/
void* MEM_BufferAlloc(uint32_t numBytes);
/*Returns the size of a given buffer*/
uint16_t MEM_BufferGetSize(void* buffer);
/*Performs a write-read-verify test accross all pools*/
uint32_t MEM_WriteReadTest(void);


/*! *********************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
********************************************************************************** */
#if defined(MEM_TRACKING) && defined(DEBUG_ASSERT)
#define MEM_ASSERT(condition) if(!(condition))while(1);
#else
#define MEM_ASSERT(condition) (void)(condition);
#endif


/*! *********************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
********************************************************************************** */

#ifdef MEM_STATISTICS
/*Statistics structure definition. Used by pools.*/
typedef struct poolStat_tag
  {
  uint16_t numBlocks;
  uint16_t allocatedBlocks;
  uint16_t allocatedBlocksPeak;
  uint16_t allocationFailures;
  uint16_t freeFailures;
#ifdef MEM_TRACKING
  uint16_t poolFragmentWaste;
  uint16_t poolFragmentWastePeak;
#endif /*MEM_TRACKING*/
  } poolStat_t;
#endif /*MEM_STATISTICS*/

#ifdef MEM_TRACKING
/*Definition for alloc indicators. Used in buffer tracking.*/
typedef enum
{
  MEM_TRACKING_FREE_c = 0,
  MEM_TRACKING_ALLOC_c,
}memTrackingStatus_t;

/*Tracking structure definition.*/
typedef PACKED_STRUCT BlockTracking_tag
{
  void *blockAddr;                  /*Addr of Msg, not that this pointer is
                                     4 byte bigger than the addr in the pool
                                     has the header of the msg is 4 bytes */
  uint16_t blockSize;               /*Size of block in bytes.*/
  uint16_t fragmentWaste;           /*Size requested by allocator.*/
  void *allocAddr;                  /*Return address of last Alloc made */
  void *freeAddr;                   /*Return address of last Free made */
  uint16_t allocCounter;            /*No of time this msg has been allocated */
  uint16_t freeCounter;             /*No of time this msg has been freed */
  memTrackingStatus_t allocStatus;  /*1 if currently allocated, 0 if currently free */
  // uint8_t padding;
}blockTracking_t;
#endif /*MEM_TRACKING*/

/*Header description for buffers.*/
typedef struct listHeader_tag
{
  listElement_t link;
  struct pools_tag *pParentPool;
}listHeader_t;

/*Buffer pools. Used by most functions*/
typedef PACKED_STRUCT pools_tag
{
  list_t anchor; /* MUST be first element in pools_t struct */
  uint16_t nextBlockSize;
  uint16_t blockSize;
#ifdef MEM_STATISTICS
  poolStat_t poolStatistics;
  uint8_t padding[2];
#endif /*MEM_STATISTICS*/   
}pools_t;

/*Buffer pool description. Used by MM_Init() for creating the buffer pools. */
typedef PACKED_STRUCT poolInfo_tag
{
  uint16_t blockSize;
  uint16_t poolSize;
 /* uint8_t padding[2]; */
}poolInfo_t;

/*! *********************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
********************************************************************************** */

#ifdef MEM_TRACKING
uint8_t MEM_Track(listHeader_t *block, memTrackingStatus_t alloc, uint32_t address, uint16_t requestedSize);
#endif /*MEM_TRACKING*/
uint8_t MEM_BufferCheck(uint8_t *p, uint32_t size);

#endif /* _MEM_MANAGER_H_ */ 
