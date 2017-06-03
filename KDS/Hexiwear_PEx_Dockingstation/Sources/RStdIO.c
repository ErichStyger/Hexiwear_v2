/**
 * \file
 * \brief Radio/remote Standard I/O module.
 * \author (c) 2013-2014 Erich Styger, http://mcuoneclipse.com/
 * \note MIT License (http://opensource.org/licenses/mit-license.html), see 'RNet_License.txt'
 *
 * This module uses queues for remote/radio standard I/O handling.
 */

//#include "RNetConf.h"
#include "RStdIO.h"
#include "FRTOS1.h"
#include "UTIL1.h"
#include "CLS1.h"
//#include "Radio.h"
//#include "RMSG.h"
//#include "RNWK.h"
//#include "RApp.h"

/* three shell queues, one for each channel.
 * E.g. the command "radio send stdin help" will place "help" into the RSTDIO_RxStdIn queue, which then will be processed by the shell.
 *  */
static xQueueHandle RSTDIO_RxStdInQ, RSTDIO_RxStdOutQ, RSTDIO_RxStdErrQ;
//static xQueueHandle RSTDIO_TxStdInQ, RSTDIO_TxStdOutQ, RSTDIO_TxStdErrQ;

/* message is
 * char-write-req 61 020501AABBCCDD00000000000000000000000000
 * 02: notification type
 * 05: size
 * 01: type
 * data, max 17 bytes
 */
#define RSTDIO_PAYLOAD_SIZE       (17) /* data size we can transmit in one message. stdout string will be added */

#define RSTDIO_QUEUE_LENGTH        64 /* items in queue, that's my buffer size */
#define RSTDIO_QUEUE_ITEM_SIZE     1 /* each item is a single character */

#define RSTDIO_QUEUE_TIMEOUT_MS   500 /* timeout for stdio queues */

//static RNWK_ShortAddrType RSTDIO_dstAddr; /* destination address */

uint8_t RSTDIO_DefaultShellBuffer[CLS1_DEFAULT_SHELL_BUFFER_SIZE]; /* default buffer which can be used by the application */

//void RSTDIO_SetDestinationAddress(RNWK_ShortAddrType addr) {
//  RSTDIO_dstAddr = addr;
//}

/*!
 * \brief Returns a queue handle for a Remote Standard I/O type
 * \param queueType Type of queue
 * \return Queue handle
 */
xQueueHandle RSTDIO_GetQueueForType(RSTDIO_QueueType queueType) {
  switch(queueType) {
    case RSTDIO_QUEUE_RX_IN:  return RSTDIO_RxStdInQ;
    case RSTDIO_QUEUE_RX_OUT: return RSTDIO_RxStdOutQ;
    case RSTDIO_QUEUE_RX_ERR: return RSTDIO_RxStdErrQ;
#if 0
    case RSTDIO_QUEUE_TX_IN:  return RSTDIO_TxStdInQ;
    case RSTDIO_QUEUE_TX_OUT: return RSTDIO_TxStdOutQ;
    case RSTDIO_QUEUE_TX_ERR: return RSTDIO_TxStdErrQ;
#endif
    default: break;
  }
  return NULL;
}

/*!
 * \brief Adds binary data to the queue.
 * \param queue Queue to be used
 * \param data Pointer to the binary data
 * \param dataSize Size of data in bytes
 */
static uint8_t AddToQueue(xQueueHandle queue, const unsigned char *data, size_t dataSize) {
  while(dataSize!=0) {
    if (xQueueSendToBack(queue, data, RSTDIO_QUEUE_TIMEOUT_MS/portTICK_PERIOD_MS)!=pdPASS) {
      return ERR_FAULT;
    }
    data++;
    dataSize--;
  }
  return ERR_OK;
}

uint8_t RSTDIO_AddToQueue(RSTDIO_QueueType queueType, const unsigned char *data, size_t dataSize) {
  switch(queueType) {
    case RSTDIO_QUEUE_RX_IN:  return AddToQueue(RSTDIO_RxStdInQ,  data, dataSize);
    case RSTDIO_QUEUE_RX_OUT: return AddToQueue(RSTDIO_RxStdOutQ, data, dataSize);
    case RSTDIO_QUEUE_RX_ERR: return AddToQueue(RSTDIO_RxStdErrQ, data, dataSize);
#if 0
    case RSTDIO_QUEUE_TX_IN:  return AddToQueue(RSTDIO_TxStdInQ,  data, dataSize);
    case RSTDIO_QUEUE_TX_OUT: return AddToQueue(RSTDIO_TxStdOutQ, data, dataSize);
    case RSTDIO_QUEUE_TX_ERR: return AddToQueue(RSTDIO_TxStdErrQ, data, dataSize);
#endif
    default: break;
  }
  return ERR_FAILED;
}

/*!
 * \brief Peeks a character from the queue without removing it.
 * \param queue Queue to be used
 * \return '\0' if the queue is empty, otherwise it contains the character. 
 */
static unsigned short RSTDIO_NofElements(xQueueHandle queue) {
  return (unsigned short)uxQueueMessagesWaiting(queue);
}

/*!
 * \brief Returns the number of elements in the queue
 * \param queueType Queue type
 * \return Number of elements in the queue, 0 if the queue is not known.
 */
uint8_t RSTDIO_NofInQueue(RSTDIO_QueueType queueType) {
  switch(queueType) {
    case RSTDIO_QUEUE_RX_IN:  return RSTDIO_NofElements(RSTDIO_RxStdInQ);
    case RSTDIO_QUEUE_RX_OUT: return RSTDIO_NofElements(RSTDIO_RxStdOutQ);
    case RSTDIO_QUEUE_RX_ERR: return RSTDIO_NofElements(RSTDIO_RxStdErrQ);
#if 0
    case RSTDIO_QUEUE_TX_IN:  return RSTDIO_NofElements(RSTDIO_TxStdInQ);
    case RSTDIO_QUEUE_TX_OUT: return RSTDIO_NofElements(RSTDIO_TxStdOutQ);
    case RSTDIO_QUEUE_TX_ERR: return RSTDIO_NofElements(RSTDIO_TxStdErrQ);
#endif
    default: break;
  }
  return 0;
}

/*!
 * \brief Receives an a message character from the queue, and returns immediately if the queue is empty.
 * \return Message character, or '\0' if there was no message.
 */
static unsigned char RSTDIO_ReceiveChar(xQueueHandle queue) {
  unsigned char ch;
  portBASE_TYPE res;

  res = xQueueReceive(queue, &ch, 0);
  if (res==errQUEUE_EMPTY) {
    return '\0';
  } else {
    return ch;
  }
}

/*!
 * \brief Returns a character from a queue
 * \param queueType Queue type
 * \return Character, or zero byte if queue is empty or unknown
 */
uint8_t RSTDIO_ReceiveQueueChar(RSTDIO_QueueType queueType) {
  switch(queueType) {
    case RSTDIO_QUEUE_RX_IN:  return RSTDIO_ReceiveChar(RSTDIO_RxStdInQ);
    case RSTDIO_QUEUE_RX_OUT: return RSTDIO_ReceiveChar(RSTDIO_RxStdOutQ);
    case RSTDIO_QUEUE_RX_ERR: return RSTDIO_ReceiveChar(RSTDIO_RxStdErrQ);
#if 0
    case RSTDIO_QUEUE_TX_IN:  return RSTDIO_ReceiveChar(RSTDIO_TxStdInQ);
    case RSTDIO_QUEUE_TX_OUT: return RSTDIO_ReceiveChar(RSTDIO_TxStdOutQ);
    case RSTDIO_QUEUE_TX_ERR: return RSTDIO_ReceiveChar(RSTDIO_TxStdErrQ);
#endif
    default: break;
  }
  return '\0';
}

#if 0
uint8_t RSTDIO_HandleStdioRxMessage(RAPP_MSG_Type type, uint8_t size, uint8_t *data, RNWK_ShortAddrType srcAddr, bool *handled, RPHY_PacketDesc *packet) {
  (void)srcAddr;
  (void)packet;
  switch(type) {
    case RAPP_MSG_TYPE_STDIN: /* <type><size><data> */
      *handled = TRUE;
      return RSTDIO_AddToQueue(RSTDIO_QUEUE_RX_IN, data, size);
    case RAPP_MSG_TYPE_STDOUT: /* <type><size><data> */
      *handled = TRUE;
      return RSTDIO_AddToQueue(RSTDIO_QUEUE_RX_OUT, data, size);
    case RAPP_MSG_TYPE_STDERR: /* <type><size><data> */
      *handled = TRUE;
      return RSTDIO_AddToQueue(RSTDIO_QUEUE_RX_ERR, data, size);
    default:
      break;
  } /* switch */
  return ERR_OK;
}
#endif

#if 0
/*!
 * \brief Flushes the given buffer/queue. The buffer will be sent over the radio.
 * \return Error code, ERR_OK for no error.
 */
static uint8_t FlushAndTxQueue(RSTDIO_QueueType queueType, RAPP_MSG_Type msgType) {
  unsigned char buf[RAPP_BUFFER_SIZE];
  uint8_t i;
  uint8_t res = ERR_OK;
  uint8_t *p, ch;
  
  p = RAPP_BUF_PAYLOAD_START(buf);
  i = 0;
  while (i<RAPP_PAYLOAD_SIZE) {
    ch = RSTDIO_ReceiveQueueChar(queueType);
    if (ch=='\0') { /* queue empty */
      break;
    }
    *p++ = ch;
    i++;
  }
  res = RAPP_PutPayload(buf, sizeof(buf), i, msgType, RSTDIO_dstAddr, RPHY_PACKET_FLAGS_NONE);
  if (res!=ERR_OK) {
    CLS1_ConstStdIOType *io = CLS1_GetStdio();

    io->stdErr('*');
    io->stdErr('F');
    io->stdErr('A');
    io->stdErr('I');
    io->stdErr('L');
    io->stdErr('*');
    io->stdErr('\n');
    return res;
  }
  return res;
}
#endif

#if 0
/*!
 * \brief Radio StdIn: sends a character to the wireless communication channel.
 * \param ch Character to send
 */
static void RSTDIO_TxStdInSendChar(uint8_t ch) {
  (void)RSTDIO_AddToQueue(RSTDIO_QUEUE_TX_IN, &ch, 1);
  if (ch=='\n' || RSTDIO_NofInQueue(RSTDIO_QUEUE_TX_IN)>RSTDIO_PAYLOAD_SIZE) { /* can send string over radio */
    (void)FlushAndTxQueue(RSTDIO_QUEUE_TX_IN, RAPP_MSG_TYPE_STDIN);
  }
}
#endif

#if 0
/*!
 * \brief Radio StdOut: sends a character to the wireless communication channel.
 * \param ch Character to send
 */
static void RSTDIO_TxStdOut(uint8_t ch) {
  (void)RSTDIO_AddToQueue(RSTDIO_QUEUE_TX_OUT, &ch, 1);
  if (ch=='\n' || RSTDIO_NofInQueue(RSTDIO_QUEUE_TX_OUT)>RSTDIO_PAYLOAD_SIZE) { /* can send string over radio */
    (void)FlushAndTxQueue(RSTDIO_QUEUE_TX_OUT, RAPP_MSG_TYPE_STDOUT);
  }
}
#endif

#if 0
/*!
 * \brief Radio StdErr: sends a character to the wireless communication channel.
 * \param ch Character to send
 */
static void RSTDIO_TxStdErr(uint8_t ch) {
  (void)RSTDIO_AddToQueue(RSTDIO_QUEUE_TX_ERR, &ch, sizeof(ch));
  if (ch=='\n' || RSTDIO_NofInQueue(RSTDIO_QUEUE_TX_ERR)>RSTDIO_PAYLOAD_SIZE) { /* can send string over radio */
    (void)FlushAndTxQueue(RSTDIO_QUEUE_TX_ERR, RAPP_MSG_TYPE_STDERR);
  }
}
#endif

#if 0
uint8_t RSTDIO_SendToTxStdio(RSTDIO_QueueType queueType, uint8_t *buf, size_t bufSize) {
  if (queueType==RSTDIO_QUEUE_TX_IN) {
    while(bufSize!=0) {
      RSTDIO_TxStdInSendChar(*buf);
      buf++; bufSize--;
    }
  } else if (queueType==RSTDIO_QUEUE_TX_OUT) {
    while(bufSize!=0) {
      RSTDIO_TxStdOut(*buf);
      buf++; bufSize--;
    }
  } else if (queueType==RSTDIO_QUEUE_TX_ERR) {
    while(bufSize!=0) {
      RSTDIO_TxStdErr(*buf);
      buf++; bufSize--;
    }
  } else {
    return ERR_FAILED; /* unknown queue? */
  }
  return ERR_OK;
}
#endif

/*!
 * \brief Radio StdIn: Reads a character from the StdIn queue and echoes it on the standard shell stdOut
 */
static void RSTDIO_RxStdInReadChar(byte *ch) {
  *ch = RSTDIO_ReceiveQueueChar(RSTDIO_QUEUE_RX_IN); /* will return '\0' if queue is empty */
}

/*!
 * \brief Checks if we have a character in the stdin queue without removing it.
 * \return TRUE if character is present.
 */
static bool RSTDIO_RxStdInKeyPressed(void) {
  return (bool)(RSTDIO_NofElements(RSTDIO_RxStdInQ)!=0); 
}

CLS1_ConstStdIOType RSTDIO_stdio = {
  (CLS1_StdIO_In_FctType)RSTDIO_RxStdInReadChar, /* stdin */
#if 1
  NULL,
  NULL,
#else
  (CLS1_StdIO_OutErr_FctType)RSTDIO_TxStdOut, /* stdout */
  (CLS1_StdIO_OutErr_FctType)RSTDIO_TxStdErr, /* stderr */
#endif
  RSTDIO_RxStdInKeyPressed /* if input is not empty */
};

CLS1_ConstStdIOTypePtr RSTDIO_GetStdio(void) {
  return &RSTDIO_stdio;
}

/*!
 * \brief Called from the application task. This function checks the Radio RX queue and checks if it contains stdio messages.
 * If so, it dispatches it to the corresponding shell queues.
 */
void RSTDIO_Print(CLS1_ConstStdIOTypePtr io) {
  unsigned char ch;

  for(;;) { /* breaks */
    ch = RSTDIO_ReceiveChar(RSTDIO_RxStdOutQ);
    if(ch=='\0') {
      break; /* get out of for loop */
    }
    io->stdOut(ch); /* output character */
  }
  for(;;) { /* breaks */
    ch = RSTDIO_ReceiveChar(RSTDIO_RxStdErrQ);
    if(ch=='\0') {
      break; /* get out of for loop */
    }
    io->stdErr(ch); /* output character */
  }
}

static void PrintHelp(const CLS1_StdIOType *io) {
  CLS1_SendHelpStr((unsigned char*)"rstdio", (unsigned char*)"Group of rstdio commands\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows help or status\r\n", io->stdOut);
//  CLS1_SendHelpStr((unsigned char*)"  daddr 0x<addr>", (unsigned char*)"Set destination node address\r\n", io->stdOut);
//  CLS1_SendHelpStr((unsigned char*)"  send (in/out/err)", (unsigned char*)"Send a string to remote stdio\r\n", io->stdOut);
}

static uint8_t PrintStatus(const CLS1_StdIOType *io) {
  uint8_t buf[32];

  CLS1_SendStatusStr((unsigned char*)"rstdio", (unsigned char*)"\r\n", io->stdOut);
#if 0
  UTIL1_strcpy(buf, sizeof(buf), (unsigned char*)"0x");
#if RNWK_SHORT_ADDR_SIZE==1
  UTIL1_strcatNum8Hex(buf, sizeof(buf), RSTDIO_dstAddr);
#else
  UTIL1_strcatNum16Hex(buf, sizeof(buf), RSTDIO_dstAddr);
#endif
  UTIL1_strcat(buf, sizeof(buf), (unsigned char*)"\r\n");
  CLS1_SendStatusStr((unsigned char*)"  dest addr", buf, io->stdOut);
#endif
  return ERR_OK;
}

uint8_t RSTDIO_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
  uint8_t res = ERR_OK;
  const uint8_t *p;
  uint16_t val16;

  if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, (char*)"rstdio help")==0) {
    PrintHelp(io);
    *handled = TRUE;
  } else if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd, (char*)"rstdio status")==0) {
    *handled = TRUE;
    return PrintStatus(io);
#if 0
  } else if (UTIL1_strncmp((char*)cmd, (char*)"rstdio send", sizeof("rstdio send")-1)==0) {
    unsigned char buf[CLS1_DEFAULT_SHELL_BUFFER_SIZE];
    RSTDIO_QueueType queue;

    if (UTIL1_strncmp((char*)cmd, (char*)"rstdio send in", sizeof("rstdio send in")-1)==0) {
      queue = RSTDIO_QUEUE_TX_IN;
      cmd += sizeof("rstdio send in");
    } else if (UTIL1_strncmp((char*)cmd, (char*)"rstdio send out", sizeof("rstdio send out")-1)==0) {
      queue = RSTDIO_QUEUE_TX_OUT;
      cmd += sizeof("rstdio send out");
    } else if (UTIL1_strncmp((char*)cmd, (char*)"rstdio send err", sizeof("rstdio send err")-1)==0) {
      queue = RSTDIO_QUEUE_TX_ERR;
      cmd += sizeof("rstdio send err");
    } else {
      return ERR_OK; /* not handled */
    }
    UTIL1_strcpy(buf, sizeof(buf), cmd);
    UTIL1_chcat(buf, sizeof(buf), '\n');
    buf[sizeof(buf)-2] = '\n'; /* have a '\n' in any case */
    if (RSTDIO_SendToTxStdio(queue, buf, UTIL1_strlen((char*)buf))!=ERR_OK) {
      CLS1_SendStr((unsigned char*)"failed!\r\n", io->stdErr);
    }
    *handled = TRUE;
  } else if (UTIL1_strncmp((char*)cmd, (char*)"rstdio daddr", sizeof("rstdio daddr")-1)==0) {
    p = cmd + sizeof("rstdio daddr")-1;
    *handled = TRUE;
    if (UTIL1_ScanHex16uNumber(&p, &val16)==ERR_OK) {
      RSTDIO_dstAddr = val16;
    } else {
      CLS1_SendStr((unsigned char*)"ERR: wrong address\r\n", io->stdErr);
      return ERR_FAILED;
    }
#endif
  }
  return res;
}

/*! \brief Deinitializes the queue module */
void RSTDIO_Deinit(void) {
  FRTOS1_vQueueUnregisterQueue(RSTDIO_RxStdInQ);
  FRTOS1_vQueueDelete(RSTDIO_RxStdInQ);
  RSTDIO_RxStdInQ = NULL;

  FRTOS1_vQueueUnregisterQueue(RSTDIO_RxStdOutQ);
  FRTOS1_vQueueDelete(RSTDIO_RxStdOutQ);
  RSTDIO_RxStdOutQ = NULL;

  FRTOS1_vQueueUnregisterQueue(RSTDIO_RxStdErrQ);
  FRTOS1_vQueueDelete(RSTDIO_RxStdErrQ);
  RSTDIO_RxStdErrQ = NULL;
#if 0
  FRTOS1_vQueueUnregisterQueue(RSTDIO_TxStdInQ);
  FRTOS1_vQueueDelete(RSTDIO_TxStdInQ);
  RSTDIO_TxStdInQ = NULL;

  FRTOS1_vQueueUnregisterQueue(RSTDIO_TxStdOutQ);
  FRTOS1_vQueueDelete(RSTDIO_TxStdOutQ);
  RSTDIO_TxStdOutQ = NULL;

  FRTOS1_vQueueUnregisterQueue(RSTDIO_TxStdErrQ);
  FRTOS1_vQueueDelete(RSTDIO_TxStdErrQ);
  RSTDIO_TxStdErrQ = NULL;
#endif
}

void RSTDIO_Init(void) {
  //RSTDIO_SetDestinationAddress(RSTDIO_CONFIG_SETTING_RSTDIO_DEFAULT_DESTINATION_ADDRESS);
  RSTDIO_RxStdInQ = FRTOS1_xQueueCreate(RSTDIO_QUEUE_LENGTH, RSTDIO_QUEUE_ITEM_SIZE);
  if (RSTDIO_RxStdInQ==NULL) {
    for(;;){} /* out of memory? */
  }
  FRTOS1_vQueueAddToRegistry(RSTDIO_RxStdInQ, "RxStdInQ");
#if configUSE_TRACE_HOOKS
  vTraceSetQueueName(RSTDIO_RxStdInQ, "RxStdInQ");
#endif

  RSTDIO_RxStdOutQ = FRTOS1_xQueueCreate(RSTDIO_QUEUE_LENGTH, RSTDIO_QUEUE_ITEM_SIZE);
  if (RSTDIO_RxStdOutQ==NULL) {
    for(;;){} /* out of memory? */
  }
  FRTOS1_vQueueAddToRegistry(RSTDIO_RxStdOutQ, "RxStdOutQ");
#if configUSE_TRACE_HOOKS
  vTraceSetQueueName(RSTDIO_RxStdOutQ, "RxStdOutQ");
#endif

  RSTDIO_RxStdErrQ = FRTOS1_xQueueCreate(RSTDIO_QUEUE_LENGTH, RSTDIO_QUEUE_ITEM_SIZE);
  if (RSTDIO_RxStdErrQ==NULL) {
    for(;;){} /* out of memory? */
  }
  FRTOS1_vQueueAddToRegistry(RSTDIO_RxStdErrQ, "RxStdErrQ");
#if configUSE_TRACE_HOOKS
  vTraceSetQueueName(RSTDIO_RxStdErrQ, "RxStdErrQ");
#endif
#if 0
  RSTDIO_TxStdInQ = FRTOS1_xQueueCreate(RSTDIO_QUEUE_LENGTH, RSTDIO_QUEUE_ITEM_SIZE);
  if (RSTDIO_TxStdInQ==NULL) {
    for(;;){} /* out of memory? */
  }
  FRTOS1_vQueueAddToRegistry(RSTDIO_TxStdInQ, "TxStdInQ");
#if configUSE_TRACE_HOOKS
  vTraceSetQueueName(RSTDIO_TxStdInQ, "TxStdInQ");
#endif

  RSTDIO_TxStdOutQ = FRTOS1_xQueueCreate(RSTDIO_QUEUE_LENGTH, RSTDIO_QUEUE_ITEM_SIZE);
  if (RSTDIO_TxStdOutQ==NULL) {
    for(;;){} /* out of memory? */
  }
  FRTOS1_vQueueAddToRegistry(RSTDIO_TxStdOutQ , "TxStdOutQ");
#if configUSE_TRACE_HOOKS
  vTraceSetQueueName(RSTDIO_TxStdOutQ, "TxStdOutQ");
#endif

  RSTDIO_TxStdErrQ = FRTOS1_xQueueCreate(RSTDIO_QUEUE_LENGTH, RSTDIO_QUEUE_ITEM_SIZE);
  if (RSTDIO_TxStdErrQ==NULL) {
    for(;;){} /* out of memory? */
  }
  FRTOS1_vQueueAddToRegistry(RSTDIO_TxStdErrQ , "TxStdErrQ");
#if configUSE_TRACE_HOOKS
  vTraceSetQueueName(RSTDIO_TxStdErrQ, "TxStdErrQ");
#endif
#endif
}
