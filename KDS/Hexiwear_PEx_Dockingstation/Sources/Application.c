/*
 * Application.c
 *
 *  Created on: 29.11.2016
 *      Author: Erich Styger Local
 */

#include "Platform.h"
#include "Application.h"
#include "FRTOS1.h"
#include "Shell.h"
#include "RNet_App.h"
#include "RGBR.h"
#include "RGBB.h"
#include "RGBG.h"
#if PL_CONFIG_HAS_HOME_LEDS
  #include "HLED1.h"
  #include "HLED2.h"
  #include "HLED3.h"
  #include "HLED4.h"
  #include "HLED5.h"
  #include "HLED6.h"
#endif
#if PL_CONFIG_HAS_KW40_COMM
  #include "KW40Comm.h"
#endif
#include "Vibro.h"
#include "GDisp1.h"
#include "LCD1.h"
#include "OLED_Pwr.h"

// command byte number
#define FIRST_BYTE (1)
#define OTHER_BYTE (0)

#define OLED_CMD_SET_COLUMN ( 0x15 )
#define OLED_CMD_SET_ROW    ( 0x75 )


#define OLED_CMD_STARTLINE (0xA1)

#define OLED_CMD_WRITERAM       (0x5C)
#define OLED_CMD_READRAM        (0x5D)
#define OLED_CMD_DISPLAYOFFSET  (0xA2)
#define OLED_CMD_DISPLAYALLOFF  (0xA4)
#define OLED_CMD_DISPLAYALLON   (0xA5)
#define OLED_CMD_NORMALDISPLAY  (0xA6)
#define OLED_CMD_INVERTDISPLAY  (0xA7)
#define OLED_CMD_FUNCTIONSELECT (0xAB)
#define OLED_CMD_DISPLAYOFF     (0xAE)
#define OLED_CMD_DISPLAYON      (0xAF)
#define OLED_CMD_PRECHARGE      (0xB1)
#define OLED_CMD_DISPLAYENHANCE (0xB2)
#define OLED_CMD_SETVSL         (0xB4)
#define OLED_CMD_SETGPIO        (0xB5)
#define OLED_CMD_PRECHARGE2     (0xB6)
#define OLED_CMD_SETGRAY        (0xB8)
#define OLED_CMD_USELUT         (0xB9)
#define OLED_CMD_PRECHARGELEVEL (0xBB)
#define OLED_CMD_VCOMH          (0xBE)
#define OLED_CMD_CONTRASTABC    (0xC1)
#define OLED_CMD_CONTRASTMASTER (0xC7)
#define OLED_CMD_MUXRATIO       (0xCA)
#define OLED_CMD_COMMANDLOCK    (0xFD)
#define OLED_CMD_HORIZSCROLL    (0x96)
#define OLED_CMD_STOPSCROLL     (0x9E)
#define OLED_CMD_STARTSCROLL    (0x9F)

/**
 * set lock command
 * the locked OLED driver MCU interface prohibits all commands
 * and memory access, except the 0xFD command
 */
#define OLED_CMD_SET_CMD_LOCK ( 0xFD /* << 8 */ )
// unlock OLED driver MCU interface for entering command (default upon reset)
#define OLED_UNLOCK           (0x12)
// lock OLED driver MCU interface for entering command
#define OLED_LOCK             (0x16)
// commands 0xA2, 0xB1, 0xB3, 0xBB, 0xBE, 0xC1 are inaccessible in both lock and unlock state (default upon reset)
#define OLED_ACC_TO_CMD_NO    (0xB0)
// commands 0xA2, 0xB1, 0xB3, 0xBB, 0xBE, 0xC1 are accessible in unlock state
#define OLED_ACC_TO_CMD_YES   (0xB1)

#define OLED_CMD_SET_OSC_FREQ_AND_CLOCKDIV (0xB3)

// clock divider
#define OLED_CLOCKDIV_1    (0x00)
#define OLED_CLOCKDIV_2    (0x01)
#define OLED_CLOCKDIV_4    (0x02)
#define OLED_CLOCKDIV_8    (0x03)
#define OLED_CLOCKDIV_16   (0x04)
#define OLED_CLOCKDIV_32   (0x05)
#define OLED_CLOCKDIV_64   (0x06)
#define OLED_CLOCKDIV_128  (0x07)
#define OLED_CLOCKDIV_256  (0x08)
#define OLED_CLOCKDIV_512  (0x09)
#define OLED_CLOCKDIV_1024 (0x0A)

/**
 * set MUX ratio
 */
#define OLED_CMD_SET_MUX_RATIO (0xCA)

/**
 * set re-map / color depth
 */
#define OLED_CMD_SET_REMAP ( 0xA0 )

// set horizontal or vertical increment
#define OLED_ADDR_INC_HOR (0x00)
#define OLED_ADDR_INC_VER (0x01)

/**
 * remap settings
 */

#define REMAP_HORIZONTAL_INCREMENT ( 0 )
#define REMAP_VERTICAL_INCREMENT   ( 1 << 0 )

#define REMAP_COLUMNS_LEFT_TO_RIGHT ( 0 )
#define REMAP_COLUMNS_RIGHT_TO_LEFT ( 1 << 1 )

#define REMAP_ORDER_ABC ( 0 )
#define REMAP_ORDER_CBA ( 1 << 2 )

#define REMAP_SCAN_UP_TO_DOWN ( 0 )
#define REMAP_SCAN_DOWN_TO_UP ( 1 << 4 )

#define REMAP_COM_SPLIT_ODD_EVEN_DIS ( 0 )
#define REMAP_COM_SPLIT_ODD_EVEN_EN  ( 1 << 5 )

#define REMAP_COLOR_RGB565 ( 1 << 6 )


#define OLED_REMAP_SETTINGS ( REMAP_ORDER_ABC | REMAP_COM_SPLIT_ODD_EVEN_EN | REMAP_COLOR_RGB565 | REMAP_COLUMNS_LEFT_TO_RIGHT | REMAP_SCAN_UP_TO_DOWN | REMAP_HORIZONTAL_INCREMENT )

typedef struct {
  uint8_t cmd, type;
} init_cmd_t;

#define CMD_BYTE   (1)
#define DATA_BYTE  (0)


static const init_cmd_t seq[] = {
    OLED_CMD_SET_CMD_LOCK,  CMD_BYTE,
    OLED_UNLOCK,            DATA_BYTE,
    OLED_CMD_SET_CMD_LOCK,  CMD_BYTE,
    OLED_ACC_TO_CMD_YES,    DATA_BYTE,
    OLED_CMD_DISPLAYOFF,    CMD_BYTE,
    OLED_CMD_SET_OSC_FREQ_AND_CLOCKDIV, CMD_BYTE,
    0xF1,                   DATA_BYTE,
    OLED_CMD_SET_MUX_RATIO, CMD_BYTE,
    0x5F,                   DATA_BYTE,
    OLED_CMD_SET_REMAP,     CMD_BYTE,
    OLED_REMAP_SETTINGS,    DATA_BYTE,
    OLED_CMD_SET_COLUMN,    CMD_BYTE,
    0x00,                   DATA_BYTE,
    0x5F,                   DATA_BYTE,
    OLED_CMD_SET_ROW,       CMD_BYTE,
    0x00,                   DATA_BYTE,
    0x5F,                   DATA_BYTE,
    OLED_CMD_STARTLINE,     CMD_BYTE,
    0x80,                   DATA_BYTE,
    OLED_CMD_DISPLAYOFFSET, CMD_BYTE,
    0x60,                   DATA_BYTE,
    OLED_CMD_PRECHARGE,     CMD_BYTE,
    0x32,                   CMD_BYTE,
    OLED_CMD_VCOMH,         CMD_BYTE,
    0x05,                   CMD_BYTE,
    OLED_CMD_NORMALDISPLAY, CMD_BYTE,
    OLED_CMD_CONTRASTABC,   CMD_BYTE,
    0x8A,                   DATA_BYTE,
    0x51,                   DATA_BYTE,
    0x8A,                   DATA_BYTE,
    OLED_CMD_CONTRASTMASTER, CMD_BYTE,
    0xCF,                   DATA_BYTE,
    OLED_CMD_SETVSL,        CMD_BYTE,
    0xA0,                   DATA_BYTE,
    0xB5,                   DATA_BYTE,
    0x55,                   DATA_BYTE,
    OLED_CMD_PRECHARGE2,    CMD_BYTE,
    0x01,                   DATA_BYTE,
    OLED_CMD_DISPLAYON,     CMD_BYTE
    };

#define RESET_LOW()   RESpin1_ClrVal()  /* RESET signal low (reset is low active) */
#define RESET_HIGH()  RESpin1_SetVal()  /* RESET signal high */

#define CMD_MODE()   D_Cpin1_ClrVal()   /* switch to command mode, D/C low */
#define DATA_MODE()  D_Cpin1_SetVal()   /* switch to data mode, D/C high */
#define CS_HIGH()    SCEpin1_SetVal()   /* CE signal high */
#define CS_LOW()     SCEpin1_ClrVal()   /* CE signal low */

static void spi_write(uint8_t ch) {
  //uint8_t dummy;

  while(SM2_SendChar(ch)!=ERR_OK) {}    /* send LSB data byte */
  while(SM2_GetCharsInTxBuf()!=0) {};  /* wait until everything is sent */
  //while(SM2_RecvChar(&dummy)!=ERR_OK) {};
}

static void OLED_SendCmd(uint8_t cmd, bool isCmd) {
  if (isCmd )  {
    CMD_MODE();
  } else {
    DATA_MODE();
  }
  CS_LOW();
  spi_write(cmd);
  CS_HIGH();
}


static uint8_t OLED_Init(void) {
  int i;

  //  LCD1_Init();

  OLED_Pwr_ClrVal(); /* OLED power off */
  vTaskDelay(pdMS_TO_TICKS(1));
  RESET_LOW();
  vTaskDelay(pdMS_TO_TICKS(1));
  RESET_HIGH();
  vTaskDelay(pdMS_TO_TICKS(1));
  OLED_Pwr_SetVal(); /* OLED power on */

  for (int i=0;i<39;i++)
  {
    OLED_SendCmd(seq[i].cmd, seq[i].type);
  }

#if 0
  if ( ERR_OK != OLED_SendCmd( OLED_CMD_SET_CMD_LOCK, FIRST_BYTE ) )
  {
      return ERR_FAILED;
  }
  if ( ERR_OK != OLED_SendCmd( OLED_UNLOCK, OTHER_BYTE ) )
  {
      return ERR_FAILED;
  }

  if ( ERR_OK != OLED_SendCmd( OLED_CMD_SET_CMD_LOCK, FIRST_BYTE ) )
  {
      return ERR_FAILED;
  }
  if ( ERR_OK != OLED_SendCmd( OLED_ACC_TO_CMD_YES, OTHER_BYTE ) )
  {
      return ERR_FAILED;
  }

  if ( ERR_OK != OLED_SendCmd( OLED_CMD_DISPLAYOFF, FIRST_BYTE ) )
  {
      return ERR_FAILED;
  }

  if ( ERR_OK != OLED_SendCmd( OLED_CMD_SET_OSC_FREQ_AND_CLOCKDIV, FIRST_BYTE ) )
  {
      return ERR_FAILED;
  }
  if ( ERR_OK != OLED_SendCmd( 0xF1, OTHER_BYTE ) )
  {
      return ERR_FAILED;
  }

  if ( ERR_OK != OLED_SendCmd( OLED_CMD_SET_MUX_RATIO, FIRST_BYTE ) )
  {
      return ERR_FAILED;
  }
  if ( ERR_OK != OLED_SendCmd( 0x5F, OTHER_BYTE ) )
  {
      return ERR_FAILED;
  }

  if ( ERR_OK != OLED_SendCmd( OLED_CMD_SET_REMAP, FIRST_BYTE ) )
  {
      return ERR_FAILED;
  }
  if ( ERR_OK != OLED_SendCmd( OLED_REMAP_SETTINGS, OTHER_BYTE ) )
  {
      return ERR_FAILED;
  }

  if ( ERR_OK != OLED_SendCmd( OLED_CMD_SET_COLUMN, FIRST_BYTE ) )
  {
      return ERR_FAILED;
  }
  if ( ERR_OK != OLED_SendCmd( 0x00, OTHER_BYTE ) )
  {
      return ERR_FAILED;
  }
  if ( ERR_OK != OLED_SendCmd( 0x5F, OTHER_BYTE ) )
  {
      return ERR_FAILED;
  }

  if ( ERR_OK != OLED_SendCmd( OLED_CMD_SET_ROW, FIRST_BYTE ) )
  {
      return ERR_FAILED;
  }
  if ( ERR_OK != OLED_SendCmd( 0x00, OTHER_BYTE ) )
  {
      return ERR_FAILED;
  }
  if ( ERR_OK != OLED_SendCmd( 0x5F, OTHER_BYTE ) )
  {
      return ERR_FAILED;
  }

  if ( ERR_OK != OLED_SendCmd( OLED_CMD_STARTLINE, FIRST_BYTE ) )
  {
      return ERR_FAILED;
  }
  if ( ERR_OK != OLED_SendCmd( 0x80, OTHER_BYTE ) )
  {
      return ERR_FAILED;
  }

  if ( ERR_OK != OLED_SendCmd( OLED_CMD_DISPLAYOFFSET, FIRST_BYTE ) )
  {
      return ERR_FAILED;
  }
  if ( ERR_OK != OLED_SendCmd( 0x60, OTHER_BYTE ) )
  {
      return ERR_FAILED;
  }

  if ( ERR_OK != OLED_SendCmd( OLED_CMD_PRECHARGE, FIRST_BYTE ) )
  {
      return ERR_FAILED;
  }
  if ( ERR_OK != OLED_SendCmd( 0x32, FIRST_BYTE ) )
  {
      return ERR_FAILED;
  }

  if ( ERR_OK != OLED_SendCmd( OLED_CMD_VCOMH, FIRST_BYTE ) )
  {
      return ERR_FAILED;
  }
  if ( ERR_OK != OLED_SendCmd( 0x05, FIRST_BYTE ) )
  {
      return ERR_FAILED;
  }

  if ( ERR_OK != OLED_SendCmd( OLED_CMD_NORMALDISPLAY, FIRST_BYTE ) )
  {
      return ERR_FAILED;
  }

  if ( ERR_OK != OLED_SendCmd( OLED_CMD_CONTRASTABC, FIRST_BYTE ) )
  {
      return ERR_FAILED;
  }
  if ( ERR_OK != OLED_SendCmd( 0x8A, OTHER_BYTE ) )
  {
      return ERR_FAILED;
  }
  if ( ERR_OK != OLED_SendCmd( 0x51, OTHER_BYTE ) )
  {
      return ERR_FAILED;
  }
  if ( ERR_OK != OLED_SendCmd( 0x8A, OTHER_BYTE ) )
  {
      return ERR_FAILED;
  }

  if ( ERR_OK != OLED_SendCmd( OLED_CMD_CONTRASTMASTER, FIRST_BYTE ) )
  {
      return ERR_FAILED;
  }
  if ( ERR_OK != OLED_SendCmd( 0xCF, OTHER_BYTE ) )
  {
      return ERR_FAILED;
  }

  if ( ERR_OK != OLED_SendCmd( OLED_CMD_SETVSL, FIRST_BYTE ) )
  {
      return ERR_FAILED;
  }
  if ( ERR_OK != OLED_SendCmd( 0xA0, OTHER_BYTE ) )
  {
      return ERR_FAILED;
  }
  if ( ERR_OK != OLED_SendCmd( 0xB5, OTHER_BYTE ) )
  {
      return ERR_FAILED;
  }
  if ( ERR_OK != OLED_SendCmd( 0x55, OTHER_BYTE ) )
  {
      return ERR_FAILED;
  }

  if ( ERR_OK != OLED_SendCmd( OLED_CMD_PRECHARGE2, FIRST_BYTE ) )
  {
      return ERR_FAILED;
  }
  if ( ERR_OK != OLED_SendCmd( 0x01, OTHER_BYTE ) )
  {
      return ERR_FAILED;
  }

  if ( ERR_OK != OLED_SendCmd( OLED_CMD_DISPLAYON, FIRST_BYTE ) )
  {
      return ERR_FAILED;
  }
#endif
}

static void AppTask(void *param) {
  RGBR_On();
  FRTOS1_vTaskDelay(pdMS_TO_TICKS(100));
  RGBR_Off();
  RGBG_On();
  FRTOS1_vTaskDelay(pdMS_TO_TICKS(100));
  RGBG_Off();
  RGBB_On();
  FRTOS1_vTaskDelay(pdMS_TO_TICKS(100));
  RGBB_Off();
#if PL_CONFIG_HAS_HOME_LEDS
  HLED1_On();
  FRTOS1_vTaskDelay(pdMS_TO_TICKS(100));
  HLED1_Off();
  HLED2_On();
  FRTOS1_vTaskDelay(pdMS_TO_TICKS(100));
  HLED2_Off();
  HLED3_On();
  FRTOS1_vTaskDelay(pdMS_TO_TICKS(100));
  HLED3_Off();
  HLED4_On();
  FRTOS1_vTaskDelay(pdMS_TO_TICKS(100));
  HLED4_Off();
  HLED5_On();
  FRTOS1_vTaskDelay(pdMS_TO_TICKS(100));
  HLED5_Off();
  HLED6_On();
  FRTOS1_vTaskDelay(pdMS_TO_TICKS(100));
  HLED6_Off();
#endif
  //Vibro_SetVal();
  //Vibro_ClrVal();

  OLED_Init();
  GDisp1_Clear();
  GDisp1_DrawBox(5, 5, 10, 20, 2, GDisp1_COLOR_BLUE);
  GDisp1_UpdateFull();
  for(;;) {
    RGBG_On();
    FRTOS1_vTaskDelay(pdMS_TO_TICKS(50));
    RGBG_Off();
    FRTOS1_vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void APP_Run(void) {
  SHELL_Init();
#if PL_CONFIG_HAS_RADIO
  RNETA_Init();
#endif
#if PL_CONFIG_HAS_KW40_COMM
  KW40Comm_Init();
  HostComm_Init();
#endif
  if (xTaskCreate(AppTask, (uint8_t *)"App", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL) != pdPASS) {
    for(;;){} /* error case only, stay here! */
  }
  vTaskStartScheduler();
}

