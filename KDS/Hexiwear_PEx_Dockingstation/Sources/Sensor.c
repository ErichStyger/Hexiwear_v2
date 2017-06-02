/*
 * Sensor.c
 *
 *  Created on: 02.06.2017
 *      Author: Erich Styger
 */

#include "Sensor.h"
#include "FRTOS1.h"
#include "Bluetooth.h"
#include "FX1.h"
#include "HostComm.h"

static void SensorTask(void *param) {
  for(;;) {
    if (BLUETOOTH_GetCurrentAppMode()==GUI_CURRENT_APP_SENSOR_TAG) {
      int16_t x, y, z;

      /* accelerometer */
      x = FX1_GetXmg();
      vTaskDelay(pdMS_TO_TICKS(10));
      y = FX1_GetYmg();
      vTaskDelay(pdMS_TO_TICKS(10));
      z = FX1_GetZmg();
      vTaskDelay(pdMS_TO_TICKS(10));
      /* transform milli-g values into centi-g (1234 mg are 123 which then are 1.23g */
      x = x/10;
      y = y/10;
      z = z/10;
      HostComm_SendAccel(x, y, z);
      /* magnetometer */
      if (FX1_GetMagX(&x)==ERR_OK) {
        x = x/10;
      } else {
        x = 0;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      if (FX1_GetMagY(&y)==ERR_OK) {
        y = y/10;
      } else {
        y = 0;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      if (FX1_GetMagZ(&z)==ERR_OK) {
        z = z/10;
      } else {
        z = 0;
      }
      HostComm_SendMag(x, y, z);

      vTaskDelay(pdMS_TO_TICKS(500));
    } else {
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  }
}

void SENSOR_Init(void) {
  if (xTaskCreate(SensorTask, (uint8_t *)"Sensor", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL) != pdPASS) {
    for(;;){} /* error case only, stay here! */
  }
}
