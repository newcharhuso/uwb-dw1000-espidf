#include "DW1000.h"
#include "DW1000Ranging.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "spi_config.h"
#include <string.h>


// TAG antenna delay defaults to 16384
// leftmost two bytes below will become the "short address"
char tag_addr[] = "7D:00:22:EA:82:60:3B:9C";

void newRange()
{
//   Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
//   Serial.print(",");
//   Serial.println(DW1000Ranging.getDistantDevice()->getRange());
uint16_t shortAddress = DW1000Ranging.getDistantDevice()->getShortAddress();
printf("Short Address: %04X\n", shortAddress);
printf(",");
printf("%f\n" , DW1000Ranging.getDistantDevice()->getRange());
}

void newDevice(DW1000Device *device)
{
//   Serial.print("Device added: ");
//   Serial.println(device->getShortAddress(), HEX);
uint16_t shortAdress = device->getShortAddress();
printf("%X\n", shortAdress);
}

void inactiveDevice(DW1000Device *device)
{
//   Serial.print("delete inactive device: ");
//   Serial.println(device->getShortAddress(), HEX);
uint16_t shortAdress = device->getShortAddress();
printf("delete inactive device: %X\n", shortAdress);
}

extern "C" {
  void app_main(void);
}

void app_main(void)
{
  //Serial.begin(115200);
  TaskHandle_t taskHandle_dwm1000 = NULL;
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
// start as tag, do not assign random short address
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
  
  while(1) {
    xTaskCreatePinnedToCore((TaskFunction_t)xTask_DWM1000, "dwm1000_loop", 2048, &DW1000Ranging, 5, &taskHandle_dwm1000, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}