#include "DW1000.h"
#include "DW1000Ranging.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4
#define PIN_IRQ static_cast<gpio_num_t>(34)
#define PIN_RST static_cast<gpio_num_t>(27)
#define PIN_SS  static_cast<gpio_num_t>(4)

// TAG antenna delay defaults to 16384
// leftmost two bytes below will become the "short address"
char tag_addr[] = "7D:00:22:EA:82:60:3B:9C";

spi_bus_config_t buscfg = {
    .mosi_io_num = SPI_MOSI,
    .miso_io_num = SPI_MISO,
    .sclk_io_num = SPI_SCK

  };



void loop()
{
  DW1000Ranging.loop();
}

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
vTaskDelay(1);//delay(1000);

  //init the configuration
  spi_bus_initialize(HSPI_HOST, &buscfg, 1);//SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

// start as tag, do not assign random short address

  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
}