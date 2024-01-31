// spi_config.h
#ifndef SPI_CONFIG_H
#define SPI_CONFIG_H

#include "driver/spi_master.h"
#include "driver/gpio.h"

// SPI pin definitions
#define SPI_SCK  static_cast<gpio_num_t>(18)
#define SPI_MISO static_cast<gpio_num_t>(19)
#define SPI_MOSI static_cast<gpio_num_t>(23)

// DW1000 pin definitions
#define DW_CS   static_cast<gpio_num_t>(4)
#define PIN_RST static_cast<gpio_num_t>(27)
#define PIN_IRQ static_cast<gpio_num_t>(34)
#define PIN_SS  static_cast<gpio_num_t>(4)


#define SPI_CHANNEL    HSPI_HOST
#define SPI_MODE       2 // Default SPI mode 2
#define SPI_CLOCK      16000000L // Default SPI clock 500KHz

// Function declarations
void init_spi();
void init_gpio();

extern spi_device_handle_t spi_handle_fast;
extern spi_device_handle_t spi_handle_slow;

void init_spi();

#endif // SPI_CONFIG_H
