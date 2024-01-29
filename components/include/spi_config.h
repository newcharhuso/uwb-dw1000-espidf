// spi_config.h
#ifndef SPI_CONFIG_H
#define SPI_CONFIG_H

#include "driver/spi_master.h"
#include "driver/gpio.h"

// SPI pin definitions
#define SPI_SCK  18
#define SPI_MISO 19
#define SPI_MOSI 23

// DW1000 pin definitions
#define DW_CS   static_cast<gpio_num_t>(4)
#define PIN_RST static_cast<gpio_num_t>(27)
#define PIN_IRQ static_cast<gpio_num_t>(34)


#define SPI_CHANNEL    HSPI_HOST
#define SPI_MODE       2 // Default SPI mode 2
#define SPI_CLOCK      500000 // Default SPI clock 500KHz

// Function declarations
void init_spi();
void init_gpio();

spi_device_handle_t spi_handle;
extern spi_device_handle_t spi_handle_fast;
extern spi_device_handle_t spi_handle_slow;

void init_spi();

#endif // SPI_CONFIG_H
