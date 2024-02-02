// spi_config.cpp
#include "spi_config.h"
#include "esp_system.h"
#include "esp_log.h"

spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096
    };

spi_device_interface_config_t fastSpiCfg = {
        .mode = 1,                                // SPI mode 0
        .clock_speed_hz = 16000000L,            // Clock out at 10 MHz
        .spics_io_num = DW_CS,               // CS pin
        .queue_size = 7,                    // We want to be able to queue 7 transactions at a time
    };
spi_device_interface_config_t slowSpiCfg = {
        .mode = 1,                                // SPI mode 0
        .clock_speed_hz = 2000000L,            // Clock out at 10 MHz
        .spics_io_num = DW_CS,               // CS pin
        .queue_size = 7,                    // We want to be able to queue 7 transactions at a time
    };

spi_device_handle_t spi;

void init_spi() {
    // Initialize the SPI bus
    esp_err_t  ret_first = spi_bus_initialize(HSPI_HOST, &buscfg, 0);
    if (ret_first != ESP_OK) {
        printf("spi_bus_add_device failed: %s\n", esp_err_to_name(ret_first));
    }
    // Add the fast and slow SPI devices
    esp_err_t ret_second = spi_bus_add_device(HSPI_HOST, &fastSpiCfg, &spi);
    if (ret_second != ESP_OK) {
        printf("spi_bus_add_device failed: %s\n", esp_err_to_name(ret_second));
    }
} 


void changeSpiFrequency(spi_device_interface_config_t newConfig) {
    esp_err_t err = spi_bus_remove_device(spi);
    if (err != ESP_OK) {
        printf("\n Error removing SPI device: %s", esp_err_to_name(err));
    }

    err = spi_bus_add_device(HSPI_HOST, &newConfig, &spi);
    if (err != ESP_OK) {
        printf("\n Error adding SPI device: %s", esp_err_to_name(err));
    }

}



void init_gpio() {
    gpio_set_direction(PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_IRQ, GPIO_MODE_INPUT);
    gpio_set_intr_type(PIN_IRQ, GPIO_INTR_ANYEDGE);
}