// spi_config.cpp
#include "spi_config.h"



spi_bus_config_t buscfg = {
    .mosi_io_num = SPI_MOSI,
    .miso_io_num = SPI_MISO,
    .sclk_io_num = SPI_SCK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4096,

};


void init_gpio() {
    gpio_set_direction(PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_IRQ, GPIO_MODE_INPUT);
    gpio_set_intr_type(PIN_IRQ, GPIO_INTR_ANYEDGE);
    // Configure interrupt handler for PIN_IRQ if needed
}

// Fast SPI configuration
spi_device_interface_config_t fastSpiCfg = {
    .mode = 0,                         // SPI mode 0
#ifdef ESP8266
    .clock_speed_hz = 20 * 1000 * 1000, // 20 MHz for ESP8266
#else
    .clock_speed_hz = 16 * 1000 * 1000, // 16 MHz for others
#endif
    .spics_io_num = DW_CS,             // CS pin
    // ... other configurations ...
};

// Slow SPI configuration
spi_device_interface_config_t slowSpiCfg = {
    .mode = 0,                         // SPI mode 0
    .clock_speed_hz = 2 * 1000 * 1000, // 2 MHz
    .spics_io_num = DW_CS,             // CS pin
    // ... other configurations ...
};

spi_device_handle_t spi_handle_fast;
spi_device_handle_t spi_handle_slow;

void init_spi() {
    // Initialize the SPI bus
    esp_err_t  ret_first = spi_bus_initialize(HSPI_HOST, &buscfg, 0);
    if (ret_first != ESP_OK) {
        printf("spi_bus_add_device failed: %s\n", esp_err_to_name(ret_first));
    }
    // Add the fast and slow SPI devices
    esp_err_t ret_second = spi_bus_add_device(HSPI_HOST, &fastSpiCfg, &spi_handle_fast);
    if (ret_second != ESP_OK) {
        printf("spi_bus_add_device failed: %s\n", esp_err_to_name(ret_second));
    }
    esp_err_t ret_third =spi_bus_add_device(HSPI_HOST, &slowSpiCfg, &spi_handle_slow);
    if (ret_third != ESP_OK) {
        printf("spi_bus_add_device failed: %s\n", esp_err_to_name(ret_third));
    }
}