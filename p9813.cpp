#include <p9813.hpp>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TAG "P9813"

P9813::P9813(
    int numModules,
    spi_host_device_t spiHost, 
    int dmaChan, 
    int pinMosi, 
    int pinSclk, 
    int pinCS)
  : numModules(numModules) {

  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = pinMosi;
  buscfg.miso_io_num = -1;
  buscfg.sclk_io_num = pinSclk;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 3 * 4;

  spi_device_interface_config_t devcfg = {};
  devcfg.command_bits = 0;
  devcfg.address_bits = 0;
  devcfg.clock_speed_hz = 10 * 1000 * 1000;
  devcfg.mode = 0;
  devcfg.spics_io_num = pinCS; 
  devcfg.queue_size = 1;                      

  ESP_ERROR_CHECK(spi_bus_initialize(spiHost, &buscfg, dmaChan));

  ESP_ERROR_CHECK(spi_bus_add_device(spiHost, &devcfg, &spi));

  tx_buffer = (uint8_t *)heap_caps_malloc((numModules + 2) * 4, MALLOC_CAP_DMA);

  tx_buffer[ 0] = 0x00;
  tx_buffer[ 1] = 0x00;
  tx_buffer[ 2] = 0x00;
  tx_buffer[ 3] = 0x00;

  tx_buffer[numModules * 4 + 0] = 0x00;
  tx_buffer[numModules * 4 + 1] = 0x00;
  tx_buffer[numModules * 4 + 2] = 0x00;
  tx_buffer[numModules * 4 + 3] = 0x00;
}


void P9813::writeColor(int index, uint32_t color) {
  writeColor(index, (color >> 16) & 0xff, (color >> 8) & 0xff, color & 0xff);
}


void P9813::writeColor(int index, uint8_t red, uint8_t green, uint8_t blue) {
  static spi_transaction_t trans = {};

  trans.length = (numModules + 2)  * 4 * 8;
  trans.tx_buffer = tx_buffer;
  trans.rx_buffer = nullptr;

  int offset = (index + 1) * 4;

    // Start by sending a byte with the format "1 1 /B7 /B6 /G7 /G6 /R7 /R6"
  tx_buffer[offset] = (1<<6) | (1<<7);
  if ((green & 0x80) == 0)  tx_buffer[offset] |= (1<<5);
  if ((green & 0x40) == 0)  tx_buffer[offset] |= (1<<4);
  if ((red & 0x80) == 0) tx_buffer[offset] |= (1<<3);
  if ((red & 0x40) == 0) tx_buffer[offset] |= (1<<2);
  if ((blue & 0x80) == 0)   tx_buffer[offset] |= (1<<1);
  if ((blue & 0x40) == 0)   tx_buffer[offset] |= (1<<0);

  tx_buffer[offset + 1] = green;
  tx_buffer[offset + 2] = red;
  tx_buffer[offset + 3] = blue;

  ESP_ERROR_CHECK(spi_device_polling_start(spi, &trans, portMAX_DELAY));

  ESP_ERROR_CHECK(spi_device_polling_end(spi, portMAX_DELAY));
}
