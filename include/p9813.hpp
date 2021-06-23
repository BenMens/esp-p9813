#ifndef _P9813_H
#define _P9813_H

#include <stdio.h>
#include "driver/spi_master.h"

class P9813 {
  private:
    spi_device_handle_t spi;
    uint8_t *tx_buffer;
    int numModules;

  public:
    P9813(
      int numModules,
      spi_host_device_t spiHost, 
      int dmaChan, 
      int pinMosi, 
      int pinSclk, 
      int pinCS);

    void writeColor(int index, uint32_t color);
    void writeColor(int index, uint8_t red, uint8_t green, uint8_t blue);
};


#endif