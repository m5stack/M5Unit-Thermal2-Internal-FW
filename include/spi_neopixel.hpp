#pragma once

#include <driver/spi_common.h>

namespace m5
{
  /// Generate RGB LED control signals using MOSI on SPI bus.
  class NeoPixel_Class
  {
  public:

    NeoPixel_Class(void) {};
    bool init(int pin, int spi_host = HSPI_HOST);

    void setColor(uint32_t rgb);
    void setColor(uint8_t r, uint8_t g, uint8_t b) { setColor(r<<16|g<<8|b); }

  private:

    int _pin;
    int _spi_port;
  };
};
