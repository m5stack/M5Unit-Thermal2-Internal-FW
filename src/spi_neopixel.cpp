#include "spi_neopixel.hpp"

#include <esp_log.h>

#include <driver/spi_master.h>
#include <driver/periph_ctrl.h>
#include <driver/gpio.h>
#include <soc/spi_reg.h>
#include <soc/soc.h>
#include <soc/rtc.h>
#include <cstring>

namespace m5 {
static uint32_t getApbFrequency(void) {
    rtc_cpu_freq_config_t conf;
    rtc_clk_cpu_freq_get_config(&conf);
    if (conf.freq_mhz >= 80) {
        return 80 * 1000000;
    }
    return (conf.source_freq_mhz * 1000000) / conf.div;
}

static uint32_t FreqToClockDiv(uint32_t fapb, uint32_t hz) {
    if (fapb <= hz) return SPI_CLK_EQU_SYSCLK;
    uint32_t div_num = fapb / (1 + hz);
    uint32_t pre     = div_num / 64u;
    div_num          = div_num / (pre + 1);
    return div_num << 12 | ((div_num - 1) >> 1) << 6 | div_num | pre << 18;
}

bool NeoPixel_Class::init(int pin, int spi_host) {
    uint32_t spi_port = (spi_host + 1);
    _spi_port         = spi_port;
    _pin              = pin;

    periph_module_enable(spi_periph_signal[spi_host].module);
    gpio_matrix_out(pin, spi_periph_signal[spi_host].spid_out, false, false);

    // gpio_matrix_out(32, spi_periph_signal[spi_host].spid_out, false, false);
    // // for DEBUG gpio_matrix_out(33, spi_periph_signal[spi_host].spiclk_out,
    // false, false);

    uint32_t clkdiv = FreqToClockDiv(getApbFrequency(), 2500000);
    WRITE_PERI_REG(SPI_CLOCK_REG(spi_port), clkdiv);
    WRITE_PERI_REG(SPI_USER_REG(spi_port),
                   SPI_USR_MOSI | SPI_USR_MISO | SPI_DOUTDIN);
    WRITE_PERI_REG(SPI_CTRL_REG(spi_port), 0);
    WRITE_PERI_REG(SPI_CTRL2_REG(spi_port), 0);
    WRITE_PERI_REG(SPI_SLAVE_REG(spi_port), 0);
    WRITE_PERI_REG(SPI_PIN_REG(spi_port), 0);
    WRITE_PERI_REG(SPI_DMA_IN_LINK_REG(spi_port), 0);
    WRITE_PERI_REG(SPI_DMA_OUT_LINK_REG(spi_port), 0);
    WRITE_PERI_REG(SPI_DMA_CONF_REG(spi_port), 0);
    WRITE_PERI_REG(SPI_MOSI_DLEN_REG(spi_port),
                   (32 << 3) - 1);  // 送信ビット数は末尾に無信号期間を含める
    memset(reinterpret_cast<void*>(SPI_W0_REG(spi_port)), 0,
           64);  // 送信バッファをゼロクリア

    return true;
}

void NeoPixel_Class::setColor(uint32_t rgb) {
    union {
        uint32_t buf32[3];
        uint8_t buf[12];
    };
    buf32[2] = 0;

    /// RGB から GRBに変換
    uint32_t grb =
        ((rgb >> 16) << 8) + (((rgb >> 8) & 0xFF) << 16) + (rgb & 0xFF);

    uint32_t bytetmp  = 0;
    uint32_t idx3step = 0;
    uint32_t mask     = 0x00800000;
    for (int i = 0; i < 24 * 3; ++i) {
        bool bit = (idx3step == 0) ? 1 : (idx3step == 1) ? (grb & mask) : 0;
        bytetmp  = (bytetmp << 1) + bit;
        if (++idx3step == 3) {
            idx3step = 0;
            mask >>= 1;
        }
        if ((i & 7) == 7) {
            buf[i >> 3] = bytetmp;
        }
    }
    uint32_t spi_port = _spi_port;
    while (READ_PERI_REG(SPI_CMD_REG(spi_port)) & SPI_USR)
        ;
    memcpy(reinterpret_cast<void*>(SPI_W0_REG(spi_port)), buf, 12);
    WRITE_PERI_REG(SPI_CMD_REG(spi_port), SPI_USR);
}
/*
  void NeoPixel_Class::setColor(uint32_t rgb)
  {
    static constexpr const uint8_t bit2byte[4] = { 0x88, 0x8E, 0xE8, 0xEE };
    union
    {
      uint32_t buf32[3];
      uint8_t buf[12];
    };

    /// RGB から GRBに変換
    uint32_t bits = ((rgb >> 16) << 8) + (((rgb >> 8) & 0xFF) << 16) + (rgb &
0xFF);

    /// SPIの1Byte で
NeoPixelの2bitを再現する。24bitぶん再現するので12Byte使用する。 for (int i = 0;
i < 12; ++i)
    {
      buf[i] = bit2byte[(bits >> (22-(i<<1))) & 3];
    }
    uint32_t spi_port = _spi_port;
    while (READ_PERI_REG(SPI_CMD_REG(spi_port)) & SPI_USR);
    memcpy(reinterpret_cast<void*>(SPI_W0_REG(spi_port)), buf, 12);
    WRITE_PERI_REG(SPI_CMD_REG(spi_port), SPI_USR);
  }
//*/
};  // namespace m5
