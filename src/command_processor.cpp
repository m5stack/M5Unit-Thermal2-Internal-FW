//! Copyright (c) M5Stack. All rights reserved.
//! Licensed under the MIT license.
//! See LICENSE file in the project root for full license information.

/* clang-format off */

// register list
// index : 0x00        = button status
//                       ※ [4:1] Flag cleared when user writes 1.
//                       [4]: button holded flag. ( 500msec keep )
//                       [3]: button clicked flag. (short click)
//                       [2]: button was released flag.
//                       [1]: button was pressed flag.
//                       [0]: button is pressed. (current state)

// index : 0x01        = temperature alarm status ( read only )
//                       [7] High temp reached high threshold
//                       [6] Ave  temp reached high threshold
//                       [5] Med  temp reached high threshold
//                       [4] Low  temp reached high threshold
//                       [3] High temp reached low threshold
//                       [2] Ave  temp reached low threshold
//                       [1] Med  temp reached low threshold
//                       [0] Low  temp reached low threshold

// index : 0x02 - 0x03 = (const) device status ( reserve )
// index : 0x04        = (const) device id ( 0x90 )
// index : 0x05        = (const) device id ( 0x64 )
// index : 0x06        = (const) firmware major version
// index : 0x07        = (const) firmware minor version
// index : 0x08        = I2C addr
// index : 0x09        = I2C addr (bit invert)

// index : 0x0A        = [0]: buzzer enable.
//                       [1]: neopixel enable.
//                       [2]: autorefresh enable.
// index : 0x0B        = [2:0]refresh rate 0b000:0.5Hz - 0b111:64Hz
// index : 0x0C        = [3:0]noise filter level

// index : 0x10        = temperature alerm area size
//                       [0-3] width  :     15:all - 0:minimum size
//                       [4-7] height :     11:all - 0:minimum size

// index : 0x11        = temperature alerm enable
//                       [7] High temp reached high threshold
//                       [6] Ave  temp reached high threshold
//                       [5] Med  temp reached high threshold
//                       [4] Low  temp reached high threshold
//                       [3] High temp reached low threshold
//                       [2] Ave  temp reached low threshold
//                       [1] Med  temp reached low threshold
//                       [0] Low  temp reached low threshold

// index : 0x12 - 0x13 = buzzer freq. 0~65535 (little endian)
// index : 0x14        = buzzer volume. 0~255
// index : 0x15 - 0x17 = Neopixel Color (R G B)
// index : 0x18 - 0x1F = reserved

// index : 0x20 - 0x21 = alarm threshold of lowest temperature.
// index : 0x22 - 0x24 = Low temperature alarm neopixel color. (R G B)
// index : 0x25        = Low temperature alarm buzzer interval x 10ms
// index : 0x26 - 0x27 = Low temperature alarm buzzer frequency. 0 ~ 65535(depends on buzzer volume setting) (little endian)
// index : 0x28 - 0x2F = reserved

// index : 0x30 - 0x31 = alarm threshold of highest temperature. value = (℃+64)*128 (little endian)
// index : 0x32 - 0x34 = High temperature alarm neopixel color. (R G B)
// index : 0x35        = High temperature alarm buzzer interval x 10ms
// index : 0x36 - 0x37 = High temperature alarm buzzer frequency. 0 ~ 65535(depends on buzzer volume setting) (little endian)
// index : 0x38 - 0x3F = reserved

// index : 0x40 - 0x6D = reserved

// index : 0x6E        = data refresh control ( 0:no new data / 1:available new data )
//                       ※ write 0: request new data.
// index : 0x6F        = subpage information (0 or 1)
// index : 0x70 - 0x71 = Median temperature (little endian)
// index : 0x72 - 0x74 = Average temperature (little endian)
// index : 0x74 - 0x75 = Most differential temperature (little endian)
// index : 0x76        = Most differential x position
// index : 0x77        = Most differential y position
// index : 0x78 - 0x79 = Lowest temperature (little endian)
// index : 0x7A        = Lowest x position
// index : 0x7B        = Lowest y position
// index : 0x7C - 0x7D = Highest temperature (little endian)
// index : 0x7E        = Highest x position
// index : 0x7F        = Highest y position

// index : 0x80 - 0x97 = Temperature data array (16x24 = 384 word) (little endian)
//                       ※ One register number contains 16 word (32 bytes) of data.
// index : 0x80        = X0~15,Y0 Temperature data array (16 word) (little endian)
// index : 0x81        = X0~15,Y1 Temperature data array (16 word) (little endian)
// index : 0x82        = X0~15,Y2 Temperature data array (16 word) (little endian)
//   ～～～～～～～～～～～～～～～
// index : 0x95        = X0~15,Y21 Temperature data array (16 word) (little endian)
// index : 0x96        = X0~15,Y22 Temperature data array (16 word) (little endian)
// index : 0x97        = X0~15,Y23 Temperature data array (16 word) (little endian)

/*
The resolution of the camera is 32x24, but the data obtained at one time is half of that.
The resulting Temperature data array is stored alternately like a chessboard.
You can use reg 0x6F `subpage information` to find out whether even or odd numbers were obtained

■ == subpage 0
□ == subpage 1
         : 　０ １ ２ ３ … 28 29 30 31
reg 0x80 : ０■ □ ■ □ … ■ □ ■ □
reg 0x81 : １□ ■ □ ■ … □ ■ □ ■
reg 0x82 : ２■ □ ■ □ … ■ □ ■ □
           ～～～～～～～～～～～
reg 0x95 : 21□ ■ □ ■ … □ ■ □ ■
reg 0x96 : 22■ □ ■ □ … ■ □ ■ □
reg 0x97 : 23□ ■ □ ■ … □ ■ □ ■
*/

// ※ Temperature value conversion formula:  (°C + 64) * 128 = value.
// e.g.
//     0 == -64.00°C
//  8192 ==   0.00°C
// 12032 ==  30.00°C
// 20992 == 100.00℃
// 65535 == 447.99℃

/* clang-format on */

#include "command_processor.hpp"

#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/rtc.h>
static rtc_cpu_freq_config_t _cpu_freq_conf_160;
static rtc_cpu_freq_config_t _cpu_freq_conf_80;

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <nvs_flash.h>

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <algorithm>

#include "spi_neopixel.hpp"
#include "i2c_master.hpp"
#include "i2c_slave.hpp"
#include "mlx90640.hpp"

#include "Button_Class.hpp"
#include "unit_thermal2.hpp"
#include "update.hpp"
#include "common.hpp"

#define setbit(x, y)     x |= (0x01 << (y))
#define clrbit(x, y)     x &= ~(0x01 << (y))
#define reversebit(x, y) x ^= (0x01 << (y))
#define getbit(x, y)     ((x) >> (y)&0x01)

namespace command_processor {

// MLX90640
static constexpr gpio_num_t PIN_IN_SDA = GPIO_NUM_21;
static constexpr gpio_num_t PIN_IN_SCL = GPIO_NUM_22;
// GROVE PORTA
static constexpr gpio_num_t PIN_EX_SDA = GPIO_NUM_33;
static constexpr gpio_num_t PIN_EX_SCL = GPIO_NUM_32;
// neopixel, buzzer, button
static constexpr gpio_num_t PIN_RGB_LED = GPIO_NUM_27;
static constexpr gpio_num_t PIN_BUZZER  = GPIO_NUM_25;
static constexpr gpio_num_t PIN_BUTTON  = GPIO_NUM_39;

static constexpr int BUZZER_LEDC_CHAN = 1;

static constexpr int I2C_DEFAULT_ADDR = 0x32;

// Flash memory cannot be accessed while NVS is being saved, so it is necessary
// to stop the operation of the receiving task.
// static std::mutex _flash_lock;
static xSemaphoreHandle _flash_locker;

static m5::I2C_Master _i2c_in;
static m5::I2C_Slave _i2c_ex;
static m5::NeoPixel_Class _led;
static m5::MLX90640_Class _mlx;
static m5::Button_Class _btn;

static uint32_t _alarm_last_time = 0;
static uint32_t _alarm_interval  = 50;

enum alarm_state_t {
    alarm_change_setting,
    alarm_none,
    alarm_high_temp1,
    alarm_high_temp2,
    alarm_low_temp1,
    alarm_low_temp2,
};
static alarm_state_t _current_alarm_state = alarm_change_setting;

volatile static uint8_t _save_nvs_countdown = 0;

static constexpr size_t MLX_TEMP_ARRAY_SIZE      = 16;
static constexpr size_t MLX_FRAMEDATA_ARRAY_SIZE = 4;
static volatile int _idx_framedata               = -1;
static volatile int _idx_tempdata                = -1;
static uint16_t* _mlx_framedatas[MLX_FRAMEDATA_ARRAY_SIZE];
static m5::MLX90640_Class::temp_data_t* _mlx_tempdatas[MLX_TEMP_ARRAY_SIZE] = {
    nullptr};
static m5::MLX90640_Class::temp_data_t* _temp_data = nullptr;
static int16_t* _diff_data;

static constexpr char NVS_KEY_REGDATA[] = "REGDATA";

static constexpr uint8_t REGINDEX_I2C_ADDR     = 0x08;
static constexpr uint8_t REGINDEX_REFRESH_CTRL = 0x6E;
static constexpr uint8_t REGINDEX_PIXEL_DATA   = 0x80;
static constexpr size_t PIXEL_DATA_SIZE        = 16 * 24 * 2;

/// Overall register data size
static constexpr size_t UNIT_REGISTER_SIZE =
    REGINDEX_PIXEL_DATA + PIXEL_DATA_SIZE;

// Data received from I2C master.
static union {
    uint8_t _i2c_rx_data[UNIT_REGISTER_SIZE];
    unit_thermal2_reg_t _reg_rx_data;
};

// Data to be sent to I2C master.
static union {
    uint8_t _i2c_tx_data[UNIT_REGISTER_SIZE + 16];
    unit_thermal2_reg_t _reg_tx_data;
};

static constexpr size_t REGISTER_WRITABLE_MAP_SIZE =
    (REGINDEX_REFRESH_CTRL + 1);
static constexpr size_t REGISTER_WRITABLE_OFFSET = 8;

// unit_thermal2_reg_t writable map
// Must not be placed in ROM to be accessed from ISR.
static constexpr uint8_t DRAM_ATTR
    REGISTER_WRITABLE_MAP[(REGISTER_WRITABLE_MAP_SIZE + 7) >> 3] = {
        0x80, 0xFF,  // 0x00 - 0x0F
        0xFF, 0xFF,  // 0x10 - 0x1F
        0xFF, 0xFF,  // 0x20 - 0x2F
        0xFF, 0xFF,  // 0x30 - 0x3F
        0xFF, 0xFF,  // 0x40 - 0x4F
        0xFF, 0xFF,  // 0x50 - 0x5F
        0xFF, 0xFE,  // 0x60 - 0x6F
};
static constexpr uint8_t DRAM_ATTR
    REGISTER_DIRECT_WRITE_MAP[(REGISTER_WRITABLE_MAP_SIZE + 7) >> 3] = {
        0x00, 0xD8,  // 0x00 - 0x0F
        0xC0, 0x00,  // 0x10 - 0x1F
        0xFF, 0x00,  // 0x20 - 0x2F
        0xFF, 0x00,  // 0x30 - 0x3F
        0x00, 0x00,  // 0x40 - 0x4F
        0x00, 0x00,  // 0x50 - 0x5F
        0x00, 0x02,  // 0x60 - 0x6E
};

static constexpr std::size_t RX_BUFFER_MAX = 0x10;
static constexpr std::size_t PARAM_MAXLEN  = 12;
static std::uint8_t _rx_buffer[RX_BUFFER_MAX][PARAM_MAXLEN];
static std::size_t _write_reg_index = 0;  // 書込み先のI2Cレジスタ番号
static std::size_t _read_reg_index = 0;  // 読出し元のI2Cレジスタ番号
static volatile bool _reg_modified = false;  // レジスタデータ更新の有無フラグ
static volatile bool _reg_read_complete       = true;
static volatile std::size_t _rx_buffer_setpos = 0;
static volatile std::size_t _rx_buffer_getpos = 0;
static std::uint8_t* _params                  = _rx_buffer[0];
static std::size_t _param_index               = 0;
static std::size_t _param_need_count          = 1;
static std::size_t _param_resetindex          = 0;
enum firmupdate_state_t {
    nothing,       // コマンド未受信;
    wait_data,     // データ待機;
    progress,      // データ受信中;
    sector_write,  // セクタブロックのフラッシュ書き込み;
    finish,        // 全行程終了;
};
static volatile firmupdate_state_t _firmupdate_state = nothing;
static std::size_t _firmupdate_index                 = 0;
static std::size_t _firmupdate_totalsize             = 0;
static std::size_t _firmupdate_result                = 0;
static std::size_t _last_command                     = 0;

static constexpr uint8_t CMD_REG_ACCESS = 0x00;
// 4Byte ファームウェア更新開始   [0]=0xF0 [1]=0x66 [2]=0x75 [3]=0xF0
static constexpr uint8_t CMD_UPDATE_BEGIN = 0xF0;
// 4Byte ファームウェアデータ送信 [0]=0xF1 [1]=0x66 [2]=0x75 [3]=0xF1
static constexpr uint8_t CMD_UPDATE_DATA = 0xF1;
// 4Byte ファームウェア更新完了   [0]=0xF2 [1]=0x66 [2]=0x75 [3]=0xF2
static constexpr uint8_t CMD_UPDATE_END = 0xF2;
// 4Byte リセット(再起動)         [0]=0xFF [1]=0x66 [2]=0x75 [3]=0xFF
static constexpr uint8_t CMD_RESET            = 0xFF;
static constexpr uint8_t UPDATE_RESULT_BROKEN = 0x01;
static constexpr uint8_t UPDATE_RESULT_ERROR  = 0x00;
static constexpr uint8_t UPDATE_RESULT_OK     = 0xF1;
static constexpr uint8_t UPDATE_RESULT_BUSY   = 0xFF;

/* clang-format off */
static inline volatile uint32_t* get_gpio_hi_reg(int_fast8_t pin) { return (pin & 32) ? &GPIO.out1_w1ts.val : &GPIO.out_w1ts; }
static inline volatile uint32_t* get_gpio_lo_reg(int_fast8_t pin) { return (pin & 32) ? &GPIO.out1_w1tc.val : &GPIO.out_w1tc; }
static inline bool gpio_in(int_fast8_t pin) { return ((pin & 32) ? GPIO.in1.data : GPIO.in) & (1 << (pin & 31)); }
static inline void gpio_hi(int_fast8_t pin) { *get_gpio_hi_reg(pin) = 1 << (pin & 31); }
static inline void gpio_lo(int_fast8_t pin) { *get_gpio_lo_reg(pin) = 1 << (pin & 31); }
/* clang-format on */

static void setBuzzer(uint32_t freq, uint32_t volume) {
    static uint32_t prev_freq = ~0u;
    static uint32_t prev_duty = ~0u;

    static constexpr auto group   = (ledc_mode_t)(BUZZER_LEDC_CHAN >> 3);
    static constexpr auto timer   = (ledc_timer_t)((BUZZER_LEDC_CHAN >> 1) & 3);
    static constexpr auto channel = (ledc_channel_t)((BUZZER_LEDC_CHAN)&7);

    if (prev_freq != freq) {
        prev_freq = freq;
        ledc_timer_config_t ledc_timer;
        ledc_timer.speed_mode      = LEDC_HIGH_SPEED_MODE;
        ledc_timer.timer_num       = timer;
        ledc_timer.duty_resolution = LEDC_TIMER_10_BIT;
        ledc_timer.freq_hz         = freq;
        ledc_timer.clk_cfg         = LEDC_USE_APB_CLK;
        ledc_timer_config(&ledc_timer);
    }
    if (freq == 0) {
        volume = 0;
    }
    uint32_t duty = 0;
    if (volume) {
        ++volume;
        duty = (volume * volume) >> 7;
    }
    if (prev_duty != duty) {
        prev_duty = duty;
        ledc_set_duty(group, channel, duty);
        ledc_update_duty(group, channel);
        *(uint32_t*)GPIO_FUNC25_OUT_SEL_CFG_REG = duty ? 0x48 : 0x100;
    }
}

static void IRAM_ATTR core0Task(void* main_handle) {
    // initialize sensor.
    _i2c_in.init(I2C_NUM_0, PIN_IN_SDA, PIN_IN_SCL);
    int retry = 16;
    _led.init(PIN_RGB_LED);
    _led.setColor(0, 2, 0);

    while (!_mlx.init(&_i2c_in) && --retry) {
        _led.setColor(2, 0, 2);
        _i2c_in.stop();

        if (retry & 1) {
            // for beta version failsafe (pin SDA SCL swapped)
            if (!_i2c_in.init(I2C_NUM_0, PIN_IN_SCL, PIN_IN_SDA)) {
                ESP_LOGE(LOGNAME, "I2C In init failure.");
                esp_restart();
            }
        } else if (!_i2c_in.init(I2C_NUM_0, PIN_IN_SDA, PIN_IN_SCL)) {
            ESP_LOGE(LOGNAME, "I2C In init failure.");
            esp_restart();
        }
    }

    if (retry == 0) {
        ESP_LOGE(LOGNAME, "MLX90640 init failure.");
        _led.setColor(2, 2, 0);
        {
            gpio_config_t io_conf;
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
            io_conf.intr_type    = GPIO_INTR_DISABLE;
            io_conf.mode         = GPIO_MODE_OUTPUT;
            io_conf.pin_bit_mask = (uint64_t)1 << PIN_IN_SDA;
            gpio_config(&io_conf);
            io_conf.pin_bit_mask = (uint64_t)1 << PIN_IN_SCL;
            gpio_config(&io_conf);
            for (int i = 0; i < 20; ++i) {
                vTaskDelay(1);
                gpio_lo(PIN_IN_SCL);
                vTaskDelay(1);
                gpio_lo(PIN_IN_SDA);
                vTaskDelay(1);
                gpio_hi(PIN_IN_SCL);
                vTaskDelay(1);
                gpio_hi(PIN_IN_SDA);
            }
        }
        esp_restart();
    }
    xSemaphoreTake(_flash_locker, portMAX_DELAY);
    _mlx.setRate(
        (m5::MLX90640_Class::refresh_rate_t)_reg_tx_data.config.refresh_rate);
    xSemaphoreGive(_flash_locker);

    for (int i = 0; i < 4; ++i) {
        _mlx_framedatas[i] = (uint16_t*)heap_caps_malloc(
            m5::MLX90640_Class::FRAME_DATA_BYTES, MALLOC_CAP_DMA);
        memset(_mlx_framedatas[i], 0x2C, m5::MLX90640_Class::FRAME_DATA_BYTES);
    }

    // running...
    size_t discard_count = 2;
    while (_firmupdate_state == nothing) {
        if ((uint8_t)_mlx.getRate() !=
            (uint8_t)_reg_tx_data.config.refresh_rate) {
            uint8_t rate = _reg_tx_data.config.refresh_rate & 7;
            _reg_tx_data.config.refresh_rate =
                (unit_thermal2_reg_t::refresh_rate_t)rate;
            xSemaphoreTake(_flash_locker, portMAX_DELAY);
            _mlx.setRate((m5::MLX90640_Class::refresh_rate_t)rate);
            xSemaphoreGive(_flash_locker);
            // Discard twice because invalid data is obtained immediately after
            // refresh rate change.
            discard_count = 2;
        }
        xSemaphoreTake(_flash_locker, portMAX_DELAY);
        int idx   = (_idx_framedata + 1) & (MLX_FRAMEDATA_ARRAY_SIZE - 1);
        bool recv = _mlx.readFrameData(_mlx_framedatas[idx]);
        xSemaphoreGive(_flash_locker);
        if (recv) {
            if (discard_count) {
                --discard_count;
            } else {
                _idx_framedata = idx;
                xTaskNotifyGive(main_handle);
            }
        } else {
            vTaskDelay(1);
        }
    }
    vTaskDelete(nullptr);
}

static void validateRegData(void) {
    uint_fast8_t i2c_addr = _reg_rx_data.config.i2c_addr;
    if (0xFF != (i2c_addr ^ _reg_rx_data.config.i2c_addr_inv) ||
        i2c_addr < 0x08 || i2c_addr >= 0x78) {
        ESP_LOGI(LOGNAME, "invalid i2c_addr:%02x  use default addr:%02x",
                 i2c_addr, I2C_DEFAULT_ADDR);
        _reg_rx_data.config.i2c_addr     = I2C_DEFAULT_ADDR;
        _reg_rx_data.config.i2c_addr_inv = ~I2C_DEFAULT_ADDR;
    }

    // Buzzer alarm minimum interval is 50ms.
    if (_reg_rx_data.lowest_alarm.buzzer_interval < 5) {
        _reg_rx_data.lowest_alarm.buzzer_interval = 5;
    }
    if (_reg_rx_data.highest_alarm.buzzer_interval < 5) {
        _reg_rx_data.highest_alarm.buzzer_interval = 5;
    }
}

static void IRAM_ATTR save_nvs(void) {
    rtc_clk_cpu_freq_set_config_fast(&_cpu_freq_conf_160);

    validateRegData();
    uint8_t new_i2c_addr = 0;
    if (_reg_rx_data.config.i2c_addr != _reg_tx_data.config.i2c_addr) {
        ESP_LOGI(LOGNAME, "I2C_Addr diff:%02x -> %02x",
                 _reg_rx_data.config.i2c_addr, _reg_tx_data.config.i2c_addr);

        if (_reg_rx_data.config.i2c_addr != _reg_tx_data.config.i2c_addr &&
            _reg_rx_data.config.i2c_addr >= 0x08 &&
            _reg_rx_data.config.i2c_addr < 0x78 &&
            0xFF == (_reg_rx_data.config.i2c_addr ^
                     _reg_rx_data.config.i2c_addr_inv)) {
            new_i2c_addr = _reg_rx_data.config.i2c_addr;
        }
        if (new_i2c_addr) {
            ESP_LOGI(LOGNAME, "I2C_Addr change:%02x -> %02x",
                     _i2c_ex.getI2CAddr(), new_i2c_addr);
            _reg_tx_data.config.i2c_addr     = new_i2c_addr;
            _reg_tx_data.config.i2c_addr_inv = ~new_i2c_addr;
        }
    }

    // Stop the sensor receiving task because the flash ROM cache is invalidated
    // when writing to NVS is executed;
    xSemaphoreTake(_flash_locker, portMAX_DELAY);
    uint32_t handle = 0;
    if (ESP_OK != nvs_open(LOGNAME, NVS_READWRITE, &handle)) {
        ESP_EARLY_LOGI(LOGNAME, "nvs open error.");
    } else {
        ESP_EARLY_LOGI(LOGNAME, "nvs save");
        nvs_set_blob(handle, NVS_KEY_REGDATA,
                     &_i2c_tx_data[REGISTER_WRITABLE_OFFSET],
                     sizeof(unit_thermal2_reg_t) - REGISTER_WRITABLE_OFFSET);

        nvs_commit(handle);
        nvs_close(handle);
        ESP_EARLY_LOGI(LOGNAME, "done.");
    }
    xSemaphoreGive(_flash_locker);
    _reg_rx_data.config.i2c_addr     = _reg_tx_data.config.i2c_addr;
    _reg_rx_data.config.i2c_addr_inv = _reg_tx_data.config.i2c_addr_inv;

    rtc_clk_cpu_freq_set_config_fast(&_cpu_freq_conf_80);
}

static void IRAM_ATTR load_nvs(void) {
    memset(_i2c_rx_data, 0, UNIT_REGISTER_SIZE);

    ESP_LOGI(LOGNAME, "load_nvs");
    uint32_t handle;
    bool loaded    = false;
    esp_err_t init = nvs_flash_init();
    if (ESP_OK != init) {
        ESP_LOGE(LOGNAME, "nvs flash init error. %d", init);
    }
    if (ESP_OK != nvs_open(LOGNAME, NVS_READONLY, &handle)) {
        ESP_LOGI(LOGNAME, "nvs open error.  start nvs erase ...");
        while (init != ESP_OK) {
            taskYIELD();
            nvs_flash_erase();
            init = nvs_flash_init();
        }
        ESP_LOGI(LOGNAME, "done.");

        _save_nvs_countdown = 1;
    } else {
        size_t len = sizeof(unit_thermal2_reg_t) - REGISTER_WRITABLE_OFFSET;
        loaded     = (ESP_OK ==
                  nvs_get_blob(handle, NVS_KEY_REGDATA,
                                   &_i2c_rx_data[REGISTER_WRITABLE_OFFSET], &len));
        if (!loaded) {
            ESP_LOGE(LOGNAME, "nvs error: can't get regdata.");
        }
        nvs_close(handle);
    }

    // Register data initial value setting.
    _reg_rx_data.status.device_id_0   = DEVICE_ID_0;             // Device ID
    _reg_rx_data.status.device_id_1   = DEVICE_ID_1;             // Device ID
    _reg_rx_data.status.version_major = FIRMWARE_MAJOR_VERSION;  // major ver
    _reg_rx_data.status.version_minor = FIRMWARE_MINOR_VERSION;  // minor ver

    if (loaded) {
        validateRegData();
    } else {
        _reg_rx_data.config.i2c_addr          = I2C_DEFAULT_ADDR;
        _reg_rx_data.config.i2c_addr_inv      = ~I2C_DEFAULT_ADDR;
        _reg_rx_data.config.function_ctrl     = 4;
        _reg_rx_data.config.refresh_rate      = _reg_rx_data.rate_16Hz;
        _reg_rx_data.config.noise_filter      = 4;
        _reg_rx_data.config.buzzer_volume     = 128;
        _reg_rx_data.config.buzzer_freq       = 4800;
        _reg_rx_data.config.temp_alarm_area   = 0xFF;
        _reg_rx_data.config.temp_alarm_enable = 0;

        _reg_rx_data.lowest_alarm.buzzer_interval = 5;
        _reg_rx_data.lowest_alarm.buzzer_freq     = 4800;
        _reg_rx_data.lowest_alarm.temp_threshold  = (0 + 64) * 128;
        _reg_rx_data.lowest_alarm.led.r           = 0;
        _reg_rx_data.lowest_alarm.led.g           = 0;
        _reg_rx_data.lowest_alarm.led.b           = 4;

        _reg_rx_data.highest_alarm.buzzer_interval = 5;
        _reg_rx_data.highest_alarm.buzzer_freq     = 4800;
        _reg_rx_data.highest_alarm.temp_threshold  = (100 + 64) * 128;
        _reg_rx_data.highest_alarm.led.r           = 4;
        _reg_rx_data.highest_alarm.led.g           = 0;
        _reg_rx_data.highest_alarm.led.b           = 0;
    }

    memcpy(&_reg_tx_data, &_reg_rx_data, sizeof(unit_thermal2_reg_t));

#if CORE_DEBUG_LEVEL > 0
    auto d = (const uint8_t*)&_reg_tx_data;
    for (size_t r = 0; r < 8; ++r) {
        ESP_LOGI(LOGNAME,
                 "reg:0x%02x: %02x %02x %02x %02x  %02x %02x %02x %02x  %02x "
                 "%02x %02x %02x  %02x %02x %02x %02x",
                 r * 16, d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8],
                 d[9], d[10], d[11], d[12], d[13], d[14], d[15]);
        d += 16;
    }
#endif
}

static bool IRAM_ATTR command(void) {
    if (_rx_buffer_getpos == _rx_buffer_setpos) {
        return false;
    }
    const std::uint8_t* params = _rx_buffer[_rx_buffer_getpos];

    switch (params[0]) {
        default:
            ESP_LOGI(LOGNAME, "unknown CMD:%02x", params[0]);
            break;

        case CMD_UPDATE_BEGIN:
            update::initCRCtable();
            break;

        case CMD_UPDATE_DATA:
            _firmupdate_result = UPDATE_RESULT_BUSY;
            ESP_EARLY_LOGI(LOGNAME, "flash:%d", _firmupdate_index);
            {
                setBuzzer(440, 32);
                _led.setColor(0, 0, 0);
                xSemaphoreTake(_flash_locker, portMAX_DELAY);
                bool result = update::writeBuffer(_firmupdate_index);
                xSemaphoreGive(_flash_locker);
                setBuzzer(0, 0);
                if (!result) {
                    ESP_EARLY_LOGE(LOGNAME, "OTA write fail");
                    _firmupdate_result = UPDATE_RESULT_ERROR;
                    _led.setColor(2, 0, 2);
                } else {
                    ESP_EARLY_LOGI(LOGNAME, "OTA write success");
                    _firmupdate_result = UPDATE_RESULT_OK;
                    _led.setColor(0, 4, 0);
                }
            }

            _firmupdate_state = firmupdate_state_t::wait_data;
            _firmupdate_index += SPI_FLASH_SEC_SIZE;
            break;

        case CMD_UPDATE_END:
            setBuzzer(0, 0);
            if (update::end()) {
                ESP_EARLY_LOGI(LOGNAME, "success! rebooting...");
                for (int i = 0; i < 8; ++i) {
                    setBuzzer(440 + (55 << (i & 3)), 32);

                    _led.setColor((i & 4) >> 1, (i & 2), (i & 1) << 1);
                    vTaskDelay(128);
                }
                _led.setColor(0, 0, 0);
                setBuzzer(0, 0);
                esp_restart();
            } else {
                _led.setColor(2, 0, 2);
                ESP_EARLY_LOGI(LOGNAME, "OTA close fail");
            }
            break;
    }

    _rx_buffer_getpos = (_rx_buffer_getpos + 1) & (RX_BUFFER_MAX - 1);
    return true;
}

void setup(void) {
    xTaskCreatePinnedToCore(core0Task, "core0Task", 8192,
                            xTaskGetCurrentTaskHandle(), 20, nullptr,
                            PRO_CPU_NUM);
    load_nvs();
    _i2c_ex.init(I2C_NUM_1, PIN_EX_SDA, PIN_EX_SCL,
                 _reg_tx_data.config.i2c_addr);
    ESP_LOGI(LOGNAME, "I2C_SLAVE init: addr=0x%02x",
             _reg_tx_data.config.i2c_addr);

    _flash_locker = xSemaphoreCreateMutex();

    gpio_config_t io_conf;
    io_conf.mode         = GPIO_MODE_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pin_bit_mask =
        1 << GPIO_NUM_0 | 1 << GPIO_NUM_2 | 1 << GPIO_NUM_4 | 1 << GPIO_NUM_5 |
        1 << GPIO_NUM_9 | 1 << GPIO_NUM_10 | 1 << GPIO_NUM_12 |
        1 << GPIO_NUM_13 | 1 << GPIO_NUM_14 | 1 << GPIO_NUM_15 |
        1 << GPIO_NUM_18 | 1 << GPIO_NUM_19 | 1 << GPIO_NUM_23 |
        1 << GPIO_NUM_25 | 1 << GPIO_NUM_26
#if CORE_DEBUG_LEVEL == 0
        | 1 << GPIO_NUM_1 | 1 << GPIO_NUM_3
#endif
        ;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (uint64_t)1 << PIN_BUTTON;
    io_conf.mode         = GPIO_MODE_INPUT;
    gpio_config(&io_conf);

    // buzzer start beep
    static constexpr auto group   = (ledc_mode_t)(BUZZER_LEDC_CHAN >> 3);
    static constexpr auto channel = (ledc_channel_t)((BUZZER_LEDC_CHAN)&7);
    static constexpr auto timer   = (ledc_timer_t)((BUZZER_LEDC_CHAN >> 1) & 3);

    ledc_channel_config_t ledc_channel;
    ledc_channel.speed_mode = group;
    ledc_channel.channel    = channel;
    ledc_channel.timer_sel  = timer;
    ledc_channel.intr_type  = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num   = PIN_BUZZER;
    ledc_channel.duty       = 0;
    ledc_channel.hpoint     = 0;
    ledc_channel_config(&ledc_channel);
    setBuzzer(0, 0);

    for (int i = 0; i < MLX_TEMP_ARRAY_SIZE; ++i) {
        _mlx_tempdatas[i] = (m5::MLX90640_Class::temp_data_t*)heap_caps_malloc(
            sizeof(m5::MLX90640_Class::temp_data_t), MALLOC_CAP_DMA);
        memset(_mlx_tempdatas[i], 0, sizeof(m5::MLX90640_Class::temp_data_t));
    }
    _diff_data = (int16_t*)heap_caps_malloc(m5::MLX90640_Class::TEMP_DATA_BYTES,
                                            MALLOC_CAP_DMA);
    memset(_diff_data, 0, m5::MLX90640_Class::TEMP_DATA_BYTES);

    TaskHandle_t idle = xTaskGetIdleTaskHandleForCPU(PRO_CPU_NUM);
    if (idle != nullptr) esp_task_wdt_delete(idle);
    idle = xTaskGetIdleTaskHandleForCPU(APP_CPU_NUM);
    if (idle != nullptr) esp_task_wdt_delete(idle);

    rtc_clk_cpu_freq_mhz_to_config(160, &_cpu_freq_conf_160);
    rtc_clk_cpu_freq_mhz_to_config(80, &_cpu_freq_conf_80);
    rtc_clk_cpu_freq_set_config_fast(&_cpu_freq_conf_80);

    ulTaskNotifyTake(pdFALSE, 50);

    _led.setColor(0);
}

void IRAM_ATTR loop(void) {
    bool reg_mod = false;
    if (ulTaskNotifyTake(pdTRUE, 5)) {
        if (_reg_modified) {
            _reg_modified = false;
            reg_mod       = true;
        }
        while (command()) {
        }
        if (_firmupdate_state != nothing) {
            return;
        }

        if (_save_nvs_countdown) {
            if (--_save_nvs_countdown == 0) {
                save_nvs();
            }
        }

        // MLX90640数据
        static int prev_idx_framedata = -1;
        if (prev_idx_framedata != _idx_framedata) {
#if DEBUG == 1
            {  // debug
                if (_idx_framedata != ((prev_idx_framedata + 1) &
                                       (MLX_FRAMEDATA_ARRAY_SIZE - 1))) {
                    ESP_LOGE(LOGNAME, "prev_idx_frame:%d  idx_frame:%d",
                             prev_idx_framedata, _idx_framedata);
                }
            }
#endif
            prev_idx_framedata = _idx_framedata;
            int idx            = (_idx_tempdata + 1) % MLX_TEMP_ARRAY_SIZE;
            _temp_data         = _mlx_tempdatas[idx];
            auto prev_temp_data =
                _mlx_tempdatas[(idx + MLX_TEMP_ARRAY_SIZE - 2) %
                               MLX_TEMP_ARRAY_SIZE];

            static constexpr int16_t noise_filter_level[] = {
                181, 256, 362, 512, 724, 1024, 1448, 2048};
            int filter_value = noise_filter_level[_mlx.getRate()];
            int filter_gain =
                (filter_value * (_reg_tx_data.config.noise_filter & 0xF)) >> 6;

            /// Determine the monitoring range based on the register setting
            /// values.
            size_t monitor_width =
                1 + (_reg_tx_data.config.temp_alarm_area & 0x0F);
            size_t monitor_height =
                1 + (_reg_tx_data.config.temp_alarm_area >> 4);

            if (monitor_width > 16) {
                monitor_width = 16;
            }
            if (monitor_height > 12) {
                monitor_height = 12;
            }

            _mlx.calcTempData(_mlx_framedatas[prev_idx_framedata], _temp_data,
                              prev_temp_data, filter_gain, monitor_width,
                              monitor_height);

            int32_t avg_temp  = _temp_data->avg_temp;
            uint32_t max_diff = 0;
            static int diff_x;
            static int diff_y;
            bool subpage                             = _temp_data->subpage;
            static constexpr int16_t noise_of_rate[] = {32,  45,  64,  90,
                                                        128, 180, 256, 359};
            int32_t noise = noise_of_rate[_mlx.getRate()];
            int32_t kmul  = _mlx.getRate() + 1;
            for (int y = 0; y < m5::MLX90640_Class::PIXEL_ROWS; ++y) {
                bool inside_monitor_area =
                    (uint32_t)((y - 12) + monitor_height) <
                    (monitor_height * 2);

                int x = subpage ^ (y & 1);
                auto diff_line =
                    &_diff_data[y * m5::MLX90640_Class::PIXEL_COLS];
                auto temp_line = &_temp_data->data[y * 16];
                do {
                    // 平均温度を0基準に温度差を求める（基準値のゆらぎを除去）
                    int32_t ref_temp = (int32_t)temp_line[x >> 1] - avg_temp;
                    diff_line[x] =
                        (diff_line[x] * kmul + ref_temp) / (kmul + 1);
                    if (inside_monitor_area &&
                        (uint32_t)((x - 16) + monitor_width) <
                            (monitor_width * 2)) {
                        // 記憶している温度差を基準にした温度変化を求める;
                        int32_t sub = ref_temp - diff_line[x];
                        int32_t s   = sub + ((sub < 0) ? noise : -noise);
                        sub         = ((s < 0) == (sub < 0)) ? s : 0;
                        if (sub < 0) {
                            sub = -sub;
                        }
                        if (max_diff < sub) {
                            max_diff = sub;
                            diff_x   = x;
                            diff_y   = y;
                        }
                    }
                } while ((x += 2) < m5::MLX90640_Class::PIXEL_COLS);
            }
            _temp_data->diff_info.y    = diff_y;
            _temp_data->diff_info.x    = diff_x;
            _temp_data->diff_info.temp = max_diff;

            _idx_tempdata = idx;
        }
    }

    // Update temperature data in I2C slave registers.
    {
        static int prev_idx = -1;
        int idx             = _idx_tempdata;
        if (prev_idx != idx) {
            if ((!getbit(_i2c_rx_data[REGINDEX_REFRESH_CTRL], 0)) ||
                ((getbit(_reg_rx_data.config.function_ctrl, 2)) &&
                 // ((getbit(_i2c_rx_data[REGINDEX_REFRESH_CTRL], 1)) &&
                 _reg_read_complete)) {
                prev_idx = idx;

                // Adjust the order so that subpages are sent alternately.
                static bool prev_subp = false;
                prev_subp             = !prev_subp;
                if (prev_subp != (bool)(idx & 1)) {
                    if (--idx < 0) {
                        idx += (sizeof(_mlx_tempdatas) /
                                sizeof(_mlx_tempdatas[0]));
                    }
                }
                _reg_read_complete                   = false;
                _i2c_rx_data[REGINDEX_REFRESH_CTRL]  = 1;
                _mlx_tempdatas[idx]->refresh_control = 1;
                memcpy(&(_i2c_tx_data[REGINDEX_REFRESH_CTRL]),
                       _mlx_tempdatas[idx],
                       sizeof(m5::MLX90640_Class::temp_data_t));
            }
        }
    }

    uint32_t msec = esp_timer_get_time() / 1000ULL;

    {
        if (reg_mod && (_reg_tx_data.config != _reg_rx_data.config)) {
            bool chg = _reg_tx_data.config.function_ctrl !=
                           _reg_rx_data.config.function_ctrl ||
                       _reg_tx_data.config.buzzer_freq !=
                           _reg_rx_data.config.buzzer_freq ||
                       _reg_tx_data.config.buzzer_volume !=
                           _reg_rx_data.config.buzzer_volume ||
                       _reg_tx_data.config.led != _reg_rx_data.config.led;
            if (chg) {
                _reg_tx_data.config.function_ctrl =
                    _reg_rx_data.config.function_ctrl;
                _reg_tx_data.config.buzzer_freq =
                    _reg_rx_data.config.buzzer_freq;
                _reg_tx_data.config.buzzer_volume =
                    _reg_rx_data.config.buzzer_volume;
                _reg_tx_data.config.led = _reg_rx_data.config.led;

                if (_current_alarm_state == alarm_state_t::alarm_none) {
                    _current_alarm_state = alarm_state_t::alarm_change_setting;
                    _alarm_last_time     = msec + _alarm_interval + 1;
                    _save_nvs_countdown  = 16;
                }
            }
        }
        // temperature alarm check.
        if (((msec - _alarm_last_time) > _alarm_interval)) {
            if (_current_alarm_state == alarm_state_t::alarm_none ||
                _current_alarm_state == alarm_state_t::alarm_change_setting) {
                _alarm_last_time = msec;
                _alarm_interval  = (_alarm_interval + 50) >> 1;
            } else {
                _alarm_last_time += _alarm_interval;
            }

            alarm_state_t alarm_state = alarm_none;

            uint_fast8_t alarm = _reg_tx_data.config.temp_alarm_enable;

            if (alarm && _temp_data) {
                uint_fast16_t threshold_low =
                    _reg_tx_data.lowest_alarm.temp_threshold;
                uint_fast16_t threshold_high =
                    _reg_tx_data.highest_alarm.temp_threshold;

                alarm &= (threshold_low >= _temp_data->min_info.temp) << 0 |
                         (threshold_low >= _temp_data->med_temp) << 1 |
                         (threshold_low >= _temp_data->avg_temp) << 2 |
                         (threshold_low >= _temp_data->max_info.temp) << 3 |
                         (threshold_high <= _temp_data->min_info.temp) << 4 |
                         (threshold_high <= _temp_data->med_temp) << 5 |
                         (threshold_high <= _temp_data->avg_temp) << 6 |
                         (threshold_high <= _temp_data->max_info.temp) << 7;

                bool high_temp = alarm & 0xF0;
                bool low_temp  = alarm & 0x0F;

                if (high_temp && low_temp) {
                    alarm_state = (_current_alarm_state == alarm_high_temp1)
                                      ? alarm_low_temp1
                                      : alarm_high_temp1;
                } else if (high_temp) {
                    alarm_state = (_current_alarm_state == alarm_high_temp1)
                                      ? alarm_high_temp2
                                      : alarm_high_temp1;
                } else if (low_temp) {
                    alarm_state = (_current_alarm_state == alarm_low_temp1)
                                      ? alarm_low_temp2
                                      : alarm_low_temp1;
                }
            }

            _reg_tx_data.status.temp_alarm = alarm;

            if (_current_alarm_state != alarm_state) {
                _current_alarm_state  = alarm_state;
                uint16_t buzzer_freq  = 0;
                uint8_t buzzer_volume = 0;

                unit_thermal2_reg_t::rgb_t led          = {0, 0, 0};
                unit_thermal2_reg_t::alarm_reg_t* alarm = nullptr;
                switch (alarm_state) {
                    default:

                        if (getbit(_reg_rx_data.config.function_ctrl, 0)) {
                            buzzer_freq   = _reg_rx_data.config.buzzer_freq;
                            buzzer_volume = _reg_rx_data.config.buzzer_volume;
                        }
                        if (getbit(_reg_rx_data.config.function_ctrl, 1)) {
                            led = _reg_rx_data.config.led;
                        }
                        break;

                    case alarm_state_t::alarm_high_temp1:
                        alarm = &_reg_rx_data.highest_alarm;
                        break;

                    case alarm_state_t::alarm_low_temp1:
                        alarm = &_reg_rx_data.lowest_alarm;
                        break;

                    case alarm_state_t::alarm_high_temp2:
                    case alarm_state_t::alarm_low_temp2:
                        break;
                }
                if (alarm != nullptr) {
                    _alarm_interval = std::max(50, alarm->buzzer_interval * 10);

                    led = alarm->led;

                    buzzer_freq   = alarm->buzzer_freq;
                    buzzer_volume = _reg_rx_data.config.buzzer_volume;
                }
                setBuzzer(buzzer_freq, buzzer_volume);
                _led.setColor(led.r, led.g, led.b);
            }
        }
    }

    {
        // Update button status.
        uint8_t btn_status = _reg_tx_data.status.button;
        if (reg_mod) {
            uint8_t btn_clear = _reg_rx_data.status.button;
            if (btn_clear) {
                // Reflects instructions to clear button status.
                btn_status &= ~btn_clear;
                _reg_rx_data.status.button = 0;
            }
        }
        _btn.setRawState(msec, !gpio_in(PIN_BUTTON));
        if (_btn.wasChangePressed()) {
            setbit(btn_status, (_btn.isPressed() ? 1 : 2));
        }
        if (_btn.isReleased()) {
            clrbit(btn_status, 0);
            if (_btn.wasClicked()) {  // button was clicked
                setbit(btn_status, 3);
            }
        } else {
            setbit(btn_status, 0);
            if (_btn.wasHold()) {  // button was hold
                setbit(btn_status, 4);
            }
        }
        _reg_tx_data.status.button = btn_status;
    }
}

/// I2C STOP時などのデータの区切りの処理;
void IRAM_ATTR closeData(void) {
    _param_index      = 0;
    _param_need_count = 1;
    _param_resetindex = 0;
}

/// I2CペリフェラルISRから1Byteずつデータを受取る処理;
bool IRAM_ATTR addData(std::uint8_t value) {
    // if (_param_index == 0)
    // ESP_EARLY_LOGE(LOGNAME, "n:%d : v:%02x", _param_need_count, value);
    _params[_param_index] = value;

    if (++_param_index == 1) {
        _last_command     = value;
        _param_resetindex = 0;
        // ESP_EARLY_LOGE(LOGNAME, "CMD:%02x", value);
        switch (value) {
            default: {
                uint32_t regindex =
                    (value > 0x80) ? 0x80 + (value - 0x80) * 32 : value;
                _read_reg_index = regindex;
                if (regindex < UNIT_REGISTER_SIZE) {
                    _i2c_ex.setTxData(&_i2c_tx_data[regindex], 16);
                    _read_reg_index = regindex + 16;
                }
                _write_reg_index  = regindex;
                _last_command     = CMD_REG_ACCESS;
                _params[0]        = CMD_REG_ACCESS;
                _param_need_count = 2;
                _param_resetindex = 1;
                return false;
            }

            case CMD_UPDATE_END:
                _param_need_count = 4;
                return false;

            case CMD_UPDATE_BEGIN:
                _param_need_count = 8;
                return false;

            case CMD_UPDATE_DATA:
                _param_need_count = 8;
                return false;
        }
    }

    if (_param_index >= _param_need_count) {
        switch (_params[0]) {
            default:
                break;

            case CMD_REG_ACCESS:
                // Check if the value is different from
                // the current register value .
                if (_write_reg_index < REGISTER_WRITABLE_MAP_SIZE &&
                    _i2c_rx_data[_write_reg_index] != value) {
                    uint_fast8_t value = _params[1];

                    size_t map_idx  = _write_reg_index >> 3;
                    size_t map_mask = 0x80 >> (_write_reg_index & 7);

                    // Check if registers are rewritable.
                    if (REGISTER_WRITABLE_MAP[map_idx] & map_mask) {
                        _i2c_rx_data[_write_reg_index] = value;

                        if (REGISTER_DIRECT_WRITE_MAP[map_idx] & map_mask) {
                            _i2c_tx_data[_write_reg_index] = value;
                            if (_write_reg_index <
                                sizeof(unit_thermal2_reg_t)) {
                                _save_nvs_countdown = 16;
                            }
                        }
                        _reg_modified = true;
                    }
                }
                ++_write_reg_index;
                --_param_index;
                return _reg_modified;

            /// ファームウェアアップデートの準備コマンド;
            case CMD_UPDATE_BEGIN:
                if ((_params[1] == DEVICE_ID_0) &&
                    (_params[2] == DEVICE_ID_1) && (_params[0] == _params[3])) {
                    rtc_clk_cpu_freq_set_config_fast(&_cpu_freq_conf_160);
                    ESP_EARLY_LOGD(LOGNAME, "CMD_UPDATE_BEGIN");
                    _firmupdate_state = firmupdate_state_t::wait_data;
                    _firmupdate_index = 0;
                    /// 途中中断した時のためリード応答にはエラーステートを設定しておく;
                    _firmupdate_result    = UPDATE_RESULT_ERROR;
                    _firmupdate_totalsize = _params[4] << 24 |
                                            _params[5] << 16 | _params[6] << 8 |
                                            _params[7];
                    update::begin(_firmupdate_totalsize);
                } else {
                    closeData();
                    return false;
                }
                break;

            /// ファームウェアアップデートのデータコマンド;
            case CMD_UPDATE_DATA:
                if (_firmupdate_state == firmupdate_state_t::progress) {
                    /// 受信したデータをupdateに蓄積;
                    if (update::addData(_params[1])) {
                        _param_index = _param_resetindex;
                        return false;
                    }
                    ESP_EARLY_LOGD(LOGNAME,
                                   "CMD_UPDATE_DATA:progress success.");

                    if (update::checkCRC32()) {
                        ESP_EARLY_LOGD(LOGNAME,
                                       "CMD_UPDATE_DATA:checkCRC32:true");
                        _firmupdate_result = UPDATE_RESULT_BUSY;
                    } else {
                        ESP_EARLY_LOGD(LOGNAME,
                                       "CMD_UPDATE_DATA:checkCRC32:false");
                        _firmupdate_result = UPDATE_RESULT_BROKEN;
                    }
                    _i2c_ex.setTxData(_firmupdate_result, 9);
                    _firmupdate_state = sector_write;
                    closeData();
                } else if ((_params[1] == DEVICE_ID_0) &&
                           (_params[2] == DEVICE_ID_1) &&
                           (_params[0] == _params[3])) {
                    ESP_EARLY_LOGD(LOGNAME, "CMD_UPDATE_DATA:setBlockCRC32");
                    // 途中中断した時のためリード応答にはエラーステートを設定しておく;
                    _firmupdate_result = UPDATE_RESULT_ERROR;
                    _firmupdate_state  = firmupdate_state_t::progress;
                    _param_need_count  = 2;
                    _param_resetindex  = 1;
                    _param_index       = _param_resetindex;
                    update::setBlockCRC32(_params[4] << 24 | _params[5] << 16 |
                                          _params[6] << 8 | _params[7]);
                    return false;
                } else {
                    closeData();
                    return false;
                }
                break;
        }

        auto new_setpos   = (_rx_buffer_setpos + 1) & (RX_BUFFER_MAX - 1);
        _rx_buffer_setpos = new_setpos;
        _param_index      = _param_resetindex;
        for (std::size_t i = 0; i < _param_resetindex; ++i) {
            _rx_buffer[new_setpos][i] = _params[i];
        }
        _params = _rx_buffer[new_setpos];

        return true;
    }
    return false;
}

void IRAM_ATTR prepareTxData(void) {
    switch (_last_command) {
        case CMD_REG_ACCESS:
            if (_read_reg_index < UNIT_REGISTER_SIZE) {
                _i2c_ex.addTxData(&_i2c_tx_data[_read_reg_index], 8);
                _read_reg_index += 8;
                if (_read_reg_index >= UNIT_REGISTER_SIZE) {
                    _i2c_tx_data[REGINDEX_REFRESH_CTRL] &= ~1;
                    _reg_read_complete = true;
                }
            } else {
                for (std::size_t i = 0; i < 8; ++i) {
                    _i2c_ex.addTxData((_read_reg_index < UNIT_REGISTER_SIZE)
                                          ? _i2c_tx_data[_read_reg_index]
                                          : 0);
                    ++_read_reg_index;
                }
            }
            break;

        case CMD_UPDATE_BEGIN:
        case CMD_UPDATE_DATA:
            _i2c_ex.setTxData(_firmupdate_result, 9);
            break;

        default:
            _i2c_ex.addTxData(0xFF);
            break;
    }
}
}  // namespace command_processor