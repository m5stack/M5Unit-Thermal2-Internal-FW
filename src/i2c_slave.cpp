//! Copyright (c) M5Stack. All rights reserved.
//! Licensed under the MIT license.
//! See LICENSE file in the project root for full license information.

#include "i2c_slave.hpp"
#include "command_processor.hpp"

#include <sdkconfig.h>
#include <driver/i2c.h>
#include <driver/periph_ctrl.h>
#include <soc/i2c_struct.h>
#include <soc/i2c_reg.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstdint>

#if __has_include(<soc/i2c_periph.h>)
#include <soc/i2c_periph.h>
#endif

namespace m5 {
// 受信FIFOバッファがこの数を上回ると割込みが発生する
static constexpr std::uint32_t i2c_rx_fifo_full_thresh_val = 1;
// 送信FIFOバッファがこの数を下回ると割込みが発生する
static constexpr std::uint32_t i2c_tx_fifo_empty_thresh_val = 8;
// SCL立上り後のSDAサンプル時間
static constexpr std::uint32_t i2c_slave_sda_sample_val = 4;
// SCL立下り後のSDAホールド時間
static constexpr std::uint32_t i2c_slave_sda_hold_val = 4;
static constexpr std::uint32_t i2c_intr_mask          = ~0;

#if __has_include(<soc/i2c_periph.h>)
static inline periph_module_t getPeriphModule(i2c_port_t num) {
    return i2c_periph_signal[num].module;
}
static inline std::uint8_t getPeriphIntSource(i2c_port_t num) {
    return i2c_periph_signal[num].irq;
}
#else
static inline periph_module_t getPeriphModule(i2c_port_t num) {
    return num == 0 ? PERIPH_I2C0_MODULE : PERIPH_I2C1_MODULE;
}
static inline std::uint8_t getPeriphIntSource(i2c_port_t num) {
    return num == 0 ? ETS_I2C_EXT0_INTR_SOURCE : ETS_I2C_EXT1_INTR_SOURCE;
}
#endif

#if defined(CONFIG_IDF_TARGET_ESP32C3)

static inline i2c_dev_t* IRAM_ATTR getDev(i2c_port_t num) {
    return &I2C0;
}
static inline std::uint32_t IRAM_ATTR getRxFifoCount(i2c_dev_t* dev) {
    return dev->sr.rx_fifo_cnt;
}
static inline std::uint32_t IRAM_ATTR getTxFifoCount(i2c_dev_t* dev) {
    return dev->sr.tx_fifo_cnt;
}
static inline void IRAM_ATTR updateDev(i2c_dev_t* dev) {
    dev->ctr.conf_upgate = 1;
}

#else

static __attribute__((always_inline)) inline i2c_dev_t* IRAM_ATTR
getDev(i2c_port_t num) {
    return num == 0 ? &I2C0 : &I2C1;
}
static __attribute__((always_inline)) inline std::uint32_t IRAM_ATTR
getRxFifoCount(i2c_dev_t* dev) {
    return dev->status_reg.rx_fifo_cnt;
}
static __attribute__((always_inline)) inline std::uint32_t IRAM_ATTR
getTxFifoCount(i2c_dev_t* dev) {
    return dev->status_reg.tx_fifo_cnt;
}
static __attribute__((always_inline)) inline void updateDev(i2c_dev_t* dev) {
}

#endif

static __attribute__((always_inline)) inline std::uintptr_t IRAM_ATTR
getFifoAddr(i2c_dev_t* dev) {
    return (std::uintptr_t) & (dev->fifo_data);
}

static __attribute__((always_inline)) inline void IRAM_ATTR
clear_txdata(i2c_dev_t* dev) {
    dev->fifo_conf.tx_fifo_rst = 1;
    dev->fifo_conf.tx_fifo_rst = 0;
}

void __attribute__((always_inline)) inline IRAM_ATTR add_txdata(
    i2c_dev_t* dev, const std::uint8_t* buf, std::size_t len) {
    uint32_t fifo_addr = getFifoAddr(dev);
    do {
        WRITE_PERI_REG(fifo_addr, *buf++);
    } while (--len);
}

void __attribute__((always_inline)) inline IRAM_ATTR add_txdata(
    i2c_dev_t* dev, std::uint8_t buf) {
    uint32_t fifo_addr = getFifoAddr(dev);
    WRITE_PERI_REG(fifo_addr, buf);
}

void __attribute__((always_inline)) inline IRAM_ATTR set_txdata(
    i2c_dev_t* dev, const std::uint8_t* buf, std::size_t len) {
    dev->fifo_conf.tx_fifo_rst = 1;
    dev->fifo_conf.tx_fifo_rst = 0;

    uint32_t fifo_addr = getFifoAddr(dev);
    do {
        WRITE_PERI_REG(fifo_addr, *buf++);
    } while (--len);
}

void __attribute__((always_inline)) inline IRAM_ATTR set_txdata(
    i2c_dev_t* dev, std::uint8_t buf, std::size_t len) {
    dev->fifo_conf.tx_fifo_rst = 1;
    dev->fifo_conf.tx_fifo_rst = 0;

    uint32_t fifo_addr = getFifoAddr(dev);
    do {
        WRITE_PERI_REG(fifo_addr, buf);
    } while (--len);
}

/// I2Cイベントハンドラ
static void IRAM_ATTR i2c_isr_handler(void* arg) {
    auto p_i2c  = (I2C_Slave*)arg;
    auto dev    = getDev(p_i2c->getI2CPort());
    bool notify = false;
    do {
        typeof(dev->int_status) int_sts;
        int_sts.val      = dev->int_status.val;
        dev->int_clr.val = int_sts.val;

        std::uint32_t rx_fifo_cnt = getRxFifoCount(dev);
        if (rx_fifo_cnt) {
            do {
                if (command_processor::addData(dev->fifo_data.val)) {
                    notify = true;
                }
            } while (--rx_fifo_cnt);
        }
        if (int_sts.trans_complete || int_sts.arbitration_lost) {
            command_processor::closeData();
        }
        while (getTxFifoCount(dev) <= i2c_tx_fifo_empty_thresh_val) {
            command_processor::prepareTxData();
        }
    } while (dev->int_status.val);
    if (notify) {
        BaseType_t xHigherPriorityTaskWoken = pdTRUE;
        vTaskNotifyGiveFromISR(p_i2c->getMainTaskHandle(),
                               &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR();
    }
}

/// I2C slave operation start processing.
void setupTask(void* args) {
    auto i2c_slave = (I2C_Slave*)args;
    esp_intr_alloc(getPeriphIntSource(i2c_slave->getI2CPort()),
                   ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3, i2c_isr_handler,
                   i2c_slave, nullptr);
    vTaskDelete(nullptr);
}

bool I2C_Slave::init(int i2c_port, int pin_sda, int pin_scl, int i2c_addr) {
    _i2c_port = (i2c_port_t)i2c_port;

    auto dev         = getDev(i2c_port);
    dev->int_ena.val = 0;
    dev->int_clr.val = ~0u;

    /// I2C peripheral setup in core0.
    /// (※ Running setup on core1 causes ISR to run on core1.)
    xTaskCreatePinnedToCore(setupTask, "setupTask", 4096, this, 21, nullptr,
                            PRO_CPU_NUM);

    _i2c_addr       = i2c_addr;
    _pin_sda        = pin_sda;
    _pin_scl        = pin_scl;
    _mainTaskHandle = xTaskGetCurrentTaskHandle();

    periph_module_enable(getPeriphModule(i2c_port));

    typeof(dev->ctr) ctrl_reg;
    ctrl_reg.val           = 0;
    ctrl_reg.sda_force_out = 1;
    ctrl_reg.scl_force_out = 1;
    dev->ctr.val           = ctrl_reg.val;

    dev->slave_addr.val = i2c_addr;

    dev->sda_hold.time   = i2c_slave_sda_hold_val;
    dev->sda_sample.time = i2c_slave_sda_sample_val;

#if defined(CONFIG_IDF_TARGET_ESP32C3)

    dev->ctr.slv_tx_auto_start_en = 1;

    dev->timeout.time_out_value = 31;
    dev->timeout.time_out_en    = 0;

    dev->filter_cfg.val       = 0;
    dev->filter_cfg.scl_en    = 1;
    dev->filter_cfg.scl_thres = 0;
    dev->filter_cfg.sda_en    = 1;
    dev->filter_cfg.sda_thres = 0;

#else

    dev->timeout.tout = 0xFFFFF;

    dev->scl_filter_cfg.en    = 1;
    dev->scl_filter_cfg.thres = 0;
    dev->sda_filter_cfg.en    = 1;
    dev->sda_filter_cfg.thres = 0;

#endif

#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32S2)

    typeof(dev->fifo_conf) fifo_conf;
    fifo_conf.val              = 0;
    fifo_conf.rx_fifo_wm_thrhd = i2c_rx_fifo_full_thresh_val;
    fifo_conf.tx_fifo_wm_thrhd = i2c_tx_fifo_empty_thresh_val;
    dev->fifo_conf.val         = fifo_conf.val;

    dev->int_ena.val = I2C_TRANS_COMPLETE_INT_ENA |
                       I2C_ARBITRATION_LOST_INT_ENA |
                       I2C_BYTE_TRANS_DONE_INT_ENA | I2C_TXFIFO_WM_INT_ENA |
                       I2C_RXFIFO_WM_INT_ENA;

#else

    typeof(dev->fifo_conf) fifo_conf;
    fifo_conf.val                 = 0;
    fifo_conf.rx_fifo_full_thrhd  = i2c_rx_fifo_full_thresh_val;
    fifo_conf.tx_fifo_empty_thrhd = i2c_tx_fifo_empty_thresh_val;
    dev->fifo_conf.val            = fifo_conf.val;

    dev->int_ena.val = I2C_TRANS_COMPLETE_INT_ENA | I2C_TRANS_START_INT_ENA |
                       I2C_ARBITRATION_LOST_INT_ENA |
                       I2C_SLAVE_TRAN_COMP_INT_ENA | I2C_TXFIFO_EMPTY_INT_ENA |
                       I2C_RXFIFO_FULL_INT_ENA;

#endif

    updateDev(dev);

    return ((ESP_OK == i2c_set_pin((i2c_port_t)i2c_port, pin_sda, pin_scl,
                                   GPIO_PULLUP_DISABLE, GPIO_PULLUP_DISABLE,
                                   I2C_MODE_SLAVE)));
}

void I2C_Slave::release(void) {
}

void IRAM_ATTR I2C_Slave::clearTxData(void) {
    clear_txdata(getDev(_i2c_port));
}

void IRAM_ATTR I2C_Slave::addTxData(const std::uint8_t* buf, std::size_t len) {
    add_txdata(getDev(_i2c_port), buf, len);
}

void IRAM_ATTR I2C_Slave::addTxData(std::uint8_t buf) {
    add_txdata(getDev(_i2c_port), buf);
}

void IRAM_ATTR I2C_Slave::setTxData(const std::uint8_t* buf, std::size_t len) {
    set_txdata(getDev(_i2c_port), buf, len);
}

void IRAM_ATTR I2C_Slave::setTxData(std::uint8_t buf, std::size_t len) {
    set_txdata(getDev(_i2c_port), buf, len);
}

}  // namespace m5