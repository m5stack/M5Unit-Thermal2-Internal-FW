//! Copyright (c) M5Stack. All rights reserved.
//! Licensed under the MIT license.
//! See LICENSE file in the project root for full license information.

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
class I2C_Slave {
   public:
    I2C_Slave(void){};
    bool init(int i2c_port, int pin_sda, int pin_scl, int i2c_addr);
    void release(void);

    void clearTxData(void);
    void addTxData(const std::uint8_t* buf, std::size_t len);
    void addTxData(std::uint8_t buf);
    void setTxData(const std::uint8_t* buf, std::size_t len);
    void setTxData(std::uint8_t buf, std::size_t len);

    __attribute__((always_inline)) inline i2c_port_t getI2CPort(void) const {
        return _i2c_port;
    }
    __attribute__((always_inline)) inline int getI2CAddr(void) const {
        return _i2c_addr;
    }
    __attribute__((always_inline)) inline int getPinSDA(void) const {
        return _pin_sda;
    }
    __attribute__((always_inline)) inline int getPinSCL(void) const {
        return _pin_scl;
    }
    __attribute__((always_inline)) inline void* getMainTaskHandle(void) const {
        return _mainTaskHandle;
    }

   private:
    i2c_port_t _i2c_port;
    int _i2c_addr;
    int _pin_sda;
    int _pin_scl;
    void* _mainTaskHandle;
};
}  // namespace m5