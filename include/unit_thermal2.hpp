#ifndef M5UNIT_THERMAL2_REG_HPP__
#define M5UNIT_THERMAL2_REG_HPP__

#include <cstdint>

#pragma pack(push)
#pragma pack(1)

struct unit_thermal2_reg_t {
    struct rgb_t {
        uint8_t r;
        uint8_t g;
        uint8_t b;
        inline bool operator==(const rgb_t& rhs) {
            return r == rhs.r && g == rhs.g && b == rhs.b;
        }
        inline bool operator!=(const rgb_t& rhs) {
            return !(*this == rhs);
        }
    };

    enum refresh_rate_t : uint8_t {
        rate_0_5Hz,
        rate_1Hz,
        rate_2Hz,
        rate_4Hz,
        rate_8Hz,
        rate_16Hz,
        rate_32Hz,
        rate_64Hz,
    };

    struct alarm_reg_t {
        uint16_t temp_threshold;
        uint16_t buzzer_freq;
        uint8_t buzzer_interval;
        rgb_t led;
    };

    struct status_reg_t {
        uint8_t button;
        uint8_t temp_alarm;
        uint16_t reserved_0x02;
        uint8_t device_id_0;
        uint8_t device_id_1;
        uint8_t version_major;
        uint8_t version_minor;
    };

    struct config_reg_t {
        uint8_t i2c_addr;
        uint8_t i2c_addr_inv;
        uint8_t function_ctrl;
        refresh_rate_t refresh_rate;
        uint8_t noise_filter;
        uint8_t reserved_0x0D;
        uint16_t reserved_0x0E;

        uint8_t temp_alarm_area;
        uint8_t temp_alarm_enable;
        uint16_t buzzer_freq;
        uint8_t buzzer_volume;
        rgb_t led;

        inline bool operator==(const config_reg_t& rhs) {
            return memcmp(this, &rhs, sizeof(config_reg_t)) == 0;
        }
        inline bool operator!=(const config_reg_t& rhs) {
            return !(*this == rhs);
        }
    };

    // 0x00 ~ 0x07
    status_reg_t status;

    // 0x08 ~ 0x1F
    config_reg_t config;
    uint8_t reserved_0x18[8];

    // 0x20 = 0x2F
    alarm_reg_t lowest_alarm;
    uint8_t reserved_0x28[8];

    // 0x30 = 0x37
    alarm_reg_t highest_alarm;
};
#pragma pack(pop)

#endif