//! Copyright (c) M5Stack. All rights reserved.
//! Licensed under the MIT license.
//! See LICENSE file in the project root for full license information.

#include "command_processor.hpp"
#include <esp_log.h>

void setup(void) {
    command_processor::setup();
}

void loop(void) {
    command_processor::loop();
}

#if !defined(ARDUINO)

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern "C" {
void loopTask(void*) {
    setup();
    for (;;) {
        loop();
        taskYIELD();
    }
    vTaskDelete(NULL);
}

void app_main() {
    xTaskCreatePinnedToCore(loopTask, "loopTask", 8192, NULL, 1, NULL,
                            APP_CPU_NUM);
}
}
#endif
