#ifndef ESP_TIMER_H
#define ESP_TIMER_H

#include <stdint.h>
#include <chrono>

inline int64_t esp_timer_get_time() {
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();
}

#endif // ESP_TIMER_H
