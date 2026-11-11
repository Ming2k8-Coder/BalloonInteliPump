#include <stdint.h>

extern "C" {
    float read_mcu_temp() { return 25.0f; }
    uint32_t esp_get_free_heap_size() { return 120000; }
}
