#ifndef NVS_H
#define NVS_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

typedef uint32_t nvs_handle_t;
typedef enum {
    NVS_READONLY,
    NVS_READWRITE
} nvs_open_mode_t;

inline esp_err_t nvs_open(const char* name, nvs_open_mode_t open_mode, nvs_handle_t *out_handle) {
    (void)name; (void)open_mode;
    if (out_handle) *out_handle = 1;
    return ESP_OK;
}

inline esp_err_t nvs_set_blob(nvs_handle_t handle, const char* key, const void* value, size_t length) {
    (void)handle; (void)key; (void)value; (void)length;
    return ESP_OK;
}

inline esp_err_t nvs_get_blob(nvs_handle_t handle, const char* key, void* out_value, size_t* length) {
    (void)handle; (void)key; (void)out_value; (void)length;
    return ESP_OK;
}

inline esp_err_t nvs_commit(nvs_handle_t handle) {
    (void)handle;
    return ESP_OK;
}

inline void nvs_close(nvs_handle_t handle) {
    (void)handle;
}

#endif // NVS_H
