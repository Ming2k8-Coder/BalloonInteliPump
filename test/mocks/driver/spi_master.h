#ifndef DRIVER_SPI_MASTER_H
#define DRIVER_SPI_MASTER_H

#include "esp_err.h"
#include <stddef.h>

typedef void* spi_device_handle_t;

typedef struct {
    uint32_t flags;
    size_t length;
    const void *tx_buffer;
    void *rx_buffer;
} spi_transaction_t;

inline esp_err_t spi_device_polling_transmit(spi_device_handle_t handle, spi_transaction_t *trans) {
    (void)handle; (void)trans;
    return ESP_OK;
}

#endif // DRIVER_SPI_MASTER_H
