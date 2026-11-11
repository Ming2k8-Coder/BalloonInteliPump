#ifndef QUEUE_H
#define QUEUE_H

#include "FreeRTOS.h"

inline QueueHandle_t xQueueCreate(uint32_t len, uint32_t item_size) {
    (void)len; (void)item_size;
    static int dummy = 0;
    return (QueueHandle_t)&dummy;
}

#endif // QUEUE_H
