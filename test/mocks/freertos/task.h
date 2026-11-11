#ifndef TASK_H
#define TASK_H

#include "FreeRTOS.h"

inline void vTaskDelay(TickType_t ticks) {
    (void)ticks;
}

#endif // TASK_H
