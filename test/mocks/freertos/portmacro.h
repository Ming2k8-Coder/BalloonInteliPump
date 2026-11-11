#ifndef PORTMACRO_H
#define PORTMACRO_H

#include <stdint.h>

typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(mux) ((void)(mux))
#define portEXIT_CRITICAL(mux) ((void)(mux))
#define pdMS_TO_TICKS(ms) ((uint32_t)(ms))
#define pdTRUE 1
#define pdFALSE 0
typedef uint32_t TickType_t;
#define portMAX_DELAY 0xFFFFFFFFUL

#endif // PORTMACRO_H
