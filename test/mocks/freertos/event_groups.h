#ifndef EVENT_GROUPS_H
#define EVENT_GROUPS_H

#include <stdint.h>
#include "FreeRTOS.h"

typedef void* EventGroupHandle_t;
typedef uint32_t EventBits_t;

inline EventGroupHandle_t xEventGroupCreate() {
    static uint32_t s_event_group = 0;
    return (EventGroupHandle_t)&s_event_group;
}

inline EventBits_t xEventGroupSetBits(EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToSet) {
    if (xEventGroup) {
        *(uint32_t*)xEventGroup |= uxBitsToSet;
        return *(uint32_t*)xEventGroup;
    }
    return 0;
}

inline EventBits_t xEventGroupClearBits(EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToClear) {
    if (xEventGroup) {
        EventBits_t orig = *(uint32_t*)xEventGroup;
        *(uint32_t*)xEventGroup &= ~uxBitsToClear;
        return orig;
    }
    return 0;
}

inline EventBits_t xEventGroupGetBits(EventGroupHandle_t xEventGroup) {
    if (xEventGroup) {
        return *(uint32_t*)xEventGroup;
    }
    return 0;
}

inline EventBits_t xEventGroupWaitBits(EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToWaitFor,
                                       const int xClearOnExit, const int xWaitForAllBits, TickType_t xTicksToWait) {
    (void)xClearOnExit; (void)xWaitForAllBits; (void)xTicksToWait;
    if (xEventGroup) return *(uint32_t*)xEventGroup;
    return 0;
}

#endif // EVENT_GROUPS_H
