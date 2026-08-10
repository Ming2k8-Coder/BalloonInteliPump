#ifndef INPUT_H
#define INPUT_H

#include <Arduino.h>
#include "Config.h"

enum InputEvent {
    EVENT_NONE = 0,
    EVENT_UP,
    EVENT_DOWN,
    EVENT_SELECT, // Enter / Rotary Click
    EVENT_BACK,   // Calibration / Exit
    EVENT_START,  // Key 'A'
    EVENT_STOP,   // Key 'A' (Stop)
    EVENT_MENU_MANUAL, // 'B'
    EVENT_TARE,        // 'C'
    EVENT_MENU_CONFIG, // 'D'
    EVENT_NUM_0, EVENT_NUM_1, EVENT_NUM_2, EVENT_NUM_3, EVENT_NUM_4,
    EVENT_NUM_5, EVENT_NUM_6, EVENT_NUM_7, EVENT_NUM_8, EVENT_NUM_9,
    EVENT_DOT,    // '*'
    EVENT_DELETE  // '#'
};

class Input {
public:
    Input();
    void begin();
    void update();
    InputEvent getEvent(); // Consumes event
    bool hasEvent();

private:
    InputEvent _queue;
    unsigned long _lastDebounce;
    
    // Keypad Globals
#ifndef USE_ROTARY_ENCODER
    char scanKeypad();
    char _lastChar;
#endif

    // Rotary Globals
#ifdef USE_ROTARY_ENCODER
    int _lastClk;
    bool _btnState;
#endif
};

extern Input userInput;

#endif // INPUT_H
