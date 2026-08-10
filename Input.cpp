#include "Input.h"

Input userInput;

// Rotary Vars
#ifdef USE_ROTARY_ENCODER
    volatile int encoderCount = 0;
    volatile unsigned long lastIsrTime = 0;

    void IRAM_ATTR isrEncoder() {
        unsigned long now = millis();
        if (now - lastIsrTime < 5) return; // Debounce
        lastIsrTime = now;
        
        int clk = digitalRead(PIN_ENC_CLK);
        int dt = digitalRead(PIN_ENC_DT);
        
        if (clk == dt) {
            encoderCount++;
        } else {
            encoderCount--;
        }
    }
#endif

Input::Input() {
    _queue = EVENT_NONE;
    _lastDebounce = 0;
}

void Input::begin() {
#ifdef USE_ROTARY_ENCODER
    pinMode(PIN_ENC_CLK, INPUT_PULLUP);
    pinMode(PIN_ENC_DT, INPUT_PULLUP);
    pinMode(PIN_ENC_SW, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), isrEncoder, CHANGE);
#else
    pinMode(PIN_KEY_R1, OUTPUT); digitalWrite(PIN_KEY_R1, HIGH);
    pinMode(PIN_KEY_R2, OUTPUT); digitalWrite(PIN_KEY_R2, HIGH);
    pinMode(PIN_KEY_R3, OUTPUT); digitalWrite(PIN_KEY_R3, HIGH);
    pinMode(PIN_KEY_R4, OUTPUT); digitalWrite(PIN_KEY_R4, HIGH);
    
    pinMode(PIN_KEY_C1, INPUT_PULLUP);
    pinMode(PIN_KEY_C2, INPUT_PULLUP);
    pinMode(PIN_KEY_C3, INPUT_PULLUP);
    pinMode(PIN_KEY_C4, INPUT_PULLUP);
    _lastChar = 0;
#endif
}

void Input::update() {
    if (_queue != EVENT_NONE) return; // Wait for consumption

    unsigned long now = millis();
    if (now - _lastDebounce < 50) return;

#ifdef USE_ROTARY_ENCODER
    // Handle Rotation
    noInterrupts();
    int count = encoderCount;
    encoderCount = 0;
    interrupts();
    
    if (count > 0) {
        _queue = EVENT_UP; // Or EVENT_NUM_X in data entry? Context matters.
        _lastDebounce = now;
    } else if (count < 0) {
        _queue = EVENT_DOWN;
        _lastDebounce = now;
    }
    
    // Handle Button
    if (digitalRead(PIN_ENC_SW) == LOW) {
        _queue = EVENT_SELECT;
        _lastDebounce = now + 200; // Long debounce for click
    }
    
    // Rotary is limited for full text entry. 
    // We assume mostly navigation. 
    // Complex mapping (Hold for Menu?) needed for feature parity.
    
#else
    // Keypad Scan
    char key = scanKeypad();
    if (key != 0 && key != _lastChar) {
        _lastChar = key;
        _lastDebounce = now;
        
        // Map Char to Event
        switch(key) {
            case 'A': _queue = EVENT_START; break;
            case 'B': _queue = EVENT_MENU_MANUAL; break;
            case 'C': _queue = EVENT_TARE; break;
            case 'D': _queue = EVENT_MENU_CONFIG; break;
            case '2': _queue = EVENT_UP; break;   // Manual Nav
            case '8': _queue = EVENT_DOWN; break; // Manual Nav
            case '5': _queue = EVENT_SELECT; break;
            case '*': _queue = EVENT_DOT; break;
            case '#': _queue = EVENT_DELETE; break;
            default:
                if (key >= '0' && key <= '9') {
                    // This is ugly but works for manual casting to proper event
                    _queue = (InputEvent)(EVENT_NUM_0 + (key - '0')); 
                }
                break;
        }
    } else if (key == 0) {
        _lastChar = 0;
    }
#endif
}

InputEvent Input::getEvent() {
    InputEvent e = _queue;
    _queue = EVENT_NONE;
    return e;
}

bool Input::hasEvent() {
    return _queue != EVENT_NONE;
}

#ifndef USE_ROTARY_ENCODER
char Input::scanKeypad() {
    byte rowPins[4] = {PIN_KEY_R1, PIN_KEY_R2, PIN_KEY_R3, PIN_KEY_R4};
    byte colPins[4] = {PIN_KEY_C1, PIN_KEY_C2, PIN_KEY_C3, PIN_KEY_C4};
    char keys[4][4] = {
        {'1','2','3','A'},
        {'4','5','6','B'},
        {'7','8','9','C'},
        {'*','0','#','D'}
    };
    
    for (int r=0; r<4; r++) {
        digitalWrite(rowPins[r], LOW);
        for (int c=0; c<4; c++) {
            if (digitalRead(colPins[c]) == LOW) {
                digitalWrite(rowPins[r], HIGH);
                return keys[r][c];
            }
        }
        digitalWrite(rowPins[r], HIGH);
    }
    return 0;
}
#endif
