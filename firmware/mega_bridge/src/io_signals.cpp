#include "io_signals.h"

namespace io_signals {

void begin() {
    pinMode(PIN_RELAY, OUTPUT);
    pinMode(PIN_LED,   OUTPUT);
    pinMode(PIN_BTN,   INPUT_PULLUP);
    digitalWrite(PIN_RELAY, LOW);
    digitalWrite(PIN_LED,   LOW);
}

void setRelay(bool on)     { digitalWrite(PIN_RELAY, on ? HIGH : LOW); }
void setMarkerLed(bool on) { digitalWrite(PIN_LED,   on ? HIGH : LOW); }
bool readButton()          { return digitalRead(PIN_BTN) == LOW; }

}  // namespace io_signals
