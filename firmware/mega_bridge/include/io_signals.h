#pragma once
#include <Arduino.h>

namespace io_signals {

constexpr uint8_t PIN_RELAY = 7;
constexpr uint8_t PIN_LED   = 8;
constexpr uint8_t PIN_BTN   = 9;

void begin();
void setRelay(bool on);
void setMarkerLed(bool on);
bool readButton();

}  // namespace io_signals
