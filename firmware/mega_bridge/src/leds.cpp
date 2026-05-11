#include "leds.h"

namespace leds {

void Ring::begin() {
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds_, NUM_LEDS);
    FastLED.clear(true);
}

void Ring::setColor(uint8_t r, uint8_t g, uint8_t b) {
    r_ = r;
    g_ = g;
    b_ = b;
    if (mode_ == 0) {
        for (auto& px : leds_) px = CRGB(r_, g_, b_);
        FastLED.show();
    }
}

void Ring::setMode(uint8_t mode) {
    mode_ = mode;
    if (mode_ == 0) {
        for (auto& px : leds_) px = CRGB(r_, g_, b_);
        FastLED.show();
    }
}

void Ring::tick() {
    if (mode_ == 0) return;
    const uint32_t now = millis();
    if (now - last_ < 50) return;
    last_ = now;
    if (mode_ == 1) {
        const bool on = (phase_++ & 0x08) == 0;
        for (auto& px : leds_) px = on ? CRGB(r_, g_, b_) : CRGB::Black;
    } else if (mode_ == 2) {
        for (uint8_t i = 0; i < NUM_LEDS; ++i) {
            leds_[i] = (i == phase_ % NUM_LEDS) ? CRGB(r_, g_, b_) : CRGB::Black;
        }
        phase_++;
    }
    FastLED.show();
}

}  // namespace leds
