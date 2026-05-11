#pragma once
#include <FastLED.h>

namespace leds {

constexpr uint8_t DATA_PIN = 6;
constexpr uint8_t NUM_LEDS = 24;

class Ring {
 public:
    void begin();
    void setColor(uint8_t r, uint8_t g, uint8_t b);
    void setMode(uint8_t mode);            // 0=fixo, 1=pisca, 2=rotação
    void tick();                           // chamar no loop pra animar

 private:
    CRGB     leds_[NUM_LEDS]{};
    uint8_t  mode_  = 0;
    uint8_t  r_     = 0;
    uint8_t  g_     = 0;
    uint8_t  b_     = 0;
    uint32_t last_  = 0;
    uint8_t  phase_ = 0;
};

}  // namespace leds
