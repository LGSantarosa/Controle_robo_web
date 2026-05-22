#pragma once
#include <SPI.h>
#include <Bitcraze_PMW3901.h>

namespace sensors {

class Flow {
 public:
    explicit Flow(uint8_t cs_pin) : pmw_(cs_pin) {}
    bool begin();
    bool read();
    int16_t dx()      const { return dx_; }
    int16_t dy()      const { return dy_; }
    uint8_t quality() const { return quality_; }
    bool    ok()      const { return ok_; }

 private:
    Bitcraze_PMW3901 pmw_;
    bool     ok_                = false;
    int16_t  dx_                = 0;
    int16_t  dy_                = 0;
    uint8_t  quality_           = 0;
    uint32_t last_recover_ms_   = 0;
};

}  // namespace sensors
