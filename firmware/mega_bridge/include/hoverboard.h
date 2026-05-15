#pragma once
#include <Arduino.h>

namespace hoverboard {

constexpr uint16_t START_FRAME   = 0xABCD;
constexpr uint8_t  FEEDBACK_SIZE = 18;     // sizeof(Feedback)

#pragma pack(push, 1)
struct Command {
    uint16_t start;
    int16_t  steer;
    int16_t  speed;
    uint16_t checksum;
};

struct Feedback {
    uint16_t start;
    int16_t  cmd1;
    int16_t  cmd2;
    int16_t  speedR_meas;
    int16_t  speedL_meas;
    int16_t  batVoltage;
    int16_t  boardTemp;
    uint16_t cmdLed;
    uint16_t checksum;
};
#pragma pack(pop)

void sendCommand(Stream& port, int16_t steer, int16_t speed);

class FeedbackParser {
 public:
    bool feed(uint8_t b);
    const Feedback& last() const { return last_; }
    uint32_t last_recv_ms() const { return last_recv_ms_; }
    bool stale(uint32_t now_ms, uint32_t timeout_ms = 200) const {
        return last_recv_ms_ == 0 || (now_ms - last_recv_ms_) > timeout_ms;
    }

 private:
    uint16_t shifter_ = 0;
    uint8_t  state_   = 0;
    uint8_t  buf_[FEEDBACK_SIZE] = {0};
    uint8_t  got_     = 0;
    Feedback last_{};
    uint32_t last_recv_ms_ = 0;
};

}  // namespace hoverboard
