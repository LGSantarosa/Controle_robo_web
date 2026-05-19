#pragma once
#include <Arduino.h>

namespace protocol {

constexpr uint8_t START0       = 0xAA;
constexpr uint8_t START1       = 0x55;
constexpr uint8_t MAX_PAYLOAD  = 64;

constexpr uint8_t FT_SET_SPEED = 0x01;
constexpr uint8_t FT_LEDS      = 0x02;
constexpr uint8_t FT_RELAY     = 0x03;

constexpr uint8_t FT_STATE     = 0x81;
constexpr uint8_t FT_IMU       = 0x82;
constexpr uint8_t FT_FLOW      = 0x83;

uint8_t computeChecksum(uint8_t type, uint8_t len, const uint8_t* payload);
void    writeFrame(Stream& port, uint8_t type, const uint8_t* payload, uint8_t len);

class Decoder {
 public:
    bool feed(uint8_t b);
    uint8_t type()    const { return type_; }
    uint8_t len()     const { return len_; }
    const uint8_t* payload() const { return buf_; }

 private:
    enum class State : uint8_t { S0, S1, TYPE, LEN, PAYLOAD, CHECK };
    State   st_   = State::S0;
    uint8_t type_ = 0;
    uint8_t len_  = 0;
    uint8_t got_  = 0;
    uint8_t buf_[MAX_PAYLOAD] = {0};
};

}  // namespace protocol
