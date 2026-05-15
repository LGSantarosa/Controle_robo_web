#include "protocol.h"

namespace protocol {

uint8_t computeChecksum(uint8_t type, uint8_t len, const uint8_t* payload) {
    uint8_t x = type ^ len;
    for (uint8_t i = 0; i < len; ++i) x ^= payload[i];
    return x;
}

void writeFrame(Stream& port, uint8_t type, const uint8_t* payload, uint8_t len) {
    uint8_t chk = computeChecksum(type, len, payload);
    port.write(START0);
    port.write(START1);
    port.write(type);
    port.write(len);
    if (len) port.write(payload, len);
    port.write(chk);
}

bool Decoder::feed(uint8_t b) {
    switch (st_) {
        case State::S0:
            if (b == START0) st_ = State::S1;
            return false;
        case State::S1:
            // Resync: depois de um 0xAA, aceita 0x55 (header completo) OU
            // outro 0xAA (header novo começando — mantém em S1).
            // Sem isso, a sequência 0xAA 0xAA 0x55 perde o frame por ir pra S0.
            if (b == START1)      st_ = State::TYPE;
            else if (b == START0) st_ = State::S1;
            else                  st_ = State::S0;
            return false;
        case State::TYPE:
            type_ = b;
            st_   = State::LEN;
            return false;
        case State::LEN:
            len_ = b;
            got_ = 0;
            if (len_ > MAX_PAYLOAD) { st_ = State::S0; return false; }
            st_  = (len_ == 0) ? State::CHECK : State::PAYLOAD;
            return false;
        case State::PAYLOAD:
            buf_[got_++] = b;
            if (got_ >= len_) st_ = State::CHECK;
            return false;
        case State::CHECK: {
            const uint8_t expected = computeChecksum(type_, len_, buf_);
            st_ = State::S0;
            return b == expected;
        }
    }
    return false;
}

}  // namespace protocol
