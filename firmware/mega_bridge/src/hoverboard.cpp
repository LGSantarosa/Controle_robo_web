#include "hoverboard.h"
#include <string.h>

namespace hoverboard {

void sendCommand(Stream& port, int16_t steer, int16_t speed) {
    Command c;
    c.start    = START_FRAME;
    c.steer    = steer;
    c.speed    = speed;
    c.checksum = (uint16_t)(START_FRAME ^ (uint16_t)steer ^ (uint16_t)speed);
    port.write(reinterpret_cast<const uint8_t*>(&c), sizeof(c));
}

bool FeedbackParser::feed(uint8_t b) {
    if (state_ == 0) {
        shifter_ = (uint16_t)((shifter_ >> 8) | ((uint16_t)b << 8));
        if (shifter_ == START_FRAME) {
            buf_[0] = (uint8_t)(START_FRAME & 0xFF);
            buf_[1] = (uint8_t)(START_FRAME >> 8);
            got_    = 2;
            state_  = 1;
        }
        return false;
    }
    buf_[got_++] = b;
    if (got_ >= FEEDBACK_SIZE) {
        state_   = 0;
        shifter_ = 0;
        Feedback f;
        memcpy(&f, buf_, sizeof(f));
        uint16_t expected = (uint16_t)(
            START_FRAME
            ^ (uint16_t)f.cmd1
            ^ (uint16_t)f.cmd2
            ^ (uint16_t)f.speedR_meas
            ^ (uint16_t)f.speedL_meas
            ^ (uint16_t)f.batVoltage
            ^ (uint16_t)f.boardTemp
            ^ f.cmdLed);
        if (expected == f.checksum) {
            last_ = f;
            return true;
        }
    }
    return false;
}

}  // namespace hoverboard
