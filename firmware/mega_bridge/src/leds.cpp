#include "leds.h"

namespace leds {

void Ring::begin() {
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds_, NUM_LEDS);
    // 24 LEDs WS2812 a 255 = ~1.44 A no 5 V — acima do orçamento do USB da
    // MEGA (500 mA), risco de brown-out durante o modo RUN (branco cheio).
    // 80 dá brilho útil pro PMW3901 enxergar o chão sem estressar a fonte.
    FastLED.setBrightness(80);
    FastLED.clear(true);
    transition_(State::BOOT);
}

void Ring::transition_(State s) {
    state_       = s;
    state_start_ = millis();
}

void Ring::resolveAuto_() {
    if (wish_error_)        transition_(State::ERROR);
    else if (wish_active_)  transition_(State::RUN);
    else                    transition_(State::IDLE);
}

void Ring::setActive(bool active) {
    wish_active_ = active;
    if (state_ == State::ERROR || state_ == State::MANUAL) return;
    // BOOT, STARTING e WAYPOINT terminam sozinhos via resolveAuto_(); só guardamos a vontade.
    if (state_ == State::BOOT || state_ == State::STARTING || state_ == State::WAYPOINT) return;
    if (active && state_ == State::IDLE) {
        transition_(State::STARTING);
    } else if (!active && state_ == State::RUN) {
        transition_(State::IDLE);
    }
}

void Ring::setError(bool err) {
    wish_error_ = err;
    if (state_ == State::MANUAL) return;
    if (err && state_ != State::ERROR) {
        transition_(State::ERROR);
    } else if (!err && state_ == State::ERROR) {
        resolveAuto_();
    }
}

void Ring::triggerWaypoint() {
    if (state_ == State::ERROR || state_ == State::MANUAL) return;
    transition_(State::WAYPOINT);
}

void Ring::setManual(uint8_t r, uint8_t g, uint8_t b, uint8_t pattern) {
    man_r_   = r;
    man_g_   = g;
    man_b_   = b;
    man_pat_ = pattern;
    transition_(State::MANUAL);
}

void Ring::clearManual() {
    if (state_ == State::MANUAL) resolveAuto_();
}

void Ring::setState(State s) {
    transition_(s);
}

void Ring::tick() {
    const uint32_t now = millis();
    if (now - last_tick_ < 16) return;  // ~60 Hz
    last_tick_ = now;
    const uint32_t t = now - state_start_;

    switch (state_) {
        case State::OFF:
            for (auto& px : leds_) px = CRGB::Black;
            break;

        case State::BOOT: {
            // Pulso branco fraco enquanto o hardware inicializa.
            const uint8_t v = (uint8_t)((sin8((t * 256UL) / 800) >> 2) + 20);
            for (auto& px : leds_) px = CRGB(v, v, v);
            if (t >= 800) {
                // No primeiro boot, se o PC já tá mandando setpoint, faz a
                // animação STARTING (verde → branco). Senão cai em IDLE.
                if (wish_error_)        transition_(State::ERROR);
                else if (wish_active_)  transition_(State::STARTING);
                else                    transition_(State::IDLE);
                return;
            }
            break;
        }

        case State::IDLE: {
            // Azul "coração batendo": dois pulsos em ~1300 ms (lub-dub + pausa).
            const uint32_t period = 1300;
            const uint32_t p      = t % period;
            uint8_t v;
            if      (p < 150) v = (uint8_t)((p * 220UL) / 150);                 // lub sobe
            else if (p < 250) v = (uint8_t)(220 - ((p - 150) * 200UL) / 100);   // lub desce
            else if (p < 350) v = 20 + (uint8_t)(((p - 250) * 200UL) / 100);    // dub sobe
            else if (p < 500) v = (uint8_t)(220 - ((p - 350) * 215UL) / 150);   // dub desce
            else              v = 5;                                             // pausa
            for (auto& px : leds_) px = CRGB(0, 0, v);
            break;
        }

        case State::STARTING: {
            // 3 piscadas verdes (200 ms on / 200 ms off) — depois branco em RUN.
            const bool on = ((t / 200) % 2) == 0;
            for (auto& px : leds_) px = on ? CRGB(0, 220, 0) : CRGB::Black;
            if (t >= 1200) { transition_(State::RUN); return; }
            break;
        }

        case State::RUN:
            // Branco cheio para iluminar o chão para o PMW3901.
            for (auto& px : leds_) px = CRGB(255, 255, 255);
            break;

        case State::WAYPOINT: {
            // Laranja piscando por ~3 s; depois volta ao estado automático.
            const bool on = ((t / 200) % 2) == 0;
            for (auto& px : leds_) px = on ? CRGB(255, 80, 0) : CRGB::Black;
            if (t >= 3000) { resolveAuto_(); return; }
            break;
        }

        case State::ERROR: {
            // Vermelho piscando — sticky até alguém chamar setError(false).
            const bool on = ((t / 250) % 2) == 0;
            for (auto& px : leds_) px = on ? CRGB(255, 0, 0) : CRGB::Black;
            break;
        }

        case State::MANUAL: {
            // Compat: 0=fixo, 1=pisca, 2=rotação (chase).
            if (man_pat_ == 0) {
                for (auto& px : leds_) px = CRGB(man_r_, man_g_, man_b_);
            } else if (man_pat_ == 1) {
                const bool on = ((t / 250) % 2) == 0;
                for (auto& px : leds_) px = on ? CRGB(man_r_, man_g_, man_b_) : CRGB::Black;
            } else {
                const uint8_t idx = (uint8_t)((t / 50) % NUM_LEDS);
                for (uint8_t i = 0; i < NUM_LEDS; ++i) {
                    leds_[i] = (i == idx) ? CRGB(man_r_, man_g_, man_b_) : CRGB::Black;
                }
            }
            break;
        }

        default:
            for (auto& px : leds_) px = CRGB::Black;
            break;
    }
    FastLED.show();
}

}  // namespace leds
