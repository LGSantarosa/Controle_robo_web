#include "leds.h"

namespace leds {

namespace {
// Cores das animações (sem o brightness global aplicado).
// Amarelo e verde foram escolhidos por luminância alta — o PMW3901 precisa
// de luz refletida do chão pra rastrear; vermelho/azul puros apagariam a cena.
constexpr CRGB COLOR_WAYPOINT = CRGB(255, 200, 0);   // amarelo
constexpr CRGB COLOR_STARTING = CRGB(0,   220, 0);   // verde
constexpr CRGB COLOR_BASE     = CRGB(255, 255, 255); // branco que mantém luz no chão

// Tempos das animações com base branca alternada.
constexpr uint16_t FLASH_ON_MS      = 120;
constexpr uint16_t FLASH_OFF_MS     = 80;
constexpr uint16_t FLASH_PERIOD_MS  = FLASH_ON_MS + FLASH_OFF_MS;  // 200
constexpr uint8_t  FLASH_CYCLES     = 5;
constexpr uint16_t FLASH_TOTAL_MS   = FLASH_PERIOD_MS * FLASH_CYCLES;  // 1000
constexpr uint16_t PMW_RECOVERY_MS  = 150;  // auto-gain do sensor reassentar
}  // namespace

void Ring::begin() {
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds_, NUM_LEDS);
    // Anel alimentado pelo step-down de 5 V / 3 A compartilhado com a Pi.
    // 100 = ~40% de brilho ≈ ~580 mA pico (anel todo branco) — Pi sob carga
    // puxa ~1,2 A, total ~1,8 A, sobra ~1,2 A de folga no step-down.
    // Subir além de ~150 sem step-down dedicado pode causar undervoltage na Pi.
    // Ajuste em campo se PMW3901 saturar em piso claro (baixar para 60–80) ou
    // perder tracking em piso escuro (subir até ~140 com cuidado).
    FastLED.setBrightness(100);
    FastLED.clear(true);
    transition_(State::BOOT);
}

void Ring::transition_(State s) {
    state_       = s;
    state_start_ = millis();
    // Gateamento do PMW3901: liga quando a animação modula a iluminação,
    // desliga quando o estado for de luz estável (RUN/IDLE/ERROR/OFF/MANUAL).
    if (s == State::WAYPOINT || s == State::STARTING) {
        gated_until_ = state_start_ + FLASH_TOTAL_MS + PMW_RECOVERY_MS;
    } else {
        gated_until_ = 0;
    }
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
            // 5 ciclos com base branca alternada: LEDs pares ficam brancos
            // (mantêm luz no chão pro PMW), ímpares piscam verde.
            // PMW3901 fica gateado pelo gated_until_ — ver transition_().
            const uint32_t phase = t % FLASH_PERIOD_MS;
            const bool color_on = phase < FLASH_ON_MS;
            for (uint8_t i = 0; i < NUM_LEDS; ++i) {
                if (i & 1) {
                    leds_[i] = color_on ? COLOR_STARTING : COLOR_BASE;
                } else {
                    leds_[i] = COLOR_BASE;
                }
            }
            if (t >= FLASH_TOTAL_MS) { transition_(State::RUN); return; }
            break;
        }

        case State::RUN:
            // Branco cheio para iluminar o chão para o PMW3901.
            for (auto& px : leds_) px = COLOR_BASE;
            break;

        case State::WAYPOINT: {
            // Mesma estrutura do STARTING, mas amarelo. Robô continua andando:
            // a base branca preserva ~50% da iluminação e o gated_until_
            // suprime o txFlow() pelos ~1150 ms até o auto-gain do PMW reassentar.
            const uint32_t phase = t % FLASH_PERIOD_MS;
            const bool color_on = phase < FLASH_ON_MS;
            for (uint8_t i = 0; i < NUM_LEDS; ++i) {
                if (i & 1) {
                    leds_[i] = color_on ? COLOR_WAYPOINT : COLOR_BASE;
                } else {
                    leds_[i] = COLOR_BASE;
                }
            }
            if (t >= FLASH_TOTAL_MS) { resolveAuto_(); return; }
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
