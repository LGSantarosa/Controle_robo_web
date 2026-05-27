#pragma once
#include <FastLED.h>

namespace leds {

// TEMP DIAG 2026-05-27: movido de 6 → 5 pra testar se pino 6 da MEGA esta'
// queimado (anel responde com lixo aleatorio mesmo com loop rodando estavel).
// Reverter pra 6 se a troca de pino nao resolver.
constexpr uint8_t DATA_PIN = 5;
constexpr uint8_t NUM_LEDS = 24;

// Estados do anel — o firmware decide sozinho na maior parte do tempo;
// o PC pode forçar via FT_LEDS (ver protocol.h).
enum class State : uint8_t {
    OFF      = 0,  // apagado
    BOOT     = 1,  // pulso branco curto durante o setup()
    IDLE     = 2,  // azul "coração batendo" (robô ligado, sem comandos do PC)
    STARTING = 3,  // 5 ciclos verde/branco (1 s) ao sair de IDLE pro RUN
    RUN      = 4,  // branco fixo pra iluminar o chão pro PMW3901
    WAYPOINT = 5,  // 5 ciclos amarelo/branco (1 s) ao chegar num ponto, depois volta
    ERROR    = 6,  // vermelho piscando, sticky até o erro sair
    MANUAL   = 7,  // override RGB+pattern legado vindo do PC
};

class Ring {
 public:
    void begin();
    void tick();

    // Eventos vindos da lógica principal
    void setActive(bool active);   // PC mandando setpoints recentes?
    void setError(bool err);       // algum sensor essencial falhou?
    void triggerWaypoint();        // robô chegou num ponto

    // Override manual (compatibilidade: PC envia RGB+pattern)
    void setManual(uint8_t r, uint8_t g, uint8_t b, uint8_t pattern);
    void clearManual();            // sai do manual e volta pro automático

    // Permite o PC pedir um estado direto via protocolo
    void setState(State s);
    State state() const { return state_; }

    // True enquanto a animação atual (STARTING, WAYPOINT) está modulando a
    // iluminação do anel — o PMW3901 vê variação de brilho/cor e reporta
    // motion fantasma. Inclui janela de recovery do auto-gain do sensor.
    // Use no txFlow() pra suprimir publicação durante esse intervalo.
    bool gated() const { return gated_until_ != 0 && millis() < gated_until_; }

 private:
    void transition_(State s);
    void resolveAuto_();            // escolhe próximo estado automático

    CRGB     leds_[NUM_LEDS]{};
    State    state_       = State::OFF;
    uint32_t state_start_ = 0;
    uint32_t last_tick_   = 0;

    bool     wish_active_ = false;
    bool     wish_error_  = false;
    uint32_t gated_until_ = 0;  // 0 = sem gating; senão millis() de fim

    uint8_t  man_r_   = 0;
    uint8_t  man_g_   = 0;
    uint8_t  man_b_   = 0;
    uint8_t  man_pat_ = 0;
};

}  // namespace leds
