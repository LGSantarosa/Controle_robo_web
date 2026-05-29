#include <Arduino.h>
#include "sensors_flow.h"

namespace sensors {

// Limiares de plausibilidade (ver read()). FLOW_MAX_COUNT é folgado: o
// movimento REAL é ~10 counts/amostra a 1 m/s (m/count ≈ 2,5 mm), então 2000 só
// barra lixo. O SQUAL do PMW3901 satura em ~168 — acima disso o byte está
// corrompido. Diagnóstico 2026-05-29 (EMI do motor no SPI via shifter MOSFET).
static constexpr int16_t FLOW_MAX_COUNT = 2000;
static constexpr uint8_t FLOW_SQUAL_MAX = 168;

bool Flow::begin() {
    ok_ = pmw_.begin();
    return ok_;
}

bool Flow::read() {
    // Se o SPI caiu (cabo solto, brown-out), tenta re-init a cada 2 s.
    // Mesmo padrão do Imu::read(); sem isso o flow morto pelo cabo soltando
    // ficava desabilitado até reboot da MEGA.
    if (!ok_) {
        const uint32_t now = millis();
        if (now - last_recover_ms_ > 2000) {
            last_recover_ms_ = now;
            begin();
        }
        return false;
    }
    int16_t dx, dy;
    pmw_.readMotionCount(&dx, &dy);
    // C5 (2026-05-29): SQUAL real (reg 0x07) em vez de 0 hardcoded. Sem isto o
    // pose_estimator zerava o peso do flow (sigmoid(quality)→alpha≈0) e o sensor
    // nunca entrava na fusão. Ler logo após readMotionCount() — mesmo quadro.
    const uint8_t squal = pmw_.readSqual();

    // Rejeita amostra corrompida por EMI do motor no SPI. Só acontece MANOBRANDO
    // (rodas em sentidos opostos → pico de chaveamento/ground bounce, e o shifter
    // MOSFET marginal não segura o sinal): dx/dy saturam em ±16000..32768 e o
    // byte de SQUAL volta 0x00/0xFF/>168. Sem este filtro o lixo entraria na
    // fusão com peso alto (na corrupção o SQUAL às vezes lê ~112, α≈0,84).
    // Descarta a amostra (não publica) — o pose_estimator trata o gap a 100 Hz.
    // Diagnóstico/medições completos: project_pmw3901_emi_motor (memória).
    if (dx > FLOW_MAX_COUNT || dx < -FLOW_MAX_COUNT ||
        dy > FLOW_MAX_COUNT || dy < -FLOW_MAX_COUNT ||
        squal > FLOW_SQUAL_MAX) {
        return false;
    }

    dx_ = dx;
    dy_ = dy;
    quality_ = squal;
    return true;
}

}  // namespace sensors
