#include <Arduino.h>
#include "sensors_flow.h"

namespace sensors {

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
    pmw_.readMotionCount(&dx_, &dy_);
    // FIXME(C5): quality fixa em 0 desativa a fusão flow no pose_estimator
    // (sigmoid em quality colapsa em 0 → alpha=0 → robô ignora o flow).
    // O lib Bitcraze PMW3901 não expõe SQUAL (registrador 0x07); fix exige
    // fork da lib OU leitura SPI manual. Detalhes e workaround (snap-to-cone
    // via LiDAR) documentados no README seção "Sensores embarcados".
    quality_ = 0;
    return true;
}

}  // namespace sensors
