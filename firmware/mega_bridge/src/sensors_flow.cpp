#include "sensors_flow.h"

namespace sensors {

bool Flow::begin() {
    ok_ = pmw_.begin();
    return ok_;
}

void Flow::read() {
    if (!ok_) return;
    pmw_.readMotionCount(&dx_, &dy_);
    // O lib do Bitcraze não expõe leitura direta do SQUAL (qualidade) —
    // mantemos 0 e podemos elevar pra um valor real numa próxima iteração.
    quality_ = 0;
}

}  // namespace sensors
