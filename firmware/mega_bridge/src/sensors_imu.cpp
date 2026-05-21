#include "sensors_imu.h"

#include <new>  // placement new usado em tryInit_(); não depender da ordem de include

namespace sensors {

bool Imu::tryInit_(uint8_t addr) {
    // Reconstroi o objeto Adafruit_BNO055 com o endereço alvo. `placement
    // new` evita alocação dinâmica (a MEGA não tem heap pra esbanjar).
    bno_.~Adafruit_BNO055();
    new (&bno_) Adafruit_BNO055(55, addr, &Wire);
    if (!bno_.begin()) return false;
    delay(10);
    bno_.setExtCrystalUse(true);
    addr_ = addr;
    return true;
}

bool Imu::begin() {
    // Tenta o endereço default 0x28; se falhar, tenta 0x29 (jumper ADR).
    ok_ = tryInit_(BNO055_ADDRESS_A) || tryInit_(BNO055_ADDRESS_B);
    return ok_;
}

bool Imu::read() {
    // Se o I²C caiu (cabo solto / brown-out), tenta re-init a cada 2 s.
    if (!ok_) {
        const uint32_t now = millis();
        if (now - last_recover_ms_ > 2000) {
            last_recover_ms_ = now;
            begin();
        }
        return false;
    }
    quat_ = bno_.getQuat();

    // Sanity check: quaternion todo zerado normalmente sinaliza barramento
    // morto. Mesma resposta: marca como caído e deixa recover periódico tentar.
    if (quat_.w() == 0.0 && quat_.x() == 0.0 && quat_.y() == 0.0 && quat_.z() == 0.0) {
        ok_ = false;
        return false;
    }

    // BNO055 reporta giroscópio em °/s por padrão — convertemos pra rad/s
    // pra casar com a convenção ROS (sensor_msgs/Imu.angular_velocity).
    const auto g_deg = bno_.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    gyro_  = imu::Vector<3>(
        g_deg.x() * (PI / 180.0),
        g_deg.y() * (PI / 180.0),
        g_deg.z() * (PI / 180.0));

    accel_ = bno_.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    return true;
}

}  // namespace sensors
