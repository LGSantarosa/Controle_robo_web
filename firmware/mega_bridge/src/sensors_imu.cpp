#include "sensors_imu.h"

namespace sensors {

bool Imu::tryInit_(uint8_t addr) {
    if (!mpu_.begin(addr, &Wire)) return false;
    addr_ = addr;
    // ±500°/s cobre o giro do skid-steer (~344°/s no nav2) com folga.
    mpu_.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu_.setAccelerometerRange(MPU6050_RANGE_4_G);
    // DLPF 94 Hz: corta ruído de motor/vibração sem atrasar demais o giro
    // (amostramos IMU a 50 Hz no main).
    mpu_.setFilterBandwidth(MPU6050_BAND_94_HZ);
    return true;
}

bool Imu::begin() {
    // Endereço default 0x68; se falhar, tenta 0x69 (jumper AD0).
    ok_ = tryInit_(0x68) || tryInit_(0x69);
    if (ok_ && !calibrated_) {
        calibrateGyro_();
        calibrated_ = true;
    }
    return ok_;
}

void Imu::calibrateGyro_() {
    // Média do giro com o robô PARADO no boot → bias de zero. O MPU6050 tem
    // offset de giro relevante; sem subtrair, o yaw integrado deriva visível.
    // Roda SÓ uma vez (flag calibrated_ no begin): recovery de barramento em
    // runtime NÃO recalibra, pra nunca capturar bias com o robô em movimento.
    const int N = 200;
    float sx = 0, sy = 0, sz = 0;
    int got = 0;
    sensors_event_t a, g, t;
    for (int i = 0; i < N; ++i) {
        if (mpu_.getEvent(&a, &g, &t)) {
            sx += g.gyro.x;
            sy += g.gyro.y;
            sz += g.gyro.z;
            ++got;
        }
        delay(3);
    }
    if (got > 0) {
        bx_ = sx / got;
        by_ = sy / got;
        bz_ = sz / got;
    }
}

bool Imu::read() {
    // Se o I²C caiu (cabo solto / brown-out), tenta re-init a cada 2 s. O bias
    // estimado no boot é preservado (mesmo chip), então não recalibra.
    if (!ok_) {
        const uint32_t now = millis();
        if (now - last_recover_ms_ > 2000) {
            last_recover_ms_ = now;
            begin();
        }
        return false;
    }

    sensors_event_t a, g, t;
    if (!mpu_.getEvent(&a, &g, &t)) {
        // Leitura falhou: barramento provavelmente morto. Marca caído e deixa
        // o recover periódico tentar de novo.
        ok_ = false;
        return false;
    }

    // getEvent já entrega giro em rad/s e accel em m/s² (convenção ROS). Só
    // subtraímos o bias do giro estimado no boot.
    gx_ = g.gyro.x - bx_;
    gy_ = g.gyro.y - by_;
    gz_ = g.gyro.z - bz_;
    ax_ = a.acceleration.x;
    ay_ = a.acceleration.y;
    az_ = a.acceleration.z;
    return true;
}

}  // namespace sensors
