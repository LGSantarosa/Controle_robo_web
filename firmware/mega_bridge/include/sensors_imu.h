#pragma once
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

namespace sensors {

// IMU = MPU6050 (6 eixos: giroscópio + acelerômetro). Substituiu o BNO055.
// Diferença que importa: o MPU6050 NÃO tem magnetômetro nem fusão interna →
// NÃO existe orientação absoluta. Entregamos só giro (rad/s) + accel (m/s²);
// o yaw é integrado da taxa do giro no pose_estimator.
//
// Montagem: o chip está DE PONTA-CABEÇA (eixo Z aponta pra BAIXO). O driver
// devolve gyro/accel no frame BRUTO do sensor (sem correção de montagem); a
// inversão de sinal do yaw (Z down → yaw = -gz) é aplicada no pose_estimator
// via param imu_yaw_sign, pra permitir ajuste de bancada sem reflashear a MEGA.
// Ver memória project_imu_mpu6050_mounting.
class Imu {
 public:
    bool begin();
    bool read();
    // gyro em rad/s, accel em m/s² — FRAME BRUTO do sensor (giro já sem bias).
    float gx() const { return gx_; }
    float gy() const { return gy_; }
    float gz() const { return gz_; }
    float ax() const { return ax_; }
    float ay() const { return ay_; }
    float az() const { return az_; }
    bool ok() const { return ok_; }

 private:
    bool tryInit_(uint8_t addr);
    void calibrateGyro_();   // estima o bias do giro com o robô parado (1x no boot)

    Adafruit_MPU6050 mpu_;
    // Endereço I²C: 0x68 (AD0=GND, default) ou 0x69 (AD0=VCC). begin() tenta os
    // dois, igual o BNO055 fazia, pra não depender do jumper.
    uint8_t  addr_ = 0x68;
    bool     ok_ = false;
    bool     calibrated_ = false;          // bias só é estimado uma vez (no boot)
    uint32_t last_recover_ms_ = 0;
    float gx_ = 0, gy_ = 0, gz_ = 0;       // rad/s (sem bias)
    float ax_ = 0, ay_ = 0, az_ = 0;       // m/s²
    float bx_ = 0, by_ = 0, bz_ = 0;       // bias do giro (rad/s), subtraído no read()
};

}  // namespace sensors
