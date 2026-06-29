#pragma once
#include <Arduino.h>   // delay/millis (antes vinham transitivos pela lib Adafruit)
#include <Wire.h>

namespace sensors {

// IMU = MPU9250 (9 eixos: giroscópio + acelerômetro + magnetômetro AK8963).
// Substituiu o MPU6050.
//
// POR ENQUANTO usamos só GYRO + ACCEL (yaw INTEGRADO da taxa do giro Z, igual
// ao 6050). O magnetômetro (AK8963, yaw ABSOLUTO → mata a deriva do yaw) fica
// pra um passo seguinte: a IMU agora está no CENTRO do robô, plana, longe da EMI
// dos motores, então o mag deve ser confiável. Os ganchos pra ele estão no .cpp
// (endereço/bypass) marcados como TODO. Ver memória project_imu_mpu9250.
//
// Montagem: chip PLANO, componentes pra CIMA (eixo Z pra cima). O driver devolve
// gyro/accel no FRAME BRUTO do sensor (sem correção de montagem); o sinal do yaw
// é aplicado no pose_estimator via param imu_yaw_sign — agora +1.0 (era -1.0
// quando a placa ficava de ponta-cabeça com o 6050).
//
// Por que NÃO a Adafruit_MPU6050: ela confere o chip-ID (0x68) e REJEITA o 9250
// (que responde 0x71). O mapa de registradores de gyro/accel do 9250 é idêntico
// ao 6050, então lemos direto por registrador (Wire) — sem dependência externa,
// reusando o Wire.setWireTimeout/recovery que o firmware já configura no setup().
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
    // Magnetômetro AK8963 (yaw absoluto, fase 1). mx/my/mz em µT, JÁ no frame do
    // gyro/accel (eixos alinhados) e com a correção de fábrica ASA aplicada.
    // magOk() = false até o AK8963 inicializar / se a leitura estourar (HOFL).
    float mx() const { return mx_; }
    float my() const { return my_; }
    float mz() const { return mz_; }
    bool magOk() const { return mag_ok_; }

 private:
    bool tryInit_(uint8_t addr);
    void calibrateGyro_();                       // bias do giro com o robô parado (1x no boot)
    bool readRaw_(float& gx, float& gy, float& gz,
                  float& ax, float& ay, float& az);   // burst + escala, SEM bias
    bool writeReg_(uint8_t reg, uint8_t val);
    bool readRegs_(uint8_t reg, uint8_t* buf, uint8_t n);
    // AK8963 (magnetômetro embutido no MPU9250, acessível via bypass em 0x0C)
    bool initMag_();                             // bypass + ASA + modo contínuo (1x no boot)
    void readMag_();                             // 1 amostra: ST1/data/ST2, ASA, alinha eixos
    bool magWrite_(uint8_t reg, uint8_t val);
    bool magRead_(uint8_t reg, uint8_t* buf, uint8_t n);

    // Endereço I²C: 0x68 (AD0=GND, default) ou 0x69 (AD0=VCC). begin() tenta os
    // dois, igual o 6050 fazia, pra não depender do jumper.
    uint8_t  addr_ = 0x68;
    bool     ok_ = false;
    bool     calibrated_ = false;          // bias só é estimado uma vez (no boot)
    uint32_t last_recover_ms_ = 0;
    float gx_ = 0, gy_ = 0, gz_ = 0;       // rad/s (sem bias)
    float ax_ = 0, ay_ = 0, az_ = 0;       // m/s²
    float bx_ = 0, by_ = 0, bz_ = 0;       // bias do giro (rad/s), subtraído no read()
    // AK8963
    bool  mag_ok_ = false;
    float mx_ = 0, my_ = 0, mz_ = 0;       // µT, frame do gyro, ASA aplicado
    float asax_ = 1, asay_ = 1, asaz_ = 1; // correção de fábrica por eixo (fuse ROM)
};

}  // namespace sensors
