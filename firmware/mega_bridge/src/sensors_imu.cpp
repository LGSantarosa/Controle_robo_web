#include "sensors_imu.h"

namespace sensors {

// --- registradores MPU9250 (gyro/accel = mesmo mapa do MPU6050) -------------
namespace {
constexpr uint8_t REG_CONFIG        = 0x1A;  // DLPF do giroscópio
constexpr uint8_t REG_GYRO_CONFIG   = 0x1B;  // fundo de escala do giro
constexpr uint8_t REG_ACCEL_CONFIG  = 0x1C;  // fundo de escala do accel
constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;  // DLPF do accel (novo no 9250)
constexpr uint8_t REG_ACCEL_XOUT_H  = 0x3B;  // início do burst (accel..temp..gyro)
constexpr uint8_t REG_PWR_MGMT_1    = 0x6B;
constexpr uint8_t REG_WHO_AM_I       = 0x75;

constexpr uint8_t WHO_MPU9250 = 0x71;        // MPU9250
constexpr uint8_t WHO_MPU9255 = 0x73;        // variante MPU9255 (aceita também)

// Escalas escolhidas (casam com o 6050 anterior):
//  ±500 °/s → 65.5 LSB/(°/s);  ±4 g → 8192 LSB/g.
constexpr float GYRO_LSB_PER_DPS = 65.5f;
constexpr float ACCEL_LSB_PER_G  = 8192.0f;
constexpr float G_TO_MS2         = 9.80665f;
// DEG_TO_RAD já é macro do Arduino.h (mesmo valor) — usamos a dele.

// TODO (yaw absoluto, passo seguinte): magnetômetro AK8963 fica em I²C 0x0C,
// acessível habilitando o BYPASS (REG_INT_PIN_CFG 0x37, bit 0x02) ou via I²C
// master interno. Aqui só gyro+accel; ao ligar o mag, ler 0x0C e fundir o
// heading no pose_estimator. Centralidade da IMU (longe da EMI) viabiliza.
}  // namespace

bool Imu::writeReg_(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool Imu::readRegs_(uint8_t reg, uint8_t* buf, uint8_t n) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    // repeated-start (endTransmission(false)): mantém o barramento pra o read.
    if (Wire.endTransmission(false) != 0) return false;
    const uint8_t got = Wire.requestFrom(addr_, n);
    if (got != n) return false;
    for (uint8_t i = 0; i < n; ++i) buf[i] = Wire.read();
    return true;
}

bool Imu::tryInit_(uint8_t addr) {
    addr_ = addr;
    uint8_t who = 0;
    if (!readRegs_(REG_WHO_AM_I, &who, 1)) return false;
    if (who != WHO_MPU9250 && who != WHO_MPU9255) return false;
    // Reset e wake-up. PLL com referência de giro (clock estável p/ a taxa).
    if (!writeReg_(REG_PWR_MGMT_1, 0x80)) return false;  // reset
    delay(100);
    writeReg_(REG_PWR_MGMT_1, 0x01);                     // clock = auto/PLL
    delay(10);
    // DLPF ~92 Hz no giro e ~99 Hz no accel: corta ruído de motor/vibração sem
    // atrasar demais o giro (amostramos a IMU a 50 Hz no main). FCHOICE=00
    // (bits do GYRO_CONFIG zerados) deixa o DLPF do CONFIG valer.
    writeReg_(REG_CONFIG,        0x02);                  // gyro DLPF_CFG=2 (~92 Hz)
    writeReg_(REG_GYRO_CONFIG,   0x08);                  // ±500 °/s, DLPF ON
    writeReg_(REG_ACCEL_CONFIG,  0x08);                  // ±4 g
    writeReg_(REG_ACCEL_CONFIG2, 0x02);                  // accel DLPF (~99 Hz)
    delay(10);
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

bool Imu::readRaw_(float& gx, float& gy, float& gz,
                   float& ax, float& ay, float& az) {
    uint8_t b[14];
    // burst de 14 bytes: accel(6) + temp(2) + gyro(6), big-endian.
    if (!readRegs_(REG_ACCEL_XOUT_H, b, 14)) return false;
    const int16_t rax = (int16_t)((b[0]  << 8) | b[1]);
    const int16_t ray = (int16_t)((b[2]  << 8) | b[3]);
    const int16_t raz = (int16_t)((b[4]  << 8) | b[5]);
    // b[6],b[7] = temperatura (não usada)
    const int16_t rgx = (int16_t)((b[8]  << 8) | b[9]);
    const int16_t rgy = (int16_t)((b[10] << 8) | b[11]);
    const int16_t rgz = (int16_t)((b[12] << 8) | b[13]);
    // escala → convenção ROS: giro em rad/s, accel em m/s².
    gx = (rgx / GYRO_LSB_PER_DPS) * DEG_TO_RAD;
    gy = (rgy / GYRO_LSB_PER_DPS) * DEG_TO_RAD;
    gz = (rgz / GYRO_LSB_PER_DPS) * DEG_TO_RAD;
    ax = (rax / ACCEL_LSB_PER_G) * G_TO_MS2;
    ay = (ray / ACCEL_LSB_PER_G) * G_TO_MS2;
    az = (raz / ACCEL_LSB_PER_G) * G_TO_MS2;
    return true;
}

void Imu::calibrateGyro_() {
    // Média do giro com o robô PARADO no boot → bias de zero. Sem subtrair, o
    // yaw integrado deriva visível. Roda SÓ uma vez (flag calibrated_ no begin):
    // recovery de barramento em runtime NÃO recalibra, pra nunca capturar bias
    // com o robô em movimento.
    const int N = 200;
    float sx = 0, sy = 0, sz = 0;
    int got = 0;
    float gx, gy, gz, ax, ay, az;
    for (int i = 0; i < N; ++i) {
        if (readRaw_(gx, gy, gz, ax, ay, az)) {
            sx += gx;
            sy += gy;
            sz += gz;
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

    float gx, gy, gz, ax, ay, az;
    if (!readRaw_(gx, gy, gz, ax, ay, az)) {
        // Leitura falhou: barramento provavelmente morto. Marca caído e deixa
        // o recover periódico tentar de novo.
        ok_ = false;
        return false;
    }

    // Só subtraímos o bias do giro estimado no boot.
    gx_ = gx - bx_;
    gy_ = gy - by_;
    gz_ = gz - bz_;
    ax_ = ax;
    ay_ = ay;
    az_ = az;
    return true;
}

}  // namespace sensors
