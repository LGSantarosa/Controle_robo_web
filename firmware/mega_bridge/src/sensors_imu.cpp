#include "sensors_imu.h"

namespace sensors {

// --- registradores MPU9250 (gyro/accel = mesmo mapa do MPU6050) -------------
namespace {
constexpr uint8_t REG_CONFIG        = 0x1A;  // DLPF do giroscópio
constexpr uint8_t REG_GYRO_CONFIG   = 0x1B;  // fundo de escala do giro
constexpr uint8_t REG_ACCEL_CONFIG  = 0x1C;  // fundo de escala do accel
constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;  // DLPF do accel (novo no 9250)
constexpr uint8_t REG_ACCEL_XOUT_H  = 0x3B;  // início do burst (accel..temp..gyro)
constexpr uint8_t REG_INT_PIN_CFG   = 0x37;  // BYPASS_EN p/ falar direto com o AK8963
constexpr uint8_t REG_USER_CTRL     = 0x6A;  // I2C master OFF p/ o bypass valer
constexpr uint8_t REG_PWR_MGMT_1    = 0x6B;
constexpr uint8_t REG_WHO_AM_I       = 0x75;

// --- AK8963 (magnetômetro embutido), I²C 0x0C via bypass ---
constexpr uint8_t AK_ADDR    = 0x0C;
constexpr uint8_t AK_WIA     = 0x00;   // = 0x48
constexpr uint8_t AK_ST1     = 0x02;   // bit0 DRDY
constexpr uint8_t AK_HXL     = 0x03;   // HXL..HZH (6 bytes) + ST2 (0x09)
constexpr uint8_t AK_CNTL1   = 0x0A;
constexpr uint8_t AK_ASAX    = 0x10;   // correção de fábrica (fuse ROM)
constexpr uint8_t AK_WHO     = 0x48;
constexpr float   AK_UT_PER_LSB = 0.15f;   // 16-bit: 4912 µT / 32760 ≈ 0.15

// WHO_AM_I aceitos. Gyro/accel são IDÊNTICOS nos quatro (mesmo mapa de
// registrador e escalas InvenSense), então o driver gyro-only serve pra todos.
// SÓ o 9250/9255 têm o AK8963 (mag) → o yaw absoluto futuro só existe se o boot
// reportar 0x71/0x73. 0x70 (GY-6500 = MPU6500) e 0x68 (MPU6050, a IMU antiga
// remontada 2026-07-01) funcionam como IMU mas SEM magnetômetro.
constexpr uint8_t WHO_MPU6050 = 0x68;        // MPU6050 (IMU antiga) — sem mag
constexpr uint8_t WHO_MPU6500 = 0x70;        // MPU6500 (GY-6500) — sem mag
constexpr uint8_t WHO_MPU9250 = 0x71;        // MPU9250 (GY-9250) — com mag
constexpr uint8_t WHO_MPU9255 = 0x73;        // MPU9255 — com mag

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
    if (who != WHO_MPU6050 && who != WHO_MPU6500 &&
        who != WHO_MPU9250 && who != WHO_MPU9255)
        return false;
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
    // Magnetômetro AK8963 (fase 1 yaw absoluto): inicia em separado — se falhar,
    // o gyro/accel seguem OK (mag_ok_ fica false, só o mag não é reportado).
    initMag_();
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
    readMag_();             // atualiza mx_/my_/mz_ (fase 1 yaw absoluto)
    return true;
}

// ---------------- AK8963 (magnetômetro) ----------------

bool Imu::magWrite_(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(AK_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool Imu::magRead_(uint8_t reg, uint8_t* buf, uint8_t n) {
    Wire.beginTransmission(AK_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(AK_ADDR, n) != n) return false;
    for (uint8_t i = 0; i < n; ++i) buf[i] = Wire.read();
    return true;
}

bool Imu::initMag_() {
    // Bypass: liga o AK8963 direto no barramento (I²C master do MPU desligado).
    writeReg_(REG_USER_CTRL, 0x00);        // I2C_MST_EN = 0
    writeReg_(REG_INT_PIN_CFG, 0x02);      // BYPASS_EN
    delay(10);
    uint8_t who = 0;
    if (!magRead_(AK_WIA, &who, 1) || who != AK_WHO) {
        mag_ok_ = false;
        return false;
    }
    // Correção de fábrica (ASA) lida no modo fuse ROM.
    magWrite_(AK_CNTL1, 0x00); delay(10);  // power down
    magWrite_(AK_CNTL1, 0x0F); delay(10);  // fuse ROM access
    uint8_t asa[3] = {128, 128, 128};
    magRead_(AK_ASAX, asa, 3);
    asax_ = (asa[0] - 128) / 256.0f + 1.0f;
    asay_ = (asa[1] - 128) / 256.0f + 1.0f;
    asaz_ = (asa[2] - 128) / 256.0f + 1.0f;
    magWrite_(AK_CNTL1, 0x00); delay(10);  // power down
    magWrite_(AK_CNTL1, 0x16); delay(10);  // contínuo modo 2 (100 Hz) + 16-bit
    mag_ok_ = true;
    return true;
}

void Imu::readMag_() {
    if (!mag_ok_) return;
    uint8_t st1 = 0;
    if (!magRead_(AK_ST1, &st1, 1) || !(st1 & 0x01)) return;  // sem amostra nova (DRDY=0)
    uint8_t b[7];                            // HXL..HZH (6, LITTLE-endian) + ST2
    if (!magRead_(AK_HXL, b, 7)) return;
    if (b[6] & 0x08) return;                 // ST2 HOFL = overflow -> descarta amostra
    const int16_t hx = (int16_t)((b[1] << 8) | b[0]);
    const int16_t hy = (int16_t)((b[3] << 8) | b[2]);
    const int16_t hz = (int16_t)((b[5] << 8) | b[4]);
    const float cx = hx * asax_ * AK_UT_PER_LSB;   // ASA por eixo do AK + µT
    const float cy = hy * asay_ * AK_UT_PER_LSB;
    const float cz = hz * asaz_ * AK_UT_PER_LSB;
    // Alinha pro frame do gyro/accel: AK X=sensor Y, AK Y=sensor X, AK Z=-sensor Z.
    mx_ = cy;
    my_ = cx;
    mz_ = -cz;
}

}  // namespace sensors
