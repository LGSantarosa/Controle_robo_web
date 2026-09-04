#pragma once
#include <Arduino.h>
#include <Wire.h>

namespace sensors {

// IMU #2 = BNO055 (9 eixos: gyro + accel + MAGNETÔMETRO, com fusão de sensores
// EMBARCADA no próprio chip). Entra AO LADO do MPU (sensors_imu.*), não no lugar:
// as duas rodam no mesmo barramento I²C da MEGA (endereços diferentes, sem
// conflito) e o PC recebe as duas em frames separados (FT_IMU e FT_IMU2).
//
// O que ela traz de novo, e por que existe:
//   - QUATERNION de orientação ABSOLUTA (modo NDOF). O MPU só dá TAXA de giro →
//     o yaw é integrado e DERIVA. A BNO055 fecha o heading no norte magnético,
//     então a deriva do trekking (percurso longo sem âncora de LiDAR) para de
//     crescer sem limite. Quem consome é o pose_estimator (correção lenta e
//     limitada, ver heading_correction em fused_odom.py).
//   - Magnetômetro dedicado (não o AK8963 mixado do 9250), reportado em µT.
//   - Um SEGUNDO gyro/accel: redundância real. Se um dos dois chips morrer no
//     I²C (já aconteceu: project_mega_i2c_hang), o outro segura a odometria.
//
// Endereço I²C: 0x28 (COM3/ADR em GND, default dos breakouts) ou 0x29 (ADR em
// VCC). begin() tenta os dois, igual o driver do MPU faz com 0x68/0x69. Nenhum
// dos dois colide com o MPU (0x68/0x69) nem com o AK8963 (0x0C).
//
// Unidades entregues por este driver (já convertidas no read()):
//   gyro  rad/s   |  accel m/s² (COM gravidade, convenção sensor_msgs/Imu)
//   mag   µT      |  quaternion adimensional, normalizado pelo chip (w,x,y,z)
// Tudo no FRAME BRUTO do sensor — correção de montagem é do lado do ROS
// (imu2_yaw_sign no pose_estimator), igual ao MPU.
//
// Por que NÃO a Adafruit_BNO055: mesma razão do MPU (ver sensors_imu.h) — a lib
// puxa Adafruit_Sensor/BusIO, ignora o Wire.setWireTimeout() que este firmware
// configura no setup() (a proteção anti-hang do I²C) e faz delays bloqueantes
// próprios. O mapa de registradores da BNO055 é trivial: um burst de 18 bytes
// (accel+mag+gyro), um de 8 (quaternion) e 1 byte de calibração.
class Bno055 {
 public:
    bool begin();
    // Lê um ciclo completo (accel+mag+gyro+quat+calib). false = barramento
    // caído; o próprio read() reagenda o re-init a cada 2 s (igual ao MPU).
    bool read();

    // --- gyro (rad/s) e accel (m/s², com gravidade), frame BRUTO do sensor ---
    float gx() const { return gx_; }
    float gy() const { return gy_; }
    float gz() const { return gz_; }
    float ax() const { return ax_; }
    float ay() const { return ay_; }
    float az() const { return az_; }
    // --- magnetômetro (µT), frame BRUTO do sensor ---
    float mx() const { return mx_; }
    float my() const { return my_; }
    float mz() const { return mz_; }
    // --- quaternion de orientação ABSOLUTA (NDOF), já normalizado pelo chip ---
    float qw() const { return qw_; }
    float qx() const { return qx_; }
    float qy() const { return qy_; }
    float qz() const { return qz_; }

    bool ok() const { return ok_; }
    // Byte de calibração cru do chip: sys<<6 | gyro<<4 | accel<<2 | mag, cada
    // campo 0..3. O pose_estimator SÓ aceita o heading absoluto com o mag
    // calibrado (≥2) — sem isso a BNO055 aponta pra um norte inventado e
    // arrastaria o robô pro lado errado. 3 no mag = "gira em 8 no ar" feito.
    uint8_t calib() const { return calib_; }

 private:
    bool tryInit_(uint8_t addr);
    bool writeReg_(uint8_t reg, uint8_t val);
    bool readRegs_(uint8_t reg, uint8_t* buf, uint8_t n);

    uint8_t  addr_ = 0x28;
    bool     ok_ = false;
    // true só até o primeiro begin() terminar. Separa o boot (pode gastar
    // ~700 ms num reset por software, robô parado) do recovery em runtime
    // (tem que ser barato: o loop alimenta os hoverboards a 50 Hz).
    bool     first_boot_ = true;
    uint32_t last_recover_ms_ = 0;
    float gx_ = 0, gy_ = 0, gz_ = 0;    // rad/s
    float ax_ = 0, ay_ = 0, az_ = 0;    // m/s²
    float mx_ = 0, my_ = 0, mz_ = 0;    // µT
    float qw_ = 1, qx_ = 0, qy_ = 0, qz_ = 0;
    uint8_t calib_ = 0;
};

}  // namespace sensors
