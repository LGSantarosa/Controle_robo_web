#pragma once
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

namespace sensors {

class Imu {
 public:
    bool begin();
    bool read();
    const imu::Quaternion& quat()  const { return quat_;  }
    const imu::Vector<3>&  gyro()  const { return gyro_;  }   // rad/s
    const imu::Vector<3>&  accel() const { return accel_; }   // m/s^2
    bool ok() const { return ok_; }

 private:
    bool tryInit_(uint8_t addr);

    // Endereço corrente — alterna entre 0x28 e 0x29 no begin() conforme
    // o ADR pin. Sem isso, se a placa subir em 0x29 (jumper) o IMU fica
    // marcado como ausente até alguém recompilar o firmware.
    uint8_t          addr_ = BNO055_ADDRESS_A;
    Adafruit_BNO055  bno_{55, BNO055_ADDRESS_A, &Wire};
    bool             ok_ = false;
    uint32_t         last_recover_ms_ = 0;
    imu::Quaternion  quat_;
    imu::Vector<3>   gyro_;
    imu::Vector<3>   accel_;
};

}  // namespace sensors
