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
    Adafruit_BNO055 bno_{55, BNO055_ADDRESS_A, &Wire};
    bool             ok_ = false;
    imu::Quaternion  quat_;
    imu::Vector<3>   gyro_;
    imu::Vector<3>   accel_;
};

}  // namespace sensors
