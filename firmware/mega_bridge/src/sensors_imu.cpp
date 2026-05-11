#include "sensors_imu.h"

namespace sensors {

bool Imu::begin() {
    ok_ = bno_.begin();
    if (ok_) {
        delay(10);
        bno_.setExtCrystalUse(true);
    }
    return ok_;
}

bool Imu::read() {
    if (!ok_) return false;
    quat_ = bno_.getQuat();

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
