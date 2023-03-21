#pragma once

#include "pros/imu.hpp"

namespace reauto {
namespace device {
class IMU {
public:
    /* implementation of an IMU */
    explicit IMU(const uint8_t port);

    virtual double getHeading(bool radians = false, bool wrap180 = true) const;
    virtual double getRotation(bool radians = false) const;

    virtual void reset(bool blocking = true);
    virtual void setHeading(const double target);

    bool isCalibrating() const;

private:
    // an internal IMU object
    pros::Imu m_imu;
};
}
}