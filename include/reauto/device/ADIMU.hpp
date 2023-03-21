#pragma once

// Averaged Dual IMU
#include "reauto/device/IMU.hpp"

namespace reauto {
namespace device {
class ADIMU: public IMU {
public:
    /* implementation of an IMU */
    explicit ADIMU(const uint8_t portA, const uint8_t portB);

    double getHeading(bool radians = false, bool wrap180 = true) const override;
    double getRotation(bool radians = false) const override;

    void reset(bool blocking = true) override;
    void setHeading(const double target) override;

private:
    // an internal secondary IMU object
    pros::Imu m_secondaryIMU;
};
}
}