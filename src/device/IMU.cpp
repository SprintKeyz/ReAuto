#include "reauto/device/IMU.hpp"
#include "reauto/math/Convert.hpp"

// warning: ternaries ahead

namespace reauto {
namespace device {
IMU::IMU(const uint8_t port): m_imu(port) {}

double IMU::getHeading(bool radians, bool wrap180) const {
    double heading = m_imu.get_heading();

    // should we wrap it to [-180, 180]
    if (wrap180) heading = math::wrap180(heading);
    return radians ? math::degToRad(heading) : heading;
}

double IMU::getRotation(bool radians) const {
    double rotation = m_imu.get_rotation();
    return radians ? math::degToRad(rotation) : rotation;
}

void IMU::reset(bool blocking) {
    m_imu.reset(blocking);
}

void IMU::setHeading(const double target) {
    m_imu.set_heading(target);
}

bool IMU::isCalibrating() const {
    return m_imu.is_calibrating();
}
}
}
