#include "reauto/device/ADIMU.hpp"
#include "reauto/device/IMU.hpp"
#include "reauto/math/Convert.hpp"

// warning: ternaries ahead

namespace reauto {
namespace device {
ADIMU::ADIMU(const uint8_t portA, const uint8_t portB): IMU(portA), m_secondaryIMU(portB) {}

double ADIMU::getHeading(bool radians, bool wrap180) const {
    // average them
    double heading = (IMU::getHeading(false, false) + m_secondaryIMU.get_heading()) / 2.0;

    // should we wrap it to [-180, 180]
    if (wrap180) heading = math::wrap180(heading);
    return radians ? math::degToRad(heading) : heading;
}

double ADIMU::getRotation(bool radians) const {
    double rotation = (IMU::getRotation() + m_secondaryIMU.get_rotation()) / 2.0;
    return radians ? math::degToRad(rotation) : rotation;
}

void ADIMU::reset(bool blocking) {
    IMU::reset(blocking);
    m_secondaryIMU.reset(blocking);

    if (blocking) {
        while (IMU::isCalibrating() || m_secondaryIMU.is_calibrating()) pros::delay(10);
    }
}

void ADIMU::setHeading(const double target) {
    IMU::setHeading(target);
    m_secondaryIMU.set_heading(target);
}
}
}