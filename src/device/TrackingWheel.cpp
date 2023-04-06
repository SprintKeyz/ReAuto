#include "reauto/device/TrackingWheel.hpp"
#include "reauto/chassis/impl/MotionChassis.hpp"
#include "reauto/math/Convert.hpp"
#include "reauto/filter/SMAFilter.hpp"
#include <map>

// the conversion factors for our units
std::map<reauto::DistanceUnits, double> conversions = {
    {reauto::DistanceUnits::IN, 1},
    {reauto::DistanceUnits::FT, 12},
    {reauto::DistanceUnits::CM, 0.393701},
    {reauto::DistanceUnits::MM, 0.0393701},
    {reauto::DistanceUnits::M, 39.3701}
};

namespace reauto {
namespace device {
TrackingWheel::TrackingWheel(const int8_t port, const double diam, const double dist): m_diam(diam), m_dist(dist), m_rotation(abs(port), std::signbit(port)), m_filter(5) {}

double TrackingWheel::getPosition(bool radians) const {
    double rotation = math::cdegToDeg(m_rotation.get_position());
    return radians ? math::degToRad(rotation) : rotation;
}

double TrackingWheel::getDistanceTraveled(DistanceUnits units) const {
    double distance = getPosition();
    double inches = math::degToIn(distance, m_diam);
    return inches * conversions[units];
}

void TrackingWheel::reset() {
    m_rotation.reset_position();
}

double TrackingWheel::getDiameter() const {
    return m_diam;
}

double TrackingWheel::getCenterDistance() const {
    return m_dist;
}

double TrackingWheel::getVelocity() {
    double pos = getPosition();
    double velocity = (pos - m_lastPos) / (MOTION_TIMESTEP / 1000.0);
    //velocity = m_filter.calculate(velocity);
    m_lastPos = pos;
    return velocity;
}
}
}