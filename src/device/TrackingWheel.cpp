#include "reauto/device/TrackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "reauto/chassis/impl/MotionChassis.hpp"
#include "reauto/math/Convert.hpp"
#include "reauto/filter/SMAFilter.hpp"
#include <iostream>
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
TrackingWheel::TrackingWheel(const int8_t port, const double diam, const double dist) : m_diam(diam), m_dist(dist), m_filter(5) {
    m_rotation = new pros::Rotation(port);
}

TrackingWheel::TrackingWheel(MotorSet* motors, const double diam, const double dist, const double rpm) : m_motors(motors), m_diam(diam), m_dist(dist), m_filter(5), m_rpm(rpm) {}

double TrackingWheel::getPosition(bool radians) const {
    double rotation = 0;

    if (m_rotation != nullptr) {
        rotation = math::cdegToDeg(m_rotation->get_position());
    }

    if (m_motors != nullptr) {
        rotation = m_motors->get_position();
    }

    return radians ? math::degToRad(rotation) : rotation;
}

double TrackingWheel::getDistanceTraveled(DistanceUnits units) const {
    double position = getPosition();

    if (m_rotation != nullptr) {
        double inches = math::degToIn(position, m_diam);
        return inches * conversions[units];
    }

    else if (m_motors != nullptr) {
        double dist = 0;
        double in;

        position /= 360; // convert to rotations

        switch (m_motors->get_gearing()) {
        case pros::MotorGears::ratio_36_to_1:
            in = 100;
            break;
        case pros::MotorGears::ratio_18_to_1:
            in = 200;
            break;
        case pros::MotorGears::ratio_6_to_1:
            in = 600;
            break;
        default:
            in = 200;
            break;
        }

        dist = (position * (m_diam * M_PI) * (m_rpm / in));
        return dist * conversions[units];
    }

    return 0;
}

void TrackingWheel::reset() {
    if (m_rotation != nullptr) {
        m_rotation->reset();
    }

    if (m_motors != nullptr) {
        m_motors->reset_position();
    }
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