#pragma once

#include "pros/rotation.hpp"
#include "reauto/device/MotorSet.hpp"
#include "reauto/filter/SMAFilter.hpp"

namespace reauto {
// the units for the tracking wheel
enum class DistanceUnits
{
    IN,
    FT,
    CM,
    MM,
    M
};

namespace device {
class TrackingWheel {
public:
    explicit TrackingWheel(const int8_t port, const double diam, const double dist);

    // ctor for a motor set without dedicated hw
    explicit TrackingWheel(MotorSet* motors, const double diam, const double dist, const double rpm);

    // get the position of the tracking wheel
    double getPosition(bool radians = false) const;

    // get the total distance traveled by the tracking wheel
    double getDistanceTraveled(DistanceUnits units = DistanceUnits::IN) const;

    // reset the encoder value to 0 units
    void reset();

    // get the diameter
    double getDiameter() const;

    // get the distance from the center of rotation
    double getCenterDistance() const;

    // get the velocity (in/s)
    double getVelocity();

private:
    const double m_diam;
    const double m_dist;
    const double m_rpm = 0;

    pros::Rotation* m_rotation = nullptr;
    reauto::MotorSet* m_motors = nullptr;

    filter::SMAFilter m_filter;

    // for velocity
    double m_lastPos = 0;
};
}
}