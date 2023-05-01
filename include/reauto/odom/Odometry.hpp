#pragma once

#include "reauto/datatypes/Point.h"
#include "reauto/device/IMU.hpp"
#include "reauto/math/Convert.hpp"
#include "pros/apix.h"
#include "reauto/device/TrackingWheel.hpp"
#include "reauto/device/TrackingWheels.hpp"
#include <iostream>
#include <cmath>

#define ODOM_TIMESTEP 10

// odometry doesnt deal with heading management!
// therefore reset pose and other heading-related
// functions are left up to the chassis class

namespace reauto {
class Odometry {
public:
    Odometry(TrackingWheels* wheels, device::IMU* imu);

    void resetPosition();
    void setPosition(Point p);
    Point getPosition();
    void startTracking();

private:
    Point m_pos;
    TrackingWheels* m_wheels;
    device::IMU* m_imu;

    // odom only supports middle/back because I'm lazy
    // expect this to change in the future!
    double m_prevMiddlePos;
    double m_prevBackPos;
    double m_prevRotationRad;

    void resetPreviousVariables();
};
}