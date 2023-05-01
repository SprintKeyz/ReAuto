#pragma once

#include "reauto/chassis/impl/MotionChassis.hpp"
#include "reauto/controller/impl/PIDController.hpp"
#include <memory>

namespace reauto {
struct ProfileData {
    double position;
    double velocity;
    double acceleration;
};

struct TrapezoidalProfileConstants {
    double maxVelocity;
    double kVelocityScale; // kV
    double maxAcceleration;
    double kAccelerationScale; // kA

    // feedback
    double kP = 0;
    double kD = 0;
};

class TrapezoidalProfile {
public:
    TrapezoidalProfile(std::shared_ptr<MotionChassis> chassis, TrapezoidalProfileConstants constants, std::shared_ptr<controller::PIDController> headingCorrectController = nullptr);

    // compute the profile for a distance
    void compute(double target, double maxV = 0, double maxA = 0);

    // follow the profile
    void followLinear();
    void followAngular();

private:
    std::shared_ptr<MotionChassis> m_chassis;
    std::shared_ptr<controller::PIDController> m_headingCorrector;
    TrapezoidalProfileConstants m_constants;

    double m_target; // the current target
    double m_maxVelocity; // the maximum velocity of the robot
    double m_maxAcceleration; // the maximum acceleration of the robot
    double m_profileMaxV = 0; // the max velocity of the profile
    double m_profileAccel = 0; // the max acceleration of the profile
    double m_timeToMaxV = 0; // the time to reach max velocity
    double m_timeFromMaxV = 0; // the time to decelerate from max velocity
    double m_timeTotal = 0; // the total profile time

    // prev position
    double m_prevPos = 0;

    // get our data at time t
    ProfileData getProfileDataAtTime(double time);
};
}