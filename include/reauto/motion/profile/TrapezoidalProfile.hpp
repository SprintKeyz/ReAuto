#pragma once

#include "reauto/chassis/impl/MotionChassis.hpp"
#include "reauto/controller/impl/PIDController.hpp"
#include "reauto/motion/profile/MotionProfile.hpp"
#include <memory>

namespace reauto {
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
    double m_maxVelocity; // the maximum velocity
    double m_maxAcceleration; // the maximum acceleration

    // the actual profile, used like this:
    // p.get(time).velocity, p.get(time).acceleration
    MotionProfile m_profile;
};
}