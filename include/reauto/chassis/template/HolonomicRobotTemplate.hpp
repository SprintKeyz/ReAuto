#pragma once

#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "reauto/chassis/template/RobotTemplate.hpp"

namespace reauto {
class HolonomicRobotTemplate: public RobotTemplate {
public:
    // set the strafe voltage of the left side
    virtual void setLeftStrafeVoltage(double voltage) = 0;

    // set the strafe voltage of the right side
    virtual void setRightStrafeVoltage(double voltage) = 0;

    // set the strafe velocity of the left side
    virtual void setLeftStrafeVelocity(double velocity) = 0;

    // set the strafe velocity of the right side
    virtual void setRightStrafeVelocity(double velocity) = 0;

    // set the strafe voltages
    virtual void setStrafeVoltage(double fwdPower, double sidePower) = 0;

    // set the strafe velocities
    virtual void setStrafeVelocity(double fwdVel, double sideVel) = 0;

protected:
    explicit HolonomicRobotTemplate() = default;
};
}