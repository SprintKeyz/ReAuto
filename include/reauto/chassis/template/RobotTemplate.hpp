#pragma once

#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"

#define TOLERANCE_DEG 5

namespace reauto {
class RobotTemplate {
public:
    // set the forward voltage of the left side
    virtual void setLeftFwdVoltage(double voltage) = 0;

    // set the forward voltage of the right side
    virtual void setRightFwdVoltage(double voltage) = 0;

    // set the forward velocity of the left side
    virtual void setLeftFwdVelocity(double velocity) = 0;

    // set the forward velocity of the right side
    virtual void setRightFwdVelocity(double velocity) = 0;

    // set the forward voltage
    virtual void setFwdVoltage(double voltage) = 0;

    // set the forward velocity
    virtual void setFwdVelocity(double velocity) = 0;

    // set the turn voltage [-L, R]
    virtual void setTurnVoltage(double voltage) = 0;

    // set the turn velocity [-L, R]
    virtual void setTurnVelocity(double velocity) = 0;

    // set the chassis brake mode
    virtual void setBrakeMode(pros::Motor_Brake mode) = 0;

    // brake the chassis
    virtual void brake() = 0;

    // get the left velocity
    virtual double getLeftVelocity() const = 0;

protected:
    explicit RobotTemplate() = default;
};
}