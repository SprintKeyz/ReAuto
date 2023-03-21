#pragma once

#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"

namespace reauto {
class RobotTemplate {
public:
    // set the forward voltage of the left motors
    virtual void setLeftVoltage(double v) = 0;

    // set the forward voltage of the right motors
    virtual void setRightVoltage(double v) = 0;

    // set the forward velocity of the left motors
    virtual void setLeftVelocity(double v) = 0;

    // set the forward velocity of the right motors
    virtual void setRightVelocity(double v) = 0;

    // set the voltages of the motors [L, R]
    virtual void setVoltages(double vLeft, double vRight) = 0;

    // set the velocities of the motors [L, R]
    virtual void setVelocities(double vLeft, double vRight) = 0;

    // set the chassis brake mode
    virtual void setBrakeMode(pros::Motor_Brake mode) = 0;

    // brake the chassis
    virtual void brake() = 0;

    // access the internal motor objects
    virtual pros::MotorGroup* getLeftMotors() = 0;
    virtual pros::MotorGroup* getRightMotors() = 0;

protected:
    explicit RobotTemplate() = default;
};
}