#pragma once

#include "pros/motor_group.hpp"
#include "reauto/chassis/template/RobotTemplate.hpp"

namespace reauto {
class TankBase: public RobotTemplate {
public:
    explicit TankBase(std::initializer_list<int8_t> leftPorts, std::initializer_list<int8_t> rightPorts, pros::Motor_Gears gearset);

    // set the forward voltage of the left motors
    void setLeftVoltage(double v);

    // set the forward voltage of the right motors
    void setRightVoltage(double v);

    // set the forward velocity of the left motors
    void setLeftVelocity(double v);

    // set the forward velocity of the right motors
    void setRightVelocity(double v);

    // set the voltages of the motors [L, R]
    void setVoltages(double vLeft, double vRight);

    // set the velocities of the motors [L, R]
    void setVelocities(double vLeft, double vRight);

    // set the chassis brake mode
    void setBrakeMode(pros::Motor_Brake mode);

    // brake the chassis
    void brake();

    // access the internal motor objects
    pros::MotorGroup* getLeftMotors();
    pros::MotorGroup* getRightMotors();

private:
    // left and right motors
    std::shared_ptr<pros::MotorGroup> m_leftMotors;
    std::shared_ptr<pros::MotorGroup> m_rightMotors;
};
}