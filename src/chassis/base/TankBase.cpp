#include "reauto/chassis/base/TankBase.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"

namespace reauto {
TankBase::TankBase(std::initializer_list<int8_t> leftPorts, std::initializer_list<int8_t> rightPorts, pros::Motor_Gears gearset)
    : m_leftMotors(std::make_shared<pros::MotorGroup>(leftPorts, gearset)),
    m_rightMotors(std::make_shared<pros::MotorGroup>(rightPorts, gearset)) {}

void TankBase::setLeftVoltage(double v) {
    m_leftMotors->move(v);
}

void TankBase::setRightVoltage(double v) {
    m_rightMotors->move(v);
}

void TankBase::setLeftVelocity(double v) {
    m_leftMotors->move_velocity(v);
}

void TankBase::setRightVelocity(double v) {
    m_rightMotors->move_velocity(v);
}

void TankBase::setVoltages(double vLeft, double vRight) {
    m_leftMotors->move(vLeft);
    m_rightMotors->move(vRight);
}

void TankBase::setVelocities(double vLeft, double vRight) {
    m_leftMotors->move_velocity(vLeft);
    m_rightMotors->move_velocity(vRight);
}

void TankBase::setBrakeMode(pros::Motor_Brake mode) {
    m_leftMotors->set_brake_mode(mode);
    m_rightMotors->set_brake_mode(mode);
}

void TankBase::brake() {
    m_leftMotors->brake();
    m_rightMotors->brake();
}

pros::MotorGroup* TankBase::getLeftMotors() {
    return m_leftMotors.get();
}

pros::MotorGroup* TankBase::getRightMotors() {
    return m_rightMotors.get();
}
}