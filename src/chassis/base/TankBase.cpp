#include "reauto/chassis/base/TankBase.hpp"
#include "reauto/math/Convert.hpp"

namespace reauto {
TankBase::TankBase(std::initializer_list<int8_t> left, std::initializer_list<int8_t> right, pros::Motor_Gears gearset) {
    // create the left and right motor groups
    m_left = std::make_shared<MotorSet>(left, gearset);
    m_right = std::make_shared<MotorSet>(right, gearset);
}

void TankBase::setLeftFwdVoltage(double voltage) {
    m_left->move(voltage);
}

void TankBase::setRightFwdVoltage(double voltage) {
    m_right->move(voltage);
}

void TankBase::setLeftFwdVelocity(double velocity) {
    m_left->move_velocity(velocity);
}

void TankBase::setRightFwdVelocity(double velocity) {
    m_right->move_velocity(velocity);
}

void TankBase::setFwdVoltage(double voltage) {
    setLeftFwdVoltage(voltage);
    setRightFwdVoltage(voltage);
}

void TankBase::setFwdVelocity(double velocity) {
    setLeftFwdVelocity(velocity);
    setRightFwdVelocity(velocity);
}

void TankBase::setTurnVoltage(double voltage) {
    setLeftFwdVoltage(voltage);
    setRightFwdVoltage(-voltage);
}

void TankBase::setTurnVelocity(double velocity) {
    setLeftFwdVelocity(velocity);
    setRightFwdVelocity(-velocity);
}

void TankBase::setBrakeMode(pros::Motor_Brake mode) {
    m_left->set_brake_mode(mode);
    m_right->set_brake_mode(mode);
}

void TankBase::brake() {
    m_left->brake();
    m_right->brake();
}

MotorSet* TankBase::getLeftMotors() const {
    return m_left.get();
}

MotorSet* TankBase::getRightMotors() const {
    return m_right.get();
}

double TankBase::getLeftVelocity() const {
    std::cout << "called" << std::endl;
    return (*m_left)[0].get_actual_velocity();
}
}