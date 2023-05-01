#include "reauto/chassis/base/MecanumBase.hpp"
#include "reauto/math/Convert.hpp"

namespace reauto {
MecanumBase::MecanumBase(std::initializer_list<int8_t> left, std::initializer_list<int8_t> right, pros::Motor_Gears gearset) {
    // create the left and right motor groups

    if (left.size() != right.size()) {
        throw std::invalid_argument("[ReAuto] Left and right motor groups must be the same size");
    }

    if (left.size() != 2) {
        throw std::invalid_argument("[ReAuto] Motor groups must be of size 2");
    }

    m_left = std::make_shared<MotorSet>(left, gearset);
    m_right = std::make_shared<MotorSet>(right, gearset);
}

void MecanumBase::setLeftFwdVoltage(double voltage) {
    m_left->move(voltage);
}

void MecanumBase::setRightFwdVoltage(double voltage) {
    m_right->move(voltage);
}

void MecanumBase::setLeftStrafeVoltage(double voltage) {
    m_left.get()[0].move(voltage);
    m_left.get()[1].move(-voltage);
}

void MecanumBase::setRightStrafeVoltage(double voltage) {
    m_right.get()[0].move(-voltage);
    m_right.get()[1].move(voltage);
}

void MecanumBase::setLeftFwdVelocity(double velocity) {
    m_left->move_velocity(velocity);
}

void MecanumBase::setRightFwdVelocity(double velocity) {
    m_right->move_velocity(velocity);
}

void MecanumBase::setLeftStrafeVelocity(double velocity) {
    m_left.get()[0].move_velocity(velocity);
    m_left.get()[1].move_velocity(-velocity);
}

void MecanumBase::setRightStrafeVelocity(double velocity) {
    m_right.get()[0].move_velocity(-velocity);
    m_right.get()[1].move_velocity(velocity);
}

void MecanumBase::setFwdVoltage(double voltage) {
    setLeftFwdVoltage(voltage);
    setRightFwdVoltage(voltage);
}

void MecanumBase::setFwdVelocity(double velocity) {
    setLeftFwdVelocity(velocity);
    setRightFwdVelocity(velocity);
}

void MecanumBase::setTurnVoltage(double voltage) {
    setLeftFwdVoltage(voltage);
    setRightFwdVoltage(-voltage);
}

void MecanumBase::setTurnVelocity(double velocity) {
    setLeftFwdVelocity(velocity);
    setRightFwdVelocity(-velocity);
}

double MecanumBase::getLeftVelocity() const {
    return (*m_left)[0].get_actual_velocity();
}

void MecanumBase::setStrafeVoltage(double fwdPower, double sidePower) {
    // assume that index 0 is front, and pos side power is right

    m_left.get()[0].move(fwdPower + sidePower);
    m_left.get()[1].move(fwdPower - sidePower);
    m_right.get()[0].move(fwdPower - sidePower);
    m_right.get()[1].move(fwdPower + sidePower);
}

void MecanumBase::setStrafeVelocity(double fwdVel, double sideVel) {
    // assume that index 0 is front, and pos side power is right
    m_left.get()[0].move_velocity(fwdVel + sideVel);
    m_left.get()[1].move_velocity(fwdVel - sideVel);
    m_right.get()[0].move_velocity(fwdVel - sideVel);
    m_right.get()[1].move_velocity(fwdVel + sideVel);
}

void MecanumBase::setBrakeMode(pros::Motor_Brake mode) {
    m_left->set_brake_mode(mode);
    m_right->set_brake_mode(mode);
}

void MecanumBase::brake() {
    m_left->brake();
    m_right->brake();
}

MotorSet* MecanumBase::getLeftMotors() const {
    return m_left.get();
}

MotorSet* MecanumBase::getRightMotors() const {
    return m_right.get();
}
}