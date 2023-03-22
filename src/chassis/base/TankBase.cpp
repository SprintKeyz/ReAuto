#include "reauto/chassis/base/TankBase.hpp"
#include "reauto/math/Convert.hpp"

namespace reauto {
TankBase::TankBase(std::initializer_list<int8_t> left, std::initializer_list<int8_t> right, pros::Motor_Gears gearset) {
    // create the left and right motor groups
    m_left = std::make_shared<pros::MotorGroup>(left, gearset);
    m_right = std::make_shared<pros::MotorGroup>(right, gearset);
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

void TankBase::setFwdRelativeTarget(double deg, double velocity) {
    double initial = m_left->get_position();

    m_left->move_relative(deg, velocity);
    m_right->move_relative(deg, velocity);

    while ((m_left->get_position() - initial - deg) >= TOLERANCE_DEG || fabs(m_left->get_position() - initial - deg) <= TOLERANCE_DEG) {
        pros::delay(15);
    }
}

void TankBase::setTurnRelativeTarget(double deg, double velocity) {
    double initial = m_left->get_position();

    m_left->move_relative(deg, velocity);
    m_right->move_relative(-deg, velocity);

    while ((m_left->get_position() - initial - deg) >= TOLERANCE_DEG || fabs(m_left->get_position() - initial - deg) <= TOLERANCE_DEG) {
        pros::delay(15);
    }
}

void TankBase::setBrakeMode(pros::Motor_Brake mode) {
    m_left->set_brake_mode(mode);
    m_right->set_brake_mode(mode);
}

void TankBase::brake() {
    m_left->brake();
    m_right->brake();
}

pros::MotorGroup* TankBase::getLeftMotors() const {
    return m_left.get();
}

pros::MotorGroup* TankBase::getRightMotors() const {
    return m_right.get();
}
}