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

    // I doubt this is the right way to do this, but it works
    for (int8_t i : left) {
        m_left.push_back(std::make_shared<pros::Motor>(i, gearset));
    }

    for (int8_t i : right) {
        m_right.push_back(std::make_shared<pros::Motor>(i, gearset));
    }
}

void MecanumBase::setLeftFwdVoltage(double voltage) {
    for (auto i : m_left) {
        i->move(voltage);
    }
}

void MecanumBase::setRightFwdVoltage(double voltage) {
    for (auto i : m_right) {
        i->move(voltage);
    }
}

void MecanumBase::setLeftFwdVelocity(double velocity) {
    for (auto i : m_left) {
        i->move_velocity(velocity);
    }
}

void MecanumBase::setRightFwdVelocity(double velocity) {
    for (auto i : m_right) {
        i->move_velocity(velocity);
    }
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

void MecanumBase::setStrafeVoltage(double fwdPower, double sidePower) {
    // assume that index 0 is front, and pos side power is right
    m_left[0]->move(fwdPower + sidePower);
    m_left[1]->move(fwdPower - sidePower);
    m_right[0]->move(fwdPower - sidePower);
    m_right[1]->move(fwdPower + sidePower);
}

void MecanumBase::setStrafeVelocity(double fwdVel, double sideVel) {
    // assume that index 0 is front, and pos side power is right
    m_left[0]->move_velocity(fwdVel + sideVel);
    m_left[1]->move_velocity(fwdVel - sideVel);
    m_right[0]->move_velocity(fwdVel - sideVel);
    m_right[1]->move_velocity(fwdVel + sideVel);
}

void MecanumBase::setFwdRelativeTarget(double deg, double velocity) {
    double initial = m_left[0]->get_position();

    for (auto i : m_left) {
        i->move_relative(deg, velocity);
    }

    for (auto i : m_right) {
        i->move_relative(deg, velocity);
    }

    while ((m_left[0]->get_position() - initial - deg) >= TOLERANCE_DEG || fabs(m_left[0]->get_position() - initial - deg) <= TOLERANCE_DEG) {
        pros::delay(15);
    }
}

void MecanumBase::setTurnRelativeTarget(double deg, double velocity) {
    double initial = m_left[0]->get_position();

    for (auto i : m_left) {
        i->move_relative(deg, velocity);
    }

    for (auto i : m_right) {
        i->move_relative(-deg, velocity);
    }

    while ((m_left[0]->get_position() - initial - deg) >= TOLERANCE_DEG || fabs(m_left[0]->get_position() - initial - deg) <= TOLERANCE_DEG) {
        pros::delay(15);
    }
}

void MecanumBase::setBrakeMode(pros::Motor_Brake mode) {
    for (auto i : m_left) {
        i->set_brake_mode(mode);
    }

    for (auto i : m_right) {
        i->set_brake_mode(mode);
    }
}

void MecanumBase::brake() {
    for (auto i : m_left) {
        i->brake();
    }

    for (auto i : m_right) {
        i->brake();
    }
}

std::vector<std::shared_ptr<pros::Motor>>* MecanumBase::getLeftMotors() {
    return &m_left;
}

std::vector<std::shared_ptr<pros::Motor>>* MecanumBase::getRightMotors() {
    return &m_right;
}
}