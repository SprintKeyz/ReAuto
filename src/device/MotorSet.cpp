#include "reauto/device/MotorSet.hpp"

namespace reauto {
MotorSet::MotorSet(std::initializer_list<int8_t> ports, pros::Motor_Gears gearset) {
    for (auto port : ports) {
        m_motors.emplace_back(port, gearset);
    }
}

pros::Motor& MotorSet::operator[](size_t index) {
    return m_motors[index];
}

void MotorSet::move(double voltage) {
    for (auto& motor : m_motors) {
        motor.move(voltage);
    }
}

void MotorSet::move_velocity(double velocity) {
    for (auto& motor : m_motors) {
        motor.move_velocity(velocity);
    }
}

void MotorSet::move_relative(double deg, double velocity) {
    for (auto& motor : m_motors) {
        motor.move_relative(deg, velocity);
    }
}

void MotorSet::set_brake_mode(pros::Motor_Brake mode) {
    for (auto& motor : m_motors) {
        motor.set_brake_mode(mode);
    }
}

void MotorSet::brake() {
    for (auto& motor : m_motors) {
        motor.brake();
    }
}

double MotorSet::get_position() const {
    return m_motors[0].get_position();
}
}