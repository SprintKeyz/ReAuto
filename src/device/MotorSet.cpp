#include "reauto/device/MotorSet.hpp"

namespace reauto {
MotorSet::MotorSet(std::initializer_list<int8_t> ports, pros::MotorGears gearset) {
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

void MotorSet::setEncoderUnits(pros::motor_encoder_units_e_t units) {
    for (auto& motor : m_motors) {
        motor.set_encoder_units(units);
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

void MotorSet::set_brake_mode(pros::MotorBrake mode) {
    for (auto& motor : m_motors) {
        motor.set_brake_mode(mode);
    }
}

void MotorSet::brake() {
    for (auto& motor : m_motors) {
        motor.brake();
    }
}

void MotorSet::reset_position() {
    for (auto& motor : m_motors) {
        motor.tare_position();
    }
}

double MotorSet::get_position() const {
    // we just need to average the positions of all the motors!
    double sum = 0;
    for (auto& motor : m_motors) {
        sum += motor.get_position();
    }

    return sum / m_motors.size();
}

pros::MotorGears MotorSet::get_gearing() const {
    return m_motors[0].get_gearing();
}
}