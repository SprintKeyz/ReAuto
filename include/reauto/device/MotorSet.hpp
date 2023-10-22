#pragma once

// this exists because PROS 4 removed the ability to get individual motors from a MotorGroup
#include "pros/abstract_motor.hpp"
#include <initializer_list>
#include "pros/motors.hpp"

namespace reauto
{
class MotorSet
{
public:
    explicit MotorSet(std::initializer_list<int8_t> ports, pros::MotorGears gearset);

    // disable copy constructor
    MotorSet(const MotorSet&) = delete;

    // add array index operator
    pros::Motor& operator[](size_t index);

    void move(double voltage);
    void move_velocity(double velocity);
    void move_relative(double deg, double velocity);
    void set_brake_mode(pros::MotorBrake mode);
    void brake();
    double get_position() const;
    void reset_position();
    pros::MotorGears get_gearing() const;

private:
    std::vector<pros::Motor> m_motors;

};
}