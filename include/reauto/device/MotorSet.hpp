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
    explicit MotorSet(std::initializer_list<int8_t> ports, pros::Motor_Gears gearset);

    // disable copy constructor
    MotorSet(const MotorSet&) = delete;

    // add array index operator
    pros::Motor& operator[](size_t index);

    void move(double voltage);
    void move_velocity(double velocity);
    void move_relative(double deg, double velocity);
    void set_brake_mode(pros::Motor_Brake mode);
    void brake();
    double get_position() const;

private:
    std::vector<pros::Motor> m_motors;

};
}