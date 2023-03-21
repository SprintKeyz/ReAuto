#pragma once

#include <initializer_list>
#include <iostream>
#include "pros/abstract_motor.hpp"
#include "pros/misc.hpp"
#include "reauto/chassis/impl/FeedForwardChassis.hpp"

namespace reauto {
enum class HolonomicMode {
    MECANUM = 1,
    X = 2,
};

class ChassisBuilder {
public:
    // ctor
    ChassisBuilder() = default;

    // pass motor ports
    ChassisBuilder& motors(std::initializer_list<int8_t> left, std::initializer_list<int8_t> right, pros::Motor_Gears gears);

    // pass reference to controller
    ChassisBuilder& controller(pros::Controller& c);

    // pass IMU port(s)
    ChassisBuilder& imu(uint8_t port);
    ChassisBuilder& imu(uint8_t portA, uint8_t portB);

    // pass tracking wheel config
    ChassisBuilder& trackingWheels(std::pair<double, double> left, std::pair<double, double> right, std::pair<double, double> back);
    ChassisBuilder& trackingWheels(std::pair<double, double> first, std::pair<double, double> second, bool centerConfig = false);

    // set tracking wheel diameter
    ChassisBuilder& setTrackingWheelDiam(double diam);

    // set the chassis track width
    ChassisBuilder& setTrackWidth(double w);

    // set the chassis wheel diameter
    ChassisBuilder& setWheelDiam(double diam);

    // set the chassis gear ratio
    ChassisBuilder& setGearRatio(double ratio);

    // enable odometry
    ChassisBuilder& odom();

    // set holonomic mode
    ChassisBuilder& holonomic(HolonomicMode mode);

    // build it!
    FeedForwardChassis* build();
};
}