#pragma once

#include <initializer_list>
#include <iostream>
#include <variant>
#include "pros/abstract_motor.hpp"
#include "pros/misc.hpp"
#include "reauto/chassis/impl/FeedForwardChassis.hpp"
#include "reauto/chassis/impl/FeedbackChassis.hpp"

namespace reauto {
enum class HolonomicMode {
    NONE = 0,
    MECANUM = 1,
    X = 2,
};

class ChassisBuilder {
public:
    // pass motor ports
    ChassisBuilder& motors(std::initializer_list<int8_t> left, std::initializer_list<int8_t> right, pros::Motor_Gears gears);

    // pass reference to controller
    ChassisBuilder& controller(pros::Controller& c);

    // pass IMU port(s)
    ChassisBuilder& imu(uint8_t port);
    ChassisBuilder& imu(uint8_t portA, uint8_t portB);

    // pass tracking wheel config (port, center dist)
    ChassisBuilder& trackingWheels(std::pair<int8_t, double> left, std::pair<int8_t, double> right, std::pair<int8_t, double> back);
    ChassisBuilder& trackingWheels(std::pair<int8_t, double> first, std::pair<int8_t, double> second, bool centerConfig = false);

    // set tracking wheel diameter
    ChassisBuilder& setTrackingWheelDiam(double diam);

    // set the chassis track width
    ChassisBuilder& setTrackWidth(double width);

    // set the chassis wheel diameter
    ChassisBuilder& setWheelDiam(double diam);

    // set the chassis gear ratio
    ChassisBuilder& setGearRatio(double ratio);

    // enable odometry
    ChassisBuilder& odom();

    // set holonomic mode
    ChassisBuilder& holonomic(HolonomicMode mode);

    // build it!
    std::shared_ptr<FeedForwardChassis> build();
    std::shared_ptr<FeedbackChassis> buildFeedbackChassis();

private:
    // wheel ports
    std::initializer_list<int8_t> m_left;
    std::initializer_list<int8_t> m_right;

    // gearset
    pros::Motor_Gears m_gearset;

    // ptr to controller
    pros::Controller* m_controller;

    // IMU ports
    uint8_t m_imuA = 0;
    uint8_t m_imuB = 0;

    // tracking wheel ports
    int8_t m_lTWheelPort = 0;
    int8_t m_rTWheelPort = 0;
    int8_t m_cTWheelPort = 0;
    int8_t m_bTWheelPort = 0;

    // tracking wheel distances
    double m_lTWheelDist = 0;
    double m_rTWheelDist = 0;
    double m_cTWheelDist = 0;
    double m_bTWheelDist = 0;

    // tracking wheel diameter
    double m_tWheelDiam = 0;

    // robot properties
    double m_trackWidth = 0;
    double m_wheelDiam = 0;
    double m_gearRatio = 0;

    // use odometry?
    bool m_odomEnabled = false;

    // set holonomic mode
    HolonomicMode m_holoMode = HolonomicMode::NONE;
};
}