#pragma once

#include <initializer_list>
#include <iostream>
#include <variant>
#include "pros/abstract_motor.hpp"
#include "pros/misc.hpp"
#include "reauto/chassis/impl/FeedForwardChassis.hpp"
#include "reauto/chassis/impl/FeedbackChassis.hpp"
#include "reauto/chassis/impl/HolonomicMode.hpp"

namespace reauto {

template <HolonomicMode HoloMode = HolonomicMode::NONE>
class ChassisBuilder {
public:
    // pass motor ports
    ChassisBuilder& motors(std::initializer_list<int8_t> left, std::initializer_list<int8_t> right, pros::Motor_Gears gears) {
        m_left = left;
        m_right = right;
        m_gearset = gears;
        return *this;
    }

    // pass reference to controller
    ChassisBuilder& controller(pros::Controller& c) {
        m_controller = &c;
        return *this;
    }

    // pass IMU port(s)
    ChassisBuilder& imu(uint8_t port) {
        m_imuA = port;
        return *this;
    }

    ChassisBuilder& imu(uint8_t portA, uint8_t portB) {
        m_imuA = portA;
        m_imuB = portB;
        return *this;
    }

    // pass tracking wheel config (port, center dist)
    ChassisBuilder& trackingWheels(std::pair<int8_t, double> left, std::pair<int8_t, double> right, std::pair<int8_t, double> back) {
        m_lTWheelPort = left.first;
        m_lTWheelDist = left.second;
        m_rTWheelPort = right.first;
        m_rTWheelDist = right.second;
        m_bTWheelPort = back.first;
        m_bTWheelDist = back.second;
        return *this;
    }

    ChassisBuilder& trackingWheels(std::pair<int8_t, double> first, std::pair<int8_t, double> second, bool centerConfig = false) {
        if (centerConfig) {
            // center + back wheels
            m_cTWheelPort = first.first;
            m_cTWheelDist = first.second;
            m_bTWheelPort = second.first;
            m_bTWheelDist = second.second;
        }

        else {
            // left + right wheels
            m_lTWheelPort = first.first;
            m_lTWheelDist = first.second;
            m_rTWheelPort = second.first;
            m_rTWheelDist = second.second;
        }
        return *this;
    }

    // set tracking wheel diameter
    ChassisBuilder& setTrackingWheelDiam(double diam) {
        m_tWheelDiam = diam;
        return *this;
    }

    // set the chassis track width
    ChassisBuilder& setTrackWidth(double width) {
        m_trackWidth = width;
        return *this;
    }

    // set the chassis wheel diameter
    ChassisBuilder& setWheelDiam(double diam) {
        m_wheelDiam = diam;
        return *this;
    }

    // set the chassis gear ratio
    ChassisBuilder& setGearRatio(double ratio) {
        m_gearRatio = ratio;
        return *this;
    }

    // enable odometry
    ChassisBuilder& odom() {
        m_odomEnabled = true;
        return *this;
    }

    // build the chassis
    std::shared_ptr<FeedForwardChassis<HoloMode>> build() {
        return std::make_shared<FeedForwardChassis<HoloMode>>(m_left, m_right, m_gearset, *m_controller, m_trackWidth, m_gearRatio, m_wheelDiam);
    }

    // build the chassis with feedback
    std::shared_ptr<FeedbackChassis<HoloMode>> buildWithFeedback() {
        // check if tracking wheels are valid
    // INVALID: only back wheel, only left or right wheel, only center wheel, no diam
        if ((m_lTWheelPort == 0 && m_rTWheelPort == 0 && m_cTWheelPort == 0) ||
            m_tWheelDiam <= 0 ||
            (m_lTWheelPort == 0 && m_cTWheelPort == 0 && m_rTWheelPort != 0) ||
            (m_lTWheelPort != 0 && m_cTWheelPort == 0 && m_rTWheelPort == 0) ||
            (m_lTWheelPort == 0 && m_cTWheelPort != 0 && m_rTWheelPort == 0 && m_bTWheelPort == 0) ||
            (m_lTWheelPort == 0 && m_cTWheelPort == 0 && m_rTWheelPort == 0 && m_bTWheelPort != 0)) {
            std::cout << "[ReAuto] ERROR: Your tracking wheel configuration is not valid. Please check your configuration." << std::endl;
            return nullptr;
        }

        if (m_imuA == 0) {
            std::cout << "[ReAuto] ERROR: You must specify an IMU port for the feedback chassis." << std::endl;
            return nullptr;
        }

        // make tracking wheel pairs for each possible wheel
        std::pair<int8_t, double> lTWheel = std::make_pair(m_lTWheelPort, m_lTWheelDist);
        std::pair<int8_t, double> rTWheel = std::make_pair(m_rTWheelPort, m_rTWheelDist);
        std::pair<int8_t, double> cTWheel = std::make_pair(m_cTWheelPort, m_cTWheelDist);
        std::pair<int8_t, double> bTWheel = std::make_pair(m_bTWheelPort, m_bTWheelDist);

        if (m_lTWheelPort != 0 && m_rTWheelPort != 0 && m_bTWheelPort != 0) {
            if (m_imuB != 0)
                return std::make_shared<FeedbackChassis<HoloMode>>(m_left, m_right, m_gearset, *m_controller, m_trackWidth, m_gearRatio, m_wheelDiam, m_imuA, m_imuB, lTWheel, rTWheel, bTWheel, m_tWheelDiam);
            else
                return std::make_shared<FeedbackChassis<HoloMode>>(m_left, m_right, m_gearset, *m_controller, m_trackWidth, m_gearRatio, m_wheelDiam, m_imuA, lTWheel, rTWheel, bTWheel, m_tWheelDiam);
        }

        // the false indicates that it is NOT a center t wheel config
        if (m_lTWheelPort != 0 && m_rTWheelDist != 0) {
            if (m_imuB != 0)
                return std::make_shared<FeedbackChassis<HoloMode>>(m_left, m_right, m_gearset, *m_controller, m_trackWidth, m_gearRatio, m_wheelDiam, m_imuA, m_imuB, lTWheel, rTWheel, false, m_tWheelDiam);
            else
                return std::make_shared<FeedbackChassis<HoloMode>>(m_left, m_right, m_gearset, *m_controller, m_trackWidth, m_gearRatio, m_wheelDiam, m_imuA, lTWheel, rTWheel, false, m_tWheelDiam);
        }

        // all that is left is the center t wheel config
        if (m_imuB != 0)
            return std::make_shared<FeedbackChassis<HoloMode>>(m_left, m_right, m_gearset, *m_controller, m_trackWidth, m_gearRatio, m_wheelDiam, m_imuA, m_imuB, cTWheel, bTWheel, true, m_tWheelDiam);
        else
            return std::make_shared<FeedbackChassis<HoloMode>>(m_left, m_right, m_gearset, *m_controller, m_trackWidth, m_gearRatio, m_wheelDiam, m_imuA, cTWheel, bTWheel, true, m_tWheelDiam);
    }

    // <build with odom here>

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
};
}