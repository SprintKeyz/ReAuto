#include "reauto/chassis/ChassisBuilder.hpp"
#include "reauto/chassis/impl/FeedbackChassis.hpp"

namespace reauto {

template <HolonomicMode HoloMode>
ChassisBuilder<HoloMode>& ChassisBuilder<HoloMode>::motors(std::initializer_list<int8_t> left, std::initializer_list<int8_t> right, pros::Motor_Gears gears) {
    m_left = left;
    m_right = right;
    m_gearset = gears;
    return *this;
}

template <HolonomicMode HoloMode>
ChassisBuilder<HoloMode>& ChassisBuilder<HoloMode>::controller(pros::Controller& c) {
    m_controller = &c;
    return *this;
}

template <HolonomicMode HoloMode>
ChassisBuilder<HoloMode>& ChassisBuilder<HoloMode>::imu(uint8_t port) {
    m_imuA = port;
    return *this;
}

template <HolonomicMode HoloMode>
ChassisBuilder<HoloMode>& ChassisBuilder<HoloMode>::imu(uint8_t portA, uint8_t portB) {
    m_imuA = portA;
    m_imuB = portB;
    return *this;
}

template <HolonomicMode HoloMode>
ChassisBuilder<HoloMode>& ChassisBuilder<HoloMode>::trackingWheels(std::pair<int8_t, double> left, std::pair<int8_t, double> right, std::pair<int8_t, double> back) {
    m_lTWheelPort = left.first;
    m_lTWheelDist = left.second;
    m_rTWheelPort = right.first;
    m_rTWheelDist = right.second;
    m_bTWheelPort = back.first;
    m_bTWheelDist = back.second;
    return *this;
}

template <HolonomicMode HoloMode>
ChassisBuilder<HoloMode>& ChassisBuilder<HoloMode>::trackingWheels(std::pair<int8_t, double> first, std::pair<int8_t, double> second, bool centerConfig) {
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

template <HolonomicMode HoloMode>
ChassisBuilder<HoloMode>& ChassisBuilder<HoloMode>::setTrackingWheelDiam(double diam) {
    m_tWheelDiam = diam;
    return *this;
}

template <HolonomicMode HoloMode>
ChassisBuilder<HoloMode>& ChassisBuilder<HoloMode>::setTrackWidth(double width) {
    m_trackWidth = width;
    return *this;
}

template <HolonomicMode HoloMode>
ChassisBuilder<HoloMode>& ChassisBuilder<HoloMode>::setWheelDiam(double diam) {
    m_wheelDiam = diam;
    return *this;
}

template <HolonomicMode HoloMode>
ChassisBuilder<HoloMode>& ChassisBuilder<HoloMode>::setGearRatio(double ratio) {
    m_gearRatio = ratio;
    return *this;
}

template <HolonomicMode HoloMode>
ChassisBuilder<HoloMode>& ChassisBuilder<HoloMode>::odom() {
    m_odomEnabled = true;
    return *this;
}

template <HolonomicMode HoloMode>
std::shared_ptr<FeedbackChassis<HoloMode>> ChassisBuilder<HoloMode>::build() {
    return std::make_shared<FeedForwardChassis<HoloMode>>(this->m_, m_right, m_gearset, *m_controller, m_trackWidth, m_gearRatio, m_wheelDiam);
}

template <HolonomicMode HoloMode>
std::shared_ptr<FeedbackChassis<HoloMode>> ChassisBuilder<HoloMode>::buildWithFeedback() {
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
}