#include "reauto/chassis/impl/FeedbackChassis.hpp"

namespace reauto {
FeedbackChassis::FeedbackChassis(std::initializer_list<int8_t> leftPorts, std::initializer_list<int8_t> rightPorts, pros::Motor_Gears gearset, BaseType type, pros::Controller& controller, double tWidth, double gearRatio, double wheelDiam, uint8_t imuA, uint8_t imuB, std::pair<int8_t, double> leftTWheel, std::pair<int8_t, double> rightTWheel, std::pair<int8_t, double> backTWheel, double tWheelDiam): FeedForwardChassis(leftPorts, rightPorts, gearset, type, controller, tWidth, gearRatio, wheelDiam) {
    m_imu = std::make_shared<device::ADIMU>(imuA, imuB);
    m_leftTWheel = std::make_shared<device::TrackingWheel>(leftTWheel.first, leftTWheel.second, tWheelDiam);
    m_rightTWheel = std::make_shared<device::TrackingWheel>(rightTWheel.first, rightTWheel.second, tWheelDiam);
    m_backTWheel = std::make_shared<device::TrackingWheel>(backTWheel.first, backTWheel.second, tWheelDiam);

    m_centerTWheel = nullptr;
    m_tWheelConfig = TrackingWheelConfig::LEFT_RIGHT_BACK;
}

FeedbackChassis::FeedbackChassis(std::initializer_list<int8_t> leftPorts, std::initializer_list<int8_t> rightPorts, pros::Motor_Gears gearset, BaseType type, pros::Controller& controller, double tWidth, double gearRatio, double wheelDiam, uint8_t imu, std::pair<int8_t, double> leftTWheel, std::pair<int8_t, double> rightTWheel, std::pair<int8_t, double> backTWheel, double tWheelDiam): FeedForwardChassis(leftPorts, rightPorts, gearset, type, controller, tWidth, gearRatio, wheelDiam) {
    m_imu = std::make_shared<device::IMU>(imu);
    m_leftTWheel = std::make_shared<device::TrackingWheel>(leftTWheel.first, leftTWheel.second, tWheelDiam);
    m_rightTWheel = std::make_shared<device::TrackingWheel>(rightTWheel.first, rightTWheel.second, tWheelDiam);
    m_backTWheel = std::make_shared<device::TrackingWheel>(backTWheel.first, backTWheel.second, tWheelDiam);

    m_centerTWheel = nullptr;
    m_tWheelConfig = TrackingWheelConfig::LEFT_RIGHT_BACK;
}

FeedbackChassis::FeedbackChassis(std::initializer_list<int8_t> leftPorts, std::initializer_list<int8_t> rightPorts, pros::Motor_Gears gearset, BaseType type, pros::Controller& controller, double tWidth, double gearRatio, double wheelDiam, uint8_t imuA, uint8_t imuB, std::pair<int8_t, double> firstTWheel, std::pair<int8_t, double> secondTWheel, bool centerConfig, double tWheelDiam): FeedForwardChassis(leftPorts, rightPorts, gearset, type, controller, tWidth, gearRatio, wheelDiam) {
    m_imu = std::make_shared<device::ADIMU>(imuA, imuB);

    if (centerConfig) {
        m_centerTWheel = std::make_shared<device::TrackingWheel>(firstTWheel.first, firstTWheel.second, tWheelDiam);
        m_backTWheel = std::make_shared<device::TrackingWheel>(secondTWheel.first, secondTWheel.second, tWheelDiam);
        m_leftTWheel = nullptr;
        m_rightTWheel = nullptr;

        m_tWheelConfig = TrackingWheelConfig::CENTER_BACK;
    }

    else {
        m_leftTWheel = std::make_shared<device::TrackingWheel>(firstTWheel.first, firstTWheel.second, tWheelDiam);
        m_rightTWheel = std::make_shared<device::TrackingWheel>(secondTWheel.first, secondTWheel.second, tWheelDiam);
        m_centerTWheel = nullptr;
        m_backTWheel = nullptr;

        m_tWheelConfig = TrackingWheelConfig::LEFT_RIGHT;
    }
}

FeedbackChassis::FeedbackChassis(std::initializer_list<int8_t> leftPorts, std::initializer_list<int8_t> rightPorts, pros::Motor_Gears gearset, BaseType type, pros::Controller& controller, double tWidth, double gearRatio, double wheelDiam, uint8_t imu, std::pair<int8_t, double> firstTWheel, std::pair<int8_t, double> secondTWheel, bool centerConfig, double tWheelDiam): FeedForwardChassis(leftPorts, rightPorts, gearset, type, controller, tWidth, gearRatio, wheelDiam) {
    m_imu = std::make_shared<device::IMU>(imu);

    if (centerConfig) {
        m_centerTWheel = std::make_shared<device::TrackingWheel>(firstTWheel.first, firstTWheel.second, tWheelDiam);
        m_backTWheel = std::make_shared<device::TrackingWheel>(secondTWheel.first, secondTWheel.second, tWheelDiam);
        m_leftTWheel = nullptr;
        m_rightTWheel = nullptr;

        m_tWheelConfig = TrackingWheelConfig::CENTER_BACK;
    }

    else {
        m_leftTWheel = std::make_shared<device::TrackingWheel>(firstTWheel.first, firstTWheel.second, tWheelDiam);
        m_rightTWheel = std::make_shared<device::TrackingWheel>(secondTWheel.first, secondTWheel.second, tWheelDiam);
        m_centerTWheel = nullptr;
        m_backTWheel = nullptr;

        m_tWheelConfig = TrackingWheelConfig::LEFT_RIGHT;
    }
}
}