#pragma once

#include "reauto/chassis/impl/FeedForwardChassis.hpp"
#include "reauto/device/TrackingWheel.hpp"
#include "reauto/device/ADIMU.hpp"

namespace reauto {
// tracking wheel config enum
enum class TrackingWheelConfig {
    LEFT_RIGHT = 0,
    CENTER_BACK = 1,
    LEFT_RIGHT_BACK = 2,
};

template <HolonomicMode HoloMode>
class FeedbackChassis: public FeedForwardChassis<HoloMode> {
    friend class BangBangController;

public:
    FeedbackChassis(std::initializer_list<int8_t> leftPorts, std::initializer_list<int8_t> rightPorts, pros::Motor_Gears gearset, pros::Controller& controller, double tWidth, double gearRatio, double wheelDiam, uint8_t imuA, uint8_t imuB, std::pair<int8_t, double> leftTWheel, std::pair<int8_t, double> rightTWheel, std::pair<int8_t, double> backTWheel, double tWheelDiam);
    FeedbackChassis(std::initializer_list<int8_t> leftPorts, std::initializer_list<int8_t> rightPorts, pros::Motor_Gears gearset, pros::Controller& controller, double tWidth, double gearRatio, double wheelDiam, uint8_t imu, std::pair<int8_t, double> leftTWheel, std::pair<int8_t, double> rightTWheel, std::pair<int8_t, double> backTWheel, double tWheelDiam);
    FeedbackChassis(std::initializer_list<int8_t> leftPorts, std::initializer_list<int8_t> rightPorts, pros::Motor_Gears gearset, pros::Controller& controller, double tWidth, double gearRatio, double wheelDiam, uint8_t imuA, uint8_t imuB, std::pair<int8_t, double> firstTWheel, std::pair<int8_t, double> secondTWheel, bool centerConfig, double tWheelDiam);
    FeedbackChassis(std::initializer_list<int8_t> leftPorts, std::initializer_list<int8_t> rightPorts, pros::Motor_Gears gearset, pros::Controller& controller, double tWidth, double gearRatio, double wheelDiam, uint8_t imu, std::pair<int8_t, double> firstTWheel, std::pair<int8_t, double> secondTWheel, bool centerConfig, double tWheelDiam);

    // getting sensor values
    inline device::TrackingWheel* getLeftTrackingWheel() const { return m_leftTWheel.get(); }
    inline device::TrackingWheel* getRightTrackingWheel() const { return m_rightTWheel.get(); }
    inline device::TrackingWheel* getCenterTrackingWheel() const { return m_centerTWheel.get(); }
    inline device::TrackingWheel* getBackTrackingWheel() const { return m_backTWheel.get(); }
    inline device::IMU* getIMU() const { return m_imu.get(); }
    inline TrackingWheelConfig getTrackingWheelConfig() const { return m_tWheelConfig; }


private:
    // sensors
    std::shared_ptr<device::IMU> m_imu = nullptr;
    std::shared_ptr<device::TrackingWheel> m_leftTWheel = nullptr;
    std::shared_ptr<device::TrackingWheel> m_rightTWheel = nullptr;
    std::shared_ptr<device::TrackingWheel> m_centerTWheel = nullptr;
    std::shared_ptr<device::TrackingWheel> m_backTWheel = nullptr;
    TrackingWheelConfig m_tWheelConfig;
};
}