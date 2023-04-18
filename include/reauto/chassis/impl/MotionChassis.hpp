#pragma once

#include "pros/abstract_motor.hpp"
#include "reauto/chassis/base/MecanumBase.hpp"
#include "reauto/chassis/base/TankBase.hpp"
#include "reauto/chassis/template/RobotTemplate.hpp"
#include "reauto/datatypes/Pose.h"
#include "reauto/device/MotorSet.hpp"
#include "reauto/chassis/impl/HolonomicMode.hpp"
#include "reauto/device/ADIMU.hpp"
#include "reauto/device/TrackingWheel.hpp"
#include "reauto/device/TrackingWheels.hpp"
#include "reauto/odom/Odometry.hpp"
#include <initializer_list>

#define MOTION_TIMESTEP 10

namespace reauto {
struct RobotMeasurements {
    double trackWidth;
    double wheelDiameter;
    double gearRatio;
};

class MotionChassis {
public:
    // this is the ONLY constructor that should be used
    explicit MotionChassis(
        std::initializer_list<int8_t> left,
        std::initializer_list<int8_t> right,
        pros::Motor_Gears gearset,
        pros::Controller& controller,
        HolonomicMode holoMode,
        uint8_t imuPort,
        uint8_t secondaryImuPort,
        int8_t firstTWheelPort,
        double firstTWheelDist,
        int8_t secondTWheelPort,
        double secondTWheelDist,
        int8_t thirdTWheelPort,
        double thirdTWheelDist,
        double tWheelDiam,
        TrackingConfiguration tConfig,
        double trackWidth,
        double wheelDiameter,
        double gearRatio
    );

    // init chassis!
    void init();

    // functions!
    void setLeftVoltage(double voltage);
    void setRightVoltage(double voltage);
    void setVoltage(double left, double right);

    void setLeftVelocity(double velocity);
    void setRightVelocity(double velocity);
    void setVelocity(double left, double right);

    void setBrakeMode(pros::Motor_Brake mode);
    void brake();

    // for driving

    // set the slew step for driver control
    // signChange is the slew step for when the sign changes
    void setSlewDrive(double normal, double signChange = 0);

    // set an exponential curve for driver control
    void setDriveExponent(double exponent);

    // set the maximum speed for driver control
    void setDriveMaxSpeed(double maxSpeed);

    // set custom behavior for driver control
    void setCustomDriveBehavior(std::function<double(double)> behavior);

    // set the controller deadband for driver control
    void setControllerDeadband(double deadband);

    // set the channels to be used for arcade drive
    void setArcadeDriveChannels(pros::controller_analog_e_t forwardChannel, pros::controller_analog_e_t turnChannel);

    // update the chassis tank drive with joystick values
    void tank(double speedScale = 127);

    // update the chassis arcade drive with joystick values
    void arcade(double speedScale = 127);

    // get the current heading
    double getHeading(bool rad = false) const;

    // set the current heading (0 to 360)
    void setHeading(double deg);

    // get the robot pose
    Pose getPose() const;

    // set the robot pose
    void setPose(Pose p);

    // get the left and right velocities
    double getLeftVelocity() const;
    double getRightVelocity() const;

    // get the tracking wheels
    TrackingWheels* getTrackingWheels() const;

    RobotMeasurements getMeasurements() const;

private:
    // base robot drivetrain
    std::shared_ptr<RobotTemplate> m_robot;
    HolonomicMode m_holoMode = HolonomicMode::NONE;

    // IMU (or ADIMU)
    std::shared_ptr<device::IMU> m_imu;

    // controller
    pros::Controller& m_controller;

    // tracking wheels
    std::shared_ptr<TrackingWheels> m_trackingWheels;

    // robot hardware measurements
    RobotMeasurements m_measurements;

    // for driver control
    double m_slewStep = 0;
    double m_slewStepSignChange = 0;
    double m_exponent = 0;
    double m_deadband = 0;
    double m_driveMaxSpeed = 0;

    // use custom behavior for driver control?
    bool m_useCustomBehavior = false;
    std::function<double(double)> m_driveCustomBehavior;

    // for arcade drive
    pros::controller_analog_e_t m_forwardChannel = pros::E_CONTROLLER_ANALOG_LEFT_Y;
    pros::controller_analog_e_t m_turnChannel = pros::E_CONTROLLER_ANALOG_RIGHT_X;

    // for slew
    double m_currentLeftVoltage = 0;
    double m_currentRightVoltage = 0;

    double calcExponentialDrive(double input);

    // odometry object!
    std::shared_ptr<Odometry> m_odom;
    Pose m_pose = { 0, 0, 0 };
};
}