#pragma once

#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "reauto/chassis/template/RobotTemplate.hpp"
#include "reauto/chassis/base/TankBase.hpp"

namespace reauto {

// keep track of the type of robot base
enum class BaseType {
    TANK = 0,
    MECANUM = 1,
};

class FeedForwardChassis {
public:
    FeedForwardChassis(std::initializer_list<int8_t> leftPorts, std::initializer_list<int8_t> rightPorts, pros::Motor_Gears gearset, BaseType type, pros::Controller& controller);
    FeedForwardChassis(std::initializer_list<int8_t> leftPorts, std::initializer_list<int8_t> rightPorts, pros::Motor_Gears gearset, BaseType type, pros::Controller& controller, double tWidth, double gearRatio, double wheelDiam);

    // virtual because we can override this in the feedback version
    virtual void drive(double distance, double velocity, bool blocking = true);
    virtual void turn(double angle, double velocity, bool blocking = true);

    // set the velocity and voltage of the motors
    void setleftVelocity(double velocity);
    void setrightVelocity(double velocity);
    void setleftVoltage(double voltage);
    void setrightVoltage(double voltage);

    // set forward and turn velocities
    void setForwardVelocity(double velocity);
    void setTurnVelocity(double velocity);

    // set forward and turn voltages
    void setForwardVoltage(double voltage);
    void setTurnVoltage(double voltage);

    // brake the motors
    void brake();

    // set the brake mode of the motors
    void setBrakeMode(pros::Motor_Brake mode);

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

private:
    std::shared_ptr<RobotTemplate> m_baseRobot;
    const BaseType m_baseRobotType;

    // a reference to the controller
    pros::Controller& m_controller;

    const double m_trackWidth = 0;
    const double m_gearRatio = 0;
    const double m_wheelDiameter = 0;

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
    double m_currentLeftVoltage;
    double m_currentRightVoltage;

    double calcExponentialDrive(double input);
};
}