#include "reauto/chassis/impl/FeedForwardChassis.hpp"
#include "reauto/chassis/impl/HolonomicMode.hpp"
#include "reauto/motion/Slew.hpp"
#include "reauto/math/Convert.hpp"

// bases
#include "reauto/chassis/base/TankBase.hpp"
#include "reauto/chassis/base/MecanumBase.hpp"
#include "reauto/chassis/template/RobotTemplate.hpp"

namespace reauto {

template <HolonomicMode HoloMode>
FeedForwardChassis<HoloMode>::FeedForwardChassis(std::initializer_list<int8_t> leftPorts, std::initializer_list<int8_t> rightPorts, pros::Motor_Gears gearset, pros::Controller& controller)
    :
    m_controller(controller),
    m_trackWidth(0.0),
    m_gearRatio(0.0),
    m_wheelDiameter(0.0) {

    switch (HoloMode) {
    case HolonomicMode::NONE:
        m_baseRobot = std::make_shared<TankBase>(leftPorts, rightPorts, gearset);
        break;
    case HolonomicMode::MECANUM:
        m_baseRobot = std::make_shared<MecanumBase>(leftPorts, rightPorts, gearset);
    }
}

template <HolonomicMode HoloMode>
FeedForwardChassis<HoloMode>::FeedForwardChassis(std::initializer_list<int8_t> leftPorts, std::initializer_list<int8_t> rightPorts, pros::Motor_Gears gearset, pros::Controller& controller, double tWidth, double gearRatio, double wheelDiam)
    :
    m_controller(controller),
    m_trackWidth(tWidth),
    m_gearRatio(gearRatio),
    m_wheelDiameter(wheelDiam) {

    switch (HoloMode) {
    case HolonomicMode::NONE:
        m_baseRobot = std::make_shared<TankBase>(leftPorts, rightPorts, gearset);
        break;
    case HolonomicMode::MECANUM:
        m_baseRobot = std::make_shared<MecanumBase>(leftPorts, rightPorts, gearset);
    }
}

template <HolonomicMode HoloMode>
void FeedForwardChassis<HoloMode>::drive(double distance, double velocity) {
    // convert distance to degrees
    double degrees = math::inToDeg(distance, m_wheelDiameter);

    // velocity is in RPM
    m_baseRobot->setFwdRelativeTarget(degrees, velocity);
}

template <HolonomicMode HoloMode>
void FeedForwardChassis<HoloMode>::turn(double angle, double velocity) {
    // TODO: Implement turns!
    throw std::runtime_error("Pure FeedForward turns are not implemented yet!");
}

template <HolonomicMode HoloMode>
void FeedForwardChassis<HoloMode>::setleftVelocity(double velocity) {
    m_baseRobot->setLeftFwdVelocity(velocity);
}

template <HolonomicMode HoloMode>
void FeedForwardChassis<HoloMode>::setrightVelocity(double velocity) {
    m_baseRobot->setRightFwdVelocity(velocity);
}

template <HolonomicMode HoloMode>
void FeedForwardChassis<HoloMode>::setleftVoltage(double voltage) {
    m_baseRobot->setLeftFwdVoltage(voltage);
}

template <HolonomicMode HoloMode>
void FeedForwardChassis<HoloMode>::setrightVoltage(double voltage) {
    m_baseRobot->setRightFwdVoltage(voltage);
}

template <HolonomicMode HoloMode>
void FeedForwardChassis<HoloMode>::setForwardVelocity(double velocity) {
    m_baseRobot->setFwdVelocity(velocity);
}

template <HolonomicMode HoloMode>
void FeedForwardChassis<HoloMode>::setTurnVelocity(double velocity) {
    m_baseRobot->setTurnVelocity(velocity);
}

template <HolonomicMode HoloMode>
void FeedForwardChassis<HoloMode>::setForwardVoltage(double voltage) {
    m_baseRobot->setFwdVoltage(voltage);
}

template <HolonomicMode HoloMode>
void FeedForwardChassis<HoloMode>::setTurnVoltage(double voltage) {
    m_baseRobot->setTurnVoltage(voltage);
}

template <HolonomicMode HoloMode>
void FeedForwardChassis<HoloMode>::setBrakeMode(pros::Motor_Brake mode) {
    m_baseRobot->setBrakeMode(mode);
}

template <HolonomicMode HoloMode>
void FeedForwardChassis<HoloMode>::brake() {
    m_baseRobot->brake();
}

template <HolonomicMode HoloMode>
void FeedForwardChassis<HoloMode>::setSlewDrive(double normal, double signChange) {
    m_slewStep = normal;
    m_slewStepSignChange = signChange > 0 ? signChange : normal;
}

template <HolonomicMode HoloMode>
void FeedForwardChassis<HoloMode>::setDriveExponent(double exponent) {
    m_exponent = exponent;
}

template <HolonomicMode HoloMode>
void FeedForwardChassis<HoloMode>::setDriveMaxSpeed(double maxSpeed) {
    m_driveMaxSpeed = maxSpeed;
}

template <HolonomicMode HoloMode>
void FeedForwardChassis<HoloMode>::setControllerDeadband(double deadband) {
    m_deadband = deadband;
}

template <HolonomicMode HoloMode>
void FeedForwardChassis<HoloMode>::setArcadeDriveChannels(pros::controller_analog_e_t forwardChannel, pros::controller_analog_e_t turnChannel) {
    m_forwardChannel = forwardChannel;
    m_turnChannel = turnChannel;
}

template <HolonomicMode HoloMode>
double FeedForwardChassis<HoloMode>::calcExponentialDrive(double input) {
    return (std::pow(input, m_exponent) / (std::pow(100, 2)));
}

template <HolonomicMode HoloMode>
void FeedForwardChassis<HoloMode>::setCustomDriveBehavior(std::function<double(double)> function) {
    m_driveCustomBehavior = function;
    m_useCustomBehavior = true;
}

template <HolonomicMode HoloMode>
void FeedForwardChassis<HoloMode>::tank(double speedScale) {
    // get left and right joystick values
    double left = m_controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    double right = m_controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // account for deadband
    left = (fabs(left) < m_deadband) ? 0 : left;
    right = (fabs(right) < m_deadband) ? 0 : right;

    // apply the custom drive behavior
    if (m_useCustomBehavior) {
        left = m_driveCustomBehavior(left);
        right = m_driveCustomBehavior(right);
    }

    else if (m_exponent != 0) {
        // apply the default exponential drive implementation
        left = calcExponentialDrive(left);
        right = calcExponentialDrive(right);
    }

    // slew if applicable
    if (m_slewStep > 0) {
        util::slew(m_currentLeftVoltage, left, std::signbit(left) != std::signbit(m_currentLeftVoltage) ? m_slewStepSignChange : m_slewStep);
        util::slew(m_currentRightVoltage, right, std::signbit(right) != std::signbit(m_currentRightVoltage) ? m_slewStepSignChange : m_slewStep);
    }

    // account for max speed
    if (m_driveMaxSpeed > 0) {
        left = std::clamp(left, -m_driveMaxSpeed, m_driveMaxSpeed);
        right = std::clamp(right, -m_driveMaxSpeed, m_driveMaxSpeed);
    }

    // apply speed scale
    left *= speedScale / 127.0;
    right *= speedScale / 127.0;

    // set the motor voltages
    m_baseRobot->setLeftFwdVoltage(left);
    m_baseRobot->setRightFwdVoltage(right);

    // update the current voltage values
    m_currentLeftVoltage = left;
    m_currentRightVoltage = right;
}

template <HolonomicMode HoloMode>
void FeedForwardChassis<HoloMode>::arcade(double speedScale) {
    // get forward and turn joystick values
    double forward = m_controller.get_analog(m_forwardChannel);
    double turn = m_controller.get_analog(m_turnChannel);

    // get the left and right motor voltages
    double left = forward - turn;
    double right = forward + turn;

    // account for deadband
    left = (fabs(left) < m_deadband) ? 0 : left;
    right = (fabs(right) < m_deadband) ? 0 : right;

    // apply the custom drive behavior
    if (m_useCustomBehavior) {
        left = m_driveCustomBehavior(left);
        right = m_driveCustomBehavior(right);
    }

    else if (m_exponent != 0) {
        // apply the default exponential drive implementation
        left = calcExponentialDrive(left);
        right = calcExponentialDrive(right);
    }

    // slew if applicable
    if (m_slewStep > 0) {
        util::slew(m_currentLeftVoltage, left, std::signbit(left) != std::signbit(m_currentLeftVoltage) ? m_slewStepSignChange : m_slewStep);
        util::slew(m_currentRightVoltage, right, std::signbit(right) != std::signbit(m_currentRightVoltage) ? m_slewStepSignChange : m_slewStep);
    }

    // account for max speed
    if (m_driveMaxSpeed > 0) {
        left = std::clamp(left, -m_driveMaxSpeed, m_driveMaxSpeed);
        right = std::clamp(right, -m_driveMaxSpeed, m_driveMaxSpeed);
    }

    // apply speed scale
    left *= speedScale / 127.0;
    right *= speedScale / 127.0;

    // set the motor voltages
    m_baseRobot->setLeftFwdVoltage(left);
    m_baseRobot->setRightFwdVoltage(right);

    // update the current voltage values
    m_currentLeftVoltage = left;
    m_currentRightVoltage = right;
}
}