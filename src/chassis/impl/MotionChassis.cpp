#include "reauto/chassis/impl/MotionChassis.hpp"
#include "reauto/chassis/impl/HolonomicMode.hpp"
#include "reauto/motion/Slew.hpp"

#include <cmath>

namespace reauto {
MotionChassis::MotionChassis(
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
    double gearRatio): m_controller(controller) {

    switch (holoMode) {
    case HolonomicMode::NONE:
        m_robot = std::make_shared<TankBase>(left, right, gearset);
        break;
    case HolonomicMode::MECANUM:
        m_robot = std::make_shared<MecanumBase>(left, right, gearset);
        break;
    default:
        throw std::runtime_error("[ReAuto] Sorry! This holonomic mode is not supported yet.");
        break;
    }

    m_holoMode = holoMode;

    if (secondaryImuPort != 0) {
        m_imu = std::make_shared<device::ADIMU>(imuPort, secondaryImuPort);
    }
    else {
        m_imu = std::make_shared<device::IMU>(imuPort);
    }

    m_trackingWheels = std::make_shared<TrackingWheels>();

    switch (tConfig) {
    case TrackingConfiguration::LRB:
        m_trackingWheels->left = std::make_shared<device::TrackingWheel>(firstTWheelPort, tWheelDiam, firstTWheelDist);
        m_trackingWheels->right = std::make_shared<device::TrackingWheel>(secondTWheelPort, tWheelDiam, secondTWheelDist);
        m_trackingWheels->back = std::make_shared<device::TrackingWheel>(thirdTWheelPort, tWheelDiam, thirdTWheelDist);
        m_trackingWheels->config = TrackingConfiguration::LRB;
        break;

    case TrackingConfiguration::CB:
        m_trackingWheels->center = std::make_shared<device::TrackingWheel>(firstTWheelPort, tWheelDiam, firstTWheelDist);
        m_trackingWheels->back = std::make_shared<device::TrackingWheel>(secondTWheelPort, tWheelDiam, secondTWheelDist);
        m_trackingWheels->config = TrackingConfiguration::CB;
        break;

    case TrackingConfiguration::LR:
        m_trackingWheels->left = std::make_shared<device::TrackingWheel>(firstTWheelPort, tWheelDiam, firstTWheelDist);
        m_trackingWheels->right = std::make_shared<device::TrackingWheel>(secondTWheelPort, tWheelDiam, secondTWheelDist);
        m_trackingWheels->config = TrackingConfiguration::LR;
        break;
    }

    m_measurements = { trackWidth, wheelDiameter, gearRatio };
}

RobotMeasurements MotionChassis::getMeasurements() const {
    return m_measurements;
}

void MotionChassis::init() {
    std::cout << "[ReAuto] Starting init..." << std::endl;

    // reset IMU
    m_imu->reset(true);

    // reset encoder positions
    if (m_trackingWheels->back != nullptr) m_trackingWheels->back->reset();
    if (m_trackingWheels->center != nullptr) m_trackingWheels->center->reset();
    if (m_trackingWheels->left != nullptr) m_trackingWheels->left->reset();
    if (m_trackingWheels->right != nullptr) m_trackingWheels->right->reset();

    // odometry
    m_odom = std::make_shared<Odometry>(m_trackingWheels.get(), m_imu.get());
    m_odom->resetPosition();
    m_odom->startTracking();

    // done!
    std::cout << "[ReAuto] Init complete!" << std::endl;
}

void MotionChassis::setLeftVoltage(double voltage) {
    m_robot->setLeftFwdVoltage(voltage);
}

void MotionChassis::setRightVoltage(double voltage) {
    m_robot->setRightFwdVoltage(voltage);
}

void MotionChassis::setVoltage(double left, double right) {
    m_robot->setLeftFwdVoltage(left);
    m_robot->setRightFwdVoltage(right);
}

void MotionChassis::setLeftVelocity(double velocity) {
    m_robot->setLeftFwdVelocity(velocity);
}

void MotionChassis::setRightVelocity(double velocity) {
    m_robot->setRightFwdVelocity(velocity);
}

void MotionChassis::setVelocity(double left, double right) {
    m_robot->setLeftFwdVelocity(left);
    m_robot->setRightFwdVelocity(right);
}

void MotionChassis::setBrakeMode(pros::Motor_Brake mode) {
    m_robot->setBrakeMode(mode);
}

void MotionChassis::brake() {
    m_robot->brake();
}

double MotionChassis::getLeftVelocity() const {
    return 0;
}

double MotionChassis::getRightVelocity() const {
    return m_trackingWheels->center->getVelocity();
}


void MotionChassis::setSlewDrive(double normal, double signChange) {
    m_slewStep = normal;
    m_slewStepSignChange = signChange > 0 ? signChange : normal;
}

void MotionChassis::setDriveExponent(double exponent) {
    m_exponent = exponent;
}

void MotionChassis::setDriveMaxSpeed(double maxSpeed) {
    m_driveMaxSpeed = maxSpeed;
}

void MotionChassis::setControllerDeadband(double deadband) {
    m_deadband = deadband;
}

void MotionChassis::setArcadeDriveChannels(pros::controller_analog_e_t forwardChannel, pros::controller_analog_e_t turnChannel) {
    m_forwardChannel = forwardChannel;
    m_turnChannel = turnChannel;
}

double MotionChassis::calcExponentialDrive(double input) {
    return (std::pow(input, m_exponent) / (std::pow(100, 2)));
}

void MotionChassis::tank(double speedScale) {
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
    m_robot->setLeftFwdVoltage(left);
    m_robot->setRightFwdVoltage(right);

    // update the current voltage values
    m_currentLeftVoltage = left;
    m_currentRightVoltage = right;
}

void MotionChassis::arcade(double speedScale) {
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
    m_robot->setLeftFwdVoltage(left);
    m_robot->setRightFwdVoltage(right);

    // update the current voltage values
    m_currentLeftVoltage = left;
    m_currentRightVoltage = right;
}

double MotionChassis::getHeading(bool rad) const {
    // wrapped [-180, 180] by default
    return m_imu->getHeading(rad);
}

void MotionChassis::setHeading(double deg) {
    m_imu->setHeading(deg);
    m_pose.theta = math::wrap180(deg);
}

Pose MotionChassis::getPose() const {
    Point p = m_odom->getPosition();
    return { p.x, p.y, m_imu->getHeading() };
}

void MotionChassis::setPose(Pose p) {
    m_odom->setPosition({ p.x, p.y });
    m_imu->setHeading(p.theta.value_or(0));
    m_pose = p;
}

TrackingWheels* MotionChassis::getTrackingWheels() const {
    return m_trackingWheels.get();
}
}