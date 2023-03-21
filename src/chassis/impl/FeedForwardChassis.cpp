#include "reauto/chassis/impl/FeedForwardChassis.hpp"
#include "reauto/math/Convert.hpp"
#include "reauto/motion/Slew.hpp"

namespace reauto {
FeedForwardChassis::FeedForwardChassis(std::initializer_list<int8_t> leftPorts, std::initializer_list<int8_t> rightPorts, pros::Motor_Gears gearset, BaseType type, pros::Controller& controller)
    :
    m_baseRobotType(type),
    m_controller(controller),
    m_trackWidth(0.0),
    m_gearRatio(0.0),
    m_wheelDiameter(0.0) {

    switch (m_baseRobotType) {
    case BaseType::TANK:
        m_baseRobot = std::make_shared<TankBase>(leftPorts, rightPorts, gearset);
        break;
    case BaseType::MECANUM:
        break;
    }
}

FeedForwardChassis::FeedForwardChassis(std::initializer_list<int8_t> leftPorts, std::initializer_list<int8_t> rightPorts, pros::Motor_Gears gearset, BaseType type, pros::Controller& controller, double tWidth, double gearRatio, double wheelDiam)
    :
    m_baseRobotType(type),
    m_controller(controller),
    m_trackWidth(tWidth),
    m_gearRatio(gearRatio),
    m_wheelDiameter(wheelDiam) {

    switch (m_baseRobotType) {
    case BaseType::TANK:
        m_baseRobot = std::make_shared<TankBase>(leftPorts, rightPorts, gearset);
        break;
    case BaseType::MECANUM:
        break;
    }
}

void FeedForwardChassis::drive(double distance, double velocity, bool blocking) {
    // convert distance to degrees
    double degrees = math::inToDeg(distance, m_wheelDiameter);

    // tolerance for movements (we can stop if within x degrees)
    double tolerance = 5.0;

    // store the initial position (for blocking)
    double initialPos = m_baseRobot->getLeftMotors()->get_position();

    // velocity is in RPM
    m_baseRobot->getLeftMotors()->move_relative(degrees, velocity);
    m_baseRobot->getRightMotors()->move_relative(degrees, velocity);

    // wait for completion
    // PROS dev team doesn't include a way to check if a motor
    // group is moving, so we have to check the position
    if (blocking) {
        while (m_baseRobot->getLeftMotors()->get_position() < (initialPos + degrees) - tolerance) {
            pros::delay(15);
        }
    }
}

void FeedForwardChassis::turn(double angle, double velocity, bool blocking) {
    // TODO: Implement turns!
}

void FeedForwardChassis::setleftVelocity(double velocity) {
    m_baseRobot->setLeftVelocity(velocity);
}

void FeedForwardChassis::setrightVelocity(double velocity) {
    m_baseRobot->setRightVelocity(velocity);
}

void FeedForwardChassis::setleftVoltage(double voltage) {
    m_baseRobot->setLeftVoltage(voltage);
}

void FeedForwardChassis::setrightVoltage(double voltage) {
    m_baseRobot->setRightVoltage(voltage);
}

void FeedForwardChassis::setForwardVelocity(double velocity) {
    m_baseRobot->setVelocities(velocity, velocity);
}

void FeedForwardChassis::setTurnVelocity(double velocity) {
    m_baseRobot->setVelocities(-velocity, velocity);
}

void FeedForwardChassis::setForwardVoltage(double voltage) {
    m_baseRobot->setVoltages(voltage, voltage);
}

void FeedForwardChassis::setTurnVoltage(double voltage) {
    m_baseRobot->setVoltages(-voltage, voltage);
}

void FeedForwardChassis::setBrakeMode(pros::Motor_Brake mode) {
    m_baseRobot->setBrakeMode(mode);
}

void FeedForwardChassis::brake() {
    m_baseRobot->brake();
}

void FeedForwardChassis::setSlewDrive(double normal, double signChange) {
    m_slewStep = normal;
    m_slewStepSignChange = signChange > 0 ? signChange : normal;
}

void FeedForwardChassis::setDriveExponent(double exponent) {
    m_exponent = exponent;
}

void FeedForwardChassis::setDriveMaxSpeed(double maxSpeed) {
    m_driveMaxSpeed = maxSpeed;
}

void FeedForwardChassis::setControllerDeadband(double deadband) {
    m_deadband = deadband;
}

void FeedForwardChassis::setArcadeDriveChannels(pros::controller_analog_e_t forwardChannel, pros::controller_analog_e_t turnChannel) {
    m_forwardChannel = forwardChannel;
    m_turnChannel = turnChannel;
}

double FeedForwardChassis::calcExponentialDrive(double input) {
    return (std::pow(input, m_exponent) / (std::pow(100, 2)));
}

void FeedForwardChassis::setCustomDriveBehavior(std::function<double(double)> function) {
    m_driveCustomBehavior = function;
    m_useCustomBehavior = true;
}

void FeedForwardChassis::tank(double speedScale) {
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
    m_baseRobot->setVoltages(left, right);

    // update the current voltage values
    m_currentLeftVoltage = left;
    m_currentRightVoltage = right;
}

void FeedForwardChassis::arcade(double speedScale) {
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
    m_baseRobot->setVoltages(left, right);

    // update the current voltage values
    m_currentLeftVoltage = left;
    m_currentRightVoltage = right;
}
}