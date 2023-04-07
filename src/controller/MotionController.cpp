#include "reauto/controller/MotionController.hpp"
#include "reauto/controller/impl/PIDController.hpp"
#include "reauto/math/Calculate.hpp"
#include "reauto/math/Convert.hpp"

namespace reauto {
MotionController::MotionController(std::shared_ptr<MotionChassis> chassis, controller::FeedbackController* linear, controller::FeedbackController* angular, double headingkP) {
    m_chassis = chassis;
    m_linear = linear;
    m_angular = angular;

    PIDExits e = {
        0.5,
        0,
        120,
        0,
        150
    };

    if (headingkP != 0) m_headingController = new controller::PIDController({ headingkP, 0, 0 }, e);
}

// drive
void MotionController::drive(double distance, double maxSpeed, double maxTime, double forceExitError) {
    m_linear->setTarget(distance);
    double initialAngle = m_chassis->getHeading();
    if (m_headingController != nullptr) m_headingController->setTarget(initialAngle);

    m_initialDistance = m_chassis->getTrackingWheels()->center->getDistanceTraveled();

    while (!m_linear->settled()) {
        // check max time
        if (maxTime != 0 && m_processTimer > maxTime) break;

        double linError = distance - (m_chassis->getTrackingWheels()->center->getDistanceTraveled() - m_initialDistance);
        double angError = initialAngle - m_chassis->getHeading();

        // check force exit error
        if (forceExitError != 0 && std::abs(linError) < forceExitError) break;

        double linear = m_linear->calculate(linError);
        double angular = (m_headingController != nullptr) ? m_headingController->calculate(angError) : 0;

        // cap linear speed to max
        linear = std::clamp(linear, -maxSpeed, maxSpeed);

        double lSpeed = linear + angular;
        double rSpeed = linear - angular;

        // ratio the speeds to respect the max speed
        double speedRatio = std::max(std::abs(lSpeed), std::abs(rSpeed)) / maxSpeed;
        if (speedRatio > 1) {
            lSpeed /= speedRatio;
            rSpeed /= speedRatio;
        }

        m_chassis->setVoltage(lSpeed, rSpeed);

        // delay
        pros::delay(MOTION_TIMESTEP);
        m_processTimer += MOTION_TIMESTEP;
    }

    m_chassis->brake();
    m_processTimer = 0;
}

void MotionController::drive(Point target, double maxSpeed, double maxTime, double forceExitError) {
    // calc distance and angle errors
    Point initial = { m_chassis->getPose().x, m_chassis->getPose().y };

    // get distance and angle to point
    double dist = calc::distance(initial, target);
    double angle = math::wrap180(calc::angleDifference(initial, target) - m_chassis->getHeading());

    double totalDist = dist;

    // tips: disable turning when close to the target (within a few inches) and multiply lateral error by cos(ang error)

    // set PID targets
    m_linear->setTarget(dist);
    m_angular->setTarget(angle);

    // update!
    while (!m_linear->settled()) {
        // check max time
        if (maxTime != 0 && m_processTimer > maxTime) break;

        Point current = { m_chassis->getPose().x, m_chassis->getPose().y };

        dist = calc::distance(current, target);
        angle = math::wrap180(calc::angleDifference(current, target) - m_chassis->getHeading());

        // check force exit error
        if (forceExitError != 0 && std::abs(dist) < forceExitError) break;

        dist *= cos(math::degToRad(angle));

        double distOutput = m_linear->calculate(dist);
        double angOutput = m_angular->calculate(angle);

        // if we are physically close and the total movement was somewhat large, we can disable turning
        // the 7.5 is from lemlib, which I'm basing this on
        bool closeToTarget = (calc::distance(current, target) < 7.5);
        if (closeToTarget) {
            angOutput = 0;
        }

        // cap the linear speeds
        distOutput = std::clamp(distOutput, -maxSpeed, maxSpeed);

        // calculate speeds
        double lSpeed = distOutput + angOutput;
        double rSpeed = distOutput - angOutput;

        // limit the speeds to respect max speed
        double speedRatio = std::max(std::abs(lSpeed), std::abs(rSpeed)) / maxSpeed;
        if (speedRatio > 1) {
            lSpeed /= speedRatio;
            rSpeed /= speedRatio;
        }

        // set the speeds
        m_chassis->setVoltage(lSpeed, rSpeed);

        // delay
        pros::delay(MOTION_TIMESTEP);
        m_processTimer += MOTION_TIMESTEP;
    }

    m_chassis->brake();
    m_processTimer = 0;
}

// turn
void MotionController::turn(double angle, double maxSpeed, bool relative, double maxTime, double forceExitError) {
    // wrap angle
    angle = math::wrap180(angle);

    if (relative) {
        m_angular->setTarget(m_chassis->getHeading() + angle);
    }

    else {
        m_angular->setTarget(angle);
    }

    while (!m_angular->settled()) {
        // check max time
        if (maxTime != 0 && m_processTimer > maxTime) return;

        double error = angle - m_chassis->getHeading();

        // check force exit error
        if (forceExitError != 0 && std::abs(error) < forceExitError) break;

        double output = m_angular->calculate(error);
        output = std::clamp(output, -maxSpeed, maxSpeed);
        m_chassis->setVoltage(output, -output);

        // delay
        pros::delay(MOTION_TIMESTEP);
        m_processTimer += MOTION_TIMESTEP;
    }

    m_chassis->brake();
    m_processTimer = 0;
}

void MotionController::turn(Point target, double maxSpeed, double maxTime, double forceExitError) {
    // calculate angle to the point
    Pose p = m_chassis->getPose();
    double angle = math::wrap180(calc::angleDifference({ p.x, p.y }, target) - p.theta);
    std::cout << "angle: " << angle << std::endl;
    turn(angle, maxSpeed, false, maxTime, forceExitError);
}
}