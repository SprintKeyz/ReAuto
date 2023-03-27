#include "reauto/controller/MotionController.hpp"
#include "reauto/controller/impl/PIDController.hpp"
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
void MotionController::drive(double distance, double maxSpeed) {
    m_linear->setTarget(distance);
    if (m_headingController != nullptr) m_headingController->setTarget(m_chassis->getHeading());

    m_initialDistance = m_chassis->getTrackingWheels()->center->getDistanceTraveled();

    while (!m_linear->settled()) {
        double linear = m_linear->calculate(m_chassis->getTrackingWheels()->center->getDistanceTraveled() - m_initialDistance);
        double angular = (m_headingController != nullptr) ? m_headingController->calculate(m_chassis->getHeading()) : 0;

        // cap speed
        angular = std::clamp(angular, -maxSpeed, maxSpeed);
        m_chassis->setVoltage(linear + angular, linear - angular);

        // delay
        pros::delay(MOTION_TIMESTEP);
    }

    m_chassis->brake();
}

// turn
void MotionController::turn(double angle, double maxSpeed, bool relative) {
    // wrap angle
    angle = math::wrap180(angle);

    if (relative) {
        m_angular->setTarget(m_chassis->getHeading() + angle);
    }

    else {
        m_angular->setTarget(angle);
    }

    while (!m_angular->settled()) {
        double output = m_angular->calculate(m_chassis->getHeading());
        output = std::clamp(output, -maxSpeed, maxSpeed);
        m_chassis->setVoltage(output, -output);
    }

    m_chassis->brake();

    // delay
    pros::delay(MOTION_TIMESTEP);
}
}