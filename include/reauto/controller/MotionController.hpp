#pragma once

#include "reauto/controller/FeedbackController.hpp"
#include "reauto/controller/impl/PIDController.hpp"
#include "reauto/chassis/impl/MotionChassis.hpp"
#include "reauto/datatypes/PIDConstants.h"
#include "reauto/datatypes/PIDExits.h"

namespace reauto {
class MotionController {
public:
    // heading exit error, exit time, and velocity time are set internally
    MotionController(std::shared_ptr<MotionChassis> chassis, controller::FeedbackController* linear, controller::FeedbackController* angular, double headingkP = 0);

    // drive
    void drive(double distance, double maxSpeed = 127);
    void drive(Point target, double maxSpeed = 127);
    void turn(double angle, double maxSpeed = 127, bool relative = false);
    void turn(Point target, double maxSpeed = 127);

private:
    // chassis
    std::shared_ptr<MotionChassis> m_chassis;

    // controllers
    controller::FeedbackController* m_linear;
    controller::FeedbackController* m_angular;

    // initial distance for linear control
    double m_initialDistance = 0;

    // for heading control (just a P/PID controller)
    controller::PIDController* m_headingController = nullptr;
};
}