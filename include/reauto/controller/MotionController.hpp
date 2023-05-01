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
    // max time is the maximum time the controller will run for
    // exit error is the error at which the controller will FORCE exit
    // thru means the controller will not dip below its max speed (it will finish at max speed)
    void drive(double distance, double maxSpeed = 127, double maxTime = 0, double forceExitError = 0, bool thru = false);
    void drive(Point target, double maxSpeed = 127, bool reverse = false, double maxTime = 0, double forceExitError = 0, bool thru = false);
    void driveToPose(Pose target, double leadToPose = 0.6, double maxSpeed = 127, bool reverse = false, double maxTime = 0, double forceExitError = 0, bool thru = false);
    void turn(double angle, double maxSpeed = 127, bool relative = false, double maxTime = 0, double forceExitError = 0, bool thru = false);
    void turn(Point target, double maxSpeed = 127, double maxTime = 0, double forceExitError = 0, bool thru = false);
    void chainDrive(std::vector<Pose> points, std::vector<bool> reverses, double exitErrorPerPoint = 0.3);

private:
    // chassis
    std::shared_ptr<MotionChassis> m_chassis;

    // controllers
    controller::FeedbackController* m_linear;
    controller::FeedbackController* m_angular;

    // initial distance for linear control
    double m_initialDistance = 0;

    // process timer (ms)
    double m_processTimer = 0;

    // for heading control (just a P/PID controller)
    controller::PIDController* m_headingController = nullptr;

    // store the last target angle! this allows heading correct to
    // lock to the desired angle instead of whatever the current angle is
    double m_lastTargetAngle = 0;

    // for drive to pose
    Point calcThirdPoint(Point start, Pose target, double leadToPose);

    // calc carrot point
    Point calcCarrotPoint(Point start, Pose target, double leadToPose);
};
}