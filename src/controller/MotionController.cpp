#include "reauto/controller/MotionController.hpp"
#include "reauto/controller/impl/PIDController.hpp"
#include "reauto/device/TrackingWheels.hpp"
#include "reauto/math/Calculate.hpp"
#include "reauto/math/Convert.hpp"

namespace reauto
{
MotionController::MotionController(std::shared_ptr<MotionChassis> chassis,
    controller::FeedbackController* linear,
    controller::FeedbackController* angular,
    double headingkP)
{
    m_chassis = chassis;
    m_linear = linear;
    m_angular = angular;

    PIDExits e = { 1, 0, 100, 0, 150 };

    if (headingkP != 0)
        m_headingController = new controller::PIDController({ headingkP, 0, 0 }, e);
}

// drive
void MotionController::drive(double distance, double maxSpeed, double maxTime,
    double forceExitError, bool thru)
{
    m_linear->setTarget(distance);
    if (m_headingController != nullptr)
        m_headingController->setTarget(m_lastTargetAngle);

    // set initial distance based on tconfig
    if (m_chassis->getTrackingWheels()->config == TrackingConfiguration::CB) {
        m_initialDistance = m_chassis->getTrackingWheels()->center->getDistanceTraveled();
    }

    else {
        m_initialDistance = m_chassis->getTrackingWheels()->left->getDistanceTraveled();
    }

    if (thru)
    {
        double error;

        if (m_chassis->getTrackingWheels()->config == TrackingConfiguration::CB) {
            error = distance - (m_chassis->getTrackingWheels()->center->getDistanceTraveled() - m_initialDistance);
        }

        else {
            error = distance - (m_chassis->getTrackingWheels()->left->getDistanceTraveled() - m_initialDistance);
        }

        while (fabs(error) > 0.25)
        {
            if (m_chassis->getTrackingWheels()->config == TrackingConfiguration::CB) {
                error = distance - (m_chassis->getTrackingWheels()->center->getDistanceTraveled() - m_initialDistance);
            }

            else {
                error = distance - (m_chassis->getTrackingWheels()->left->getDistanceTraveled() - m_initialDistance);
            }

            // spin motors
            int multiplier = (error > 0) ? 1 : -1;
            m_chassis->setVoltage(maxSpeed * multiplier, maxSpeed * multiplier);
        }

        m_chassis->brake();
    }

    else
    {

        while (!m_linear->settled())
        {
            // check max time
            if (maxTime != 0 && m_processTimer > maxTime)
                break;

            double linError;

            if (m_chassis->getTrackingWheels()->config == TrackingConfiguration::CB) {
                linError = distance -
                (m_chassis->getTrackingWheels()->center->getDistanceTraveled() -
                    m_initialDistance);
            }

            else {
                linError = distance -
                (m_chassis->getTrackingWheels()->left->getDistanceTraveled() -
                    m_initialDistance);
            }
    
            double angError = m_lastTargetAngle - m_chassis->getHeading();

            // check force exit error
            if (forceExitError != 0 && std::abs(linError) < forceExitError)
                break;

            double linear = m_linear->calculate(linError);
            double angular = (m_headingController != nullptr)
                ? m_headingController->calculate(angError)
                : 0;

            if (linError < 6) {
                angular = 0;
            }

            // cap linear speed to max
            linear = std::clamp(linear, -maxSpeed, maxSpeed);

            // if thru, set linear speed to max
            if (thru)
                linear = maxSpeed;

            double lSpeed = linear + angular;
            double rSpeed = linear - angular;

            // ratio the speeds to respect the max speed
            double speedRatio =
                std::max(std::abs(lSpeed), std::abs(rSpeed)) / maxSpeed;
            if (speedRatio > 1)
            {
                lSpeed /= speedRatio;
                rSpeed /= speedRatio;
            }

            m_chassis->setVoltage(lSpeed, rSpeed);

            // delay
            pros::delay(MOTION_TIMESTEP);
            m_processTimer += MOTION_TIMESTEP;
        }

        if (forceExitError == 0)
            m_chassis->brake();
        m_processTimer = 0;
    }
}

void MotionController::drive(Point target, double maxSpeed, bool invert,
    double maxTime, double forceExitError, bool thru)
{
    // calc distance and angle errors
    Point initial = { m_chassis->getPose().y, m_chassis->getPose().x };

    target = { target.y, target.x };

    // get distance and angle to point
    double dist = calc::distance(initial, target);
    double angle = math::wrap180(calc::angleDifference(initial, target) -
        m_chassis->getHeading());

    // tips: disable turning when close to the target (within a few inches) and
    // multiply lateral error by cos(ang error)

    // set PID targets
    m_linear->setTarget(dist);
    m_angular->setTarget(angle);

    // set last target angle
    m_lastTargetAngle = angle;

    // update!
    while (!m_linear->settled())
    {
        // check max time
        if (maxTime != 0 && m_processTimer > maxTime)
            break;

        Point current = { m_chassis->getPose().y, m_chassis->getPose().x };

        //std::cout << "Current is: " << current.x << ", " << current.y << std::endl;
        //std::cout << "Target is: " << target.x << ", " << target.y << std::endl;

        dist = calc::distance(current, target);
        angle = math::wrap180(calc::angleDifference(current, target) -
            m_chassis->getHeading());

        // check force exit error
        if (forceExitError != 0 && std::abs(dist) < forceExitError)
            break;

        dist *= cos(math::degToRad(angle));

        if (invert /*|| angle > 90*/)
        {
            angle = math::wrap180(angle + 180);
        }

        double distOutput = m_linear->calculate(dist);
        double angOutput = m_angular->calculate(angle);

        // if we are physically close and the total movement was somewhat large, we
        // can disable turning the 7.5 is from lemlib, which I'm basing this on
        bool closeToTarget = (calc::distance(current, target) < 4);
        if (closeToTarget)
        {
            angOutput = 0;
        }

        // cap the linear speeds
        distOutput = std::clamp(distOutput, -maxSpeed, maxSpeed);

        // if thru, set linear speed to max
        if (thru)
            distOutput = maxSpeed;

        // calculate speeds
        double lSpeed = distOutput + angOutput;
        double rSpeed = distOutput - angOutput;

        // limit the speeds to respect max speed
        double speedRatio = std::max(std::abs(lSpeed), std::abs(rSpeed)) / maxSpeed;
        if (speedRatio > 1)
        {
            lSpeed /= speedRatio;
            rSpeed /= speedRatio;
        }

        // set the speeds
        m_chassis->setVoltage(lSpeed, rSpeed);

        // delay
        pros::delay(MOTION_TIMESTEP);
        m_processTimer += MOTION_TIMESTEP;
    }

    if (forceExitError == 0)
        m_chassis->brake();
    m_processTimer = 0;
}

// caclulate current target point for boomerang
Point MotionController::calcCarrotPoint(Point start, Pose target,
    double leadToPose)
{
    // get current pose
    Pose current = m_chassis->getPose();

    // get distance to target
    double dist = calc::distance({ current.x, current.y }, { target.x, target.y });

    // get boomerang point
    double boomerangX = target.x - dist * sin(math::degToRad(target.theta.value())) * leadToPose;
    double boomerangY = target.y - dist * cos(math::degToRad(target.theta.value())) * leadToPose;

    return { boomerangX, boomerangY };
}

// boomerang
void MotionController::driveToPose(Pose target, double leadToPose,
    double maxSpeed, bool invert,
    double maxTime, double forceExitError,
    bool thru)
{
    // calc distance and angle errors
    Point initial = { m_chassis->getPose().y, m_chassis->getPose().x };

    // get distance and angle to point
    double distToTarget = calc::distance(initial, { target.y, target.x });

    // set PID targets - angle target is our current heading at first
    m_linear->setTarget(distToTarget);
    m_angular->setTarget(m_chassis->getHeading());

    // set last target angle
    // this is the target theta because we are using boomerang!
    m_lastTargetAngle = target.theta.value();

    // update!
    while (!m_linear->settled())
    {
        // check max time
        if (maxTime != 0 && m_processTimer > maxTime)
            break;

        Point current = { m_chassis->getPose().y, m_chassis->getPose().x };

        double distError = calc::distance(current, { target.y, target.x });
        // angle error is the angle to the boomerang point
        Point boomerang = calcCarrotPoint(initial, target, leadToPose);

        // get angle to boomerang point - this is our target angle AND our angle
        // error our target always changes so our error always changes...
        double angTarget = math::wrap180(calc::angleDifference(current, boomerang) -
            m_chassis->getHeading());

        // set our target angle to the angle to the boomerang point

        // check force exit error
        if (forceExitError != 0 && std::abs(distError) < forceExitError)
            break;

        // distError *= cos(math::degToRad(angleError)); - unnecessary because it's
        // a boomerang movement!

        if (invert)
        {
            angTarget = math::wrap180(angTarget + 180);
        }

        double distOutput = m_linear->calculate(distError);
        double angOutput = m_angular->calculate(angTarget);

        // if we are physically close and the total movement was somewhat large, we
        // can disable turning
        bool closeToTarget = (calc::distance(current, { target.y, target.x }) < 4);
        if (closeToTarget)
        {
            angOutput = 0;
        }

        // cap the linear speeds
        distOutput = std::clamp(distOutput, -maxSpeed, maxSpeed);

        // if thru, set linear speed to max
        if (thru)
            distOutput = maxSpeed;

        // calculate speeds
        double lSpeed = distOutput + angOutput;
        double rSpeed = distOutput - angOutput;

        // limit the speeds to respect max speed
        double speedRatio = std::max(std::abs(lSpeed), std::abs(rSpeed)) / maxSpeed;
        if (speedRatio > 1)
        {
            lSpeed /= speedRatio;
            rSpeed /= speedRatio;
        }

        // set the speeds
        m_chassis->setVoltage(lSpeed, rSpeed);

        // delay
        pros::delay(MOTION_TIMESTEP);
        m_processTimer += MOTION_TIMESTEP;
    }

    if (forceExitError == 0)
        m_chassis->brake();
    m_processTimer = 0;
}

// turn
void MotionController::turn(double angle, double maxSpeed, bool relative,
    double maxTime, double forceExitError, bool thru)
{
    // wrap angle
    angle = math::wrap180(angle);

    if (relative)
    {
        m_angular->setTarget(math::wrap180(m_chassis->getHeading() + angle));
        m_lastTargetAngle = math::wrap180(m_chassis->getHeading() + angle);
    }

    else
    {
        m_angular->setTarget(angle);
        m_lastTargetAngle = angle;
    }

    while (!m_angular->settled())
    {
        // check max time
        if (maxTime != 0 && m_processTimer > maxTime)
            return;

        double error = math::wrap180(angle - m_chassis->getHeading());

        // check force exit error
        if (forceExitError != 0 && std::abs(error) < forceExitError)
            break;

        double output = m_angular->calculate(error);
        output = std::clamp(output, -maxSpeed, maxSpeed);

        // if thru, set angular speed to max
        if (thru)
            output = maxSpeed;

        m_chassis->setVoltage(output, -output);

        // delay
        pros::delay(MOTION_TIMESTEP);
        m_processTimer += MOTION_TIMESTEP;
    }

    if (forceExitError == 0)
        m_chassis->brake();
    m_processTimer = 0;
}

void MotionController::swing(double angle, double maxSpeed, double forceOneSide, bool relative, double maxTime, double forceExitError, bool thru) {
    // wrap angle
    angle = math::wrap180(angle);

    if (relative)
    {
        m_angular->setTarget(math::wrap180(m_chassis->getHeading() + angle));
        m_lastTargetAngle = math::wrap180(m_chassis->getHeading() + angle);
    }

    else
    {
        m_angular->setTarget(angle);
        m_lastTargetAngle = angle;
    }

    while (!m_angular->settled())
    {
        // check max time
        if (maxTime != 0 && m_processTimer > maxTime)
            return;

        double error = math::wrap180(angle - m_chassis->getHeading());

        // check force exit error
        if (forceExitError != 0 && std::abs(error) < forceExitError)
            break;

        double output = m_angular->calculate(error);
        output = std::clamp(output, -maxSpeed, maxSpeed);

        // if thru, set angular speed to max
        if (thru)
            output = maxSpeed;

        // check what side to set output to
        if (forceOneSide == 0) {
            // pick fastest side
            // powering right side turns right
            if (output > 0) {
                m_chassis->setVoltage(output, 0);
            }

            else {
                m_chassis->setVoltage(0, output);
            }
        }

        else if (forceOneSide == 1) {
            // force left side
            m_chassis->setVoltage(output, 0);
        }

        else if (forceOneSide == 2) {
            // force right side
            m_chassis->setVoltage(0, output);
        }

        // delay
        pros::delay(MOTION_TIMESTEP);
        m_processTimer += MOTION_TIMESTEP;
    }

    if (forceExitError == 0)
        m_chassis->brake();
    m_processTimer = 0;
}

void MotionController::turn(Point target, double maxSpeed, double maxTime,
    double forceExitError, bool thru)
{
    // calculate angle to the point
    Pose p = m_chassis->getPose();
    double angle = math::wrap180(calc::angleDifference({ p.y, p.x }, target) -
        p.theta.value_or(0));
    turn(angle, maxSpeed, false, maxTime, forceExitError, thru);
}
} // namespace reauto
