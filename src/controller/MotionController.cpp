#include "reauto/controller/MotionController.hpp"
#include "reauto/controller/impl/PIDController.hpp"
#include "reauto/math/Calculate.hpp"
#include "reauto/math/Convert.hpp"

namespace reauto
{
    MotionController::MotionController(std::shared_ptr<MotionChassis> chassis, controller::FeedbackController *linear, controller::FeedbackController *angular, double headingkP)
    {
        m_chassis = chassis;
        m_linear = linear;
        m_angular = angular;

        PIDExits e = {
            0.5,
            0,
            120,
            0,
            150};

        if (headingkP != 0)
            m_headingController = new controller::PIDController({headingkP, 0, 0}, e);
    }

    // drive
    void MotionController::drive(double distance, double maxSpeed, double maxTime, double forceExitError, bool thru)
    {
        m_linear->setTarget(distance);
        double initialAngle = m_chassis->getHeading();
        if (m_headingController != nullptr)
            m_headingController->setTarget(initialAngle);

        m_initialDistance = m_chassis->getTrackingWheels()->center->getDistanceTraveled();

        if (thru)
        {
            double error = distance - (m_chassis->getTrackingWheels()->center->getDistanceTraveled() - m_initialDistance);

            while (fabs(error) > 0.25) {
                error = distance - (m_chassis->getTrackingWheels()->center->getDistanceTraveled() - m_initialDistance);

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

                double linError = distance - (m_chassis->getTrackingWheels()->center->getDistanceTraveled() - m_initialDistance);
                double angError = initialAngle - m_chassis->getHeading();

                // check force exit error
                if (forceExitError != 0 && std::abs(linError) < forceExitError)
                    break;

                double linear = m_linear->calculate(linError);
                double angular = (m_headingController != nullptr) ? m_headingController->calculate(angError) : 0;

                // cap linear speed to max
                linear = std::clamp(linear, -maxSpeed, maxSpeed);

                // if thru, set linear speed to max
                if (thru)
                    linear = maxSpeed;

                double lSpeed = linear + angular;
                double rSpeed = linear - angular;

                // ratio the speeds to respect the max speed
                double speedRatio = std::max(std::abs(lSpeed), std::abs(rSpeed)) / maxSpeed;
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

    void MotionController::drive(Point target, double maxSpeed, bool reverse, double maxTime, double forceExitError, bool thru)
    {
        // calc distance and angle errors
        Point initial = {m_chassis->getPose().x, m_chassis->getPose().y};

        // get distance and angle to point
        double dist = calc::distance(initial, target);
        double angle = math::wrap180(calc::angleDifference(initial, target) - m_chassis->getHeading());

        // tips: disable turning when close to the target (within a few inches) and multiply lateral error by cos(ang error)

        // set PID targets
        m_linear->setTarget(dist);
        m_angular->setTarget(angle);

        // update!
        while (!m_linear->settled())
        {
            // check max time
            if (maxTime != 0 && m_processTimer > maxTime)
                break;

            Point current = {m_chassis->getPose().x, m_chassis->getPose().y};

            dist = calc::distance(current, target);
            angle = math::wrap180(calc::angleDifference(current, target) - m_chassis->getHeading());

            // check force exit error
            if (forceExitError != 0 && std::abs(dist) < forceExitError)
                break;

            dist *= cos(math::degToRad(angle));

            if (reverse || angle > 90)
            {
                angle = math::wrap180(angle + 180);
            }

            double distOutput = m_linear->calculate(dist);
            double angOutput = m_angular->calculate(angle);

            // if we are physically close and the total movement was somewhat large, we can disable turning
            // the 7.5 is from lemlib, which I'm basing this on
            bool closeToTarget = (calc::distance(current, target) < 7.5);
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

    Point calcBoomerangPoint(Pose initial, Pose target, double leadToPose)
    {
        // calc distance and angle errors
        double dist = calc::distance({initial.x, initial.y}, {target.x, target.y});
        double angle = std::atan2(target.y - initial.y, target.x - initial.x) - initial.theta.value_or(0);

        return {
            initial.x + (dist - leadToPose) * std::cos(angle),
            initial.y + (dist - leadToPose) * std::sin(angle)};
    }

    // boomerang
    void MotionController::driveToPose(Pose target, double leadToPose, double maxSpeed, double maxTime, double forceExitError, bool thru)
    {
        bool firstRun = true;

        while (true)
        {
            // calc boomerang point
            Point current = {m_chassis->getPose().x, m_chassis->getPose().y};

            // check max time
            if (maxTime != 0 && m_processTimer > maxTime)
                break;

            // calc boomerang point
            Point boomerang = calcBoomerangPoint(m_chassis->getPose(), target, leadToPose);

            // drive to boomerang point
            double dist = calc::distance(current, boomerang);
            double angle = math::wrap180(calc::angleDifference(current, boomerang) - m_chassis->getHeading());

            // calc distance to target for force exit error
            double distToTarget = calc::distance(boomerang, {target.x, target.y});

            // check force exit error
            if (forceExitError != 0 && distToTarget < forceExitError)
                break;

            // set PID targets
            m_linear->setTarget(dist, firstRun);
            m_angular->setTarget(angle, firstRun);
            firstRun = false;

            // update!
            double distOutput = m_linear->calculate(dist);
            double angOutput = m_angular->calculate(angle);

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
    void MotionController::turn(double angle, double maxSpeed, bool relative, double maxTime, double forceExitError, bool thru)
    {
        // wrap angle
        angle = math::wrap180(angle);

        if (relative)
        {
            m_angular->setTarget(m_chassis->getHeading() + angle);
        }

        else
        {
            m_angular->setTarget(angle);
        }

        while (!m_angular->settled())
        {
            // check max time
            if (maxTime != 0 && m_processTimer > maxTime)
                return;

            double error = angle - m_chassis->getHeading();

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

    void MotionController::turn(Point target, double maxSpeed, double maxTime, double forceExitError, bool thru)
    {
        // calculate angle to the point
        Pose p = m_chassis->getPose();
        double angle = math::wrap180(calc::angleDifference({p.x, p.y}, target) - p.theta.value_or(0));
        std::cout << "angle: " << angle << std::endl;
        turn(angle, maxSpeed, false, maxTime, forceExitError, thru);
    }
}
