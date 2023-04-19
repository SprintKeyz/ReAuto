#include "reauto/controller/MotionController.hpp"
#include "reauto/controller/impl/PIDController.hpp"
#include "reauto/math/Calculate.hpp"
#include "reauto/math/Convert.hpp"

namespace reauto
{
    MotionController::MotionController(std::shared_ptr<MotionChassis> chassis,
                                       controller::FeedbackController *linear,
                                       controller::FeedbackController *angular,
                                       double headingkP)
    {
        m_chassis = chassis;
        m_linear = linear;
        m_angular = angular;

        PIDExits e = {1, 0, 100, 0, 150};

        if (headingkP != 0)
            m_headingController = new controller::PIDController({headingkP, 0, 0}, e);
    }

    // drive
    void MotionController::drive(double distance, double maxSpeed, double maxTime,
                                 double forceExitError, bool thru)
    {
        m_linear->setTarget(distance);
        if (m_headingController != nullptr)
            m_headingController->setTarget(m_lastTargetAngle);

        m_initialDistance =
            m_chassis->getTrackingWheels()->center->getDistanceTraveled();

        if (thru)
        {
            double error =
                distance -
                (m_chassis->getTrackingWheels()->center->getDistanceTraveled() -
                 m_initialDistance);

            while (fabs(error) > 0.25)
            {
                error = distance -
                        (m_chassis->getTrackingWheels()->center->getDistanceTraveled() -
                         m_initialDistance);

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

                double linError =
                    distance -
                    (m_chassis->getTrackingWheels()->center->getDistanceTraveled() -
                     m_initialDistance);
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

    void MotionController::drive(Point target, double maxSpeed, bool reverse,
                                 double maxTime, double forceExitError, bool thru)
    {
        // calc distance and angle errors
        Point initial = {m_chassis->getPose().x, m_chassis->getPose().y};

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

            Point current = {m_chassis->getPose().x, m_chassis->getPose().y};

            dist = calc::distance(current, target);
            angle = math::wrap180(calc::angleDifference(current, target) -
                                  m_chassis->getHeading());

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

            // if we are physically close and the total movement was somewhat large, we
            // can disable turning the 7.5 is from lemlib, which I'm basing this on
            bool closeToTarget = (calc::distance(current, target) < 5);
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

    // calculate third point for boomerang
    Point MotionController::calcThirdPoint(Point start, Pose target,
                                           double leadToPose)
    {
        double h = sqrt(pow(start.x - target.x, 2) + pow(start.y - target.y, 2));
        double x =
            target.x - (h * sin(math::degToRad(target.theta.value()))) * leadToPose;
        double y =
            target.y - (h * cos(math::degToRad(target.theta.value()))) * leadToPose;

        return {x, y};
    }

    // caclulate current target point for boomerang
    Point MotionController::calcCarrotPoint(Point start, Pose target,
                                            double leadToPose)
    {
        // get current pose
        Pose current = m_chassis->getPose();

        // t is a value 0-1 that represents how far along the line we are
        // to calculate it, we find the dist from the start to the current point, and
        // divide by the dist from the start to the target
        double t = calc::distance(start, {current.x, current.y}) /
                   calc::distance(start, {target.x, target.y});

        // if t is greater than 1, we are past the target point, so we just return the
        // target point
        if (t > 1)
            return {target.x, target.y};

        // calculate the carrot point
        Point three = calcThirdPoint(start, target, leadToPose);
        double boomerangX = ((1 - t) * ((1 - t) * start.x + t * three.x) +
                             t * ((1 - t) * three.x + t * target.x));
        double boomerangY = ((1 - t) * ((1 - t) * start.y + t * three.y) +
                             t * ((1 - t) * three.y + t * target.y));

        return {boomerangX, boomerangY};
    }

    void MotionController::chainDrive(std::vector<Pose> points,
                                      std::vector<bool> reverses,
                                      double exitErrorPerPoint)
    {
        // our linear error is dist to the final point
        // our angular error is the angle to the next point
        // we force exit when we are within exitErrorPerPoint of the next point

        // calc distance and angle errors
        Point initial = {m_chassis->getPose().x, m_chassis->getPose().y};

        // get distance and angle to point
        double dist = calc::distance(initial, {points.back().x, points.back().y});
        double angle =
            math::wrap180(calc::angleDifference(initial, {points[0].x, points[0].y}) -
                          m_chassis->getHeading());

        // tips: disable turning when close to the target (within a few inches) and
        // multiply lateral error by cos(ang error)

        // set PID targets
        m_linear->setTarget(dist);
        m_angular->setTarget(angle);

        // set last target angle
        m_lastTargetAngle =
            calc::angleDifference(initial, {points.back().x, points.back().y}) -
            m_chassis->getHeading();

        // update!
        // for each point
        int iter = 0;

        for (Pose p : points)
        {
            if (points.back().x == p.x && points.back().y == p.y)
            {
                // drive to the final point
                drive({p.x, p.y}, p.theta.value_or(127));
                break;
            }

            else
            {
                double linErrorToNextPoint = calc::distance(
                    {m_chassis->getPose().x, m_chassis->getPose().y}, {p.x, p.y});

                while (linErrorToNextPoint > exitErrorPerPoint)
                {
                    Point current = {m_chassis->getPose().x, m_chassis->getPose().y};

                    linErrorToNextPoint = calc::distance(current, {p.x, p.y});
                    dist = calc::distance(current, {points.back().x, points.back().y});
                    angle = math::wrap180(calc::angleDifference(current, {p.x, p.y}) -
                                          m_chassis->getHeading());

                    dist *= cos(math::degToRad(angle));

                    if (reverses.size() >= iter && reverses[iter])
                    {
                        angle = math::wrap180(angle + 180);
                    }

                    double distOutput = m_linear->calculate(dist);
                    double angOutput = m_angular->calculate(angle);

                    // if we are physically close and the total movement was somewhat large,
                    // we can disable turning the 7.5 is from lemlib, which I'm basing this
                    // on
                    bool closeToTarget = (calc::distance(current, {p.x, p.y}) < 5);
                    if (closeToTarget)
                    {
                        angOutput = 0;
                    }

                    double maxSpeed = p.theta.value_or(127);

                    // cap the linear speeds
                    distOutput = std::clamp(distOutput, -maxSpeed, maxSpeed);

                    // calculate speeds
                    double lSpeed = distOutput + angOutput;
                    double rSpeed = distOutput - angOutput;

                    // limit the speeds to respect max speed
                    double speedRatio =
                        std::max(std::abs(lSpeed), std::abs(rSpeed)) / maxSpeed;
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
            }
        }

        m_processTimer = 0;
    }

    // boomerang
    void MotionController::driveToPose(Pose target, double leadToPose,
                                       double maxSpeed, bool reverse,
                                       double maxTime, double forceExitError,
                                       bool thru)
    {
        // calc distance and angle errors
        Point initial = {m_chassis->getPose().x, m_chassis->getPose().y};

        // get distance and angle to point
        double distToTarget = calc::distance(initial, {target.x, target.y});

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

            Point current = {m_chassis->getPose().x, m_chassis->getPose().y};

            double distError = calc::distance(current, {target.x, target.y});
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

            if (reverse)
            {
                angTarget = math::wrap180(angTarget + 180);
            }

            double distOutput = m_linear->calculate(distError);
            double angOutput = m_angular->calculate(angTarget);

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
            m_angular->setTarget(m_chassis->getHeading() + angle);
            m_lastTargetAngle = m_chassis->getHeading() + angle;
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

    void MotionController::turn(Point target, double maxSpeed, double maxTime,
                                double forceExitError, bool thru)
    {
        // calculate angle to the point
        Pose p = m_chassis->getPose();
        double angle = math::wrap180(calc::angleDifference({p.x, p.y}, target) -
                                     p.theta.value_or(0));
        std::cout << "angle: " << angle << std::endl;
        turn(angle, maxSpeed, false, maxTime, forceExitError, thru);
    }
} // namespace reauto
