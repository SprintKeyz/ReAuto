#include "reauto/motion/profile/TrapezoidalProfile.hpp"
#include <cmath>

namespace reauto {
TrapezoidalProfile::TrapezoidalProfile(std::shared_ptr<MotionChassis> chassis, TrapezoidalProfileConstants constants, std::shared_ptr<controller::PIDController> headingCorrectController) {
    m_chassis = chassis;
    m_constants = constants;
    m_headingCorrector = headingCorrectController;
}

void TrapezoidalProfile::compute(double target, double maxV, double maxA)
{
    m_target = target;
    m_maxVelocity = maxV == 0 ? m_constants.maxVelocity : maxV;
    m_maxAcceleration = maxA == 0 ? m_constants.maxAcceleration : maxA;

    // init some variables
    double timeToMaxV = m_maxVelocity / m_maxAcceleration; // time to reach max velocity
    double timeFromMaxV = 0; // time to start decelerating
    double timeTotal = 0; // total time of the profile
    double profileMaxV = 0; // the max velocity of the profile

    std::vector<double> times = { 0 };
    std::vector<double> positions = { 0 };
    std::vector<double> velocities = { 0 };
    std::vector<double> accelerations = { 0 };

    double a = m_maxVelocity / timeToMaxV;
    double timeAtMaxV = m_target / m_maxVelocity - timeToMaxV;

    // check if the profile is a triangle or trapezoid
    if (m_maxVelocity * timeToMaxV > m_target) {
        // triangle profile
        timeToMaxV = sqrt(m_target / a);
        timeFromMaxV = timeToMaxV;
        timeTotal = 2.0 * timeToMaxV;
        profileMaxV = a * timeToMaxV;
    }

    else {
        // trapezoid profile
        timeFromMaxV = timeAtMaxV + timeToMaxV;
        timeTotal = timeFromMaxV + timeToMaxV;
        profileMaxV = m_maxVelocity;
    }

    while (times.back() < timeTotal) {
        double time = times.back() + (MOTION_TIMESTEP / 1000.0); // seconds
        times.push_back(time);

        if (time < timeToMaxV) {
            // accelerate to max velocity
            velocities.push_back(a * time);
            accelerations.push_back(a);
        }

        else if (time < timeFromMaxV) {
            // cruise
            velocities.push_back(profileMaxV);
            accelerations.push_back(0);
        }

        else if (time < timeTotal) {
            // decelerate to 0
            double decel_time = time - timeFromMaxV;
            velocities.push_back(profileMaxV - a * decel_time);
            accelerations.push_back(-a);
        }

        else {
            // stop
            velocities.push_back(0);
            accelerations.push_back(0);
        }

        positions.push_back(positions.back() + velocities.back() * (MOTION_TIMESTEP/1000.0));
    }

    m_profile.setProfile(times, positions, velocities, accelerations);

    // print times
    for (int i = 0; i < times.size(); i++) {
        std::cout << times[i] << ", ";
    }
}

void TrapezoidalProfile::followLinear() {
    double time = 0;
    double prevError = 0;

    double initialDist = m_chassis->getTrackingWheels()->center->getDistanceTraveled();
    double lastTime = 0;

    // if heading correct
    double initialAngle = m_chassis->getHeading();

    if (m_headingCorrector != nullptr) {
        m_headingCorrector->setTarget(initialAngle);
    }

    while (!m_profile.isConcluded(time)) {
        MotionProfileData setpoint = m_profile.get(time);

        std::cout << "time: " << time << ", position: " << setpoint.position << ", velocity: " << setpoint.velocity << ", acceleration: " << setpoint.acceleration << std::endl;

        // get time error (for accuracy) - 1 is perfect
        double dt = (lastTime == 0) ? MOTION_TIMESTEP : pros::millis() - lastTime;
        // double error_dt = dt / MOTION_TIMESTEP; - doesn't work, can flip between <> 1

        // integrate feedback
        double current = m_chassis->getTrackingWheels()->center->getDistanceTraveled() - initialDist;
        double error = setpoint.position - current;
        double deriv = (error - prevError) / (dt / 1000.0) - setpoint.velocity;

        // controller output
        double feedbackOutput = m_constants.kP * error + m_constants.kD * deriv;

        // calculate heading correction
        double angular = (m_headingCorrector != nullptr) ? m_headingCorrector->calculate(initialAngle - m_chassis->getHeading()) : 0;

        // calculate output
        double output = (setpoint.velocity * m_constants.kVelocityScale + setpoint.acceleration * m_constants.kAccelerationScale) + feedbackOutput;
        double left = output + angular;
        double right = output - angular;

        // calculate max speed and ratio the outputs
        double max = std::max(std::abs(left), std::abs(right)) / 127.0;
        if (max > 1) {
            left /= max;
            right /= max;
        }

        std::cout << "left: " << left << ", right: " << right << std::endl;
        
        // set voltage
        m_chassis->setVoltage(left, right);

        prevError = error;
        lastTime = pros::millis();
        pros::delay(MOTION_TIMESTEP);
        time += (MOTION_TIMESTEP / 1000.0);
    }

    m_chassis->brake();
}

void TrapezoidalProfile::followAngular() {

}

}
