#include "reauto/motion/profile/TrapezoidalProfile.hpp"
#include <cmath>

namespace reauto {
TrapezoidalProfile::TrapezoidalProfile(std::shared_ptr<MotionChassis> chassis, TrapezoidalProfileConstants constants, std::shared_ptr<controller::PIDController> headingCorrectController) {
    m_chassis = chassis;
    m_constants = constants;
    m_headingCorrector = headingCorrectController;
}

ProfileData TrapezoidalProfile::getProfileDataAtTime(double time) {
    double pos = 0;
    double vel = 0;
    double acc = 0;
    
    if (time < m_timeToMaxV) {
        vel = m_profileAccel * time;
        acc = m_profileAccel;
    }

    else if (time < m_timeFromMaxV) {
        vel = m_profileMaxV;
        acc = 0;
    }

    else if (time < m_timeTotal) {
        vel = m_profileMaxV - m_profileAccel * (time - m_timeFromMaxV);
        acc = -m_profileAccel;
    }

    pos = m_prevPos + vel * time;
    m_prevPos = pos;
    return { pos, vel, acc };
}

void TrapezoidalProfile::compute(double target, double maxV, double maxA)
{
    m_target = target;
    m_maxVelocity = maxV == 0 ? m_constants.maxVelocity : maxV;
    m_maxAcceleration = maxA == 0 ? m_constants.maxAcceleration : maxA;

    // init some variables
    m_timeToMaxV = m_maxVelocity / m_maxAcceleration; // time to reach max velocity
    m_timeFromMaxV = 0; // time to start decelerating
    m_timeTotal = 0; // total time of the profile
    m_profileMaxV = 0; // the max velocity of the profile

    m_profileAccel = m_maxVelocity / m_timeToMaxV;
    double timeAtMaxV = m_target / m_maxVelocity - m_timeToMaxV;

    // check if the profile is a triangle or trapezoid
    if (m_maxVelocity * m_timeToMaxV > m_target) {
        // triangle profile
        m_timeToMaxV = sqrt(m_target / m_profileAccel);
        m_timeFromMaxV = m_timeToMaxV;
        m_timeTotal = 2.0 * m_timeToMaxV;
        m_profileMaxV = m_profileAccel * m_timeToMaxV;
    }

    else {
        // trapezoid profile
        m_timeFromMaxV = timeAtMaxV + m_timeToMaxV;
        m_timeTotal = m_timeFromMaxV + m_timeToMaxV;
        m_profileMaxV = m_maxVelocity;
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

    while (m_timeTotal > time) {
        ProfileData setpoint = getProfileDataAtTime(time);

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
