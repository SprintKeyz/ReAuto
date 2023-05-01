#include "reauto/controller/impl/PIDController.hpp"
#include "reauto/chassis/impl/MotionChassis.hpp"
#include "reauto/motion/Slew.hpp"
#include "pros/apix.h"

#include <cmath>

namespace reauto {
namespace controller {
PIDController::PIDController(std::vector<IPIDConstants> constants, PIDExits exits, double pStartI, double slew): m_constantTable(constants) {
    m_exits = exits;
    m_slew = slew;
    m_pStartI = pStartI;
}

PIDController::PIDController(PIDConstants constants, PIDExits exits, double pStartI, double slew): m_constantTable({ {constants.kP, constants.kI, constants.kD, 0} }) {
    m_exits = exits;
    m_slew = slew;
    m_pStartI = pStartI;
}

void PIDController::resetController() {
    m_prevError = 0;
    m_integral = 0;
    m_derivative = 0;
    m_smallErrorTimer = 0;
    m_largeErrorTimer = 0;
    m_velocityTimer = 0;
    m_lastOutput = 0;
    m_lastTime = 0;
}

void PIDController::resetErrors() {
    m_smallErrorTimer = 0;
    m_largeErrorTimer = 0;
    m_velocityTimer = 0;
}

void PIDController::setTarget(double target, bool reset) {
    m_constants = m_constantTable.get(target);
    m_target = target;
    m_error = target;
    if (reset) resetController();
}

void PIDController::setConstants(PIDConstants constants) {
    m_constantTable = InterpolatedConstants({ {constants.kP, constants.kI, constants.kD, 0} });
}

void PIDController::setConstants(std::vector<IPIDConstants> constants) {
    m_constantTable = InterpolatedConstants(constants);
}

PIDConstants PIDController::getConstants(double error) {
    return m_constantTable.get(error);
}

double PIDController::calculate(double error) {
    // time is current millis - last millis
    // we use this method to increase accuracy
    m_error = error;

    double dt = (m_lastTime == 0) ? MOTION_TIMESTEP : pros::millis() - m_lastTime;

    //std::cout << "Error: " << m_error << ", dt: " << dt << std::endl;

    // this represents how much we are off by in time
    // in a perfect world, this would be 1
    // double error_dt = dt / MOTION_TIMESTEP; - doesnt work because integral is multiplied
    // and error_dt can flip between <1 and >1

    m_integral += m_error * (dt / 1000.0); // divide by 1000 to get seconds
    m_derivative = (m_error - m_prevError) / (dt / 1000.0);

    if (m_pStartI != 0 && fabs(m_error) > m_pStartI) {
        m_integral = 0;
    }

    if (std::signbit(m_error) != std::signbit(m_prevError)) {
        m_integral = 0;
    }

    m_prevError = m_error;
    m_lastTime = pros::millis();

    double output = (m_error * m_constants.kP) + (m_integral * m_constants.kI) + (m_derivative * m_constants.kD);

    if (m_slew != 0 && fabs(m_target) >= 6) {
        // only slew at the start (within first 3 inches)
        // edit: or the end (within last 3 inches)
        // ensure target >= 6 to do this!
        if ((fabs(m_error) > fabs(m_target) - 3.0)) {
            util::slew(m_lastOutput, output, m_slew);
        }
    }

    m_lastOutput = output;
    return output;
}

int debug = 0; 

bool PIDController::settled() {
    // check small error
    // print error
    if (debug == 10) {
        std::cout << "Error: " << m_error << std::endl;
        debug = 0;
    }
    debug++;
    if (fabs(m_error) <= m_exits.smallError) {
        m_smallErrorTimer += MOTION_TIMESTEP;
        m_largeErrorTimer = 0;

        if (m_smallErrorTimer >= m_exits.smallTime) {
            resetErrors();
            return true;
        }
    }

    else {
        m_smallErrorTimer = 0;
    }

    // check large error
    if (fabs(m_error) <= m_exits.largeError && fabs(m_error) > m_exits.smallError) {
        m_largeErrorTimer += MOTION_TIMESTEP;
        m_smallErrorTimer = 0;

        if (m_largeErrorTimer >= m_exits.largeTime) {
            resetErrors();
            return true;
        }
    }

    else {
        m_largeErrorTimer = 0;
    }

    // check velocity
    if (fabs(m_derivative) <= 0.05) {
        m_velocityTimer += MOTION_TIMESTEP;

        if (m_velocityTimer >= m_exits.velocityTimeout) {
            resetErrors();
            std::cout << "Exit velocity" << std::endl;
            return true;
        }
    }

    else {
        m_velocityTimer = 0;
    }

    return false;
}
}
}
