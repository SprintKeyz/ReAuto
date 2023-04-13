#pragma once

#include "reauto/chassis/impl/MotionChassis.hpp"
#include "reauto/controller/impl/PIDController.hpp"
#include "reauto/datatypes/PIDConstants.h"

namespace reauto {
enum class TuningCompoment {
    KP_VALUE,
    KI_VALUE,
    KD_VALUE,
    ERROR,
    TARGET,
};

class TuningHelper {
public:
    TuningHelper(pros::Controller& controller, std::shared_ptr<MotionChassis> chassis, std::shared_ptr<controller::PIDController> lin, std::shared_ptr<controller::PIDController> ang);

    // run distance tuner
    void runDistance();

    // run angle tuner
    void runAngle();

private:
    pros::Controller& m_controller;
    std::shared_ptr<MotionChassis> m_chassis;
    std::shared_ptr<controller::PIDController> m_lin;
    std::shared_ptr<controller::PIDController> m_ang;

    // the distance values to tune for
    std::vector<double> m_distances = { 3, 6, 12, 24, 36 };

    // the angle values to tune for
    std::vector<double> m_angles = { 10, 25, 45, 90, 135 };

    // the current target distance or angle
    double m_target = 0;

    // the current constants
    PIDConstants m_constants = { 0, 0, 0 };

    // update a value
    void update(TuningCompoment comp, double value);

    // store time at last update
    double m_lastUpdate = 0;

    // has run?
    bool m_hasRun = false;
};
}