#pragma once

#include "reauto/controller/FeedbackController.hpp"
#include "reauto/datatypes/IPIDConstants.h"
#include "reauto/datatypes/InterpolatedConstants.hpp"
#include "reauto/datatypes/PIDConstants.h"
#include "reauto/datatypes/PIDExits.h"
#include <vector>

namespace reauto {
namespace controller {
class PIDController: public FeedbackController {
public:
    PIDController(std::vector<IPIDConstants> constants, PIDExits exits, double pStartI = 0, double slew = 0);
    PIDController(PIDConstants constants, PIDExits exits, double pStartI = 0, double slew = 0);

    // set the controller target
    void setTarget(double target) override;

    // calculate the controller output
    double calculate(double current) override;

    // check if the controller is settled
    bool settled() override;

private:
    double m_target = 0;
    double m_error = 0;
    double m_prevError = 0;
    double m_slew = 0;
    double m_pStartI = 0; // when to start integrating?

    PIDExits m_exits;
    InterpolatedConstants m_constantTable;

    // the constants to use for the movement
    PIDConstants m_constants;

    // timers
    double m_smallErrorTimer = 0;
    double m_largeErrorTimer = 0;
    double m_velocityTimer = 0;

    // integral and derivative terms
    double m_integral = 0;
    double m_derivative = 0;

    // track time
    double m_lastTime = 0;

    // reset our errors
    void resetController();
    void resetErrors();

    // for slew
    double m_lastOutput = 0;
};
}
}