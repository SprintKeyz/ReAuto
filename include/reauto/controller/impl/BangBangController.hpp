#pragma once

#include "reauto/controller/FeedbackController.hpp"

namespace reauto {
namespace controller {
class BangBangController: public FeedbackController {
public:
    BangBangController(double exitError = 0.5);

    // set the controller target
    void setTarget(double target, bool resetController = true) override;

    // calculate the controller output
    double calculate(double error) override;

    // check if the controller is settled
    bool settled() override;

    // there's no set max speed, since that's handled by the chassis
    // this always returns -127 or 127

private:
    double m_target;
    double m_error;
    double m_exitError;
};
}
}