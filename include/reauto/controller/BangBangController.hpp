#include "reauto/chassis/impl/MotionChassis.hpp"
#include "reauto/controller/FeedbackController.hpp"

#pragma once

namespace reauto {
namespace controller {
class BangBangController: public FeedbackController {
public:
    BangBangController(std::shared_ptr<MotionChassis> chassis);

    // set the controller target
    void setTarget(double target) override;

    // calculate the controller output
    double calculate(double error) override;

    // check if the controller is settled
    bool settled() override;

    // there's no set max speed, since that's handled by the chassis
    // this always returns -127 or 127

private:
    std::shared_ptr<MotionChassis> m_chassis;
};
}
}