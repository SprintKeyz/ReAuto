#pragma once

#include <iostream>

namespace reauto {
namespace controller {
class FeedbackController {
public:
    virtual void setTarget(double target) = 0;
    virtual double calculate(double error) = 0;
    virtual bool settled() = 0;
};
}
}