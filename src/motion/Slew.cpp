#include "reauto/motion/Slew.hpp"

void reauto::util::slew(double current, double& target, double step) {
    if (std::fabs(target - current) > step)
    {
        // desired is outside SLEW_STEP of current, so we need to change it.
        target = current + std::copysign(step, target - current);
    }
}