#pragma once

#include "reauto/chassis/impl/MotionChassis.hpp"
#include "reauto/math/Calculate.hpp"

namespace reauto {
namespace motion {
class PurePursuit {
public:
    PurePursuit(MotionChassis* chassis);
    void follow(const char* path, int timeout, double lookahead, bool reverse = false, double maxSpeed = 127);

private:
    MotionChassis* m_chassis;
    double m_distTraveled = 0;
};
}
}