#pragma once

#include "reauto/chassis/impl/MotionChassis.hpp"
#include "reauto/motion/purepursuit/PathGen.hpp"
#include <map>

namespace reauto
{
class PurePursuitFollower
{
public:
    PurePursuitFollower(std::shared_ptr<MotionChassis> chassis);

    // add a path to the paths repositiory
    void addPath(std::vector<Pose> points, PathConstraints constraints, std::string name, double spacing = 6, double smoothing = 0.81);

    // follow a path
    void follow(std::string name, double lookahead = 6, bool reverse = false);

private:
    std::shared_ptr<MotionChassis> m_chassis;
    std::map<std::string, std::vector<Waypoint>> m_paths;
    double m_lookahead = 6;

    // functions
    int closestPoint(Pose current, std::string name);
    Pose calculateTarget(Pose prevLookahad, Pose current, std::string name);
};
}