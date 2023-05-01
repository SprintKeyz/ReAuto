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
    PathConstraints m_constraints;

    double m_lookahead = 6;
    // iterator to the last closest point
    std::vector<Waypoint>::iterator m_lastClosest;
    int m_lastLookaheadIndex = 0;
    int m_lastLookaheadT = 0;

    // functions
    int findClosestPoint(std::vector<Waypoint> path, Pose currentPose);
    Pose findLookaheadPoint(std::string pathName);
    double calculateCurvature(Pose lookaheadPoint);
    std::pair<double, double> calculateWheelSpeeds(double velocity, double curvature);
    void reset();
};
}