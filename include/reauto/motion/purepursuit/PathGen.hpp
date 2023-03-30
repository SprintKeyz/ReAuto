#pragma once

#include <vector>
#include "reauto/datatypes/Pose.h"

// ReAuto pure pursuit path generator
// this takes in a set of points and generates a completed path
// thanks to theo and lib7482 for help and inspiration!

struct Waypoint {
    // these are to be used for pure pursuit
    double x;
    double y;
    double distance = 0; /* dist from beginning of path to this point */
    double velocity = 0;
    double curvature = 0;
};

namespace reauto
{
class PurePursuitGenerator {
public:
    PurePursuitGenerator() = default;
    std::vector<Waypoint> injectPoints(std::vector<Waypoint> points, double spacing = 6);
    // smoother should take in a value between 0.75 and 0.98 (lower is more accurate to the original path)
    std::vector<Waypoint> smoothPath(std::vector<Waypoint> points, double smoothing = 0.81);
    // calculate the distance to each waypoint
    std::vector<Waypoint> calculateDistances(std::vector<Waypoint> points);
    // calculate the curvature of each waypoint on the path
    std::vector<Waypoint> calculateCurvature(std::vector<Waypoint> points);
    // calculate the velocity of each waypoint on the path
    std::vector<Waypoint> calculateVelocity(std::vector<Waypoint> points, double maxVelocity = 127, double maxAcceleration = 127);

    // final calculation of the path
    std::vector<Waypoint> calculatePath(std::vector<Pose> points, double spacing = 6, double smoothing = 0.81, double maxVelocity = 127, double maxAcceleration = 127);
};
}
