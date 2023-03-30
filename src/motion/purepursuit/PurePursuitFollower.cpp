#include "reauto/motion/purepursuit/PurePursuitFollower.hpp"
#include "reauto/math/Calculate.hpp"
#include <string>

namespace reauto {
PurePursuitFollower::PurePursuitFollower(std::shared_ptr<MotionChassis> chassis) {
    m_chassis = chassis;
}

void PurePursuitFollower::addPath(std::vector<Pose> points, PathConstraints constraints, std::string name, double spacing, double smoothing) {
    PurePursuitGenerator generator;
    m_paths.insert({name, generator.generatePath(points, constraints, spacing, smoothing)});
}

int PurePursuitFollower::closestPoint(Pose current, std::string name) {
    // borrowed from lemlib. common liam W.
    int index = 0;
    double minDistance = 10000000;
    double dist;

    for (int i = 0; i < m_paths.size(); i++) {
        dist = calc::distance({current.x, current.y}, {m_paths[name][i].x, m_paths[name][i].y});
        if (dist < minDistance) {
            minDistance = dist;
            index = i;
        }
    }

    return index;
}

Pose PurePursuitFollower::calculateTarget(Pose prevLookahad, Pose current, std::string name) {
    // init variables
    Pose lookahead = prevLookahad;
    double t;

    for (int i=0; i<m_paths[name].size() - 1; i++) {
        t = calc::lineCircleIntersect({m_paths[name][i].x, m_paths[name][i].y}, {m_paths[name][i+1].x, m_paths[name][i+1].y}, current, m_lookahead);
    }
}

void PurePursuitFollower::follow(std::string name, double lookahead, bool reverse) {
    std::vector<Waypoint> path = m_paths[name];
    m_lookahead = lookahead;
}
}