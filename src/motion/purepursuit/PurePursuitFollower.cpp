#include "reauto/motion/purepursuit/PurePursuitFollower.hpp"
#include "reauto/math/Calculate.hpp"
#include <string>
#include <cmath>

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

Pose lerp(Waypoint p1, Waypoint p2, double t) {
    return {p1.x + (p2.x - p1.x) * t, p1.y + (p2.y - p1.y) * t, p1.velocity};
}

Pose PurePursuitFollower::calculateLookahead(Pose prevLookahead, Pose current, std::string name) {
    // init variables
    Pose lookahead = prevLookahead;
    double t;

    for (int i=0; i<m_paths[name].size() - 1; i++) {
        t = calc::lineCircleIntersect({m_paths[name][i].x, m_paths[name][i].y}, {m_paths[name][i+1].x, m_paths[name][i+1].y}, current, m_lookahead);

        if (t != -1 && i >= prevLookahead.theta) {
            lookahead = lerp(m_paths[name][i], m_paths[name][i+1], t);
            lookahead.theta = i;
        }
    }

    return lookahead;
}

void PurePursuitFollower::follow(std::string name, double lookahead, bool reverse) {
    std::vector<Waypoint> path = m_paths[name];
    m_lookahead = lookahead;
}
}