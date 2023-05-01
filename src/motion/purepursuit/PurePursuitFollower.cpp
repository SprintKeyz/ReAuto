#include "reauto/motion/purepursuit/PurePursuitFollower.hpp"
#include "reauto/math/Calculate.hpp"
#include "reauto/math/Convert.hpp"
#include <string>
#include <cmath>

namespace reauto {
PurePursuitFollower::PurePursuitFollower(std::shared_ptr<MotionChassis> chassis) {
    m_chassis = chassis;
}

void PurePursuitFollower::addPath(std::vector<Pose> points, PathConstraints constraints, std::string name, double spacing, double smoothing) {
    PurePursuitGenerator generator;
    m_paths.insert({ name, generator.generatePath(points, constraints, spacing, smoothing) });
    m_constraints = constraints;
}

Pose PurePursuitFollower::findLookaheadPoint(std::string pathName) {
    std::vector<Waypoint> path = m_paths[pathName];
    Pose current = m_chassis->getPose();

    // check if iterator is valid

    if ((m_lastClosest != path.end()) && calc::distance({ current.x, current.y }, { path.back().x, path.back().y }) < m_lookahead) {
        m_lastClosest = path.end() - 2;
        m_lastLookaheadT = 1;
    }
}

int sign(double x) {
    if (x < 0) {
        return -1;
    }
    else if (x > 0) {
        return 1;
    }
    else {
        return 0;
    }
}

double PurePursuitFollower::calculateCurvature(Pose lookaheadPoint) {
    Pose currentPose = m_chassis->getPose();
    Pose diff = { lookaheadPoint.x - currentPose.x, lookaheadPoint.y - currentPose.y, 0 };

    double head = math::degToRad((currentPose.theta.value_or(0) * -1) + 90);
    double a = -std::tan(head);
    double c = -a * currentPose.x - currentPose.y;
    double x = std::abs(a * lookaheadPoint.x + 1.0 * lookaheadPoint.y + c) / std::sqrt(a * a + 1);

    int side = sign(std::sin(head) * diff.x - std::cos(head) * diff.y);
    double curvature = 2.0 * x / std::pow(calc::distance({ currentPose.x, currentPose.y }, { lookaheadPoint.x, lookaheadPoint.y }), 2);

    return std::pow(curvature, 2) * side;
}

std::pair<double, double> PurePursuitFollower::calculateWheelSpeeds(double velocity, double curvature) {
    double leftSpeed = velocity * (2.0 + m_chassis->getMeasurements().trackWidth * curvature) / 2.0;
    double rightSpeed = velocity * (2.0 - m_chassis->getMeasurements().trackWidth * curvature) / 2.0;

    leftSpeed = std::clamp(leftSpeed, -m_constraints.maxVel * -1, m_constraints.maxVel);
    rightSpeed = std::clamp(rightSpeed, -m_constraints.maxVel * -1, m_constraints.maxVel);

    leftSpeed = (leftSpeed / (M_1_PI * m_chassis->getMeasurements().wheelDiameter)) * 360;
    rightSpeed = (rightSpeed / (M_1_PI * m_chassis->getMeasurements().wheelDiameter)) * 360;

    return { leftSpeed, rightSpeed };
}

void PurePursuitFollower::reset() {
    m_lastLookaheadIndex = 0;
    m_lastLookaheadT = 0;
}
}