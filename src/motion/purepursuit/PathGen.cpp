#include "reauto/motion/purepursuit/PathGen.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <sys/timespec.h>

namespace reauto
{
void PurePursuitGenerator::injectPoints(std::vector<Waypoint>& points, double spacing)
{
    std::vector<Waypoint> newPoints;

    for (int i = 0; i < points.size() - 1; i++)
    {
        Waypoint p1 = points[i];
        Waypoint p2 = points[i + 1];

        Waypoint vect = { p2.x - p1.x, p2.y - p1.y };
        double magnitude = sqrt(pow(vect.x, 2) + pow(vect.y, 2));

        // calculate the number of points to inject
        int numPoints = std::ceil(magnitude / spacing);

        // normalize the vector
        vect = { vect.x / magnitude, vect.y / magnitude };

        // multiply vector by spacing
        vect = { vect.x * spacing, vect.y * spacing };

        for (int j = 0; j < numPoints; j++)
        {
            Waypoint newPoint = { p1.x + (vect.x * j), p1.y + (vect.y * j) };
            newPoints.push_back(newPoint);
        }
    }

    // add the last point
    newPoints.push_back(points[points.size() - 1]);
    points = newPoints;
}

void PurePursuitGenerator::smoothPath(std::vector<Waypoint>& points, double smoothing) {
    // adapted from team 2618's code
    std::vector<Waypoint> newPath = points;

    double a = 1 - smoothing;
    double tolerance = 0.001;

    double change = tolerance;
    while (change >= tolerance) {
        change = 0;
        for (int i = 1; i < points.size() - 1; i++) {
            double aux = newPath[i].x;
            newPath[i].x += a * (points[i].x - newPath[i].x) + smoothing * (newPath[i - 1].x + newPath[i + 1].x - (2 * newPath[i].x));
            change += std::abs(aux - newPath[i].x);

            aux = newPath[i].y;
            newPath[i].y += a * (points[i].y - newPath[i].y) + smoothing * (newPath[i - 1].y + newPath[i + 1].y - (2 * newPath[i].y));
            change += std::abs(aux - newPath[i].y);
        }
    }

    points = newPath;
}

void PurePursuitGenerator::calculateDistances(std::vector<Waypoint>& points) {
    for (int i = 0; i < points.size() - 1; i++) {
        Waypoint p1 = points[i];
        Waypoint p2 = points[i + 1];

        Waypoint vect = { p2.x - p1.x, p2.y - p1.y };
        double magnitude = sqrt(pow(vect.x, 2) + pow(vect.y, 2));

        points[i].distance = magnitude;
    }
}

double wDist(Waypoint p1, Waypoint p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

double calcPointCurvature(Waypoint prev, Waypoint curr, Waypoint next) {
    double distOne = wDist(curr, prev);
    double distTwo = wDist(curr, next);
    double distThree = wDist(next, prev);

    double sidesProduct = distOne * distTwo * distThree;
    double semiPerim = (distOne + distTwo + distThree) / 2.0;

    double triangleArea = sqrt(semiPerim * (semiPerim - distOne) * (semiPerim - distTwo) * (semiPerim - distThree));

    double rad = sidesProduct / (4 * triangleArea);
    double curvature = std::isnormal(1.0 / rad) ? 1.0 / rad : 0;
    return std::pow(curvature, 2);
}

void PurePursuitGenerator::calculateCurvatures(std::vector<Waypoint>& points) {
    points[0].curvature = 0.0;

    for (int i = 1; i < points.size() - 1; i++) {
        points[i].curvature = calcPointCurvature(points[i - 1], points[i], points[i + 1]);
    }
}

void calculateVelocities(std::vector<Waypoint>& points, PathConstraints constraints) {
    points.back().velocity = constraints.endVel;

    for (int i = points.size() - 1; i > 0; i--) {
        Waypoint start = points[i];
        Waypoint end = points[i - 1];

        // if the user knows "better", we'll listen
        if (start.velocity != 0) {
            continue;
        }

        double desiredVel = constraints.turnK ? std::min(constraints.maxVel, constraints.turnK / points[i].curvature) : constraints.maxVel;
        double dist = wDist(start, end);

        // solve for final velocity
        double maxAttainableVel = std::sqrt(pow(start.velocity, 2) + (2.0 * constraints.decelLimit * dist));

        double vel = std::min(desiredVel, maxAttainableVel);
        end.velocity = vel;
    }
}

std::vector<Waypoint> PurePursuitGenerator::generatePath(std::vector<Pose> points, PathConstraints constraints, double spacing, double smoothing) {
    std::vector<Waypoint> waypoints;

    for (Pose p : points) {
        waypoints.push_back({ p.x, p.y, 0, p.theta.value_or(0), 0 });
    }

    injectPoints(waypoints, spacing);
    smoothPath(waypoints, smoothing);
    calculateDistances(waypoints);
    calculateCurvatures(waypoints);
    calculateVelocities(waypoints, constraints);

    return waypoints;
}
}