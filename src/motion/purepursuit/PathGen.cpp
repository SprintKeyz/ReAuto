#include "reauto/motion/purepursuit/PathGen.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <sys/timespec.h>

namespace reauto
{
std::vector<Waypoint> PurePursuitGenerator::injectPoints(std::vector<Waypoint> points, double spacing)
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
    return newPoints;
}

std::vector<Waypoint> PurePursuitGenerator::smoothPath(std::vector<Waypoint> points, double smoothing) {
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

    return newPath;
}

std::vector<Waypoint> PurePursuitGenerator::calculateDistances(std::vector<Waypoint> points) {
    std::vector<Waypoint> newPoints = points;

    for (int i = 0; i < newPoints.size() - 1; i++) {
        Waypoint p1 = newPoints[i];
        Waypoint p2 = newPoints[i + 1];

        Waypoint vect = { p2.x - p1.x, p2.y - p1.y };
        double magnitude = sqrt(pow(vect.x, 2) + pow(vect.y, 2));

        newPoints[i].distance = magnitude;
    }

    return newPoints;
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

std::vector<Waypoint> PurePursuitGenerator::calculateCurvatures(std::vector<Waypoint> points) {
    
}
}