#include "reauto/math/Calculate.hpp"

constexpr auto eps = 1e-14;

double reauto::calc::distance(Point a, Point b)
{
    return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}

double reauto::calc::angleDifference(Point a, Point b)
{
    double targetAngle = atan2(b.y - a.y, b.x - a.x);

    return reauto::math::radToDeg(targetAngle);
}

double reauto::calc::lineCircleIntersect(const Pose p1, const Pose p2, const Pose cp, double r)
{
    Pose d = {p2.x - p1.x, p2.y - p1.y};
    Pose f = {p1.x - cp.x, p1.y - cp.y};

    double a = d.x * d.x + d.y * d.y;
    double b = 2 * (f.x * d.x + f.y * d.y);
    double c = f.x * f.x + f.y * f.y - r * r;

    double discriminant = b * b - 4 * a * c;

    if (discriminant >= 0) {
        discriminant = sqrt(discriminant);

        double t1 = (-b - discriminant) / (2 * a);
        double t2 = (-b + discriminant) / (2 * a);

        // pick the further intersection
        if (t2 >= 0 && t2 <= 1) return t2;
        else if (t1 >= 0 && t1 <= 1) return t1;
    }

    return -1;
}