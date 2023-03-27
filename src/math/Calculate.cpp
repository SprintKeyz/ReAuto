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

std::vector<Point> reauto::calc::getLineCircleIntersects(const Point p1, const Point p2, const Point cp, double r)
{
    // this function gets the intersection points between
    // a line and a circle

    // FORMAT OF CALL  : intersects(p1, p2, cp, r, segment)
    // INPUTS          : p1 -> the first point on the line
    //                   p2 -> the second point on the line
    //                   cp -> the center point of the circle
    //                   r -> the radius of the circle

    // adapted from https://rosettacode.org/wiki/Line_circle_intersection

    // we are using a line segment
    bool segment = true;

    std::vector<Point> res;
    auto x0 = cp.x;
    auto y0 = cp.y;
    auto x1 = p1.x;
    auto y1 = p1.y;
    auto x2 = p2.x;
    auto y2 = p2.y;
    auto A = y2 - y1;
    auto B = x1 - x2;
    auto C = x2 * y1 - x1 * y2;
    auto a = pow(A, 2) + pow(B, 2);
    double b, c;
    bool bnz = true;

    if (fabs(B) >= eps)
    {
        b = 2 * (A * C + A * B * y0 - pow(B, 2) * x0);
        c = pow(C, 2) + 2 * B * C * y0 - pow(B, 2) * (pow(r, 2) - pow(x0, 2) - pow(y0, 2));
    }
    else
    {
        b = 2 * (B * C + A * B * x0 - pow(A, 2) * y0);
        c = pow(C, 2) + 2 * A * C * x0 - pow(A, 2) * (pow(r, 2) - pow(x0, 2) - pow(y0, 2));
        bnz = false;
    }
    auto d = pow(b, 2) - 4 * a * c; // discriminant
    if (d < 0)
    {
        return res;
    }

    // checks whether a point is within a segment
    auto within = [x1, y1, x2, y2](double x, double y)
    {
        auto d1 = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)); // distance between end-points
        auto d2 = sqrt(pow(x - x1, 2) + pow(y - y1, 2));   // distance from point to one end
        auto d3 = sqrt(pow(x2 - x, 2) + pow(y2 - y, 2));   // distance from point to other end
        auto delta = d1 - d2 - d3;
        return fabs(delta) < eps; // true if delta is less than a small tolerance
    };

    auto fx = [A, B, C](double x)
    {
        return -(A * x + C) / B;
    };

    auto fy = [A, B, C](double y)
    {
        return -(B * y + C) / A;
    };

    auto rxy = [segment, &res, within](double x, double y)
    {
        if (!segment || within(x, y))
        {
            res.push_back({ x, y });
        }
    };

    double x, y;
    if (d == 0.0)
    {
        // line is tangent to circle, so just one intersect at most
        if (bnz)
        {
            x = -b / (2 * a);
            y = fx(x);
            rxy(x, y);
        }
        else
        {
            y = -b / (2 * a);
            x = fy(y);
            rxy(x, y);
        }
    }
    else
    {
        // two intersects at most
        d = sqrt(d);
        if (bnz)
        {
            x = (-b + d) / (2 * a);
            y = fx(x);
            rxy(x, y);
            x = (-b - d) / (2 * a);
            y = fx(x);
            rxy(x, y);
        }
        else
        {
            y = (-b + d) / (2 * a);
            x = fy(y);
            rxy(x, y);
            y = (-b - d) / (2 * a);
            x = fy(y);
            rxy(x, y);
        }
    }

    return res;
}