#pragma once

#include "reauto/datatypes/Point.h"
#include "reauto/math/Convert.hpp"
#include <cmath>
#include <vector>

namespace reauto
{
namespace calc
{
double distance(Point a, Point b);
double angleDifference(Point a, Point b);
std::vector<Point> getLineCircleIntersects(Point p1, Point p2, Point c, double r);
}
}