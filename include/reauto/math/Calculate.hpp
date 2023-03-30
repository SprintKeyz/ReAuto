#pragma once

#include "reauto/datatypes/Pose.h"
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
double lineCircleIntersect(Pose p1, Pose p2, Pose c, double r);
}
}