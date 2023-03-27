#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>
#include "reauto/math/spline.h"
#include "reauto/datatypes/Point.h"

namespace reauto
{
class InterpolatedLUT
{
public:
    void addPoint(double x, double y);
    void addPoints(std::vector<Point> points);
    void setPoints(std::vector<Point> points);

    void create();

    double get(double input) const;

private:
    tk::spline m_spline;
    std::vector<std::pair<double, double>> m_points;
};
}