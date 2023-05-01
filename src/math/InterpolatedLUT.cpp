#include "reauto/math/InterpolatedLUT.hpp"

namespace reauto
{
void InterpolatedLUT::addPoint(double x, double y)
{
    m_points.push_back(std::make_pair(x, y));
}

void InterpolatedLUT::addPoints(std::vector<Point> points)
{
    for (Point point : points)
    {
        m_points.push_back(std::make_pair(point.x, point.y));
    }
}

void InterpolatedLUT::setPoints(std::vector<Point> points)
{
    m_points.clear();
    addPoints(points);
    create();
}

void InterpolatedLUT::create()
{
    std::sort(m_points.begin(), m_points.end());

    std::vector<double> x;
    std::vector<double> y;

    for (std::pair<double, double> point : m_points)
    {
        x.push_back(point.first);
        y.push_back(point.second);
    }

    m_spline.set_points(x, y, tk::spline::cspline);
    m_spline.make_monotonic();
}

double InterpolatedLUT::get(double input) const
{
    if (input <= m_points[0].first)
        return m_points[0].second;

    if (input >= m_points[m_points.size() - 1].first)
        return m_points[m_points.size() - 1].second;

    return m_spline(input);
}
}