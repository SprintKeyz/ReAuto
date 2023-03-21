#pragma once

#include <cmath>

constexpr double operator"" _deg(long double x)
{
    return static_cast<double>(x);
}

constexpr double operator"" _deg(unsigned long long int x)
{
    return static_cast<double>(x);
}

constexpr double operator"" _rad(long double x)
{
    return static_cast<double>(x * 180 / M_PI);
}

constexpr double operator"" _rad(unsigned long long int x)
{
    return static_cast<double>(x * 180 / M_PI);
}