#pragma once

#include <cmath>

constexpr double operator"" _in(long double x)
{
    return static_cast<double>(x);
}

constexpr double operator"" _in(unsigned long long int x)
{
    return static_cast<double>(x);
}

constexpr double operator"" _cm(long double x)
{
    return static_cast<double>(x / 2.54);
}

constexpr double operator"" _cm(unsigned long long int x)
{
    return static_cast<double>(x / 2.54);
}

constexpr double operator"" _m(long double x)
{
    return static_cast<double>(x * 39.3701);
}

constexpr double operator"" _m(unsigned long long int x)
{
    return static_cast<double>(x * 39.3701);
}

constexpr double operator"" _tile(long double x)
{
    return static_cast<double>(x * 24.0);
}

constexpr double operator"" _tile(unsigned long long int x)
{
    return static_cast<double>(x * 24.0);
}