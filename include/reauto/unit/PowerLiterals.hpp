#pragma once

#include <cmath>

constexpr double operator"" _pct(long double x)
{
    return static_cast<double>((x / 100.0) * 127.0);
}

constexpr double operator"" _pct(unsigned long long int x)
{
    return static_cast<double>((x / 100.0) * 127.0);
}

constexpr double operator"" _v(long double x)
{
    return static_cast<double>(x);
}

constexpr double operator"" _v(unsigned long long int x)
{
    return static_cast<double>(x);
}

constexpr double operator"" _mv(long double x)
{
    return static_cast<double>((x / 12000.0) * 127.0);
}

constexpr double operator"" _mv(unsigned long long int x)
{
    return static_cast<double>((x / 12000.0) * 127.0);
}