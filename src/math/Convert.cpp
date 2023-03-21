#include "reauto/math/Convert.hpp"

namespace reauto {
namespace math {
double degToRad(double deg)
{
    return deg * M_PI / 180.0;
}
double radToDeg(double rad)
{
    return rad * 180.0 / M_PI;
}
double cdegToDeg(double cdeg)
{
    return cdeg / 100.0;
}

double degToIn(double deg, double diameter)
{
    return (deg / 360) * (M_PI * diameter);
}

double inToDeg(double in, double diameter)
{
    double arcLength = (M_PI * diameter) / 360;
    return in / arcLength;
}

double wrap180(double deg)
{
    while (deg > 180 || deg < -180)
    {
        if (deg > 180)
        {
            deg -= 360;
        }

        else if (deg < -180)
        {
            deg += 360;
        }
    }

    return deg;
}
}
}