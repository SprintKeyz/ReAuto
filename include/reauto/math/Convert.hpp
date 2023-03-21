#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

namespace reauto {
namespace math {
double degToRad(double deg);
double radToDeg(double rad);
double cdegToDeg(double cdeg);
double degToIn(double deg, double diam);
double inToDeg(double in, double diam);
double wrap180(double deg);
}
}