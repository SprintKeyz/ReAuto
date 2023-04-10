#pragma once

#include <optional>

struct Pose
{
    double x;
    double y;
    std::optional<double> theta; /* we made it this just so it was out of bounds 0-360 */
};