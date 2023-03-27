#pragma once

struct Pose
{
    double x;
    double y;
    double theta = 361; /* we made it this just so it was out of bounds 0-360 */
};