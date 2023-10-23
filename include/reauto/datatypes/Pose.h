#pragma once

#include <optional>

struct Pose
{
	double x;
	double y;
	std::optional<double> theta; /* we made it this just so it was out of bounds 0-360 */
};

// add and subtract poses
Pose operator+(const Pose& a, const Pose& b);
Pose operator-(const Pose& a, const Pose& b);

// multiply and divide poses
double operator*(const Pose& a, const Pose& b);