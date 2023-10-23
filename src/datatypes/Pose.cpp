#include "reauto/datatypes/Pose.h"

// add and subtract poses
Pose operator+(const Pose& a, const Pose& b) {
    return { a.x + b.x, a.y + b.y, a.theta };
}

Pose operator-(const Pose& a, const Pose& b) {
    return { a.x - b.x, a.y - b.y, a.theta };
}

// multiply and divide poses
double operator*(const Pose& a, const Pose& b) {
    return a.x * b.x + a.y * b.y;
}