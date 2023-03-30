#pragma once

struct PathConstraints {
    double maxVel = 0;
    double decelLimit = 0;
    double turnK = 2.0; // how closely to follow turns

    double startVel = 0;
    double endVel = 0;
};