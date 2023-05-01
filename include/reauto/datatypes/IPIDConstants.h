#pragma once

// interpolated PID constants

struct IPIDConstants
{
    double kP;
    double kI;
    double kD;
    // the error at which we should fully use
    // these constants
    double error;
};