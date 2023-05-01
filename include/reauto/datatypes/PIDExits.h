#pragma once

struct PIDExits {
    double smallError;
    double largeError;
    double smallTime = 150;
    double largeTime = 150;
    double velocityTimeout = 250;
};