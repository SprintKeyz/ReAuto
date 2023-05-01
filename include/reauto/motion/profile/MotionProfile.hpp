#pragma once

#include <iostream>
#include <vector>

// this class will be a data structure class
namespace reauto {
struct MotionProfileData {
    double position;
    double velocity;
    double acceleration;
};

class MotionProfile {
public:
    MotionProfile();
    void setProfile(std::vector<double> times, std::vector<double> positions, std::vector<double> velocities, std::vector<double> accelerations);
    MotionProfileData get(double time);
    bool isConcluded(double time); // are we done?

private:
    std::vector<double> m_times;
    std::vector<double> m_positions;
    std::vector<double> m_velocities;
    std::vector<double> m_accelerations;
};
}