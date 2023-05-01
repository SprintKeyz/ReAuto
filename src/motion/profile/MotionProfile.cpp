#include "reauto/motion/profile/MotionProfile.hpp"
#include <algorithm>

namespace reauto {
MotionProfile::MotionProfile() {
    m_times = { 0 };
    m_positions = { 0 };
    m_velocities = { 0 };
    m_accelerations = { 0 };
}

void MotionProfile::setProfile(std::vector<double> times, std::vector<double> positions, std::vector<double> velocities, std::vector<double> accelerations) {
    m_times = times;
    m_positions = positions;
    m_velocities = velocities;
    m_accelerations = accelerations;
}

MotionProfileData MotionProfile::get(double time) {
    MotionProfileData data;

    // find the index of the time
    auto iter = std::find(m_times.begin(), m_times.end(), time);

    if (iter == m_times.end()) {
        // time not found
        data.position = 0;
        data.velocity = 0;
        data.acceleration = 0;

        return data;
    }

    int index = iter - m_times.begin();

    data.position = m_positions[index];
    data.velocity = m_velocities[index];
    data.acceleration = m_accelerations[index];

    return data;
}

bool MotionProfile::isConcluded(double time) {
    return (time >= m_times.back());
}
}