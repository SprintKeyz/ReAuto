#pragma once

#include "reauto/device/TrackingWheel.hpp"

// not technically a device, but it's a collection of devices
// plus it was under "odom" previously which it definitely should not have been

namespace reauto {
enum class TrackingConfiguration {
    LRB, // left, right, back
    CB, // center, back
    LR // left, right
};

struct TrackingWheels {
    std::shared_ptr<device::TrackingWheel> left = nullptr;
    std::shared_ptr<device::TrackingWheel> right = nullptr;
    std::shared_ptr<device::TrackingWheel> center = nullptr;
    std::shared_ptr<device::TrackingWheel> back = nullptr;
    TrackingConfiguration config;
};
}