#pragma once

// this is used for static casting and accessing
// holonomic functioonality from the FeedForwardChassis

enum class HolonomicMode {
    NONE = 0,
    MECANUM = 1,
    X = 2,
    H = 3,
};