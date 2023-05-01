#pragma once

#include "pros/motors.hpp"
#include "pros/adi.hpp"

namespace reauto {
namespace device {
class Catapult {
public:
    Catapult(const uint8_t motor, const uint8_t limit, pros::Motor_Gears gearset = pros::Motor_Gears::blue, int speed = 127);

    void setSpeed(const int speed);
    void setBrakeMode(const pros::Motor_Brake brake);
    // an amount of time to wait after the limit is pressed before stopping the catapult
    void setStopDelay(const unsigned int delay);
    // an amount of time to wait before polling the limit switch to stop
    void setLimitPollDelay(const unsigned int delay);
    void fire(const unsigned int time = 400);
    void load();
    void fireAsync();
    bool isFiring() const;

private:
    pros::Motor m_motor;
    pros::adi::DigitalIn m_limit;
    int m_speed;
    bool m_firing = false;
    int m_stopDelay = 0;
    int m_limitPollDelay = 150;
};
}
}