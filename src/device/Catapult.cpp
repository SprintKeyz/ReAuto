#include "reauto/device/Catapult.hpp"

namespace reauto {
namespace device {
Catapult::Catapult(const uint8_t motor, const uint8_t limit, pros::MotorGears gearset, int speed):
    m_motor(motor, gearset),
    m_limit(limit),
    m_speed(speed) {}

bool Catapult::isFiring() const {
    return m_firing;
}

void Catapult::setSpeed(const int speed) {
    m_speed = speed;
}

void Catapult::setBrakeMode(const pros::MotorBrake brake) {
    m_motor.set_brake_mode(brake);
}

void Catapult::setStopDelay(const unsigned int delay) {
    m_stopDelay = delay;
}

void Catapult::setLimitPollDelay(const unsigned int delay) {
    m_limitPollDelay = delay;
}

void Catapult::fireAsync() {
    // create a task to fire the catapult
    m_firing = true;

    pros::Task fireTask([this]() {
        m_motor.move(m_speed);

        // delay a bit before checking the limit again
        pros::delay(m_limitPollDelay);

        while (!m_limit.get_value()) {
            pros::delay(10);
        }

        pros::delay(m_stopDelay);

        m_motor.brake();
        m_firing = false;
        });
}

void Catapult::fire(const unsigned int time) {
    fireAsync();
    pros::delay(time);
}

void Catapult::load() {
    if (!m_limit.get_value()) {
        int limitDelay = m_limitPollDelay;
        m_limitPollDelay = 0;
        fireAsync();
        m_limitPollDelay = limitDelay;
    }
}
}
}