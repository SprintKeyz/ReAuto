#include "reauto/odom/Odometry.hpp"
#include "reauto/device/IMU.hpp"

namespace reauto {
Odometry::Odometry(TrackingWheels* wheels, device::IMU* imu): m_wheels(wheels), m_imu(imu) {}

void Odometry::resetPreviousVariables() {
    m_prevRotationRad = 0;
    m_prevBackPos = 0;
    m_prevMiddlePos = 0;
}

void Odometry::resetPosition() {
    m_pos = { 0, 0 };
    resetPreviousVariables();
}

void Odometry::setPosition(Point p) {
    m_pos = p;
    resetPreviousVariables();
}

Point Odometry::getPosition() {
    return m_pos;
}

void Odometry::startTracking() {
    resetPosition();

    pros::Task odomTask {
        [this] {
            std::cout << "[ReAuto] Started odometry" << std::endl;

            double currentRotationRad = m_imu->getRotation(true);
            double middlePos = m_wheels->center->getPosition();
            double backPos = m_wheels->back->getPosition();

            m_prevRotationRad = currentRotationRad;
            m_prevMiddlePos = middlePos;
            m_prevBackPos = backPos;

            while (true) {
                middlePos = m_wheels->center->getPosition();
                backPos = m_wheels->back->getPosition();
                currentRotationRad = m_imu->getRotation(true);

                double dHeading = currentRotationRad - m_prevRotationRad;

                double dFwd = math::degToIn(middlePos - m_prevMiddlePos, m_wheels->center->getDiameter()) - m_wheels->center->getCenterDistance() * dHeading;
                double dSide = math::degToIn(backPos - m_prevBackPos, m_wheels->back->getDiameter()) + m_wheels->back->getCenterDistance() * dHeading;

                double cosHeading = cos(m_imu->getHeading(true));
                double sinHeading = sin(m_imu->getHeading(true));

                double dX = cosHeading * dFwd - sinHeading * dSide;
                double dY = sinHeading * dFwd + cosHeading * dSide;

                m_prevMiddlePos = middlePos;
                m_prevBackPos = backPos;
                m_prevRotationRad = currentRotationRad;

                m_pos.x += dX;
                m_pos.y += dY;

                pros::delay(ODOM_TIMESTEP);
            }
        }
    };
}
}