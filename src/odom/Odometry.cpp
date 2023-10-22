#include "reauto/odom/Odometry.hpp"
#include "reauto/device/IMU.hpp"

namespace reauto {
Odometry::Odometry(TrackingWheels* wheels, device::IMU* imu) : m_wheels(wheels), m_imu(imu) {}

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

    pros::Task odomTask{
        [this] {
            std::cout << "[ReAuto] Started odometry" << std::endl;

            double currentRotationRad = m_imu->getRotation(true);

            // set all to 0. We'll change them if needed
            double middlePos = 0;
            double backPos = 0;
            double leftPos = 0;
            double rightPos = 0;

            if (m_wheels->back != nullptr) {
                backPos = m_wheels->back->getPosition();
            }

            if (m_wheels->center != nullptr) {
                middlePos = m_wheels->center->getPosition();
            }

            if (m_wheels->left != nullptr) {
                leftPos = m_wheels->left->getPosition();
            }

            if (m_wheels->right != nullptr) {
                rightPos = m_wheels->right->getPosition();
            }

            m_prevRotationRad = currentRotationRad;
            m_prevMiddlePos = middlePos;
            m_prevBackPos = backPos;
            m_prevLeftPos = leftPos;
            m_prevRightPos = rightPos;

            while (true) {
                middlePos = (m_wheels->center != nullptr) ? m_wheels->center->getPosition() : 0;
                backPos = (m_wheels->back != nullptr) ? m_wheels->back->getPosition() : 0;
                leftPos = (m_wheels->left != nullptr) ? m_wheels->left->getPosition() : 0;
                rightPos = (m_wheels->right != nullptr) ? m_wheels->right->getPosition() : 0;

                currentRotationRad = m_imu->getRotation(true);

                // calculate the change in rotation
                double deltaLeftPos = leftPos - m_prevLeftPos;
                double deltaRightPos = rightPos - m_prevRightPos;
                double deltaMiddlePos = middlePos - m_prevMiddlePos;
                double deltaBackPos = backPos - m_prevBackPos;
                double dHeading = currentRotationRad - m_prevRotationRad;

                // update previous values
                m_prevLeftPos = leftPos;
                m_prevRightPos = rightPos;
                m_prevMiddlePos = middlePos;
                m_prevBackPos = backPos;
                m_prevRotationRad = currentRotationRad;

                double dFwd = 0, dSide = 0;

                // calculate vertical and horizontal movement IN INCHES
                switch (m_wheels->config) {
                case TrackingConfiguration::CB:
                    dFwd = math::degToIn(deltaMiddlePos, m_wheels->center->getDiameter()) - m_wheels->center->getCenterDistance() * dHeading;
                    dSide = math::degToIn(deltaBackPos, m_wheels->back->getDiameter()) - m_wheels->back->getCenterDistance() * dHeading;
                    break;

                case TrackingConfiguration::LR:
                    dFwd = math::degToIn(deltaLeftPos, m_wheels->left->getDiameter()) - m_wheels->left->getCenterDistance() * dHeading;
                    break;

                case TrackingConfiguration::LRB:
                    dFwd = math::degToIn(deltaLeftPos, m_wheels->left->getDiameter()) - m_wheels->left->getCenterDistance() * dHeading;
                    dSide = math::degToIn(deltaBackPos, m_wheels->back->getDiameter()) - m_wheels->back->getCenterDistance() * dHeading;
                    break;

                case TrackingConfiguration::NA:
                    // user has no unpowered wheels (same as LR because it's abstracted)
                    dFwd = math::degToIn(deltaLeftPos, m_wheels->left->getDiameter()) - m_wheels->left->getCenterDistance() * dHeading;
                    break;
                }

                // calculate cosine and sine of rotation
                double cosHeading = cos(currentRotationRad);
                double sinHeading = sin(currentRotationRad);

                // calculate the change in position
                double dX = dFwd * cosHeading - dSide * sinHeading;
                double dY = dFwd * sinHeading + dSide * cosHeading;

                // update the position
                m_pos.x += dX;
                m_pos.y += dY;

                pros::delay(ODOM_TIMESTEP);
            }
        }
    };
}
}