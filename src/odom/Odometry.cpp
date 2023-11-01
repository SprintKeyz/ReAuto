#include "reauto/odom/Odometry.hpp"
#include "reauto/device/IMU.hpp"
#include "reauto/device/TrackingWheels.hpp"
#include "reauto/math/Convert.hpp"
#include <cmath>
#include <iostream>

namespace reauto {
Odometry::Odometry(TrackingWheels* wheels, device::IMU* imu, OdomPrefs prefs) : m_wheels(wheels), m_imu(imu) {
    if (prefs == OdomPrefs::PREFER_RIGHT_WHEEL) {
        m_preferRightWheel = false; // we NEED to fix odomprefs, always returns prefer right wheel
    }
}

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
                leftPos = math::inToDeg(m_wheels->left->getDistanceTraveled(), m_wheels->left->getDiameter());
            }

            if (m_wheels->right != nullptr) {
                rightPos = leftPos = math::inToDeg(m_wheels->left->getDistanceTraveled(), m_wheels->left->getDiameter());
            }

            m_prevRotationRad = currentRotationRad;
            m_prevMiddlePos = middlePos;
            m_prevBackPos = backPos;
            m_prevLeftPos = leftPos;
            m_prevRightPos = rightPos;

            while (true) {
                middlePos = (m_wheels->center != nullptr) ? m_wheels->center->getPosition() : 0;
                backPos = (m_wheels->back != nullptr) ? m_wheels->back->getPosition() : 0;
                leftPos = (m_wheels->left != nullptr) ? math::inToDeg(m_wheels->left->getDistanceTraveled(), m_wheels->left->getDiameter()) : 0;
                rightPos = (m_wheels->right != nullptr) ? math::inToDeg(m_wheels->right->getDistanceTraveled(), m_wheels->right->getDiameter()) : 0;

                currentRotationRad = m_imu->getRotation(true);

                // calculate the change in rotation
                double deltaLeftPos = leftPos - m_prevLeftPos;
                double deltaRightPos = rightPos - m_prevRightPos;
                double deltaMiddlePos = middlePos - m_prevMiddlePos;
                double deltaBackPos = backPos - m_prevBackPos;
                double dRotation = currentRotationRad - m_prevRotationRad;

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
                    dFwd = math::degToIn(deltaMiddlePos, m_wheels->center->getDiameter()) - m_wheels->center->getCenterDistance() * dRotation;
                    dSide = math::degToIn(deltaBackPos, m_wheels->back->getDiameter()) - m_wheels->back->getCenterDistance() * dRotation;
                    break;

                case TrackingConfiguration::LR:
                    if (m_preferRightWheel) {
                        dFwd = math::degToIn(deltaRightPos, m_wheels->right->getDiameter()) - m_wheels->right->getCenterDistance() * dRotation;
                    }

                    else {
                        dFwd = math::degToIn(deltaLeftPos, m_wheels->left->getDiameter()) - m_wheels->left->getCenterDistance() * dRotation;
                    }
                    break;

                case TrackingConfiguration::LRB:
                    dFwd = math::degToIn(deltaLeftPos, m_wheels->left->getDiameter()) - m_wheels->left->getCenterDistance() * dRotation;
                    dSide = math::degToIn(deltaBackPos, m_wheels->back->getDiameter()) - m_wheels->back->getCenterDistance() * dRotation;
                    break;

                case TrackingConfiguration::NA:
                    // user has no unpowered wheels (same as LR because it's abstracted)
                    if (m_preferRightWheel) {
                        dFwd = math::degToIn(deltaRightPos, m_wheels->right->getDiameter()) - m_wheels->right->getCenterDistance() * dRotation;
                    }

                    else {
                        dFwd = math::degToIn(deltaLeftPos, m_wheels->left->getDiameter()) - (m_wheels->left->getCenterDistance() * dRotation);
                    }

                    break;
                }

                // calculate cosine and sine of rotation
                double cosHeading = cos(currentRotationRad);
                double sinHeading = sin(currentRotationRad);

                double dX, dY;

                // calculate the change in position
                if (m_wheels->config == TrackingConfiguration::NA || m_wheels->config == TrackingConfiguration::LR) {
                    dX = dFwd * sinHeading;
                    dY = dFwd * cosHeading;
                }

                else {
                    dX = dFwd * cosHeading - dSide * sinHeading;
                    dY = dFwd * sinHeading + dSide * cosHeading;
                }

                if (!std::isnan(dX) && !std::isnan(dY)) {
                    // update the position
                    m_pos.x += dX;
                    m_pos.y += dY;
                }

                pros::delay(ODOM_TIMESTEP);
            }
        }
    };
}
}