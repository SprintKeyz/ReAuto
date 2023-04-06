#pragma once

#include "pros/abstract_motor.hpp"
#include "reauto/chassis/template/RobotTemplate.hpp"
#include "reauto/device/MotorSet.hpp"
#include <initializer_list>

namespace reauto
{
class TankBase: public RobotTemplate
{
public:
    explicit TankBase(std::initializer_list<int8_t> left, std::initializer_list<int8_t> right, pros::Motor_Gears gearset);

    void setLeftFwdVoltage(double voltage) override;
    void setRightFwdVoltage(double voltage) override;

    void setLeftFwdVelocity(double velocity) override;
    void setRightFwdVelocity(double velocity) override;

    void setFwdVoltage(double voltage) override;
    void setFwdVelocity(double velocity) override;

    void setTurnVoltage(double voltage) override;
    void setTurnVelocity(double velocity) override;

    void setBrakeMode(pros::Motor_Brake mode) override;
    void brake() override;

    // get motors
    MotorSet* getLeftMotors() const;
    MotorSet* getRightMotors() const;

    // get the left velocity
    double getLeftVelocity() const override;

private:
    std::shared_ptr<MotorSet> m_left;
    std::shared_ptr<MotorSet> m_right;
};
}
