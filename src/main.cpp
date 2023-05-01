#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "reauto/api.hpp"
#include "reauto/device/Catapult.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::adi::Pneumatics cataPiston('B', false);
pros::adi::Pneumatics expansionPiston('C', false);
pros::Motor intakeMotor(10, pros::Motor_Gears::blue);

auto chassis =
reauto::ChassisBuilder<>()
.motors({ -20, -6, 1 }, { 5, 3, -2 }, pros::Motor_Gears::blue)
.controller(master)
.imu(7)
.trackingWheels({ -12, 1.168 }, { 11, 4.51705 }, 2.75, true)
.setTrackWidth(11_in)
.build();

std::vector<IPIDConstants> linConstants = {
    {28.2, 52, 3.7, 0},
    {28.7, 52, 3.71, 6}
};

std::vector<IPIDConstants> angConstants = {
    {6.802, 20, 0.72, 0},
    {6.8, 20, 0.76, 90}
};

PIDExits linExits = {
    0.1,
    0.4,
    50,
    140,
    250 };

PIDExits angExits = {
    0.5,
    1,
    60,
    150,
    250 };

auto linearPID = std::make_shared<reauto::controller::PIDController>(linConstants, linExits, 2.2);
auto angularPID = std::make_shared<reauto::controller::PIDController>(angConstants, angExits, 10);
auto controller = std::make_shared<reauto::MotionController>(chassis, linearPID.get(), angularPID.get(), 3.2);

std::shared_ptr<reauto::device::Catapult> cata = std::make_shared<reauto::device::Catapult>(8, 'A');

void initialize()
{
  // chassis->strafe(80, 100);
  chassis->init();
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol()
{
  chassis->setSlewDrive(24.0, 5.0);
  chassis->setDriveExponent(3);
  chassis->setControllerDeadband(12);
  chassis->setDriveMaxSpeed(100_pct);
  double pistonTime = pros::millis();

  while (true)
  {
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
      cata->fireAsync();
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
      intakeMotor = -127;
    }

    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
      intakeMotor = 127;
    }
    else
    {
      intakeMotor = 0;
    }

    // expansion
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X))
    {
      // chassis.turn(90, 50);
      expansionPiston.set_value(true);
      pistonTime = pros::millis();
    }

    if (expansionPiston.get_state() && pros::millis() - pistonTime > 800)
    {
      expansionPiston.set_value(false);
      pistonTime = pros::millis();
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
    {
      cataPiston.toggle();
      pros::delay(150);
    }

    chassis->tank();
    pros::delay(10);
  }
}