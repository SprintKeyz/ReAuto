#include "main.h"
#include "pros/abstract_motor.hpp"
#include "reauto/api.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::adi::DigitalIn cataLimit('A');
pros::adi::Pneumatics cataPiston('B', false);

pros::adi::Pneumatics expansionPiston('C', false);

pros::Motor intakeMotor(10, pros::Motor_Gears::blue);

pros::Motor cata(4);

#define CATA_SPEED 127

auto chassis =
reauto::ChassisBuilder<>()
.motors({ -20, -6, 1 }, { 5, 3, -2 }, pros::Motor_Gears::blue)
.controller(master)
.imu(7)
.trackingWheels({ -12, 1.168 }, { 11, 4.51705 }, 2.75, true)
.setTrackWidth(11_in)
.build();

// velocities in in/s
reauto::TrapezoidalProfileConstants constants = {
  90,
  2.06,
  59.7,
  0.308,
  2.02,
  0.08,
};

PIDExits hcExits = {
  0.25,
  0.85,
  60,
  150,
  400
};

PIDConstants hcConstants = { 3.8, 0, 0 };

auto hc = std::make_shared<reauto::controller::PIDController>(hcConstants, hcExits);
auto profile = std::make_shared<reauto::TrapezoidalProfile>(chassis, constants, hc);

std::vector<IPIDConstants> linConstants = {
  {14.97, 0.0, 1.51, 3},
  {10.4, 0, 1.3, 6 },
  { 9.3, 0, 0.85, 12 },
  { 8.22, 0, 0.9, 18 },
  { 10.79, 0, 1.29, 24 },
};

std::vector<IPIDConstants> angConstants = {
  {6.2, 0, 0.39, 15},
  {5.2, 0, 0.485, 30},
  {5.2, 0, 0.502, 45},
  {5.18, 0, 0.59, 90},
};

PIDExits linExits = {
  0.1,
  0.4,
  50,
  140,
  250
};

PIDExits angExits = {
  0.5,
  1,
  60,
  150,
  250
};

auto linearPID = std::make_shared<reauto::controller::PIDController>(linConstants, linExits, 0, 16);
auto angularPID = std::make_shared<reauto::controller::PIDController>(angConstants, angExits, 10, 0);

auto controller = std::make_shared<reauto::MotionController>(chassis, linearPID.get(), angularPID.get(), 3.2);

void initialize()
{
  // chassis->strafe(80, 100);
  chassis->init();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{
  //profile->compute(18_in);
  //profile->followLinear();

  //profile->compute(90_deg);
  //profile->followAngular();

  //profile->compute(24_in);
  //profile->followLinear();

  // chained move to points
  controller->drive({ 46, 21 }, 100_pct, 0, 3.5_in);
  controller->drive({ 30, 18 }, 100_pct, 0, 3.5_in);
  controller->drive({ 21, -5 }, 100_pct, 0, 3.5_in);
  controller->drive({ 37, 23 }, 100_pct, 0, 3.2_in);
  controller->drive({ 56, -65 }, 100_pct, 0, 3.5_in);
  controller->drive({ 18, 34 }, 100_pct);
  controller->drive({ 0, 0 }, 100_pct);
  controller->turn(0);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
  chassis->setSlewDrive(24.0, 5.0);
  chassis->setDriveExponent(3);
  chassis->setControllerDeadband(12);
  chassis->setDriveMaxSpeed(100_pct);

  double initialTime = pros::millis();
  double pistonTime = pros::millis();

  bool firing = false;
  bool shouldBeStopped = true;

  // var for debugging
  int debug = 0;

  while (true)
  {
    // cata
    // here's how it works:

    // the catapult starts off touching the limit switch
    // then, when the button is pressed, it should go down a bit more, then fire
    // we delay to wait for the fire to complete, then retract it again until it hits the limit
    // at which point we stop and wait for next press

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
      // go down and fire
      firing = true;
      initialTime = pros::millis();
      cata = -CATA_SPEED;
      shouldBeStopped = false;
    }

    if (firing)
    {
      if (pros::millis() - initialTime >= 600)
      {
        // fired!
        cata = 0;
        firing = false;
      }
    }

    if (!cataLimit.get_value() && !shouldBeStopped)
    {
      // we have fired, wait a second and go down
      if (pros::millis() - initialTime >= 700)
      {
        cata = -CATA_SPEED;
      }
    }

    if (!firing && cataLimit.get_value())
    {
      // we have retracted, stop!
      shouldBeStopped = true;
      cata = 0;
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

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      cataPiston.toggle();
      pros::delay(150);
    }

    chassis->tank();

    Pose chassisPos = chassis->getPose();

    if (debug == 13)
    {
      debug = 0;
      printf("X: %f, Y: %f, Angle: %f\n", chassisPos.x, chassisPos.y, chassisPos.theta);
      //printf("Angle: %f\n", chassis->getHeading());
    }

    debug++;
    pros::delay(15);
  }
}