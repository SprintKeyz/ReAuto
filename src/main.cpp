#include "main.h"
#include "pros/abstract_motor.hpp"
#include "reauto/api.hpp"
#include "reauto/filter/SMAFilter.hpp"
#include "reauto/motion/profile/MotionProfile.hpp"
#include "reauto/motion/profile/TrapezoidalProfile.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

 /**
  * Runs initialization code. This occurs as soon as the program is started.
  *
  * All other competition modes are blocked by initialize; it is recommended
  * to keep execution time for this mode under a few seconds.
  */

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
.trackingWheels({ 12, 0.5 }, { 11, 4 }, 2.75, true)
.setTrackWidth(11.25_in)
.build();

PIDExits exits = {
    0.1,
    0,
    120,
    0,
    150,
};

PIDConstants hK = {
    0.1,
    0,
    0,
};

auto headingController = std::make_shared<reauto::controller::PIDController>(hK, exits);

reauto::TrapezoidalProfileConstants k = {
    12,
    2.5,
    4,
    1.57,
    31.5,
    8 };

std::shared_ptr<reauto::TrapezoidalProfile> profile = std::make_shared<reauto::TrapezoidalProfile>(chassis, k, headingController);

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
  /*chassis->setSlewDrive(24.0, 5.0);
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
  }*/

  reauto::filter::SMAFilter filter(5);

  while (true) {
    chassis->tank();
    double leftVel = chassis->getLeftVelocity();
    std::cout << "Velocity: " << leftVel << std::endl;
    pros::delay(15);
  }
}
