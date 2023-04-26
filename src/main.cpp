#include "main.h"
#include "pros/abstract_motor.hpp"
#include "reauto/api.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::adi::DigitalIn cataLimit('A');
pros::adi::Pneumatics cataPiston('B', true);

pros::adi::Pneumatics expansionPiston('C', false);

pros::Motor intakeMotor(10, pros::Motor_Gears::blue);

pros::Motor cata(4);

#define CATA_SPEED 127

auto chassis =
reauto::ChassisBuilder<>()
.motors({ -20, -6, 1 }, { 5, 3, -2 }, pros::Motor_Gears::blue)
.controller(master)
.imu(7)
.trackingWheels({ -12, 0.001 }, { 11, 4.282 }, 2.75, true)
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
    400 };

PIDConstants hcConstants = { 2.1, 0, 0 };

auto hc = std::make_shared<reauto::controller::PIDController>(hcConstants, hcExits);
auto profile = std::make_shared<reauto::TrapezoidalProfile>(chassis, constants, hc);

std::vector<IPIDConstants> linConstants = {
    {28.2, 52, 3.7, 0},
    {28.7, 52, 3.71, 6},
    {24.5, 74, 3.28, 12},
    {24.4, 69, 3.28, 24},
};

#define TURN_AMT 24

/*
lin:
{14.97, 0.0, 1.51, 3},
    {10.4, 0, 1.3, 6},
    {9.3, 0, 0.85, 12},
    {8.22, 0, 0.9, 18},
    {10.79, 0, 1.29, 24},*/

std::vector<IPIDConstants> angConstants = {
    {6.802, 20, 0.72, 0},
    {6.78, 20, 0.767, 90}
};

/*
ang:
{6.2, 0, 0.39, 15},
    {5.2, 0, 0.485, 30},
    {5.2, 0, 0.502, 45},
    {5.18, 0, 0.59, 90},*/

PIDExits linExits = {
    0.25,
    0.75,
    50,
    80,
    250 };

PIDExits angExits = {
    0.5,
    1,
    50,
    80,
    250 };

auto linearPID = std::make_shared<reauto::controller::PIDController>(linConstants, linExits, 2.5);
auto angularPID = std::make_shared<reauto::controller::PIDController>(angConstants, angExits, 10);

auto controller = std::make_shared<reauto::MotionController>(chassis, linearPID.get(), angularPID.get(), 0);

auto helper = std::make_shared<reauto::TuningHelper>(master, chassis, linearPID, angularPID);

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

void cataTask()
{
  cata = -CATA_SPEED;
  pros::delay(600);
  // fired, retract until hit switch
  cata = -CATA_SPEED;

  while (!cataLimit.get_value())
  {
    pros::delay(10);
  }

  cata.brake();
}

void fireCata()
{
  pros::Task fireCataTask(cataTask);
}

void soloWP() {
  chassis->setPose({ 0, 0, 0_deg });

  // get the roller
  controller->drive(6.4);
  intakeMotor.move_relative(240, 600);
  pros::delay(150);
  // go shoot
  controller->drive(-13, 100_pct);
  controller->turn(-11, 100_pct);
  fireCata();
  pros::delay(400);

  // get three stack
  controller->drive(3.8);
  intakeMotor = -127;
  controller->turn(-135);
  controller->drive(36, 40_pct);

  // shoot
  controller->turn(-35, 100_pct);
  controller->drive(-5, 100_pct);
  fireCata();
  pros::delay(400);

  // get three stack
  controller->turn(-135);
  controller->drive(42, 40_pct);

  // turn to shoot
  controller->turn(-61, 100_pct);
  controller->drive(-5, 100_pct);

  // shoot
  fireCata();
  pros::delay(400);

  //intakeMotor = 0;

  // turn and get roller
  controller->turn(-135);
  controller->drive(33);
  controller->turn(-90);
  controller->drive(3.8);

  // get roller
  intakeMotor.move_relative(240, 600);
}

void autoTwoShots() {
  chassis->setPose({ 0, 0, 0_deg });

  // get the roller
  controller->drive(6.4);
  intakeMotor.move_relative(240, 600);
  pros::delay(150);
  // go shoot
  controller->drive(-13, 100_pct);
  controller->turn(-11, 100_pct);
  fireCata();
  pros::delay(400);

  // get three stack
  controller->drive(3.8);
  intakeMotor = -127;
  controller->turn(-135);
  controller->drive(36, 40_pct);

  // shoot
  controller->turn(-35, 100_pct);
  controller->drive(-5, 100_pct);
  fireCata();
  pros::delay(400);
}

void soloWPNoFirstShot() {
  chassis->setPose({ 0, 0, 0_deg });

  // get the roller
  controller->drive(6.4);
  intakeMotor.move_relative(240, 600);
  pros::delay(150);

  // get three stack
  controller->drive(-9.8);
  intakeMotor = -127;
  controller->turn(-135);
  controller->drive(36, 55_pct);

  // shoot
  controller->turn(-35, 100_pct);
  controller->drive(-5, 100_pct);
  fireCata();
  pros::delay(400);

  // get three stack
  controller->turn(-135);
  controller->drive(42, 40_pct);

  // turn to shoot
  controller->turn(-61, 100_pct);
  controller->drive(-5, 100_pct);

  // shoot
  fireCata();
  pros::delay(400);

  intakeMotor = 0;

  // turn and get roller
  controller->turn(-135);
  controller->drive(33);
  controller->turn(-90);
  controller->drive(3.8);

  // get roller
  intakeMotor.move_relative(360, 600);
}

void rollerOnly() {
  chassis->setPose({ 0, 0, 0_deg });

  // get the roller
  controller->drive(6.4);
  intakeMotor.move_relative(240, 600);
  pros::delay(150);
}

void skills() {
  chassis->setPose({ 0, 0, 0_deg });

  // get the roller
  controller->drive(6.4);
  intakeMotor.move_relative(360, 600);
  pros::delay(150);
  // go shoot
  controller->drive(-13, 100_pct);
  controller->turn(-11, 100_pct);
  fireCata();
  pros::delay(400);

  // get three stack
  controller->drive(2.2);
  intakeMotor = -127;
  controller->turn(-135);
  controller->drive(36, 40_pct);

  // shoot
  controller->turn(-35, 100_pct);
  controller->drive(-5, 100_pct);
  fireCata();
  pros::delay(400);

  // get three stack
  controller->turn(-135);
  controller->drive(42, 40_pct);

  // turn to shoot
  controller->turn(-61, 100_pct);
  controller->drive(-5, 100_pct);

  // shoot
  fireCata();
  pros::delay(400);

  intakeMotor = 0;

  // turn and get roller
  controller->turn(-135);
  controller->drive(33);
  controller->turn(-90);
  controller->drive(3.8);

  // get roller
  intakeMotor.move_relative(360, 600);
  pros::delay(150);

  // back up
  controller->drive(-7);
  controller->turn(-115);

  // expand
  expansionPiston.extend();

  /*controller->turn(-180);
  controller->drive(16);

  // spin roller
  intakeMotor.move_relative(360, 600);
  pros::delay(150);

  // back up
  controller->drive(-11);
  controller->turn(-45);
  controller->drive(-35, 40_pct);

  // turn and shoot
  controller->turn(135);
  controller->drive(-5);
  fireCata();
  pros::delay(400);

  // turn and get next 3
  controller->turn(-45);
  controller->drive(36, 40_pct);

  // turn and shoot
  controller->turn(-70);
  fireCata();
  pros::delay(400);

  // turn and drive to corner
  controller->turn(-35);
  controller->drive(40);

  // spin around to shoot
  controller->turn(45);*/
}

void autonomous()
{
  // profile->compute(18_in);
  // profile->followLinear();

  // profile->compute(90_deg);
  // profile->followAngular();

  // profile->compute(24_in);
  // profile->followLinear();
  
  //soloWP();
  soloWPNoFirstShot();
  //autoTwoShots();
  //skills();
  //rollerOnly();
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

void computeDistCenter() {
  double currentHeading = chassis->getHeading(true);
  double distCenterWheel = chassis->getTrackingWheels()->center->getDistanceTraveled();
  double distBackWheel = chassis->getTrackingWheels()->back->getDistanceTraveled();

  std::cout << "center distance: " << distCenterWheel/currentHeading << std::endl;
  std::cout << "back distance: " << distBackWheel/currentHeading << std::endl;
}

void opcontrol()
{
  //controller->drive({ 12, -10 }, 60_pct);
  //controller->turn(90);
  //controller->drive(-10, 60_pct);
  //computeDistCenter();

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

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
    {
      cataPiston.toggle();
      pros::delay(150);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
      autonomous();
    }

    chassis->tank();

    Pose chassisPos = chassis->getPose();

    if (debug == 13)
    {
      debug = 0;
      printf("X: %f, Y: %f, Angle: %f\n", chassisPos.x, chassisPos.y, chassisPos.theta.value());
      // printf("Angle: %f\n", chassis->getHeading());
    }

    debug++;
    pros::delay(15);
  }
}