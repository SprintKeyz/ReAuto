#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "reauto/api.hpp"
#include "reauto/chassis/impl/MotionChassis.hpp"
#include "reauto/controller/impl/PIDController.hpp"
#include "reauto/datatypes/PIDConstants.h"
#include "reauto/datatypes/PIDExits.h"
#include "reauto/device/Catapult.hpp"
#include "reauto/motion/profile/TrapezoidalProfile.hpp"
#include <memory>

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::adi::Pneumatics cataPiston('B', false);
pros::adi::Pneumatics expansionPiston('C', false);
pros::Motor intakeMotor(10, pros::MotorGears::blue);

auto chassis =
reauto::ChassisBuilder<>()
.motors({ 20, 2 }, { -19, -1 }, pros::MotorGears::blue)
.controller(master)
.imu(6)
.setChassisConstants(10.5, 3.25, 260)
.build();

PIDConstants headingConstants = { 5.2, 0.0, 0.0 };
PIDExits headingExits = { 0.5, 0.75, 180, 450 };

auto headingPID = std::make_shared<reauto::controller::PIDController>(headingConstants, headingExits);

// motion profile
reauto::TrapezoidalProfileConstants constants = { 95, 1.34, 28, 0.5 };
auto profile = reauto::TrapezoidalProfile(chassis, constants, nullptr);

// pure pursuit
reauto::motion::PurePursuit purePursuitController(chassis.get());

void initialize()
{
	chassis->init();
	chassis->setBrakeMode(pros::MotorBrake::hold);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	purePursuitController.follow("/usd/path1.txt", 10000, 10);
}

void opcontrol()
{
	//chassis->setSlewDrive(24.0, 5.0);
	chassis->setDriveExponent(3);
	chassis->setControllerDeadband(12);
	chassis->setDriveMaxSpeed(100_pct);
	double pistonTime = pros::millis();

	while (true)
	{
		chassis->arcade();
		pros::delay(10);
	}
}