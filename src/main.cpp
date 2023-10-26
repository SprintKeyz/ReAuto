#include "main.h"
#include "pros/misc.hpp"
#include "reauto/api.hpp"
#include "reauto/motion/purepursuit/PurePursuit.hpp"
#include "reauto/odom/Odometry.hpp"
#include <fstream>
#include <memory>
#include <streambuf>
#include <string>

using namespace reauto;

pros::Controller master(pros::E_CONTROLLER_MASTER);

auto chassis =
	ChassisBuilder<>()
		.motors({14, -19, -18}, {16, 15, -13}, pros::MotorGears::blue)
		.controller(master)
		.setChassisConstants(10.5_in, 3.25_in, 360)
		.imu(21)
		.odomPrefs(OdomPrefs::PREFER_RIGHT_WHEEL) /* optional, don't add this line unless needed */
		.build();

// the catapult is free spinning, so no need to add any functionality to reauto.
pros::Motor cata(12);
pros::Motor intake(11);

// walls
pros::adi::Pneumatics walls('A', false);
pros::adi::Pneumatics doinker('B', false);
pros::adi::Pneumatics climb('E', false);

// set up PID controller for lateral movement
std::vector<IPIDConstants> latConstants = {
	{0, 0, 0, 0}
};

// angular movement
std::vector<IPIDConstants> angConstants = {
	{0, 0, 0, 0}
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

auto linPID = std::make_shared<controller::PIDController>(latConstants, linExits);
auto angPID = std::make_shared<controller::PIDController>(angConstants, angExits);
auto controller = std::make_shared<MotionController>(chassis, linPID.get(), angPID.get());

auto purePursuit = std::make_shared<motion::PurePursuit>(chassis.get());

void initialize() {
	chassis->init();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	purePursuit->follow("/usd/path.txt", 20000, 10);

	// PID testing
	//controller->drive(12_in);
	//controller->turn(90_deg);
}

void opcontrol() {
	chassis->setDriveExponent(3);
 	chassis->setControllerDeadband(12);

	while (true) {
		// step our tank drive loop, handled by reauto
		chassis->tank();

		// get pose
		Pose p = chassis->getPose();
		std::cout << "X: " << p.x << ", Y: " << p.y << ", Angle: " << p.theta.value_or(0) << std::endl;

		// intake controls
		if (master.get_digital(DIGITAL_R1)) {
			intake.move_voltage(-12000);
		}

		else if (master.get_digital(DIGITAL_R2)) {
			intake.move_voltage(12000);
		}

		else {
			intake.brake();
		}

		// catapult controls
		if (master.get_digital(DIGITAL_L1)) {
			cata.move_voltage(10800);
		}

		else {
			cata.move_voltage(0);
		}
		
		// pneumatic walls
		if (master.get_digital_new_press(DIGITAL_L2)) {
			walls.toggle();
		}

		if (master.get_digital_new_press(DIGITAL_A)) {
			doinker.toggle();
		}

		// climb
		if (master.get_digital_new_press(DIGITAL_X)) {
			climb.toggle();
		}

		pros::delay(10);
	}
}