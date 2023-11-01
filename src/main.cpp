#include "main.h"
#include "pros/misc.hpp"
#include "reauto/api.hpp"
#include "reauto/motion/purepursuit/PurePursuit.hpp"
#include "reauto/odom/Odometry.hpp"
#include <fstream>
#include <memory>
#include <streambuf>
#include <string>

pros::Controller master(pros::E_CONTROLLER_MASTER);

auto chassis =
	reauto::ChassisBuilder<>()
		.motors({14, -19, -18}, {16, 15, -13}, pros::MotorGears::blue)
		.controller(master)
		.setChassisConstants(10.5_in, 3.25_in, 360)
		.imu(21)
		.build();

// the catapult is free spinning, so no need to add any functionality to reauto.
pros::Motor cata(12);
pros::Motor intake(11);

// walls
pros::adi::Pneumatics walls('A', false);
pros::adi::Pneumatics doinker('B', false);
pros::adi::Pneumatics climb('E', false);

#define LAT_MOVE 12
#define ANG_MOVE 45

// set up PID controller for lateral movement
std::vector<IPIDConstants> latConstants = {
	{ 24, 0, 0, 0},
	{ 12, 0, 0.4, 6},
	{ 12, 0, 0.4, 12},
	{ 12, 0, 0.6, 18},
	{ 12, 0, 0.7, 24},
	{12, 0, 0.9, 60}
};

// angular movement
std::vector<IPIDConstants> angConstants = {
	{3.8, 0, 0.21, 45},
	{3.6, 0, 0.24, 90},
	{3.4, 0, 0.24, 120}
};

PIDExits latExits = {
	0.1,
	0.5,
	50,
	140,
	250
};

PIDExits angExits = {
	0.5,
	2,
	60,
	150,
	250
};

auto latPID = std::make_shared<reauto::controller::PIDController>(latConstants, latExits, 0, 5);
auto angPID = std::make_shared<reauto::controller::PIDController>(angConstants, angExits, 5);
auto controller = std::make_shared<reauto::MotionController>(chassis, latPID.get(), angPID.get());

auto purePursuit = std::make_shared<reauto::motion::PurePursuit>(chassis.get());

void initialize() {
	chassis->init();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	//purePursuit->follow("/usd/path.txt", 20000, 10);

	// PID testing
	//controller->turn(90_deg);

	controller->turn(-32_deg);
	controller->drive({0, 82}, 80_pct);
	controller->drive({-6, 82}, 100_pct, true);
	controller->drive({-28, 77}, 80_pct, true);
	controller->drive({-63, 60}, 80_pct, true);
	controller->turn(0_deg, 80_pct, false, 750);
	controller->drive(-12_in, 100_pct, 1000);

	pros::delay(5000);

	// drive back
	controller->drive({-6, 80}, 80_pct);
	controller->drive({0, 0}, 60_pct);

	/*controller->turn(50_deg, 127, false, 750);
	controller->drive(82_in, 100, 4500); // this should be 82
	controller->turn(125_deg, 127, false, 750);
	controller->drive(-22_in, 127, 1500);
	controller->turn(65_deg, 127, false, 750);
	controller->drive(-14_in, 127, 1500);
	controller->turn(155_deg, 127, false, 750);
	controller->drive(-24_in, 127, 1500);
	controller->turn(-45_deg, 100, false, 750); // TODO: TUNE THIS!!!*/
}

void opcontrol() {
	//controller->drive(LAT_MOVE);
	//controller->turn(ANG_MOVE);

	chassis->setDriveExponent(3);
 	chassis->setControllerDeadband(12);

	//controller->drive({12, 12}, 127);

	while (true) {
		// step our tank drive loop, handled by reauto
		chassis->tank();

		// get pose
		Pose p = chassis->getPose();
		//std::cout << "X: " << p.x << ", Y: " << p.y << ", Angle: " << p.theta.value_or(0) << std::endl;

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