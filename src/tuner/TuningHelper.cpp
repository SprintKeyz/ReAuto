#include "reauto/tuner/TuningHelper.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

namespace reauto {
TuningHelper::TuningHelper(pros::Controller& controller, std::shared_ptr<MotionChassis> chassis, std::shared_ptr<controller::PIDController> lin, std::shared_ptr<controller::PIDController> ang): m_controller(controller), m_chassis(chassis), m_lin(lin), m_ang(ang) {}

void TuningHelper::runDistance() {
    // target is the first distance
    m_target = m_distances[0];

    // var to edit (kP, kI, kD)
    TuningCompoment editing = TuningCompoment::KP_VALUE;

    // the value of the variable being edited
    double value = 0;

    // the current index of the distance array
    int index = 0;

    // set initial constants to PID constants
    m_constants = m_lin->getConstants(m_target);

    // print initial screen
    m_controller.print(0, 0, "kP: %f", m_constants.kP);
    pros::delay(80);
    m_controller.print(1, 0, "Target: %f", m_target);
    pros::delay(80);
    m_controller.print(2, 0, "Error: 0");
    pros::delay(80);

    // run a loop for the controller!
    while (true) {
        if (m_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            // cycle to next tuning variable
            switch (editing) {
            case TuningCompoment::KP_VALUE:
                editing = TuningCompoment::KD_VALUE;
                value = m_constants.kD;
                break;

            case TuningCompoment::KD_VALUE:
                editing = TuningCompoment::KI_VALUE;
                value = m_constants.kI;
                break;

            case TuningCompoment::KI_VALUE:
                editing = TuningCompoment::KP_VALUE;
                value = m_constants.kP;
                break;
            }

            // update the controller screen
            update(editing, value);
        }

        if (m_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            // cycle to previous tuning variable
            switch (editing) {
            case TuningCompoment::KP_VALUE:
                editing = TuningCompoment::KD_VALUE;
                value = m_constants.kD;
                break;

            case TuningCompoment::KD_VALUE:
                editing = TuningCompoment::KI_VALUE;
                value = m_constants.kI;
                break;

            case TuningCompoment::KI_VALUE:
                editing = TuningCompoment::KP_VALUE;
                value = m_constants.kP;
                break;
            }

            // update the controller screen
            update(editing, value);
        }

        // check if the up button is pressed
        if (m_controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            // increment the value
            value += 0.1;

            // update the controller screen
            update(editing, value);
        }

        // check if the down button is pressed
        if (m_controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            // decrement the value
            value -= 0.1;

            // update the controller screen
            update(editing, value);
        }

        // check if the B button is pressed
        if (m_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            // check if an index is the last index of the array
            int next_val = m_distances[(index + 1) % m_distances.size()];
            index = (index + 1) % m_distances.size();

            // update the target
            update(TuningCompoment::TARGET, next_val);
        }

        // check if the A button is pressed
        if (m_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            if (m_hasRun) {
                // reset the chassis by driving slowly backwards to the og point
                double initial = m_chassis->getTrackingWheels()->center->getDistanceTraveled();
                while (m_chassis->getTrackingWheels()->center->getDistanceTraveled() - initial > 0) {
                    m_chassis->setVoltage(-35, -35);
                }

                // brake
                m_chassis->brake();
                m_hasRun = false;
            }

            else {

                // run the controller!
                m_lin->setTarget(m_target);

                // get our initial distance
                double initial = m_chassis->getTrackingWheels()->center->getDistanceTraveled();

                // run a loop for the controller!
                while (!m_lin->settled()) {
                    // update the error
                    double lin_error = m_target - (m_chassis->getTrackingWheels()->center->getDistanceTraveled() - initial);

                    // calculate the output
                    double output = m_lin->calculate(lin_error);

                    // try to update the controller screen
                    update(TuningCompoment::ERROR, lin_error);

                    // set the output
                    m_chassis->setVoltage(output, output);

                    // delay for 10ms
                    pros::delay(10);
                }

                // clear the screen
                m_controller.clear();
                // print "SETTLED"
                pros::delay(80);
                m_controller.print(0, 0, "SETTLED");
                pros::delay(80);
                m_controller.print(0, 0, "A -> reset");
                pros::delay(80);

                m_hasRun = true;

                m_chassis->brake();
            }
        }

        // delay for 20ms
        pros::delay(20);
    }
}

void TuningHelper::runAngle() {
    // target is the first distance
    m_target = m_angles[0];

    // var to edit (kP, kI, kD)
    TuningCompoment editing = TuningCompoment::KP_VALUE;

    // the value of the variable being edited
    double value = 0;

    // the current index of the distance array
    int index = 0;

    // set initial constants to PID constants
    m_constants = m_ang->getConstants(m_target);

    // print initial screen
    m_controller.print(0, 0, "kP: %f", m_constants.kP);
    pros::delay(80);
    m_controller.print(1, 0, "Target: %f", m_target);
    pros::delay(80);
    m_controller.print(2, 0, "Error: 0");
    pros::delay(80);

    // run a loop for the controller!
    while (true) {
        if (m_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            // cycle to next tuning variable
            switch (editing) {
            case TuningCompoment::KP_VALUE:
                editing = TuningCompoment::KD_VALUE;
                value = m_constants.kD;
                break;

            case TuningCompoment::KD_VALUE:
                editing = TuningCompoment::KI_VALUE;
                value = m_constants.kI;
                break;

            case TuningCompoment::KI_VALUE:
                editing = TuningCompoment::KP_VALUE;
                value = m_constants.kP;
                break;
            }

            // update the controller screen
            update(editing, value);
        }

        if (m_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            // cycle to previous tuning variable
            switch (editing) {
            case TuningCompoment::KD_VALUE:
                editing = TuningCompoment::KI_VALUE;
                value = m_constants.kI;
                break;

            case TuningCompoment::KI_VALUE:
                editing = TuningCompoment::KP_VALUE;
                value = m_constants.kP;
                break;

            case TuningCompoment::KP_VALUE:
                editing = TuningCompoment::KD_VALUE;
                value = m_constants.kD;
                break;
            }

            // update the controller screen
            update(editing, value);
        }

        // check if the up button is pressed
        if (m_controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            // increment the value
            value += 0.1;

            // update the controller screen
            update(editing, value);
        }

        // check if the down button is pressed
        if (m_controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            // decrement the value
            value -= 0.1;

            // update the controller screen
            update(editing, value);
        }

        // check if the B button is pressed
        if (m_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            // check if an index is the last index of the array
            int next_val = m_angles[(index + 1) % m_angles.size()];
            index = (index + 1) % m_angles.size();

            // update the target
            update(TuningCompoment::TARGET, next_val);
        }

        // check if the A button is pressed
        if (m_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            if (m_hasRun) {
                // reset the chassis by driving slowly backwards to the og point
                while (m_chassis->getHeading() > 0) {
                    m_chassis->setVoltage(-48, 48);
                }

                // brake
                m_chassis->brake();
                m_hasRun = false;
            }

            else {

                // run the controller!
                m_ang->setTarget(m_target);

                // run a loop for the controller!
                while (!m_ang->settled()) {
                    // update the error
                    double ang_error = m_target - m_chassis->getHeading();

                    // calculate the output
                    double output = m_ang->calculate(ang_error);

                    // try to update the controller screen
                    update(TuningCompoment::ERROR, ang_error);

                    // set the output
                    m_chassis->setVoltage(output, -output);

                    // delay for 10ms
                    pros::delay(10);
                }

                // clear the screen
                m_controller.clear();
                // print "SETTLED"
                pros::delay(80);
                m_controller.print(0, 0, "SETTLED");
                pros::delay(80);
                m_controller.print(0, 0, "A -> reset");
                pros::delay(80);

                m_hasRun = true;

                m_chassis->brake();
            }
        }

        // delay for 20ms
        pros::delay(20);
    }
}

void TuningHelper::update(TuningCompoment comp, double value) {
    switch (comp) {
    case TuningCompoment::KP_VALUE:
        // update the kp value
        m_constants.kP = value;
        // update the controller screen
        m_controller.clear_line(0);
        pros::delay(50);
        m_controller.print(0, 0, "kP: %f", value);
        break;

    case TuningCompoment::KI_VALUE:
        // update the ki value
        m_constants.kI = value;
        // update the controller screen
        m_controller.clear_line(0);
        pros::delay(50);
        m_controller.print(0, 0, "kI: %f", value);
        break;

    case TuningCompoment::KD_VALUE:
        // update the kd value
        m_constants.kD = value;
        // update the controller screen
        m_controller.clear_line(0);
        pros::delay(50);
        m_controller.print(0, 0, "kD: %f", value);
        break;

    case TuningCompoment::TARGET:
        // update the target
        m_target = value;
        // update the controller screen
        m_controller.clear_line(1);
        pros::delay(50);
        m_controller.print(1, 0, "Target: %f", value);
        break;

    case TuningCompoment::ERROR:
        // update the controller screen
        if (pros::millis() - m_lastUpdate < 80) return;
        m_controller.clear_line(2);
        pros::delay(50);
        m_controller.print(2, 0, "Error: %f", value);
        break;
    }

    // must delay 80ms or update won't be applied
    if (comp != TuningCompoment::ERROR && pros::millis() - m_lastUpdate < 80) {
        pros::delay(80);
    }

    // update the controller
    if (comp != TuningCompoment::ERROR) {
        m_lin->setConstants(m_constants);
        m_ang->setConstants(m_constants);
    }

    std::cout << "Constants: " << m_ang->getConstants(0).kP << ", " << m_ang->getConstants(0).kI << ", " << m_ang->getConstants(0).kD << std::endl;
}
}