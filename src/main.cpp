#include "main.h"
#include "extended_chassis.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/pid.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/adi.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include "devices.h"
#include "auton_selector.h"
#include "auton_routes.h"
#include "testing.h"
#include "old_systems.h"
// Task function for arm control

// initialize function. Runs on program startup
void initialize()
{
    deScores.set_value(LOW);
    stopper.set_value(HIGH);
    // controller.clear(); // clear controller screen
    lcd::initialize();   // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    pros::lcd::set_text_align(pros::lcd::Text_Align::CENTER);
}

void autonomous()
{
    all_motors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
    competitionSelector.runSelection();
    all_motors.brake();
    delay(2000);
    all_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
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

// this is a failsafe incase testing functions in opcontrol haven't been commented out
bool inCompetition = false;
bool red = false;
bool colorSortEnabled = true; // Color sorting is on by default
void onCenter_button()
{
    red = !red;
    competitionSelector.displaySelectionBrain();
}

void competition_initialize()
{

    inCompetition = true;
    // show current route on brain screen
    competitionSelector.displaySelectionBrain();

    // run buttons once to print values on screen
    on_left_button();
    on_right_button();

    // assign buttons to actions in auton selector
    lcd::register_btn0_cb(on_left_button);
    lcd::register_btn2_cb(on_right_button);

    lcd::register_btn1_cb(onCenter_button);
}

const double SMOOTHING_DENOMINATOR = 100; // Used to normalize the exponential curve
const double EXPONENTIAL_POWER = 2;       // Controls how aggressive the curve is
// Helper function that makes joystick input more precise for small movements
// while maintaining full power at maximum joystick
double logDriveJoystick(double joystickPCT)
{
    // Get the absolute value for calculation
    double magnitude = fabs(joystickPCT);

    // Calculate the smoothed value
    double smoothedValue = pow(magnitude, EXPONENTIAL_POWER) / SMOOTHING_DENOMINATOR;

    // Restore the original sign (positive or negative)
    return joystickPCT >= 0 ? smoothedValue : -smoothedValue;
}

void handleDriveTrain()
{

    // get left y and right y positions
    double leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    double rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // convert to pct
    leftY /= 1.27;
    rightY /= 1.27;

    leftY = logDriveJoystick(leftY);
    rightY = logDriveJoystick(rightY);

    // convert to gearset
    leftY *= 6;
    rightY *= 6;

    // skills change

    left_motors.move_velocity(leftY);
    right_motors.move_velocity(rightY);
}
void handleLoader()
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
        loader.set_value(!loader.get_value());
    }
}

void handleWings()
{
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
        deScores.set_value(HIGH);
    }
    else
    {
        deScores.set_value(LOW);
    }
}

void handleIntake()
{
    /*
    ballSensor.set_led_pwm(100);

    //bool justSaw=false;
    // activates on pressing R1


    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && ballSensor.get_hue()<20)
    {
        intake.move(127);
        delay(75);
        backGate.set_value(HIGH);
        delay(260);
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)&& (195<ballSensor.get_hue()<220)&&!(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))){
        backGate.set_value(LOW);
        intake.move(127);
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        intake.move(127);
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
        intake.move(-127);
        backGate.set_value(LOW);

    }
    else if(!(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))&&!(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) || controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)))
    {
        backGate.set_value(LOW);
        intake.brake();

    }*/
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
        intakeTop.move(127);
        intake.move(127);
        smallIntake.move(127);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
        /*if (colorSortEnabled)
        {
            if (red == true)
            {
                if (ballSensor.get_hue() < 20)
                {
                    intake.move(127);
                    intakeTop.move(-127);
                    delay(50);
                }
                else
                {
                    intake.move(127);
                    intakeTop.move(127);
                }
            }
            else if (red == false)
            {
                if (195 < ballSensor.get_hue() && ballSensor.get_hue() < 220)
                {
                    intake.move(127);
                    intakeTop.move(-127);
                }
                else
                {
                    intake.move(127);
                    intakeTop.move(127);
                }
            }
        }
        else
        {
            intake.move(127);
            intakeTop.move(127);
            smallIntake.move(-127);
        }*/
       intake.move(127);
        intakeTop.move(127);
        smallIntake.move(-127);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
        intake.move(-127);
        intakeTop.move(-127);
        smallIntake.move(127);
    }
    else
    {
        intake.brake();
        intakeTop.brake();
        //backGate.set_value(LOW);
        smallIntake.brake();
    }
}
void handleStopper()
{
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
        stopper.set_value(LOW);
    }
    else{
        stopper.set_value(HIGH);
    }
}

void multiTake()
{
    while (true)
    {
        handleIntake();
    }
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
    // left_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
    // right_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    ballSensor.set_led_pwm(100);
    // backGate.set_value(LOW);
    // bool buttonsPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

    /* pros::Task IntakeTask([]
                         { multiTake();});*/
    // csort::color_sort_task(nullptr);
    colorSortEnabled = false; // Disable color sorting for competition
    // loop forever
    while (true)
    {
        // Handle color sort toggle
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
        {
            colorSortEnabled = !colorSortEnabled;
            controller.print(0, 0, "Sort: %s", colorSortEnabled ? "ON " : "OFF");
        }

        // THIS WHOLE IF STATEMENT SHOULD BE COMMENTED OUT IN COMPS
        if (!inCompetition)
        {
            /*bool buttonsPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
            if (buttonsPressed){
                IntakeTask.suspend();
            }*/

            testAuton();
        }
        handleDriveTrain();
        //handleSmallIntake();
        handleStopper();
        handleLoader();
        handleWings();
        handleIntake();
        // handleIntake();

        pros::lcd::print(3, "hue: %f", ballSensor.get_hue());
        pros::lcd::print(4, "prox: %f", ballSensor.get_proximity());
        pros::lcd::print(5, "bright: %f", ballSensor.get_brightness());
        pros::lcd::print(6, "raw: %f", ballSensor.get_raw());
        pros::lcd::print(7, "saturation: %f", ballSensor.get_saturation());

        // delay to save resources
        pros::delay(20);
    }
}