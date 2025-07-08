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


void intake_control_task(void *param)
{

    int stuckCount = 0;
    int requiredStuck = 10;
    int lastReset = pros::millis();
    int resetCD = 5000;
    while (true)
    {

        if (abs(intake.get_voltage()) > 0 && intake.get_torque() > 1 && lastReset + resetCD < pros::millis())
        {
            stuckCount++;
        }
        else
        {
            stuckCount = 0;
        }

        if (stuckCount > requiredStuck)
        {
            int initVoltage = intake.get_voltage();
            intake.move(-127);
            delay(200);
            if (intake.get_voltage() == -127)
            {
                intake.move(initVoltage);
            }
            intake.move(0);
            lastReset = pros::millis();
            stuckCount = 0;
        }

        delay(100);

        // print all variables used
        pros::lcd::print(0, "Intake Voltage: %d", intake.get_voltage());
        pros::lcd::print(1, "Intake Torque: %f", intake.get_torque());
        pros::lcd::print(2, "Stuck Count: %d", stuckCount);
        pros::lcd::print(3, "Last Reset: %d", lastReset);
        pros::lcd::print(4, "Current Time: %d", pros::millis());
        pros::lcd::print(5, "Reset CD: %d", resetCD);
    }
}
// Task function for arm control


// initialize function. Runs on program startup
void initialize()
{

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

void handleClamp()
{

    // activates on pressing B
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    {

        // clamp or unclamp based on toggled variable

        clamp.set_value(clamp.get_value() == LOW ? HIGH : LOW);

        // print the state of the clamp on the controller screen
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
    arm_motors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
    intakeOverride = false;
    armOverride = false;
    ringSens.set_led_pwm(100);
    intake.move(127);
    //pros::Task ColorSortTask([]
    //                         { handleColorSortTwo(0); });
    //csort::color_sort_task(nullptr);
    
    // loop forever
    while (true)
    {

        // THIS WHOLE IF STATEMENT SHOULD BE COMMENTED OUT IN COMPS
        if (!inCompetition)
        {
            testAuton();
        }
        //ringSens.set_led_pwm(100);
        //ringSens.set_integration_time(10);
        handleDriveTrain();
        csort::handleIntake();
        //handleColorSortTwo(0);
        //csort::handleColorSort();
        handleClamp();
        handleArm();
        handleLeftDoinker();
        handleRightDoinker();
        //handleAllianceMacro();
        //handleIntakeLift();
        //handleCornerMacro();
        // handleHangMacro();

        // print arm motor voltage and efficiency to brain
        //pros::lcd::print(1, "Arm Motor Voltage: %i", arm_motors.get_voltage());
        //pros::lcd::print(2, "Arm Motor Efficiency: %f", arm_motors.get_efficiency());
        //pros::lcd::print(3, "Arm Target Velocity %f", arm_motors.get_target_velocity());

        // delay to save resources
        pros::delay(20);
    }
    
}