#include "main.h"
#include "extended_chassis.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/pid.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/adi.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include <cstdlib>
#include "devices.h"
#include "auton_selector.h"
#include "auton_routes.h"
#include "testing.h"
#include "old_systems.h"

// Global variables needed for arm control
static double targetPos = 0;
static bool armMoving = false;
static const double armThreshold = 0.25; // Adjust as needed

// Task function for arm control
void arm_control_task(void *param)
{
    double currentPos;
    double error;
    double nextMovement;
    int goalCount = 0;

    while (true)
    {
        if (armMoving)
        {
            // current position in centidegrees, convert to degrees
            currentPos = armRot.get_angle() / 100.0;

            // normalize error to [-180,180]
            currentPos = currentPos - 360 * (currentPos > 180) + 360 * (currentPos < -180);

            // calculate how far arm is from target
            error = targetPos - currentPos;

            if (fabs(error) < armThreshold)
            { // goal has been met

                goalCount++;
                if (goalCount > 3)
                {
                    // reset PID for next usage
                    armPID.reset();

                    // stop arm motors in place
                    arm_motors.brake();

                    // stop running the PID code
                    armMoving = false;
                    goalCount = 0;
                }
            }
            else
            { // goal has not been met
                goalCount = 0;
                // determine how far to move based on PID
                nextMovement = armPID.update(error);

                // ensure values fit bounds of motor voltage
                nextMovement = std::clamp(nextMovement, -600.0, 600.0);

                // move arm motors based on PID
                arm_motors.move_velocity(nextMovement);
            }

            // collect and print data involving pid on screen

            pros::lcd::print(6, "Arm State: %s", armMoving ? "Moving" : "Idle");
            pros::lcd::print(3, "Arm Current Pos: %f", currentPos);
            pros::lcd::print(4, "Arm Target Pos: %f", targetPos);
            pros::lcd::print(7, "error: %f", error);
            pros::lcd::print(5, "Arm Next Movement: %f", nextMovement);
        }

        // Add a small delay to prevent the task from hogging CPU
        pros::delay(20);
    }
}

void setArm(int position)
{
    // Validate input
    if (position < 1 || position > 3)
    {
        return; // Invalid position
    }

    if (position == 1)
    {
        targetPos = 0; // Bottom position
    }
    else if (position == 2)
    {
        targetPos = 33; // Middle position
    }
    else if (position == 3)
    {
        targetPos = 134; // Top position
    }
    armMoving = true;
}

// aliases for specific positions
void setArmBottom() { setArm(1); }
void setArmMid() { setArm(2); }
void setArmTop() { setArm(3); }

void initialize_arm_position()
{
    // Move arm down at moderate speed but low power
    arm_motors.set_voltage_limit(4000); // Limit to 4V for gentle movement
    arm_motors.move_velocity(-50);      // Move down at moderate speed
    arm_motors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);

    // Wait until arm stalls (high current, low velocity)
    while (true)
    {
        // Get current draw and velocity
        double velocity = arm_motors.get_actual_velocity();
        int current = arm_motors.get_current_draw();

        // If we detect high current (stall) and low velocity, we've hit bottom
        if (current > 1500 && std::abs(velocity) < 5)
        {
            arm_motors.brake();
            arm_motors.set_voltage_limit(12000); // Reset to full voltage
            armRot.reset_position();
            break;
        }

        pros::delay(20); // Small delay to prevent hogging CPU
    }
}

bool isRedAlliance = false;

void on_center_button()
{ // swaps team for color sort

    /*isRedAlliance = !isRedAlliance;

    lcd::clear_line(1);
    lcd::print(1, "Team: %s", isRedAlliance ? "Red Team" : "Blue Team");*/
}

// this function is called during the color_sort_task
// it is the actions that are taken to sort out a ring
void tossRing()
{

    intake.move(-127);
    delay(1000);
}

bool currentlySorting = false;

void color_sort_task(void *param)
{
    int hueMin, hueMax; // these are endpoints for acceptable ring colors
    if (isRedAlliance)
    {
        hueMin = 330; // min < max b/c it loops around 360 degrees
        hueMax = 45;
    }
    else
    {
        hueMin = 0; // placeholder values
        hueMax = 0;
    }
    // detected ring is out of bounds of acceptable color range
    if ((colorSens.get_hue() < hueMin || colorSens.get_hue() > hueMax) && colorSens.get_proximity() < 20)
    {
        currentlySorting = true;
        tossRing();
        currentlySorting = false;
    }
    delay(20);
}

// initialize function. Runs on program startup
void initialize()
{

    lcd::initialize();   // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    clamp.set_value(HIGH);

    // initialize_arm_position();
    arm_motors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);

    // create arm control task
    Task arm_task(arm_control_task, nullptr, "Arm Control Task");

    // create color sort task
    // Task csort_task(color_sort_task, nullptr, "Color Sort Task");

    // set optical sensor for color sort
    // colorSens.set_led_pwm(100);

    pros::lcd::set_text_align(pros::lcd::Text_Align::CENTER);

    // print odometry position to brain screen
    /*pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

            // delay to save resources
            pros::delay(20);
        }
    });*/
}

void autonomous()
{
    left_motors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
    right_motors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
    getAutonSelector().runSelectedAuton();
    left_motors.brake();
    right_motors.brake();
    delay(2000);
    left_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
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
    getAutonSelector().displayCurrentSelection();

    // run buttons once to print values on screen
    on_left_button();
    on_center_button();
    on_right_button();

    // assign buttons to actions in auton selector
    lcd::register_btn0_cb(on_left_button);
    lcd::register_btn1_cb(on_center_button);
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

    left_motors.move_velocity(leftY);
    right_motors.move_velocity(rightY);
}

void handleIntake()
{

    // manual controls are overridden if color sort mechanism is active
    if (!currentlySorting)
    {

        // intake
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            intake.move(127);
        }
        // outtake
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            intake.move(-127);
        }
        // no movement without button pressed
        else
        {
            intake.brake();
        }
    }
}

void handleClamp()
{

    // activates on pressing B
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    {

        // clamp or unclamp based on toggled variable

        clamp.set_value(clamp.get_value() == LOW ? HIGH : LOW);

        // print the state of the clamp on the controller screen
        controller.clear_line(1);
        controller.print(1, 1, clamp.get_value() == LOW ? "Clamped" : "");
    }
}

void handleDoinky()
{

    // activates on pressing B
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
    {

        // clamp or unclamp based on toggled variable

        doinker.set_value(doinker.get_value() == LOW ? HIGH : LOW);

        // print the state of the clamp on the controller screen
        // controller.clear_line(1);
        // controller.print(1,1,.get_value() == LOW ? "Clamped" : "");
    }
}

void handleArm()
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
    {
        setArmBottom();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
    {
        setArmMid();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
        setArmTop();
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

    // loop forever
    while (true)
    {

        // THIS WHOLE IF STATEMENT SHOULD BE COMMENTED OUT IN COMPS
        if (!inCompetition)
        {
            testAuton();
        }
        handleDriveTrain();
        handleIntake();
        handleClamp();
        handleArm();
        handleDoinky();

        // delay to save resources
        pros::delay(20);
    }
}