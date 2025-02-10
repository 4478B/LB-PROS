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
#include "devices.h"
#include "auton_selector.h"
#include "auton_routes.h"
#include "testing.h"
#include "old_systems.h"
#include "color_sort.h"

// Global variables needed for arm control

namespace ArmPos{

    const double bottom = 0;
    const double mid = 22;
    const double top = 132;
    const double mid_high = 44;
    const double push_top = 152;
    const double alliance = 205;

}
static double targetPos = 0;
static double lastPos = 0;
static bool armMoving = false;
static const double armThreshold = 0.25; // Adjust as needed

void intake_control_task(void* param){
    
    int stuckCount = 0;
    int requiredStuck = 10;
    int lastReset = pros::millis();
    int resetCD = 5000;
    while(true){

        if(abs(intake.get_voltage()) > 0 && intake.get_torque() > 1 && lastReset + resetCD < pros::millis()){
            stuckCount++;
        }
        else{
            stuckCount = 0;
        }

        if(stuckCount > requiredStuck){
            int initVoltage = intake.get_voltage();
            intake.move(-127);
            delay(200);
            if(intake.get_voltage() == -127){
                intake.move(initVoltage);
            }
            intake.move(0);
            lastReset = pros::millis();
            stuckCount = 0;
        }

        delay(100);
        
        //print all variables used
        pros::lcd::print(0, "Intake Voltage: %d", intake.get_voltage());
        pros::lcd::print(1, "Intake Torque: %f", intake.get_torque());
        pros::lcd::print(2, "Stuck Count: %d", stuckCount);
        pros::lcd::print(3, "Last Reset: %d", lastReset);
        pros::lcd::print(4, "Current Time: %d", pros::millis());
        pros::lcd::print(5, "Reset CD: %d", resetCD);

    }

}
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
            currentPos = currentPos - 360 * (currentPos > 240) + 360 * (currentPos < -240);

            // calculate how far arm is from target
            error = targetPos - currentPos;
            if(targetPos == ArmPos::push_top && error >5){
                arm_motors.move(127);
                delay(20);
                continue;
            }
    
            if (fabs(error) < armThreshold)
            { // goal has been met

                goalCount++;
                if (goalCount > 3)
                {
                    // reset PID for next usage
                    armPID.reset();

                    // stop arm motors in place
                    arm_motors.move(0);
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

                // clamps movements to [-600,600]
                nextMovement = std::clamp(nextMovement, -600.0, 600.0);
                
            }

            
            // move arm motors based on PID
            arm_motors.move_velocity(nextMovement);
        }
        else
        {
            arm_motors.brake();
        }

        // collect and print data involving pid on screen
        /*
        pros::lcd::print(6, "Arm State: %s", armMoving ? "Moving" : "Idle");
        pros::lcd::print(3, "Arm Current Pos: %f", currentPos);
        pros::lcd::print(4, "Arm Target Pos: %f", targetPos);
        pros::lcd::print(7, "error: %f", error);
        pros::lcd::print(5, "Arm Next Movement: %f", nextMovement);
        */
        // Add a small delay to prevent the task from hogging CPU
        pros::delay(20);
    }
}
namespace csort {
    bool sortingEnabled = false;
    Hue targetHue = BLUE_RING_HUE;
    int lastRingDetectionTime = 0;

    void color_sort_task(void* param) {
        while (true) {
            if (sortingEnabled) {
                if(isRingDetected(targetHue)){
                    lastRingDetectionTime = pros::millis();
                }
                delay(10);
            } else {
                delay(100);
            }
        }
    }
}

void setArm(int position)
{
    lastPos = targetPos;
    targetPos = position;
    armMoving = true;
}

// aliases for specific positions
void setArmBottom() { setArm(ArmPos::bottom); }
void setArmMid() { setArm(ArmPos::mid); }
void setArmTop() { setArm(ArmPos::top); }
void setArmAlliance() { setArm(ArmPos::alliance); }

// initialize function. Runs on program startup
void initialize()
{

    // controller.clear(); // clear controller screen
    lcd::initialize();   // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    clamp.set_value(HIGH);
    right_doinker.set_value(LOW);

    // initialize_arm_position();
    arm_motors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);

    ringSens.set_led_pwm(100); // Set the LED brightness to maximum for better detection
    ringSens.set_integration_time(10);


    // create arm control task
    Task arm_task(arm_control_task, nullptr, "Arm Control Task");
    // color sort task
    //Task csort_task(csort::color_sort_task, nullptr, "Color Sort Task");
    //Task intake_task(intake_control_task, nullptr, "Intake Control Task");

    pros::lcd::set_text_align(pros::lcd::Text_Align::CENTER);

    // print odometry position to brain screen
    /*
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources

            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
                chassis.setPose(0,0,imu.get_heading());
            }
            pros::delay(20);

            
        }
    });
    */
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

    left_motors.move_velocity(leftY);
    right_motors.move_velocity(rightY);
}
namespace csort {

    int ringTossCounter = 0;
    int detectionTimeout = 0;
    const int sortingDistance = 290;
    double intakeStartPosition;

    void handleIntake() {
        if (!sortingEnabled) {
            // Simple intake control without sorting
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                intake.move(127);
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                intake.move(-127);
            } else {
                intake.brake();
            }
        } else {
            // Manual controls are overridden if color sort mechanism is active
            // Intake
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {

                // If a ring is detected within the sorting distance, continue intake
                if (detectionTimeout > 0 && sortingDistance + intakeStartPosition > intake.get_position()) {
                    intake.move(127);
                    detectionTimeout--;
                    ringTossCounter = 20;
                }
                // If a ring is detected, start the intake and set the start position
                else if (lastRingDetectionTime + 50 > pros::millis()) {
                    intake.move(127);
                    intakeStartPosition = intake.get_position();
                    detectionTimeout = 30;
                } else {
                    // If toss counter is active, brake the intake
                    if (ringTossCounter > 0) {
                        intake.brake();
                        ringTossCounter--;
                    } else {
                        // Continue intake if no ring is detected
                        intake.move(127);
                    }
                }

                // Print debug information to the LCD
                pros::lcd::print(4, "Time since last detection: %d", pros::millis() - lastRingDetectionTime);
                pros::lcd::print(5, "Intake start position: %f", intakeStartPosition);
                pros::lcd::print(6, "Detection timeout: %d", detectionTimeout);
                pros::lcd::print(7, "Ring toss counter: %d", ringTossCounter);
            }
            // Outtake
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                intake.move(-127);
            }
            // No movement without button pressed
            else {
                intake.brake();
            }
        }
    }

    int sortHoldDuration = 0;

    void handleColorSort() {
        // Set the LED brightness to maximum for better detection
        ringSens.set_led_pwm(100);
        ringSens.set_integration_time(10);

        // Toggle the hue to be tossed when X is pressed
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            targetHue = (targetHue.hue == BLUE_RING_HUE.hue) ? RED_RING_HUE : BLUE_RING_HUE;
            // print hue to the brain screen
            lcd::print(2, "Sorting out: %s", targetHue.hue == BLUE_RING_HUE.hue ? "Blue" : "Red");
        }
        // Hold X to enable/disable sorting
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            sortHoldDuration++;
            if (sortHoldDuration > 30) {
                sortingEnabled = !sortingEnabled;
                if (sortingEnabled) {
                    // Enable LED for better detection
                    ringSens.set_led_pwm(100);
                    ringSens.set_integration_time(10);
                } else {
                    // Disable LED to save power
                    //ringSens.set_led_pwm(0);
                    //ringSens.set_integration_time(100);
                    // print sorting status to the brain screen
                    lcd::print(2, "Sorting out: Disabled");
                }
            }
        } else {
            sortHoldDuration = 0;
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
        controller.print(0, 0, clamp.get_value() == HIGH ? "Clamped" : "Uncl         ");
    }
}

void handleLeftDoinker()
{

    // activates on pressing LEFT
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
    {

        left_doinker.set_value(left_doinker.get_value() == LOW ? HIGH : LOW);
    }
}

void handleRightDoinker()
{

    // activates on pressing LEFT
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {

        right_doinker.set_value(right_doinker.get_value() == LOW ? HIGH : LOW);
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
        if(targetPos == ArmPos::mid){
            int initVoltage = intake.get_voltage();
            intake.move(-127);
            delay(30);
            intake.move(initVoltage);
            setArm(ArmPos::mid_high);
        }
        else{
        setArmMid();
        }
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
        if(targetPos == ArmPos::top){
            setArm(ArmPos::push_top);
        }
        else{
            int initVoltage = intake.get_voltage();
            intake.move(-127);
            delay(30);
            intake.move(initVoltage);
            setArmTop();
        }
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
    {
        setArmAlliance();
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

    // loop forever
    while (true)
    {

        // THIS WHOLE IF STATEMENT SHOULD BE COMMENTED OUT IN COMPS
        if (!inCompetition)
        {
            testAuton();
        }
        
        handleDriveTrain();
        csort::handleIntake();
        //csort::handleColorSort();
        handleClamp();
        handleArm();
        handleLeftDoinker();
        handleRightDoinker();

        // delay to save resources
        pros::delay(20);
    }
}