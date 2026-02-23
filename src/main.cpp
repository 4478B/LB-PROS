/**
 * @file main.cpp
 * @brief Main entry point for the VEX robot program (PROS + LemLib framework).
 *
 * This file contains the four PROS lifecycle callbacks:
 *   - initialize()        : runs once on startup before anything else
 *   - competition_initialize() : runs before autonomous when connected to a competition switch
 *   - autonomous()        : runs during the 15-second autonomous period
 *   - opcontrol()         : runs during the 1m45s driver-control period
 *
 * It also defines all the per-subsystem driver-control handler functions
 * (drivetrain, intake, lift, wings, etc.) that are called every loop in opcontrol().
 *
 * Framework:
 *   PROS  – https://pros.cs.purdue.edu/  (real-time OS for VEX V5)
 *   LemLib – https://lemlib.readthedocs.io/ (motion planning / odometry)
 */

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
#include "opticalAlign.h"
#include "old_systems.h"

// initialize function. Runs on program startup
void initialize()
{
    deScores.set_value(LOW);
    lift.set_value(LOW);
    //stopper.set_value(HIGH);
    // controller.clear(); // clear controller screen
    lcd::initialize();   // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    pros::lcd::set_text_align(pros::lcd::Text_Align::CENTER);

    // Screen task to display distance sensor values and pose info
    /*pros::Task screen_task([] {
        while (true) {
            pros::lcd::clear_line(0);
            pros::lcd::print(0, "Dist Size: %d Dist: %.0fmm", backDistance.get_object_size(), backDistance.get());
            pros::lcd::clear_line(1);
            pros::lcd::print(1, "X: %.1f Y: %.1f", chassis.getPose().x, chassis.getPose().y);
            pros::lcd::clear_line(2);
            pros::lcd::print(2, "Theta: %.1f", chassis.getPose().theta);
            
            pros::delay(100);
        }
    });*/
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

// Safety flag: set to true when connected to a competition/field switch.
// testAuton() checks this and skips its manual trigger logic during a real match.
bool inCompetition = false;

// Alliance color for color-sorting; true = Red alliance, false = Blue alliance.
// Toggled by pressing the center brain screen button during competition_initialize().
bool red = false;

// Whether the intake should automatically reject balls of the opposing alliance color.
// Toggled with the UP button on the controller during opcontrol.
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

// --- Joystick Expo Curve Constants ---
// Joystick values are passed through a power curve (x^3 / DENOMINATOR) so that
// slow movements are more precise while full-stick input still reaches 100% power.
const double SMOOTHING_DENOMINATOR = 10000; // Used to normalize the exponential curve
const double EXPONENTIAL_POWER = 3;       // Controls how aggressive the curve is

// Helper function that makes joystick input more precise for small movements
// while maintaining full power at maximum joystick deflection.
// Input range: -100 to 100 (percent). Output range: -100 to 100 (percent).
double logDriveJoystick(double joystickPCT)
{
    // Get the absolute value for calculation
    double magnitude = fabs(joystickPCT);

    // Calculate the smoothed value
    double smoothedValue = pow(magnitude, EXPONENTIAL_POWER) / SMOOTHING_DENOMINATOR;

    // Restore the original sign (positive or negative)
    return joystickPCT >= 0 ? smoothedValue : -smoothedValue;
}

/**
 * Tank-drive handler called every opcontrol loop iteration.
 * Reads left/right joystick Y-axes, applies the expo curve, scales
 * the result to motor velocity units (±600 RPM for 600 RPM "blue" cartridge),
 * and commands the drivetrain motor groups.
 */
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
/**
 * Toggles the ball loader (match-loader gate) pneumatic on each press of RIGHT.
 * The loader is a pneumatic piston that controls the ball-intake gate at the field wall.
 */
void handleLoader()
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
        loader.set_value(!loader.get_value());
    }
}

/**
 * Controls the de-scoring wings (side pneumatics that push game objects off goals).
 * Held HIGH while L2 is depressed; returns LOW when released.
 */
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
        //frontGate.set_value(HIGH);
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
        //frontGate.set_value(LOW);

    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
        intake.move(-127);
        intakeTop.move(-127);
        smallIntake.move(127);
        //frontGate.set_value(LOW);

    }
    else
    {
        intake.brake();
        intakeTop.brake();
        //backGate.set_value(LOW);
        smallIntake.brake();
        //frontGate.set_value(LOW);

    }
    
    
}

/**
 * Current active intake handler (replaces handleIntake).
 *
 * Button mapping:
 *   R1 + R2  – Reverse bottom intake, run top intake backwards (eject)
 *   R1 + L1  – Run both intake stages forward (full intake)
 *   R1 only  – Run bottom intake forward, top at low power (feed into scorer)
 *   R2 only  – Run both intake stages in reverse (spit out)
 *   none     – Brake all intake motors, raise front gate, lower stopperTwo
 *
 * Note: intakeTop runs at low power (10) during R1-only to avoid jamming.
 */
void handleIntakeNew(){
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
        intake.move(127);
        intakeTop.move(-127);
        //smallIntake.move(127);
        //stopperTwo.set_value(HIGH);
        //frontGate.set_value(LOW);
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        if(lift.get_value() == HIGH){
            intake.move(127);
            intakeTop.move(127);
        }
        else{
            intake.move(127);
            intakeTop.move(127);
        }      
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    { 
       
       intake.move(127);
       intakeTop.move(10);
        //frontGate.set_value(LOW);
    }
     else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
        intake.move(-127);
        intakeTop.move(-127);
        //frontGate.set_value(LOW);

    }
     else
    {
        intake.brake();
        //intakeTop.brake();

        //backGate.set_value(LOW);
        smallIntake.brake();
        frontGate.set_value(HIGH);
        stopperTwo.set_value(LOW);
        intakeTop.brake();

    }
}

/** Toggles the front gate pneumatic on each press of DOWN. */
void handlefrontGate()
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
        frontGate.set_value(!frontGate.get_value());
    }
}

/** Toggles the lift pneumatic (raises/lowers ball-scoring lift) on each press of DOWN. */
void handleLift(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
        lift.set_value(!lift.get_value());
    }
}

/**
 * Controls the ball stopper pneumatic.
 * The stopper blocks the ball path so balls don't fall back out of the intake.
 * Held HIGH while L1 is depressed; returns LOW when released.
 */
void handleStopper()
{

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
        stopper.set_value(HIGH);
        //intakeTop.move(127);
    }
    else{
        stopper.set_value(LOW);
        

       //stopperTwo.set_value(HIGH);
    }


}

void multiTake()
{
    while (true)
    {
        handleIntake();
    }
}
void outputHeading(){
    while (true)
    {
        std::cout << "Heading: " << imu1.get_heading() << std::endl;
        std::cout << "Heading: " << imu2.get_heading() << std::endl;
        std::cout << " " << std::endl;
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
    //lift.set_value(LOW);
    chassis.setPose(0, 0, 0);
    // backGate.set_value(LOW);
    // bool buttonsPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    /*    pros::Task IntakeTask([]
                         { outputHeading();});*/
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
        /*
        if (controller.get_digital(E_CONTROLLER_DIGITAL_B))
        {
            alignToLongGoal(93,false);
        }  */
        handleDriveTrain();
        //handleSmallIntake();
        handleStopper();
        handleLift();
        handleLoader();
        handleWings();
        handleIntakeNew();
        handlefrontGate();
        // handleIntake();

        pros::lcd::print(3, "imu: %f", imu.get_roll());
        pros::lcd::print(4, "prox: %f", ballSensor.get_proximity());
        pros::lcd::print(5, "bright: %f", ballSensor.get_brightness());
        pros::lcd::print(6, "raw: %f", ballSensor.get_raw());
        pros::lcd::print(7, "saturation: %f", ballSensor.get_saturation());

        // delay to save resources
        pros::delay(20);
    }
}