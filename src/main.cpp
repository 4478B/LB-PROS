#include "main.h"
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


// Global variables needed for arm control
static double targetPos = 0;
static bool armMoving = false;
static const double armThreshold = 1.0; // Adjust as needed

// Task function for arm control
void arm_control_task(void* param) {
    double currentPos;
    double error;
    double nextMovement;
    
    while (true) {
        if (armMoving) {
            // current position in centidegrees, convert to degrees
            currentPos = armRot.get_angle()/100.0;
            
            // normalize error to [-180,180]
            currentPos = currentPos - 360 * (currentPos > 180)
                                  + 360 * (currentPos < -180);

            // calculate how far arm is from target
            error = targetPos - currentPos;

            if (fabs(error) < armThreshold) { // goal has been met
                // reset PID for next usage
                armPID.reset();

                // stop arm motors in place
                arm_motors.brake();

                //stop running the PID code
                armMoving = false;
            }
            else { // goal has not been met
                // determine how far to move based on PID
                nextMovement = armPID.update(error);

                //ensure values fit bounds of motor voltage
                nextMovement = std::clamp(nextMovement,-127.0,127.0);

                // move arm motors based on PID
                arm_motors.move(nextMovement);
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

// initialize function. Runs on program startup
void initialize() {

    lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    
    // configure motors for arm movement
    armRot.reset_position(); 
    arm_motors.set_brake_mode(E_MOTOR_BRAKE_HOLD);

    // show current route on brain screen
    getAutonSelector().displayCurrentSelection();

    // assign buttons to actions in auton selector
    lcd::register_btn0_cb(on_left_button);
	lcd::register_btn2_cb(on_right_button);

    // create arm control task
    Task arm_task(arm_control_task, nullptr, "Arm Control Task");

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

void autonomous() {
    getAutonSelector().runSelectedAuton();
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

void testCombinedPID() {
    double nextMovement = 0;
    
    while(true) {
        // Lateral PID accumulation (Left Joystick)
        int controllerOutput = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        if(abs(controllerOutput) >= 20) { // deadzone of 20 volts
            nextMovement += controllerOutput / 100.0;
        }
        
        // Trigger lateral movement with UP button
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            chassis.moveToPoint(0, nextMovement, 4000);
            nextMovement = 0;
        }
        
        // Angular PID triggering with X button (Right Joystick)
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            double x = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            double y = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
            
            // Calculate absolute angle (0-360 degrees)
            double theta;
            if (fabs(x) < 20 && fabs(y) < 20) { // deadzone check
                continue; // skip if joystick is in deadzone
            }
            
            if (y == 0) {
                theta = (x >= 0) ? 90.0 : 270.0;
            } else {
                // Calculate absolute angle in degrees
                theta = atan2(x, y) * 180.0 / M_PI;
                
                // Convert to 0-360 range
                if (theta < 0) {
                    theta += 360.0;
                }
            }
            
            // Turn to absolute heading
            chassis.turnToHeading(theta, 4000);  // Assuming turnToAngle uses absolute angles
        }
        
        // Display information
        controller.print(1, 1, "Dist: %f", nextMovement);
        
        double rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        double rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        double currentAngle = atan2(rightX, rightY) * 180.0 / M_PI;
        if (currentAngle < 0) currentAngle += 360.0;
        controller.print(2, 1, "Target Angle: %f", currentAngle);
        
        pros::delay(100);
    }
}

void handleDriveTrain(){

	// get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

}

void handleIntake(){

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
        intake.move(127);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
        intake.move(-127);
    }
    else {
        intake.brake();
    }

}


void handleClamp(){

    // activates on pressing B
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    {

        // clamp or unclamp based on toggled variable
        
        clamp.set_value(clamp.get_value() == LOW ? HIGH : LOW);

        // print the state of the clamp on the controller screen
        controller.clear_line(1);
        controller.set_text(1,1,clamp.get_value() == LOW ? "Clamped" : "");
    }
    
}

void handleArm() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        targetPos = 0; // bottom position; the starting value in code
        armMoving = true;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        targetPos = 27; // middle position; grabs from intake 
        armMoving = true;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        targetPos = 130; // top position; scoring on wall stake
        armMoving = true;
    }
}

void handleColorSort(){

    

}

/*int countPoints = 1;

void handleSavePoint(){
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
        std::cout<<"Point " + countPoints + ": " + chassis.getPose();

    }

}*/









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
void opcontrol() {

	// loop forever
    while (true) {
        
        //testCombinedPID();
        handleDriveTrain();
        handleIntake();
        handleClamp();
        handleArm();

        // delay to save resources
        pros::delay(20);
    }
}