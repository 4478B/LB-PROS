#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/pid.hpp"
#include "pros/adi.h"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include <cstdlib>

using namespace pros;
using namespace lemlib;


// left motor group
MotorGroup left_motors({-8, -9, -10}, pros::MotorGearset::blue);
// right motor group
MotorGroup right_motors({5, 6, 7}, pros::MotorGearset::blue);

MotorGroup arm_motors({-3, 4}, pros::MotorGearset::blue);

// controller definition
Controller controller(pros::E_CONTROLLER_MASTER);

Motor intake(1);

adi::Port clamp('B', pros::E_ADI_DIGITAL_OUT);

PID armPID(1, 0, 2);

Rotation armRot(2);

bool clampState = true;


// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11, // 11 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

pros::Imu imu(20);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
						&throttle_curve // log drive
);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    armRot.reset_position();

	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(7,"Arm Rotation: %f", armRot.get_angle()/100.0); // arm position

            // delay to save resources
            pros::delay(20);
        }
    });
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

void testAngularPID(){

    while(true){
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){

            double x = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
            double y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
            double theta;
            // Handle division by zero case
            if (y == 0) {
            theta = (x >= 0) ? 90.0 : 270.0;
            }

            // Calculate initial angle in radians
            double angleRad = atan2(x, y);  // Using atan2 instead of atan for proper quadrant handling
    
            // Convert to degrees
            double angleDeg = angleRad * 180.0 / M_PI;
    
            // Convert to bearing (clockwise from North)
            // 1. Make angle positive (0 to 360)
            if (angleDeg < 0) {
                angleDeg += 360.0;
            }
    
            // 2. Convert to bearing notation
            theta = angleDeg;

            chassis.turnToHeading(theta, 4000);

            pros::delay(240);
        }
        pros::delay(20);

    }

}

void testLateralPID(){

    double nextMovement = 0;
    while(true){
        int controllerOutput = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        if(abs(controllerOutput) >= 20){ // deadzone of 20 volts

            // 12 in/sec at max
            // y / 10 / 1000msec
            // 127 / 100 / 100msec

            nextMovement += controllerOutput / 100.0;
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            chassis.moveToPoint(0, nextMovement, 4000);
            nextMovement = 0;
        }
        controller.print(1,1, "Dist: %f", nextMovement);
        pros::delay(100);
    }


}





/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
 ASSET(path_jerryio_txt);
void autonomous() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    
	chassis.follow(path_jerryio_txt, 15, 2000);
	

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

    bool clampCD = false;

void handleClamp(){

    // activates on pressing B
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    {
        // toggle clamp states
        clampState = !clampState;

        // clamp or unclamp based on toggled variable
        clamp.set_value(clampState ? HIGH : LOW);

        // print the state of the clamp on the controller screen
        controller.clear_line(1);
        controller.set_text(1,1,clampState ? "Clamped" : "");
    }
    
}

double error = 0;
double targetPos = 0, currentPos = 0;
double nextMovement = 0;
double armThreshold = 1; // threshold for goal to be met
bool armMoving = false;

void handleArm(){


    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
        targetPos = 10; // bottom position; the starting value in code
        armMoving = true;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
    {
        targetPos = 35; // middle position; grabs from intake 
        armMoving = true;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
        targetPos = 140; // top position; scoring on wall stake
        armMoving = true;
    }

    if(armMoving == true){

        // current position in centidegrees, convert to degrees
        currentPos = armRot.get_angle()/100.0;
        
        // normalize error to [-180,180]
        if(currentPos > 180){
            currentPos -= 360;
        }
        else if(currentPos < -180){
            currentPos += 360;
        }


        // calculate how far arm is from target
        error = targetPos - currentPos;

        if(fabs(error) < armThreshold){ // goal has been met

            // reset PID for next usage
            armPID.reset();

            // stop arm motors in place
            arm_motors.brake();

            //stop running the PID code
            armMoving = false;
            
        }
        else // goal has not been met
        {
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
        pros::lcd::print(7,"error: %f", error);
        pros::lcd::print(5, "Arm Next Movement: %f", nextMovement); 

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
void opcontrol() {

	// loop forever
    while (true) {
        
        //testAngularPID();  // overrides other functions for testing purposes
        //testLateralPID();
        handleDriveTrain();
        handleIntake();
        handleClamp();
        handleArm();

        // delay to save resources
        pros::delay(20);
    }
}