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
    lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    
    // configure motors for arm movement
    armRot.reset_position(); 
    arm_motors.set_brake_mode(E_MOTOR_BRAKE_HOLD);

    // assign buttons to actions in auton selector
	lcd::register_btn1_cb(on_center_button);
    lcd::register_btn0_cb(on_center_button);


    pros::lcd::set_text_align(pros::lcd::Text_Align::CENTER);
	
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

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
 ASSET(test4_txt);
void autonomous() {
    
    
    
    // SKILLS ROUTE 1 (from skills_aio.txt)
    
    // set position to starting spot (needs tweaking)
    chassis.setPose(-58.7, -14.3, 315);
    // back up to hit goal1
    chassis.moveToPose(-51, -22, 315,1000, {.forwards=false},false);
    // clamp goal1 & intake (preload)
    clamp.set_value(HIGH);
    intake.move(127);
    // grab ring1 for goal1
    chassis.moveToPose(-24,-24,170,2000,{.minSpeed=72, .earlyExitRange=4});
    //grab ring2 for goal1
    chassis.moveToPose(-24,-48,270,2000,{.minSpeed=72, .earlyExitRange=4},false);
    //grab ring3 for goal1
    chassis.moveToPose(-48,-48,200,2000,{},false);
    //grab ring4 for goal1
    chassis.moveToPose(-48,-59,180,2000,{},false);
    //grab ring5 for goal1
    chassis.moveToPoint(-59,-48,2000,{},false);
    //small wait so ring5 intakes
    delay(400);
    //back goal1 into corner
    chassis.moveToPose(-63, -63, 45, 2000, {.forwards=false},false);
    //stop intake
    intake.brake();
    //unclamp goal1
    clamp.set_value(LOW);
    

    //approach goal2
    chassis.moveToPose(-48,5,180,2000,{},false);
    //back into goal2
    chassis.moveToPose(-48,24,180,2000,{},false);
    //clamp goal2 & intake
    clamp.set_value(HIGH);
    intake.move(127);
    //grab ring1 for goal2
    chassis.moveToPose(-24,24,20,2000,{.minSpeed=72, .earlyExitRange=4},false);
    //grab ring2 for goal2
    chassis.moveToPose(-24,48,290,2000,{.minSpeed=72, .earlyExitRange=4},false);
    //grab ring3 for goal2
    chassis.moveToPose(-48,48,200,2000,{},false);
    //grab ring4 for goal2
    chassis.moveToPose(-48,59,180,2000,{},false);
    //grab ring5 for goal2
    chassis.moveToPoint(-59,48,2000,{},false);
    //small wait so ring5 intakes
    delay(400);
    //back goal2 into corner
    chassis.moveToPose(-63, 63, 135, 2000, {.forwards=false},false);
    //stop intake
    intake.brake();
    //unclamp goal2
    clamp.set_value(LOW);

    
    
    /*controller.set_text(1,1,"Started I <3 Mikey");
	chassis.follow(test4_txt, 20, 20000,true,false);
    intake.move(127);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.cancelAllMotions();
    controller.clear_line(1);
    controller.set_text(1,1,"Stopped");*/
    std::cout<<"Done";
	while(true){
        delay(20);
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

double error = 0;
double targetPos = 0, currentPos = 0;
double nextMovement = 0;
double armThreshold = 2; // threshold for goal to be met
bool armMoving = false;

void handleArm(){


    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
    {
        targetPos = 0; // bottom position; the starting value in code
        armMoving = true;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
    {
        targetPos = 27; // middle position; grabs from intake 
        armMoving = true;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
        targetPos = 130; // top position; scoring on wall stake
        armMoving = true;
    }

    if(armMoving == true){

        // current position in centidegrees, convert to degrees
        currentPos = armRot.get_angle()/100.0;
        
        // normalize error to [-180,180]
        currentPos = currentPos - 360 * (currentPos > 180)
                                + 360 * (currentPos < -180);


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