#include "auton_routes.h"
#include "lemlib/chassis/chassis.hpp"
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
#include "old_systems.h"
#include "pros/rtos.h"
#include "testing.h"
#include <iomanip>
#include "color_sort.h"

// These our functions made for backwards-compatibility with VEXCode routes
 
void drivePIDOdom(double goalInches, bool clamping, double clampDistInches)
{
    // Step 1: Get the current robot pose from odometry.
    Pose poseInit(chassis.getPose(true));

    // Step 2: Calculate goal coordinates in global space based on current pose and movement distance.
    // Convert the robot's heading angle from bearing notation to unit circle angles.
    double unitCircleAngle = (M_PI / 2 - poseInit.theta);

    // Compute the target position using trigonometry.
    float goalX = poseInit.x + goalInches * cos(unitCircleAngle);
    float goalY = poseInit.y + goalInches * sin(unitCircleAngle);

    // Step 3: Create a goal pose with the same heading as the current pose.
    Pose poseGoal(goalX, goalY, poseInit.theta);

    // Step 4: Determine whether the movement is forward or backward.
    bool isForwards = goalInches > 0;

    // Step 5: Move to the calculated point, either clamped or unclamped.
    if (clamping)
    {
        // Move to point with clamping to prevent overshoot.
        chassis.MoveToPointClamp(poseGoal.x, poseGoal.y, 4000, clampDistInches, {.forwards = isForwards});
    }
    else
    {
        // Move to point without clamping.
        chassis.moveToPoint(poseGoal.x, poseGoal.y, 4000, {.forwards = isForwards});
    }

    // Step 6: Print debug information for testing pose calculations.
    // Output trimmed to 3 decimal places to fit the screen.
    pros::lcd::print(3, "Pose Init: X: %.3f, Y: %.3f, Th: %.3f", poseInit.x, poseInit.y, poseInit.theta);
    pros::lcd::print(4, "Pose Goal: X: %.3f, Y: %.3f, Th: %.3f", poseGoal.x, poseGoal.y, poseGoal.theta);
    pros::lcd::print(5, "Unit Angle: %.3f", unitCircleAngle);
}

void driveInchesClamp(double gDist, double cDist = .5)
{
    drivePID(gDist, true, cDist);
}

/*void chassis.turnToHeading(float theta)
{
    chassis.turnToHeading(theta, 2000);
}*/

// This method is designed for testing sections of autons separately

// IF CALLED IN COMPETITION/WITH COMM SWITCH:
// -- functions as regular delay
// IF CALLED NOT IN COMPETITION & WITHOUT COMM SWITCH:
// -- delays until user presses X or it times out
// -- prints section information to controller screen

// in the future we can make it print information about ending positions

/*******************************************************
 *              Section Based Functions                *
 *******************************************************/

// Global variables for auton section tracking
int autonSection = 0;

// Function to print times and pose information
void printTimes(int section, int deltaTime, int totalTime, Pose pose) {
    std::cout << std::setw(10) << section << " | "
              << std::setw(10) << deltaTime << " | "
              << std::setw(10) << totalTime << " | "
              << std::setw(10) << pose.x << " | "
              << std::setw(10) << pose.y << " | "
              << std::setw(10) << pose.theta << " | "
              << std::endl;
}

// Function to handle end of section logic
bool endSection(int delay) {
    // Functions as normal delay during competition
    if (inCompetition) {
        pros::delay(delay);
        return false;
    } else {
        bool altPath = false;
        // Handle updating timers
        int startTime = pros::millis();
        int deltaTime = startTime - prevTime;
        totalTime += deltaTime;
        prevTime = startTime;

        // Print timer positions to console for permanent logging
        Pose poseInit = chassis.getPose();
        printTimes(autonSection, deltaTime, totalTime, poseInit);

        // Print timer positions on screen for temporary logging
        pros::lcd::print(5, "Auton Section: %i", autonSection);
        pros::lcd::print(6, "Section Time: %i", deltaTime);
        pros::lcd::print(7, "Total Time: %i", totalTime);

        // While button hasn't been pressed and hasn't timed out
        while (pros::millis() - startTime < delay) {
            // Timeout override: Break if X button is pressed
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
                break;
            }
            // Alternate path override: Return alternate route indicator if A button is pressed
            else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
                altPath = true;
                break;
            }
            // Heading override: Adjust heading if Y button is pressed
            else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
                pros::lcd::print(1, "Old Heading: %f", chassis.getPose().theta);
                while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
                    pros::lcd::print(2, "New Heading: %f", chassis.getPose().theta);
                    pros::delay(20);
                }
                // Store corrected pose
                Pose poseCorrected = Pose(poseInit.x, poseInit.y, chassis.getPose().theta);
                printTimes(autonSection, 0, 0, poseCorrected);
                break;
            }
            // Position override: Adjust position if B button is pressed
            else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                float initHeading = poseInit.theta;
                while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                    pros::delay(20);
                }
                // Turn back to initial heading
                chassis.turnToHeading(initHeading, 2000);
                // Store corrected pose
                Pose poseCorrected = Pose(chassis.getPose().x, chassis.getPose().y, poseInit.theta);
                printTimes(autonSection, 0, 0, poseCorrected);
                break;
            }
            pros::delay(20);
        }
        // Updates controller screen with section information
        autonSection++;

        // Returns false if no alternate path is taken
        return altPath;
    }
}

// This file includes all of the routes coded in PROS for our robot
// The routes should have linked path.jerryio files for reference
void soloPushRight(int i){
    ringSens.set_led_pwm(100);
    clamp.set_value(HIGH);
    chassis.setPose(0,0,-212);
    setArmAlliance();
    delay(500);
    drivePID(-6,600,120);
    setArmBottom();
    chassis.turnToHeading(-32,800,{}, false);

    intake.move(60);
    drivePID(24,700,120);
    waitUntilAnyIntake(700);
    intake.brake();
    chassis.turnToHeading(60,600,{}, false);
    drivePID(-27,800,35);
    clamp.set_value(LOW);
    chassis.turnToHeading(-2,600,{}, false);
    intake.move(127);
    drivePID(26,800,130);
    chassis.turnToHeading(151,700,{}, false);
    clamp.set_value(HIGH);
    drivePID(59,1500,60);
    setArm(70);
    intake.move(80);
    drivePID(29,800,120);
    intake.move(40);
    waitUntilAnyIntake(500);
    intake.brake();
    drivePID(-24,800,120);
    chassis.turnToHeading(60,700,{}, false);
    drivePID(-42,1000,35);
    clamp.set_value(LOW);
    intake.move(127);
    chassis.turnToHeading(160,700,{}, false);
    drivePID(26,800,130);
    all_motors.set_brake_mode(E_MOTOR_BRAKE_COAST);
    drivePID(-60,800,120);
    intake.brake();
}

void soloPushLeft(int i){
    ringSens.set_led_pwm(100);
    clamp.set_value(HIGH);
    chassis.setPose(0,0,212);
    setArmAlliance();
    delay(500);
    drivePID(-6,600,120);
    setArmBottom();
    chassis.turnToHeading(32,800,{}, false);

    intake.move(60);
    drivePID(24,700,120);
    waitUntilAnyIntake(700);
    intake.brake();
    chassis.turnToHeading(-60,600,{}, false);
    drivePID(-27,800,35);
    clamp.set_value(LOW);
    chassis.turnToHeading(2,600,{}, false);
    intake.move(127);
    drivePID(26,800,130);
    chassis.turnToHeading(-151,700,{}, false);
    clamp.set_value(HIGH);
    drivePID(59,1500,60);
    setArm(70);
    intake.move(80);
    drivePID(29,800,120);
    intake.move(40);
    waitUntilAnyIntake(500);
    intake.brake();
    drivePID(-24,800,120);
    chassis.turnToHeading(-60,700,{}, false);
    drivePID(-42,1000,35);
    clamp.set_value(LOW);
    intake.move(127);
    chassis.turnToHeading(-160,700,{}, false);
    drivePID(26,800,130);
    drivePID(-55,800,120);
    intake.brake();
}
void ladyBrownRushRight(int i){
    /*
    ringSens.set_led_pwm(100);
    clamp.set_value(HIGH);
    chassis.setPose(0,0,291);
    setArm(60);
    intake.move(127);
    //waitUntilRedIntake();
    left_doinker.set_value(HIGH);
    drivePID(36,1000);
    intake.brake();
   drivePID(-16,800,30);
   left_doinker.set_value(LOW);
   chassis.turnToHeading(273,400,{},false);
   drivePID(6,600);
   setArm(220);
 delay(350);
   chassis.turnToHeading(10,650,{},false);
   setArm(80);
drivePID(-22,1000);
clamp.set_value(LOW);
chassis.turnToHeading(50,700,{},false);
intake.move(127);
left_doinker.set_value(HIGH);
drivePID(20,700);
intake.move(-40);
drivePID(50,1300,50);
drivePID(-10,600,50);
chassis.turnToHeading(195,700,{},false);
intake.move(127);
left_doinker.set_value(LOW);
chassis.turnToHeading(180,250,{},false);
drivePID(20,1000);*/
 chassis.setPose(0,0,0);
 right_doinker.set_value(HIGH);
 chassis.moveToPose(5,35,330,2000,{},false);
}
void newRingSideRight(int i){

    //setup
    ringSens.set_led_pwm(100);
    clamp.set_value(HIGH);
    ringSens.set_led_pwm(100);
    chassis.setPose(0,0,293);
    ringSens.set_led_pwm(100);
    intake.move(127);
    ringSens.set_led_pwm(100);
    //waitUntilRedIntake();
    right_doinker.set_value(HIGH);
    ringSens.set_led_pwm(100);

    // get to ring stack mid
    drivePID(45,1100);
   
    
   // slow intake and grab ring, back up with rings
    intake.move(40);
   drivePID(-27,950,66);
   intake.brake();
   right_doinker.set_value(LOW);
   
   
   // turn to face mogo and grab it
   chassis.turnToHeading(47,600,{}, false);

   drivePID(-25,900,40);
   clamp.set_value(LOW);
   delay(100);

  
   // intake ring in intake on goal

   //face line of rings
   chassis.turnToHeading(16,650,{}, false);

   // score line of rings
   intake.move(127);
   drivePID(38,900,35);
   chassis.turnToHeading(159,800,{}, false);
   drivePID(60,1300,25);
   clamp.set_value(HIGH);
   delay(500);
   intake.move(70);
   drivePID(35,800,40);
    waitUntilBlueIntake(500);
    intake.brake();
    setArmMid();
    drivePID(-48,800,40);
    clamp.set_value(LOW);
    drivePID(21,700);
    delay(100);
    intake.move(127);
    left_doinker.set_value(HIGH);
    chassis.turnToHeading(90,900,{}, false);
    intake.brake();
    intake.move(-10);
    left_doinker.set_value(LOW);
    drivePID(26,800,40);
    delay(200);
    drivePID(-9.5,600,160);
    setArmAlliance();
    delay(600);
    drivePID(-10,600);
    setArmBottom();
    chassis.turnToHeading(270,800,{}, false);
    drivePID(22,1000,30);
   /*
   // face corner
   chassis.turnToHeading(125,600,{}, false);

   // go to corner
   drivePID(36,1300);
   
   
   // doink corner
   chassis.turnToHeading(18,650,{}, false);
   right_doinker.set_value(HIGH);
   drivePID(28,1550);
   drivePID(-1.5,500,160);

   // face 
   chassis.turnToHeading(-10,800,{}, false);
   
 
   
   drivePID(-20,1000);
   chassis.turnToHeading(-175,1000,{}, false);
   chassis.turnToHeading(-160,400,{}, false);
   right_doinker.set_value(LOW);

   drivePID(32,1500,120);
   intake.move(80);
   drivePID(25,1000,40);
   waitUntilAnyIntake(700);
   intake.brake();
   drivePID(-25,600);
   setArmMid();
   intake.move(127);
   chassis.turnToHeading(90,1000,{}, false);
   setArmAlliance();*/
}
void newRingSideLeft(int i){
    ringSens.set_led_pwm(100);
    clamp.set_value(HIGH);
    ringSens.set_led_pwm(100);
    chassis.setPose(0,0,-293);
    ringSens.set_led_pwm(100);
    intake.move(127);
    ringSens.set_led_pwm(100);
    //waitUntilRedIntake();
    right_doinker.set_value(HIGH);
    ringSens.set_led_pwm(100);

    // get to ring stack mid
    drivePID(45,1100);
   
    
   // slow intake and grab ring, back up with rings
    intake.move(40);
   drivePID(-27,950,66);
   intake.brake();
   right_doinker.set_value(LOW);
   
   
   // turn to face mogo and grab it
   chassis.turnToHeading(-47,600,{}, false);

   drivePID(-20,900,40);
   clamp.set_value(LOW);
   delay(300);

  
   // intake ring in intake on goal
   intake.move(127);

   //face line of rings
   chassis.turnToHeading(-357,650,{}, false);

   // score line of rings
   drivePID(38,900,35);
   chassis.turnToHeading(-150,650,{}, false);
   drivePID(60,1300,60);
   clamp.set_value(HIGH);
   intake.move(45);
   drivePID(42,800);
    waitUntilRedIntake(500);
    intake.brake();
    setArmMid();
    drivePID(-43,800,120);
    clamp.set_value(LOW);
    drivePID(20,700);
    delay(100);
    left_doinker.set_value(HIGH);
    chassis.turnToHeading(-90,900,{}, false);
    intake.move(127);
    left_doinker.set_value(LOW);
    drivePID(5,700);
    setArmAlliance();
    delay(600);
    drivePID(-10,600);
    setArmBottom();
    chassis.turnToHeading(-270,700,{}, false);
    drivePID(22,1000,30);
}
void progSkills(int i)
{

    // initial setup
    chassis.setPose(-54, 0, 270);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    
    // score on alliance stake
    setArmAlliance();
    delay(600);
    clamp.set_value(HIGH);

    // back up to align with goal
    drivePID(-8, 500,170);
    // endSection(50000);
    setArmMid();
    chassis.turnToHeading(0, 575,{}, false);
    setArmBottom();

    // go to clamp
    drivePID(-18,700);
    drivePID(-10, 600, 42.5);
    clamp.set_value(LOW);
    delay(100);
    drivePID(4, 500,170);

    // turn & go to ring1
    chassis.turnToHeading(90, 575,{}, false);
    intake.move(127);
    drivePID(24,800);
    

    // turn to approach wall stake
    chassis.turnToHeading(126, 550,{}, false);
    drivePID(28.8,800);

    // turn & go to wall stake
    setArmMid();
    chassis.turnToHeading(180, 550,{}, false);
    

    intake.move(127);
    //side stake 1
    drivePID(23,1000,30);
    //delay(500);
    //delay(500);
    delay(450);
    intake.move(-32);
    delay(20);
    setArm(150);
    drivePID(3,150,190);
    intake.brake();
    delay(400);
    setArm(57);
    //delay(300);
    //endSection(2000);

    //back up to align with 3 rings
    drivePID(-16,800);
    intakeOverride = true;

    // turn and go to 3 rings
    //setArmBottom();
    intake.move(127);
    chassis.turnToHeading(270, 550,{}, false);
    drivePID(77, 1500, 30);

    // back up to align with last ring
    drivePID(-19,800);
    // turn & go to last ring
    chassis.turnToHeading(180, 550,{}, false);
    drivePID(25, 750, 40);
    // back up to align with corner
    drivePID(-22,750);

    // turn and go to corner
    chassis.turnToHeading(45, 900,{}, false);
    drivePID(-31, 900, 35);
    intake.move(-90);
    clamp.set_value(HIGH);
    //endSection(500);
    
    chassis.turnToHeading(45, 400,{}, false);
    
    //chassis.setPose(0,0,45);

 
    // move forward to align with goal 2
    
    drivePID(14.8,800);
    intake.move(127);
    chassis.turnToHeading(181, 900,{}, false);
    //chassis.moveToPose(-48,-36,180,7000,{.forwards=false, .maxSpeed = 50},false);
    drivePID(-78,2000,40);
    drivePID(2,400,160);
    drivePID(-7,500,30);
    //clamp goal 2
    clamp.set_value(LOW);

    delay(100);
    //endSection(50000);
    drivePID(5, 500);
    //endSection(500);
    chassis.turnToHeading(84, 600,{}, false);
    intake.move(127);
    //grab first ring
    drivePID(22,800);
    chassis.turnToHeading(50, 600,{}, false);
    drivePID(29.1,900);
    //endSection(500);
    //side stake 2
    intakeOverride = false;
    setArmMid();
    chassis.turnToHeading(0, 650,{}, false);
    drivePID(26,1000,30);
    delay(450);
    //delay(500);
    //delay(500);
    intake.move(-32);
    delay(20);
    setArm(150);
    drivePID(3,150,190);
    intake.brake();
    delay(400);
    setArm(57);
    //delay(300);

    //endSection(500);
    drivePID(-20,800);
    intakeOverride = true;
    intake.move(127);
    chassis.turnToHeading(268 , 700,{}, false);
    //pick up line of 3 rings
    drivePID(84, 1500, 30);
    drivePID(-20,800);
    chassis.turnToHeading(0, 550,{}, false);
    drivePID(25, 750, 40);
    drivePID(-22,800);
    chassis.turnToHeading(135, 700,{}, false);
    
    // TEMP POSE
    /*chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    clamp.set_value(LOW);
    chassis.setPose(0, 0, imu.get_heading());
    delay(500);*/

    //drop in corner
    drivePID(-31, 900, 35);
    intake.move(-90);
    clamp.set_value(HIGH);
    chassis.turnToHeading(135, 400,{}, false);

    //endSection(1000);
    setArmBottom();
    drivePID(25,800);
    chassis.turnToHeading(130, 500,{}, false);
    intake.move(41);
    //intake.move(127);
    //goal 3
    drivePID(92,2000,130);
    setArmMid();
    intakeOverride = false;
    chassis.turnToHeading(222, 700,{}, false);
    //drive towards goal 3
    intake.move(127);
    drivePID(-22,700);
    drivePID(-21 ,900,35);

    clamp.set_value(LOW);
    drivePID(3,500,160);
    intake.brake();
    chassis.turnToHeading(90, 800,{}, false);
    //alliance stake 2
    drivePID(26,700,40);
    delay(200);
    intake.move(-7);
    drivePID(-9.5,600,160);
    intake.brake();
    setArmAlliance();
    delay(600);
    drivePID(-15,500,130);
    setArmBottom();
    drivePID(11.5,500,130);
    intakeOverride = true;
    //ring 1 for goal 3
    intake.move(127);
    delay(100);
    chassis.turnToHeading(305, 700,{}, false);
    drivePID(37,1000);
    //ring 2 for goal 3
    chassis.turnToHeading(0, 540,{}, false);
    drivePID(21,800);
    //ring 3 and 4
    chassis.turnToHeading(90, 550,{}, false);
    drivePID(32.5,800);
    drivePID(-10,500,170);
    //ring 5
    chassis.turnToHeading(0, 550,{}, false);
    drivePID(15,800);
    chassis.turnToHeading(225, 650,{}, false);
    //drop in corner
    drivePID(-33,1000,25);
    intake.move(-30);
    delay(100);
    clamp.set_value(HIGH);
    chassis.turnToHeading(255, 400,{}, false);
    //move out of corner
    drivePID(32,900);
    intake.move(-127);
    chassis.turnToHeading(149, 800,{}, false);
    //push goal 4 into corner
    drivePID(150,2000,170);
    chassis.turnToHeading(128, 700,{}, false);
    setArmAlliance();
    intake.brake();
    all_motors.set_brake_mode(E_MOTOR_BRAKE_COAST);
    drivePID(-100,1050,60);
    delay(600);
    drivePID(4,500);
    

    

    /*
        chassis.setPose(-54, 0, 270);
        int startTime = pros::millis();
        chassis.turnToHeading(270 + 180, 2000);
        endSection(50000);
        chassis.turnToHeading(270 + 180 + 90, 2000);
        endSection(50000);
        chassis.turnToHeading(270 + 180 + 90 + 45, 2000);*/

    /*
    // PROG SKILLS ROUTE 1 (reference is skills_aio.txt)
    // set position to starting spot (needs tweaking)
    chassis.setPose(-60, 0, 0);
    // back up to hit goal1
    chassis.moveToPose(-48, -24, 315, 2000, {.forwards = false}, true);
    // clamp goal1
    delay(1500);
    clamp.set_value(LOW);

    intake.move(127);
    // score preload
    delay(1000);
    // grab ring1 for goal1
    chassis.moveToPoint(-24, -24, 2000, {}, false);
    // grab ring2 for goal1
    chassis.moveToPoint(-24, -48, 2000, {}, false);
    // grab ring3 for goal1
    chassis.moveToPoint(-59, -48, 3000, {}, false);
    delay(1000);
    // grab ring4 for goal1
    chassis.moveToPoint(-48, -48, 2000, {.forwards = false}, false);
    // grab ring5 for goal1
    chassis.moveToPoint(-48, -59, 2000, {}, false);
    // small delay so ring5 intakes
    delay(1000);
    // back goal1 into corner
    chassis.moveToPose(-63, -63, 45, 2000, {.forwards = false}, false);
    // stop intake
    intake.brake();
    // unclamp goal1
    clamp.set_value(HIGH);*/
    /*
        clamp.set_value(HIGH);
        chassis.setPose(-54, 0, 270);
        chassis.moveToPoint(-48, 0, 5000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 12}, false);
        chassis.turnToHeading(0, 2000);
        chassis.moveToPoint(-48, -28, 5000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 12}, false);
        clamp.set_value(LOW);
        endSection(50000);
        chassis.turnToHeading(90, 2000);
        intake.move(127);
        //chassis.setPose(-48, -28, 0);
        chassis.moveToPoint(-20, -26, 5000, {.forwards = true, .minSpeed = 72}, false);
        delay(1000);
        chassis.moveToPoint(0, -55, 5000, {.forwards = true, .minSpeed = 40}, false);
        delay(2000);
        endSection(50000);
        chassis.moveToPoint(0, -52, 5000, {.forwards = false, .minSpeed = 72}, false);
        chassis.turnToHeading(270, 2000);
        chassis.moveToPoint(-60, -52, 5000, {.forwards = true, .minSpeed = 72}, false);
        endSection(50000);
        chassis.moveToPoint(-48,-52, 5000, {.forwards = true, .minSpeed = 72}, false);
        chassis.turnToHeading(180, 2000);
        chassis.moveToPoint(-48, -55, 5000, {.forwards = false, .minSpeed = 72}, false);
        chassis.moveToPoint(-48, -52, 5000, {.forwards = false, .minSpeed = 72}, false);
    */

    /*
    //approach goal2
    chassis.moveToPose(-48,5,180,2000,{},false);
    //back into goal2
    chassis.moveToPose(-48,24,180,2000,{},false);
    //clamp goal2 & intake
    clamp.set_value(LOW);
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
    //small delay so ring5 intakes
    delay(400);
    //back goal2 into corner
    chassis.moveToPose(-63, 63, 135, 2000, {.forwards=false},false);
    //stop intake
    intake.brake();
    //unclamp goal2
    clamp.set_value(HIGH);

    // May need to add an odom resetter here
    // look up how they work but tldr is they have robot go into wall and then reset position of odom based on knowing they are at wall
    // it prevents drift

    */
}
void blueGoalSide(int i)
{

    // mapped in redGoalSide.txt
    clamp.set_value(HIGH);
    // rush goal and clamp
    chassis.setPose(52, -63.4, -270);
    chassis.moveToPoint(20, -58, 5000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 12}, false);
    chassis.moveToPose(3, -49, -240, 2000, {.forwards = false, .lead = .2, .minSpeed = 40}, false);
    clamp.set_value(LOW);
    endSection(100);

    // grab ring1 for goal1
    intake.move(127);
    chassis.moveToPoint(34, -45, 4000, {}, false);
    delay(150);
    intake.brake();
    endSection(0);

    // unclamp goal1
    clamp.set_value(HIGH);
    endSection(300);

    // face goal2
    chassis.moveToPoint(36, -36, 600, {.maxSpeed = 40}, false);
    delay(500);
    chassis.turnToHeading(180, 1000);
    endSection(100);

    // goto goal2 and clamp
    chassis.moveToPose(22, -36, 60, 2000, {.forwards = false, .minSpeed = 15}, false);
    clamp.set_value(LOW);
    intake.move(127);

    endSection(200);

    /*// grab ring1 for goal2
    //intake.move(127);
    setArmTop();
    chassis.moveToPoint(52, -6, 4000, {.minSpeed = 40}, false);
    setArmBottom();
    delay(500);
    intake.move(127);
    delay(400);
    intake.brake();
    chassis.moveToPoint(59, -40, 4000, {.forwards = false, .minSpeed = 40}, false);
    clamp.set_value(HIGH);
    //while(ringSens.)
    endSection(500);

    //
    //chassis.moveToPoint(-12, -12, 5000, {}, false);
    chassis.moveToPoint(64, 5, 4000, {.minSpeed = 40}, false);
    clamp.set_value(LOW);

    // score on alliance stake
    chassis.moveToPose(79,-13.5,-60,5000, {.forwards = false, .minSpeed = 72},false);
    intake.move(127);
    endSection(1000);*/

    // go to middle with arm up
    intake.brake();
    setArmTop();
    chassis.moveToPoint(5, 0, 3000, {.maxSpeed = 70}, false);
}

void redGoalSide(int i) // edit this one and mirror it for the other part
{

    // mapped in redGoalSide.txt
    clamp.set_value(HIGH);
    // rush goal and clamp
    chassis.setPose(-52, -63.4, 270);
    chassis.moveToPoint(-20, -58, 5000, {.forwards = false, .minSpeed = 72, .earlyExitRange = 12}, false);
    chassis.moveToPose(-3, -49, 240, 2000, {.forwards = false, .lead = .2, .minSpeed = 40}, false);
    clamp.set_value(LOW);
    endSection(100);

    // grab ring1 for goal1
    intake.move(127);
    chassis.moveToPoint(-34, -45, 4000, {}, false);
    delay(150);
    intake.brake();
    endSection(0);

    // unclamp goal1
    clamp.set_value(HIGH);
    endSection(300);

    // face goal2
    chassis.moveToPoint(-36, -36, 600, {.maxSpeed = 40}, false);
    chassis.turnToHeading(180, 1000);
    endSection(100);

    // goto goal2 and clamp
    chassis.moveToPose(-22, -36, 240, 2000, {.forwards = false, .minSpeed = 15}, false);
    clamp.set_value(LOW);
    intake.move(127);

    endSection(200);

    // grab ring1 for goal2
    // intake.move(127);
    setArmTop();
    chassis.moveToPoint(-52, -6, 4000, {.minSpeed = 40}, false);
    setArmBottom();
    delay(500);
    intake.move(127);
    delay(400);
    intake.brake();
    chassis.moveToPoint(-59, -40, 4000, {.forwards = false, .minSpeed = 40}, false);
    clamp.set_value(HIGH);
    // while(ringSens.)
    endSection(500);

    //
    // chassis.moveToPoint(-12, -12, 5000, {}, false);
    chassis.moveToPoint(-64, 5, 4000, {.minSpeed = 40}, false);
    clamp.set_value(LOW);

    // score on alliance stake
    chassis.moveToPose(-79, -13.5, 60, 5000, {.forwards = false, .minSpeed = 72}, false);
    intake.move(127);
    endSection(1000);

    // go to middle with arm up
    intake.brake();
    setArmTop();
    chassis.moveToPoint(-5, 0, 2000, {.maxSpeed = 70}, false);
}
void blueRingSide(int i)
{

    drivePID(-27);
    driveInchesClamp(-7, 30);
    delay(500);
    intake.move(127);
    endSection(10000);

    chassis.turnToHeading(-55, 2000);
    delay(500);
    drivePID(27); //+3; nvm
    delay(500);
    drivePID(-4);
    endSection(10000);

    chassis.turnToHeading(-145, 2000);
    drivePID(20);
    delay(500);
    endSection(10000);

    drivePID(-10);
    chassis.turnToHeading(-110, 2000);
    drivePID(11, 30);
    delay(500);
    drivePID(-6);
    chassis.turnToHeading(-145, 2000);
    endSection(10000);

    drivePID(-15);
    chassis.turnToHeading(-55, 2000);
    endSection(10000);

    drivePID(-11); //-3; nvm
    drivePID(-20);
    setArmTop();
    chassis.turnToHeading(-145, 2000);
    drivePID(7);
    endSection(10000);

    /*
    chassis.moveToPoint(58,-12,5000,{.forwards=true,.minSpeed=20},false);
    chassis.moveToPoint(64,-1,5000,{.forwards=false,.minSpeed=20},false);
    */
    /*
    chassis.moveToPoint(47,-11,5000,{.forwards=true,.minSpeed=20},false);
    chassis.moveToPoint(21,-24,5000,{.forwards=false,.minSpeed=20},false);
    clamp.set_value(LOW);
    chassis.moveToPose(23,-54,180,5000,{.forwards=true,.lead=.2,.minSpeed=20},false);*/
}
void redRingSide(int i)
{
}
void redGoalSidePostWPI(int i)
{ // in tylers routes, scores 5 on goalside, doinkers goalside without clamping

    // rush goal and set doinker down
    clamp.set_value(HIGH);
    chassis.setPose(-54, -61, 90);
    chassis.moveToPoint(-20, -58, 5000, {.minSpeed = 72, .earlyExitRange = 12}, false);
    chassis.moveToPose(-12, -49, 45, 2000, {.lead = .2, .minSpeed = 60}, false);
    right_doinker.set_value(LOW);
    endSection(50000);

    // back up with goal doinkered
    chassis.moveToPoint(-17, -55, 2000, {.forwards = false, .minSpeed = 40}, false);
    endSection(50000);

    // undoinker and turn to ring1
    right_doinker.set_value(HIGH);
    chassis.turnToPoint(-28, -48, 2000, {}, false);
    endSection(50000);

    // approach and intake ring1
    chassis.moveToPoint(-28, -48, 2000, {}, true);
    delay(150);
    intake.move(127);
    while (chassis.isInMotion())
    {
        delay(20);
    }
    endSection(50000);

    // turn to goal1
    intake.brake();
    chassis.turnToHeading(180, 2000);
    endSection(50000);

    // goto goal1, clamp, and then intake both rings
    chassis.moveToPoint(-29, -27, 2000, {.forwards = false, .maxSpeed = 50}, false);
    clamp.set_value(LOW);
    intake.move(127);
    endSection(50000);

    // back up to line up with corner
    chassis.moveToPoint(-33, -62, 2000, {.forwards = false, .minSpeed = 40}, false);
}
void WPIAWP(int i)
{

    clamp.set_value(HIGH);
    // inital pose beside alliance stake
    chassis.setPose(-61, 12, 270 - 49);

    // score on alliance stake
    setArmTop();
    delay(1000);
    setArmBottom();
    endSection(500);

    // chassis.moveToPose(-48,-12,270,5000, {.forwards = false, .minSpeed = 72},false);
    //  clamp goal1
    chassis.moveToPoint(-29.2, 20, 2000, {.forwards = false}, false);
    clamp.set_value(LOW);
    endSection(500);

    // grab ring1 for goal1 (ringside midfield)
    intake.move(127);
    chassis.moveToPoint(-27, 47, 2000, {}, false);
    endSection(200);

    // grab ring2 for goal1 (ringside upper)
    chassis.moveToPoint(-15, 55, 2000, {}, false);
    endSection(500);

    // back up to ringside turning point
    chassis.moveToPoint(-33.3, 47.5, 2000, {.forwards = false}, false);
    intake.brake();
    endSection(500);

    intake.move(127);
    chassis.moveToPoint(-15, 47, 2000, {}, false);
    endSection(500);

    chassis.moveToPose(-40, 47.5, 45, 3000, {.forwards = false}, false);
    endSection(500);

    chassis.moveToPoint(-20, 20, 2000, {}, false);

    /*
    // turn to face goalside
    chassis.turnToHeading(180,2000);
    endSection(50000);

    // rush to goalside (motion chaining)
    chassis.moveToPoint(-38.2,-25.4,2000,{.minSpeed=70,.earlyExitRange=6},false);
    intake.move(127);
    clamp.set_value(HIGH);
    endSection(50000);

    // get ring1 for goal2
    chassis.moveToPoint(-24,-48,2000,{},false);
    //here we need to tweak delay to hold ring
    endSection(50000);

    // face goal2
    intake.brake();
    chassis.turnToHeading(180,2000);
    endSection(50000);

    //clamp goal2
    chassis.moveToPoint(-24,-28,2000,{.forwards=false},false);
    clamp.set_value(LOW);
    endSection(50000);

    //turn to posts
    setArmTop();
    chassis.turnToHeading(45,2000);
    endSection(5000);

    //touch posts
    chassis.moveToPoint(5,0,2000,{.maxSpeed=60},false);

    */
}

void safeAWPLeft(int i)
{
    clamp.set_value(HIGH);
    chassis.setPose(0, 0, 212);
    ringSens.set_led_pwm(100);
    delay(2000);

    // arm functions
    setArmAlliance();
    delay(600);
    drivePID(-6,800);
    //endSection(700);

    // move to alliance ring and score it
    chassis.turnToHeading(143, 700, {}, false);
    intake.move(127);
    setArmBottom();
    drivePID(24, 1000, 30);
    delay(300);
    intake.move(70);
    drivePID(16,800);
    waitUntilAnyIntake(400);
    intake.brake();
    // chassis.turnToHeading(180 ,1000,{},false);

    drivePID(-29, 800);
    chassis.turnToHeading(247, 700, {}, false);

    // go to goal and clamp
    drivePID(-20, 800,40);
    drivePID(-18, 800, 35);
    clamp.set_value(LOW);
    intake.move(127);
    delay(100);
    chassis.turnToHeading(0, 800, {}, false);
    intake.move(127);

    // score ring 2
    drivePID(29, 800, 45);
    delay(300);
    //endSection(500);

    // go to middle
    chassis.turnToHeading(170, 800, {}, false);
    drivePID(30);
    setArmAlliance();
    arm_motors.move(30);
    left_motors.brake();
    right_motors.brake();
    /*clamp.set_value(HIGH);
    chassis.setPose(0, 0, 221);

    // arm functions
    setArmAlliance();
    delay(1000);
    drivePID(-6);
    setArmTop();
    endSection(700);

    // move to alliance ring and score it
    chassis.turnToHeading(159, 1000, {}, false);
    intake.move_velocity(90);
    drivePID(27, 1000, 45);
    setArmBottom();
    delay(300);
    drivePID(-15, 1000);
    intake.move_velocity(75);
    waitUntilAnyIntake(2000);
    intake.move(-50);
    delay(100);
    intake.brake();
    chassis.turnToHeading(252, 700, {}, false);
    intake.brake();

    // go to goal and clamp
    drivePID(-18, 1500);
    drivePID(-11, 1000, 42.5);
    clamp.set_value(LOW);
    delay(300);
    intake.move(127);
    endSection(500);
    chassis.turnToHeading(20, 1000, {}, false);

    // score ring 2
    drivePID(29, 1000, 45);
    endSection(500);

    // go to middle
    chassis.turnToHeading(165, 1000, {}, false);
    setArmTop();
    left_motors.move(42);
    right_motors.move(42);
    delay(5000);
    left_motors.brake();
    right_motors.brake();*/
}

void safeAWPRight(int i)
{
    clamp.set_value(HIGH);
    chassis.setPose(0, 0, -212);
    ringSens.set_led_pwm(100);
    delay(2000);

    // arm functions
    setArmAlliance();
    delay(600);
    drivePID(-6,800);
    //endSection(700);

    // move to alliance ring and score it
    chassis.turnToHeading(-143, 700, {}, false);
    intake.move(127);
    setArmBottom();
    drivePID(24, 1000, 30);
    delay(300);
    intake.move(70);
    drivePID(16,800);
    waitUntilAnyIntake(400);
    intake.brake();
    // chassis.turnToHeading(180 ,1000,{},false);

    drivePID(-29, 800);
    chassis.turnToHeading(-247, 700, {}, false);

    // go to goal and clamp
    drivePID(-20, 800,40);
    drivePID(-18, 800, 35);
    clamp.set_value(LOW);
    intake.move(127);
    delay(100);
    chassis.turnToHeading(0, 800, {}, false);
    intake.move(127);

    // score ring 2
    drivePID(29, 800, 45);
    delay(300);
    //endSection(500);

    // go to middle
    chassis.turnToHeading(-170, 800, {}, false);
    drivePID(30);
    setArmAlliance();
    arm_motors.move(30);
    left_motors.brake();
    right_motors.brake();

    /* THIRD GOAL LINEUP
    chassis.turnToHeading(-90, 1000, {}, false);
    drivePID(-24,1700,70);
    clamp.set_value(HIGH);
    chassis.turnToHeading(-270, 1000, {}, false);
    drivePID(-28,1700,70);*/

    /*clamp.set_value(HIGH);
    chassis.setPose(0, 0, 221);

    // arm functions
    setArmAlliance();
    delay(1000);
    drivePID(-6);
    setArmTop();
    endSection(700);

    // move to alliance ring and score it
    chassis.turnToHeading(159, 1000, {}, false);
    intake.move_velocity(90);
    drivePID(27, 1000, 45);
    setArmBottom();
    delay(300);
    drivePID(-15, 1000);
    intake.move_velocity(75);
    waitUntilAnyIntake(2000);
    intake.move(-50);
    delay(100);
    intake.brake();
    chassis.turnToHeading(252, 700, {}, false);
    intake.brake();

    // go to goal and clamp
    drivePID(-18, 1500);
    drivePID(-11, 1000, 42.5);
    clamp.set_value(LOW);
    delay(300);
    intake.move(127);
    endSection(500);
    chassis.turnToHeading(20, 1000, {}, false);

    // score ring 2
    drivePID(29, 1000, 45);
    endSection(500);

    // go to middle
    chassis.turnToHeading(165, 1000, {}, false);
    setArmTop();
    left_motors.move(42);
    right_motors.move(42);
    delay(5000);
    left_motors.brake();
    right_motors.brake();*/
}

void safe4RingRight(int i){
    clamp.set_value(HIGH);
    chassis.setPose(0, 0, -212);
    ringSens.set_led_pwm(100);
    // arm functions
    setArmAlliance();
    delay(600);
    drivePID(-6,800);
    //endSection(700);

    // move to alliance ring and score it
    chassis.turnToHeading(-143, 700, {}, false);
    intake.move(127);
    setArmBottom();
    drivePID(24, 1000, 30);
    delay(300);
    intake.move(80);
    drivePID(16,800);
    waitUntilAnyIntake(400);
    intake.brake();
    // chassis.turnToHeading(180 ,1000,{},false);

    drivePID(-29, 800);
    chassis.turnToHeading(-247, 700, {}, false);

    // go to goal and clamp
    drivePID(-20, 800,40);
    drivePID(-18, 800, 35);
    clamp.set_value(LOW);
    intake.move(127);
    delay(100);
    chassis.turnToHeading(0, 800, {}, false);
    intake.move(127);

    // score ring 2
    drivePID(29, 800, 45);
    chassis.turnToHeading(270, 800, {}, false);
    drivePID(16,800);
    drivePID(-7,500);
    chassis.turnToHeading(235, 800, {}, false);
    drivePID(7,500);
    drivePID(-9,500);
    
    chassis.turnToHeading(-180, 800, {}, false);
    drivePID(30,800,170);
    setArmAlliance();
    arm_motors.move(30);
    left_motors.brake();
    right_motors.brake();

    //endSection(500);



}
void safe4RingLeft(int i){
    clamp.set_value(HIGH);
    chassis.setPose(0, 0, 212);
    ringSens.set_led_pwm(100);

    // arm functions
    setArmAlliance();
    delay(600);
    drivePID(-6,800);
    //endSection(700);

    // move to alliance ring and score it
    chassis.turnToHeading(143, 700, {}, false);
    intake.move(127);
    setArmBottom();
    drivePID(24, 800, 45);
    delay(300);
    intake.move(80);
    drivePID(16,800);
    waitUntilAnyIntake(400);
    intake.brake();
    // chassis.turnToHeading(180 ,1000,{},false);

    drivePID(-29, 800);
    chassis.turnToHeading(247, 700, {}, false);

    // go to goal and clamp
    drivePID(-20, 800,40);
    drivePID(-18, 800, 35);
    clamp.set_value(LOW);
    intake.move(127);
    delay(100);
    chassis.turnToHeading(0, 800, {}, false);
    intake.move(127);

    // score ring 2
    drivePID(29, 800, 45);
    chassis.turnToHeading(-270, 800, {}, false);
    drivePID(16,800);
    drivePID(-7,500);
    chassis.turnToHeading(-235, 800, {}, false);
    drivePID(7,500);
    drivePID(-12,500);
    
    chassis.turnToHeading(170, 800, {}, false);
    drivePID(30,800,170);
    setArmAlliance();
    arm_motors.move(30);
    left_motors.brake();
    right_motors.brake();

    //endSection(500);



}
void danburyRedRS(int i)
{
    clamp.set_value(HIGH);
    chassis.setPose(0, 0, 212);

    // arm functions
    setArmAlliance();
    delay(500);
    drivePID(-6);
    endSection(100);

    // move to alliance ring and score it
    chassis.turnToHeading(140, 1000, {}, false);
    intake.move(80);
    drivePID(31, 1000, 45);
    setArmBottom();
    delay(150);
    intake.move(20);
    // chassis.turnToHeading(180 ,1000,{},false);

    drivePID(-19, 1000);
    delay(500);
    intake.brake();
    chassis.turnToHeading(243, 700, {}, false);
    intake.move(10);

    // go to goal and clamp
    drivePID(-20, 1500,40);
    drivePID(-18, 1000, 35);
    clamp.set_value(LOW);
    intake.move(127);
    delay(300);
    chassis.turnToHeading(0, 1000, {}, false);
    intake.move(-50);
    delay(100);
    intake.move(127);

    // score ring 2
    drivePID(26, 1000, 45);
    endSection(500);

    right_doinker.set_value(HIGH);
    chassis.turnToHeading(90, 1000, {}, false);
    drivePID(17,800,39);
    drivePID(-17,1000);
    // go to middle
    chassis.turnToHeading(145, 1000, {}, false);
    right_doinker.set_value(LOW);
    chassis.turnToHeading(170, 1000, {}, false);
    drivePID(40);
    arm_motors.move(30);
    left_motors.brake();
    right_motors.brake();
    /*clamp.set_value(HIGH);
    chassis.setPose(0, 0, 221);

    // arm functions
    setArmAlliance();
    delay(1000);
    drivePID(-6);
    setArmTop();
    endSection(700);

    // move to alliance ring and score it
    chassis.turnToHeading(159, 1000, {}, false);
    intake.move_velocity(90);
    drivePID(27, 1000, 45);
    setArmBottom();
    delay(300);
    drivePID(-15, 1000);
    intake.move_velocity(75);
    waitUntilAnyIntake(2000);
    intake.move(-50);
    delay(100);
    intake.brake();
    chassis.turnToHeading(252, 700, {}, false);
    intake.brake();

    // go to goal and clamp
    drivePID(-18, 1500);
    drivePID(-11, 1000, 42.5);
    clamp.set_value(LOW);
    delay(300);
    intake.move(127);
    endSection(500);
    chassis.turnToHeading(20, 1000, {}, false);

    // score ring 2
    drivePID(29, 1000, 45);
    endSection(500);

    // go to middle
    chassis.turnToHeading(165, 1000, {}, false);
    setArmTop();
    left_motors.move(42);
    right_motors.move(42);
    delay(5000);
    left_motors.brake();
    right_motors.brake();*/
}

void danburyBlueRS(int i){
    clamp.set_value(HIGH);
    chassis.setPose(0, 0, -212);

    // arm functions
    setArmAlliance();
    delay(500);
    drivePID(-6);
    endSection(100);

    // move to alliance ring and score it
    chassis.turnToHeading(-140, 1000, {}, false);
    intake.move(80);
    drivePID(31, 1000, 45);
    setArmBottom();
    delay(150);
    intake.move(20);
    // chassis.turnToHeading(180 ,1000,{},false);

    drivePID(-19, 1000);
    delay(500);
    intake.brake();
    chassis.turnToHeading(-243, 700, {}, false);
    intake.move(10);

    // go to goal and clamp
    drivePID(-20, 1500,40);
    drivePID(-18, 1000, 35);
    clamp.set_value(LOW);
    intake.move(127);
    delay(300);
    chassis.turnToHeading(-0, 1000, {}, false);
    intake.move(-50);
    delay(100);
    intake.move(127);

    // score ring 2
    drivePID(26, 1000, 45);
    endSection(500);

    right_doinker.set_value(HIGH);
    chassis.turnToHeading(-90, 1000, {}, false);
    drivePID(17,800,39);
    drivePID(-17,1000);
    // go to middle
    chassis.turnToHeading(-145, 1000, {}, false);
    right_doinker.set_value(LOW);
    chassis.turnToHeading(-170, 1000, {}, false);
    drivePID(40);
    arm_motors.move(30);
    left_motors.brake();
    right_motors.brake();
}

void fullawpV1(int i)
{

    // initial pose
    chassis.setPose(-58.622, 23.677, 180);
    clamp.set_value(HIGH);

    // push blue ring in front of alliance stake
    chassis.moveToPoint(-58.622, 5.504, 4321, {}, false);
    endSection(987654321);

    // align with alliance stake and score
    chassis.turnToHeading(240, 4321, {}, true);
    setArmAlliance();
    delay(1000);
    endSection(987654321);

    // back up to goal and clamp
    chassis.moveToPoint(-23.622, 23.622, 4321, {.forwards = false}, true);
    chassis.waitUntil(5);
    setArmBottom();
    chassis.waitUntilDone();
    clamp.set_value(LOW);
    endSection(987654321);

    // turn to face ring1 for goal1
    chassis.turnToHeading(0, 4321);
    endSection(987654321);

    // goto ring1 and score it
    intake.move(127);
    chassis.moveToPoint(-23.622, 47, 4321, {}, false);
    endSection(987654321);

    // turn to ring2 intermediate point
    chassis.turnToHeading(225, 4321, {}, true);
    waitUntilRedIntake(1000);
    intake.brake();
    chassis.waitUntilDone();
    endSection(987654321);

    // goto ring2 intermediate point
    intake.move(127);
    chassis.moveToPoint(-47, 23.622, 4321, {}, false);
    intake.brake();
    endSection(987654321);

    // turnto ring2
    chassis.turnToHeading(180, 4321);
    endSection(987654321);

    // hold ring2
    chassis.moveToPoint(-47.229, -9.762, 4321, {}, true);
    chassis.waitUntil(5);
    clamp.set_value(HIGH);
    waitUntilRedIntake(3000);
    chassis.waitUntilDone();
    endSection(987654321);

    // turnto goal2
    chassis.turnToHeading(300, 4321);
    endSection(987654321);

    // goto goal2 and clamp
    chassis.moveToPoint(-23.686, -23.686, 4321, {.forwards = false}, false);
    clamp.set_value(LOW);
    endSection(987654321);

    // turnto ring1 for goal2
    chassis.turnToHeading(180, 4321);
    endSection(987654321);

    // goto ring1 and score
    intake.move(127);
    chassis.moveToPoint(-23.622, -47.227, 4321, {}, true);
    waitUntilRedIntake(1000);
    chassis.waitUntilDone();
    endSection(987654321);

    // turn to ladder
    chassis.turnToHeading(20, 4321);
    endSection(987654321);

    // move arm up and score on ladder
    setArmMid();
    chassis.moveToPoint(-5, -5, 4321);
    endSection(987654321);
}

void oldRedRingSide(int i) // 4 ring, red ringside, ported from vexcode
{

    // drive and clamp, also start running the intake
    clamp.set_value(HIGH);
    drivePID(-27);
    // driveInchesClamp(-7, 30);
    drivePID(-4, 1500, 40);
    clamp.set_value(LOW);
    delay(500);
    intake.move(127);

    // turns the robot, moves toward ring1 to inake it
    chassis.turnToHeading(75, 2000);
    drivePID(15);
    delay(500);
    // drivePID(-4);

    // robot turns again, heads toward ring2, intakes
    // then the robot turns, drives, and intakes the next ring, before turning again
    chassis.turnToHeading(145, 2000);
    drivePID(12.9, 3000, 20);
    delay(500);
    drivePID(-25);
    chassis.turnToHeading(-123, 2000);
    drivePID(10);
    drivePID(6.4, 20);
    delay(1000);
    drivePID(-48);
    chassis.turnToHeading(120, 2000);

    // arm moves up, then turns toward the middle in order to touch the ladder
    setArmTop();
    drivePID(47);
    intake.move(63);
    setArmBottom();
    delay(300);
    intake.brake();
    delay(200);
    drivePID(-10);
    intake.move(127);
    chassis.turnToHeading(-60, 2000);
}

void redRingRush(int i)
{ // this route uses the new doinker mech to rush rings very fast

    // initial states
    chassis.setPose(0, 0, 66 * i);
    clamp.set_value(HIGH);

    // put down ringrush mech
    right_doinker.set_value(HIGH);

    // approach rings
    drivePID(43);
    endSection(100);

    // back up with rings
    drivePID(-10);
    endSection(100);

    // turn to be perpendicular to ring line
    chassis.turnToHeading(90 * i, 2000);
    endSection(100);

    // back up with rings and lift right_doinker
    drivePID(-14);
    right_doinker.set_value(LOW);
    endSection(100);

    // drive back a little more to align with goal
    drivePID(-5);
    endSection(100);

    // align to back up into goal
    chassis.turnToHeading(307 * i, 2000);
    delay(100);

    // back up into goal and clamp
    drivePID(-35, 2000, 35);
    clamp.set_value(LOW);
    delay(150);
    drivePID(6, 1000);
    endSection(100);

    // turn to ring line
    chassis.turnToHeading(-1 * i, 2000);
    endSection(100);

    // intake ring line
    intake.move(127);
    chassis.turnToHeading(2 * i, 2000);
    drivePID(48, 5000, 5);
    endSection(1000);
    intake.brake();

    // turn to peload
    endSection(100);
    drivePID(-10, 2000);
    chassis.turnToHeading(-100 * i, 2000);
    intake.move(127);
    drivePID(65, 3000, 10);
}

void redGoalSideSugarRush(int i)
{
    clamp.set_value(HIGH);
    right_doinker.set_value(HIGH);
    chassis.setPose(0, 0, 90);
    drivePID(40, 1000, 70);
    endSection();

    chassis.turnToHeading(275, 1000, {}, false);
    right_doinker.set_value(LOW);
    endSection();

    chassis.turnToHeading(320, 1000);
    intake.move(127);
    drivePID(15);
    delay(270);
    intake.brake();
    // drivePID(8);
    endSection(100);

    chassis.turnToHeading(180, 1000, {}, false);
    drivePID(-27);
    clamp.set_value(LOW);
    endSection(500);

    chassis.turnToHeading(270, 1000, {}, false);
    intake.move(127);
    drivePID(35, 1000);
    endSection();

    chassis.turnToHeading(190, 1000, {}, false);
    right_doinker.set_value(HIGH);
    drivePID(37, 1000);
    endSection();

    chassis.turnToHeading(90, 1000, {}, false);
    endSection();

    intake.brake();
    drivePID(20, 1000);
    clamp.set_value(HIGH);
    endSection();

    right_doinker.set_value(LOW);
    chassis.turnToHeading(225, 1000, {}, false);
    endSection();

    drivePID(-24, 1000, 80);
}
void blueGoalSideSugarRush(int i)
{
    clamp.set_value(HIGH);
    right_doinker.set_value(HIGH);
    chassis.setPose(0, 0, -90);
    drivePID(40, 1000, 70);
    endSection();

    chassis.turnToHeading(-275, 1000, {}, false);
    right_doinker.set_value(LOW);
    endSection();

    chassis.turnToHeading(-320, 1000);
    intake.move(127);
    drivePID(15);
    delay(270);
    intake.brake();
    drivePID(8);
    endSection(100);

    chassis.turnToHeading(-180, 1000, {}, false);
    drivePID(-24);
    clamp.set_value(LOW);
    endSection(500);

    chassis.turnToHeading(-270, 1000, {}, false);
    intake.move(127);
    drivePID(35, 1000);
    endSection();

    chassis.turnToHeading(-190, 1000, {}, false);
    right_doinker.set_value(HIGH);
    drivePID(36, 1000);
    endSection();

    chassis.turnToHeading(-90, 1000, {}, false);
    endSection();

    intake.brake();
    drivePID(20, 1000);
    clamp.set_value(HIGH);
    endSection();

    right_doinker.set_value(LOW);
    chassis.turnToHeading(-225, 1000, {}, false);
    endSection();

    drivePID(-24, 1000, 80);
}
