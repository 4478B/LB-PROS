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

void outake(int time){
    intake.move(-127);//intake out then in
    intakeTop.move(-127);
    smallIntake.move(127);

    delay(time);
}
void intakeAll(int time){
    intake.move(127);//intake out then in
    intakeTop.move(127);
    smallIntake.move(-127);

    delay(time);
}
void intakeStop(){
    intake.brake();
    intakeTop.brake();
    smallIntake.brake();
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
void printTimes(int section, int deltaTime, int totalTime, Pose pose)
{
    std::cout << std::setw(10) << section << " | "
              << std::setw(10) << deltaTime << " | "
              << std::setw(10) << totalTime << " | "
              << std::setw(10) << pose.x << " | "
              << std::setw(10) << pose.y << " | "
              << std::setw(10) << pose.theta << " | "
              << std::endl;
}

// Function to handle end of section logic
bool endSection(int delay)
{
    // Functions as normal delay during competition
    if (inCompetition)
    {
        pros::delay(delay);
        return false;
    }
    else
    {
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
        while (pros::millis() - startTime < delay)
        {
            // Timeout override: Break if X button is pressed
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
            {
                break;
            }
            // Alternate path override: Return alternate route indicator if A button is pressed
            else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
            {
                altPath = true;
                break;
            }
            // Heading override: Adjust heading if Y button is pressed
            else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
            {
                pros::lcd::print(1, "Old Heading: %f", chassis.getPose().theta);
                while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
                {
                    pros::lcd::print(2, "New Heading: %f", chassis.getPose().theta);
                    pros::delay(20);
                }
                // Store corrected pose
                Pose poseCorrected = Pose(poseInit.x, poseInit.y, chassis.getPose().theta);
                printTimes(autonSection, 0, 0, poseCorrected);
                break;
            }
            // Position override: Adjust position if B button is pressed
            else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
            {
                float initHeading = poseInit.theta;
                while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
                {
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

void fullAWPLeft(int i)
{
    
   //fullAWPLeft();
   chassis.setPose(0,0,69);
   intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);

   drivePID(48,2000,10);
   drivePID(-8,800);

   chassis.turnToHeading(315,850,{},false);

   drivePID(-18.8,1200,30);
   outake(100);

   stopper.set_value(LOW);
    intakeTop.move(127);
    intake.move(127);
    smallIntake.move(-127);
    


   chassis.turnToHeading(315,850,{},true);
   intakeTop.move(127);
    intake.move(127);
    smallIntake.move(-127);
   delay(1000);
   stopper.set_value(HIGH);

    //intake.move(-127);
    loader.set_value(HIGH);
    drivePID(58.5,2000,30); 
                //53.5 before

   chassis.turnToHeading(273,1400,{},false);
   //smallIntake.move(127);

   drivePID(35,1000,14);
   chassis.turnToHeading(270,500,{},false);
   
   drivePID(-2.5,300);
   drivePID(4,300);
   drivePID(-5,500,100);
   chassis.turnToHeading(270,300,{},false);
   loader.set_value(LOW);
   
   stopper.set_value(LOW);
   intakeStop();

   drivePID(-30,800,30);
    
   outake(100);

    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);

    chassis.turnToHeading(270,1000,{},true);
    delay(1100);

    stopper.set_value(HIGH);

    drivePID(10,500);
    drivePID(-13,1000,160);


    
}
void fullAWPRight(int i)
{
    chassis.setPose(0,0,-70);
   intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);
   stopper.set_value(HIGH);


   drivePID(48,2000,10);
   drivePID(-8.3,800);

   chassis.turnToHeading(-135,850,{},false);

   drivePID(16.1,1200,30);
   intake.move(-127);
   outake(100);

    //delay(2000);

   //stopper.set_value(LOW);
    outake(300);
   outake(500);
   chassis.turnToHeading(-138,850,{},false);


    
    
    drivePID(-55.5,1700,30); //+.5
                //53.5 before
    loader.set_value(HIGH);
    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);
   chassis.turnToHeading(-273,800,{},false);
   //smallIntake.move(127);

   
   drivePID(35,1200,14);
   chassis.turnToHeading(-270,500,{},false);
   
   drivePID(-1.5,300);
   drivePID(4,300);
   drivePID(-5,700,100);
   chassis.turnToHeading(-272,300,{},false);
   //loader.set_value(LOW);
   
   stopper.set_value(LOW);
   intakeStop();

   drivePID(-30,1000,30);
    
   outake(100);

    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);

    chassis.turnToHeading(-268,1000,{},true);
    delay(600);

    intakeStop();

    //stopper.set_value(HIGH);

    drivePID(15,4000,20);
    //drivePID(-13,1000,160);
}

void testPID(int i){
    chassis.setPose(0,0,270);
    drivePID(48,2000);
    //delay(500);
    chassis.turnToHeading(180,2000,{},false);
    //delay(500);
    drivePID(48,2000);
    //delay(500);
    chassis.turnToHeading(0,2000,{},false);
    //delay(500);
}
void fullLocalAWP(int i){
    chassis.setPose(0,0,0);
    loader.set_value(HIGH); 
    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);
    drivePID(29.4,900);//go to loader
    chassis.turnToHeading(-270,600,{},false);
    drivePID(20,900,23);//get loader balls
    //chassis.turnToHeading(-270,400,{},false);
    drivePID(1.5,200);//shimmy 
    drivePID(-5,200,130);
    chassis.turnToHeading(-271,400,{},false);
    
    loader.set_value(LOW); 
    stopper.set_value(LOW);
    //delay(100);
    outake(50);
    intakeStop(); 
    drivePID(-28,800);
    

    outake(50);

    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);

    drivePID(-5,300);
    chassis.turnToHeading(-270,1000,{},false); 
    //delay(500);
    drivePID(17.5,800);
    
    chassis.turnToHeading(-134,700,{},false);//go to other side of field
    stopper.set_value(HIGH);
    intakeAll(1);
    drivePID(52.5,1200,45);
    delay(150);
    outake(1);
    //delay(700);
    chassis.turnToHeading(-134,700,{},false);//go to other side of field
    delay(400);
    drivePID(-15.5,800);

    chassis.turnToHeading(-179,850,{},false);//go to other loader
    intakeAll(1);
    drivePID(51,1300);
    drivePID(-12.5,600,150);
    //outake(1);
    intakeStop();
    chassis.turnToHeading(-231,700,{},false);
    stopper.set_value(LOW);
    intakeAll(1);
    drivePID(-10.5,700,80); //22 if top
   
    
   //drivePID(3,400,150);
}
void tylerAuton(int i){
    chassis.setPose(0,0,-67);//start pose
    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);
    drivePID(41,1250,60);//get 3 mid
    chassis.turnToHeading(-10,750,{},false);
    drivePID(17,650,50);//get 2 under goal
    loader.set_value(HIGH);
    chassis.turnToHeading(-10,850,{},false);
    drivePID(-10,1000,60);
    chassis.turnToHeading(-38,850,{},false);
    drivePID(-16,700);
    loader.set_value(LOW);
    chassis.turnToHeading(-136.5,950,{},false);//turn to mid goal
    outake(50);
    delay(50);
    drivePID(10,600);
    //delay(275); //drop 3 balls mid goal
    //chassis.turnToHeading(-139,300,{},false);
    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);
    drivePID(-53.3,1600);//drive to matchloader
    loader.set_value(HIGH);//matchloader down
    chassis.turnToHeading(-270,1000,{},false);//turn to matchloader
    drivePID(23,1200,25);//drive into matchloader
    chassis.turnToHeading(-270,200,{},false);
    drivePID(-30,800);//go to highgoal
    outake(1);
    drivePID(3,500);
    stopper.set_value(LOW);
    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);
    drivePID(-7,500);
    


}
void skills(int i){
    
    chassis.setPose(0,0,0);
    
    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);
    drivePID(31.8,1500,30);//go to loader
    chassis.turnToHeading(270,1000,{},false);
    loader.set_value(HIGH); 
    delay(500);
    drivePID(20,1200,23);//get loader balls
    chassis.turnToHeading(270,400,{},false);
    drivePID(1.5,500);//shimmy
    drivePID(-3,500);
    drivePID(5,700);//shimmy
    drivePID(-5,500);
    chassis.turnToHeading(269,600,{},false);
    
    loader.set_value(LOW); 
    stopper.set_value(LOW);
    outake(50);
    intakeStop(); 
    drivePID(-25,1500);
    

    outake(100);

    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);

    drivePID(-2,200);
    chassis.turnToHeading(270,1000,{},false);
    delay(3000);
    stopper.set_value(HIGH);
    drivePID(20,1000);
    chassis.turnToHeading(135,1100,{},false);//go to other side of field
    drivePID(49,1300,30);
    drivePID(-10,1000);
    chassis.turnToHeading(180,1000,{},false);//go to other loader
    drivePID(50,2300,15);
    chassis.turnToHeading(223,1400,{},false);

    drivePID(32,1000);
    chassis.turnToHeading(270,1000,{},false);
    outake(50);
    intakeStop();
    loader.set_value(HIGH);
    stopper.set_value(LOW); 
    drivePID(-22,1000);
    


    outake(100);


    intake.move(127);
    intakeTop.move(127); 
    smallIntake.move(-127);


    drivePID(-2,200);
    chassis.turnToHeading(272,1000,{},false);
    delay(1000);
    stopper.set_value(HIGH);
    drivePID(15,1000);
    chassis.turnToHeading(270,1000,{},false);
    delay(500);
    drivePID(22,1500,30);//get loader balls
    chassis.turnToHeading(270,400,{},false);
    drivePID(-1.5,700);//shimmy
    drivePID(5,500);
    drivePID(-1.5,500);
    drivePID(5,500);
    drivePID(-5,500);
    chassis.turnToHeading(268,500,{},false);
    loader.set_value(LOW);
    stopper.set_value(LOW);
    outake(50);
    intakeStop();
    drivePID(-30,1500,30);//go to score
    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);
    chassis.turnToHeading(270,500,{},false);
    drivePID(-2,200);
    delay(4500);
    
    drivePID(17,1000);
    stopper.set_value(HIGH);
    chassis.turnToHeading(145,1000,{},false);//
    outake(1000);
    drivePID(-28.67,1000,30);
    chassis.turnToHeading(157,1000,{},false);//
    drivePID(-45,2000,50);
    drivePID(8,1000,100); 

}

