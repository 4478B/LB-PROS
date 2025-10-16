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
   drivePID(-9,1000);

   chassis.turnToHeading(315,850,{},false);

   drivePID(-20.8,800);
    intakeTop.move(127);
    intake.move(127);
    smallIntake.move(127);

   stopper.set_value(LOW);
    delay(500);


   chassis.turnToHeading(315,850,{},true);
   intakeTop.move(127);
    intake.move(127);
    smallIntake.move(-127);
   delay(1700);
   stopper.set_value(HIGH);

    //intake.move(-127);
    loader.set_value(HIGH);
    drivePID(60.5,2000,30); 
                //53.5 before

   chassis.turnToHeading(273,1400,{},false);
   //smallIntake.move(127);

   drivePID(35,1000,18);
   chassis.turnToHeading(270,500,{},false);
   
   drivePID(-1.5,300);
   drivePID(4,300);
   drivePID(-5,700,100);
   chassis.turnToHeading(270,500,{},false);
   loader.set_value(LOW);
   
   
   drivePID(-30,1000,30);
    stopper.set_value(LOW);
   intake.move(-127);//intake out then in
        intakeTop.move(-127);
        smallIntake.move(127);

    delay(100);

    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);

    chassis.turnToHeading(270,1000,{},true);
    delay(1000);

    stopper.set_value(HIGH);

    drivePID(10,500);
    drivePID(-13,1000,160);


    
}
void fullAWPRight(int i)
{
    chassis.setPose(0,0,116);
    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);
    deScores.set_value(HIGH);
    drivePID(48,2000);
    delay(200);
    drivePID(-33,1500);
}

void testPID(int i){
    chassis.setPose(0,0,0);
    drivePID(48,2000);
    //delay(500);
    chassis.turnToHeading(180,2000,{},false);
    //delay(500);
    drivePID(48,2000);
    //delay(500);
    chassis.turnToHeading(0,2000,{},false);
    //delay(500);
}

void skills(int i){
    chassis.setPose(0,0,0);
    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);
    drivePID(29,1000);//go to loader
    chassis.turnToHeading(270,1000,{},false);
    loader.set_value(HIGH); 
    delay(500);
    drivePID(20,1000,30);//get loader balls
    chassis.turnToHeading(270,400,{},false);
    drivePID(-1.5,500);//shimmy
    drivePID(5,500);
    drivePID(-1.5,500);
    drivePID(5,500);
    drivePID(-5,500);
    chassis.turnToHeading(270,600,{},false);
    loader.set_value(LOW); 
    drivePID(-25,1500);
    stopper.set_value(LOW);

        intake.move(-127);//intake out then in
        intakeTop.move(-127);
        smallIntake.move(127);

    delay(100);

    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);

    drivePID(-2,200);
    chassis.turnToHeading(270,1000,{},false);
    delay(3000);
    stopper.set_value(HIGH);
    drivePID(20,1000);
    chassis.turnToHeading(135,1000,{},false);//go to other side of field
    drivePID(49,1300,30);
    drivePID(-10,1000);
    chassis.turnToHeading(180,1000,{},false);//go to other loader
    drivePID(50,2300,15);
    chassis.turnToHeading(225,1000,{},false);
    drivePID(33,1000);
    chassis.turnToHeading(270,1000,{},false);
    loader.set_value(HIGH); 
    drivePID(-20,1000);
    stopper.set_value(LOW);


            intake.move(-127);//intake out then in
        intakeTop.move(-127);
        smallIntake.move(127);

    delay(100);

    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);


    drivePID(-2,200);
    chassis.turnToHeading(270,1000,{},false);
    delay(1000);
    stopper.set_value(HIGH);
    drivePID(15,1000);
    chassis.turnToHeading(270,1000,{},false);
    delay(500);
    drivePID(20,1500,40);//get loader balls
    chassis.turnToHeading(270,400,{},false);
    drivePID(-1.5,700);//shimmy
    drivePID(5,500);
    drivePID(-1.5,500);
    drivePID(5,500);
    drivePID(-5,500);
    chassis.turnToHeading(270,500,{},false);
    loader.set_value(LOW);
    drivePID(-25,1500,30);//go to score
    chassis.turnToHeading(270,500,{},false);
    stopper.set_value(LOW);
    delay(5000);
    drivePID(21,1000);
    stopper.set_value(HIGH);

    chassis.turnToHeading(150,1000,{},false);//
    drivePID(-18,1000,30);
    chassis.turnToHeading(160,1000,{},false);//
    drivePID(-60,2000,100);
    drivePID(20,1000,100); 

}

