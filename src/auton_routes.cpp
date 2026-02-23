/**
 * @file auton_routes.cpp
 * @brief All autonomous routines and motion helper functions.
 *
 * This file is the main body of auton code. It contains:
 *   - Helper wrappers (drivePIDOdom, outake, intakeAll, intakeStop) that simplify
 *     calling the raw motion / intake APIs.
 *   - endSection() – a development tool for splitting and timing auton sections.
 *   - Every competition and skills autonomous routine.
 *
 * Typical auton structure:
 *   1. chassis.setPose(x, y, theta)     – set starting position for odometry
 *   2. intake / mechanism setup
 *   3. Sequence of drivePID() / chassis.turnToHeading() / chassis.moveToPose() calls
 *   4. Mechanism actions between movements (outake, stopper, loader, etc.)
 *
 * Motion primitives used:
 *   drivePID(inches, timeout, kP)      – straight drive via motor encoders (old_systems.h)
 *   chassis.turnToHeading(deg, ms)     – turn to absolute heading using LemLib
 *   chassis.moveToPose(x, y, θ, ms)   – curve to a pose using LemLib odometry
 *   drivePIDOdom(inches)               – straight drive computed from current odometry pose
 */

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
#include "opticalAlign.h"
#include "pros/rtos.h"
#include "testing.h"
#include <iomanip>

// Backwards-compatibility helpers (originally written for VEXCode-style autons)

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

// ─── Intake Shorthand Helpers ──────────────────────────────────────────────────

/** Run all intake motors in reverse for `time` ms (ejects balls). */
void outake(int time){
    intake.move(-127);
    intakeTop.move(-127);
    smallIntake.move(127);
    delay(time);
}

/** Run bottom intake and small intake forward for `time` ms (intakes balls). */
void intakeAll(int time){
    intake.move(127);
    smallIntake.move(-127);
    delay(time);
}

/** Immediately stop all intake motors. */
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

void park(){
    chassis.setPose(28.569,-48.805,90);
    chassis.moveToPose(58.844,-19.425,4,2000,{.forwards=true, .lead=.5},false);
    intakeAll(1);
    double initial = imu.get_roll();
    double goal=initial+5; //tune the addition to make it work
    while(imu.get_roll()<goal){
        all_motors.move(80);
    }
    while(imu.get_roll()>initial+1){
        all_motors.move(80);
    }
    while(imu.get_roll()<goal){
        all_motors.move(80);
    }
    while(imu.get_roll()>initial+1){
        all_motors.move(80);
    }
    all_motors.brake();
    //drivePID(80,2000,13);
    //drivePID(-35,1000,10);
}

// ─── Autonomous Routines ──────────────────────────────────────────────────────

/**
 * fullAWPLeft – Full Autonomous Win Point route from the left side of the field.
 * Starts at heading 69°. Scores in mid goal, loads balls from the match-loader,
 * and ends scoring in the high goal.
 */
void fullAWPLeft(int i)
{
    
   //fullAWPLeft();
   chassis.setPose(0,0,69);
   intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);

   drivePID(47,2000,10);
   drivePID(-9,800);

   chassis.turnToHeading(315,850,{},false);

   drivePID(-16.8,1200,30);
    outake(40);

   stopper.set_value(LOW);
    intakeTop.move(127);
    intake.move(127);
    smallIntake.move(-127);
    
 

   chassis.turnToHeading(315,200,{},true);
   intakeTop.move(127);
    intake.move(127);
    smallIntake.move(-80);
   delay(170);
   stopper.set_value(HIGH);

    //intake.move(-127);
    loader.set_value(HIGH);
    drivePID(54.3,2000,30); 
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
    stopper.set_value(LOW);

    drivePID(-13,1000,160);


    
}
/**
 * fullAWPRight – Full AWP route from the right side of the field.
 * Mirror of fullAWPLeft, starting at heading -70°.
 */
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
   //chassis.turnToHeading(-272,300,{},false);
   alignToLongGoal(-272,false);
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

/** testPID – Simple 48-inch drive + 180° turn used to tune PID constants. */
void testPID(int i){
    chassis.setPose(0,0,0);
    drivePID(48,2000);
    chassis.turnToHeading(180,2500,{},false);
    //delay(500);
    //drivePID(48,2000);
    //delay(500);
    //chassis.turnToHeading(0,2500,{},false);
    //delay(500);
}
/**
 * fullLocalAWP – Local-coordinate AWP (no odometry moveToPose).
 * Uses drivePID + turnToHeading only. Scores loader balls, crosses field, repeats.
 * Superseded by newfullLocalAWP.
 */
void fullLocalAWP(int i){
    chassis.setPose(0,0,0);
    loader.set_value(HIGH); 
    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);
    drivePID(29,950);//go to loader
    chassis.turnToHeading(-270,700,{},false);
    drivePID(20,900,23);//get loader balls
    //chassis.turnToHeading(-270,400,{},false);
    drivePID(1.5,200);//shimmy 
    drivePID(-5,200,130);
    chassis.turnToHeading(-271,400,{},false);

    //alignToLongGoal(-273,false);

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

    chassis.turnToHeading(-270,1000,{},false); 
    drivePID(-5,300);
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

    chassis.turnToHeading(-177,850,{},false);//go to other loader
    intakeAll(1);
    drivePID(51,1300);
    drivePID(-10.7,600,150);
    //outake(1);
    intakeStop();
    chassis.turnToHeading(-231,700,{},false);
    stopper.set_value(LOW);
    //intakeAll(1);
    drivePID(-12.5,700,80); //22 if top
    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-40);

   
    
   //drivePID(3,400,150);
}
/**
 * newfullLocalAWP – Current primary competition AWP route (left side, heading 69°).
 * Sequence: intake mid balls → clamp mobile goal → cross field →
 *           load match-loader balls → score in high goal.
 * Uses drivePID + turnToHeading (no odometry moveToPose).
 */
void newfullLocalAWP(int i){
 chassis.setPose(0,0,69);
   intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);

    drivePID(41,1050,60);
    drivePID(-14,800);

   chassis.turnToHeading(315,850,{},false);

   drivePID(-15.3,900);
   outake(70);

   stopper.set_value(LOW);
    intakeTop.move(127);
    intake.move(127);
    smallIntake.move(-127);
    
 

   chassis.turnToHeading(315,200,{},true);
   intakeTop.move(127);
    intake.move(127);
    smallIntake.move(-80);
   delay(170);
   stopper.set_value(HIGH);

    //intake.move(-127);
    //loader.set_value(HIGH);
    drivePID(15.5,800);
    chassis.turnToHeading(182,850,{},false);
    drivePID(49,1300);
    drivePID(-8,700,150);
    chassis.turnToHeading(48,850,{},false);
    outake(1);
    drivePID(13,800);
    //outake(100);
    //intakeAll(1); 
    outake(200);
    intakeAll(1);
    drivePID(-52,1450);//drive to matchloader
    loader.set_value(HIGH);//matchloader down 
    chassis.turnToHeading(270,850,{},false);//turn to matchloader
    drivePID(25,1000,21);//drive into matchloader
    chassis.turnToHeading(273,400,{},false);
    drivePID(-30,800);//go to highgoal
    outake(50);
    drivePID(3,100);
    stopper.set_value(LOW);
    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);
    drivePID(-7,500);
}
/**
 * odomAWP – Odometry-based AWP using moveToPose for curved paths.
 * Runs two match-loader cycles using globally-tracked coordinates.
 * Recalibrates pose mid-route to correct for odometry drift.
 */
void odomAWP(int i){
    chassis.setPose(0,0,180);
    loader.set_value(HIGH); 
    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);
    drivePID(29.3,950);//go to loader
    chassis.turnToHeading(270,1000,{},false);
    drivePID(20,900,23);//get loader balls
    //chassis.turnToHeading(-270,400,{},false);
    drivePID(1.5,200);//shimmy 
    drivePID(-5,200,130);
    chassis.turnToHeading(270,400,{},false);

    //alignToLongGoal(-273,false);

    loader.set_value(LOW); 
    stopper.set_value(LOW);
    //delay(100);
    outake(50);
    intakeStop(); 
    drivePID(-28,800);
    

    //outake(50);

    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);
    drivePID(-10,300,80);
    chassis.turnToHeading(270,600,{},false); 
    drivePID(-10,100);
    delay(100);
    intakeStop();
    chassis.setPose(-27.397, -47.202,chassis.getPose().theta); // recalibrate pose after odom drift
    chassis.moveToPose(-34.936,-36.884,45,2500,{.forwards=true, .lead=.9, .minSpeed=50},false);
    stopper.set_value(HIGH);
    intakeAll(1);
    chassis.moveToPose(-6.762,-7.52,45,2000,{.forwards=true, .lead=.5, .minSpeed=50},false);
    outake(500);
    intakeAll(1);
    chassis.moveToPose(-21.974,-22.202,0,2500,{.forwards=false, .lead=.5, .minSpeed=70},false);
    chassis.moveToPose(-49.75,43.549,270,2700,{.forwards=true, .lead=.52},false);
    loader.set_value(HIGH);
    chassis.turnToHeading(270,200,{},false);
    
    drivePID(25,800,23);//get loader balls
    //chassis.turnToHeading(-270,400,{},false);
    drivePID(1.5,300);//shimmy 
    drivePID(-5,200,130);
    chassis.turnToHeading(273,250,{},false);

    //alignToLongGoal(-273,false);

    loader.set_value(LOW); 
    stopper.set_value(LOW);
    //delay(100);
    outake(50);
    intakeStop(); 
    drivePID(-28,800);
    

    //outake(50);

    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);

    drivePID(-10,300);
    chassis.turnToHeading(270,1000,{},false); 
    

    
}
/**
 * odomAWPHigh – Odometry AWP that also targets the elevated high goal.
 * Uses moveToPose for approach curves and a spawned background task to
 * lower the loader at the right moment during motion.
 */
void odomAWPHigh(int i){
    chassis.setPose(0,0,0);
    stopper.set_value(LOW);
    lift.set_value(HIGH);
    //deScores.set_value(HIGH);
    intake.move(127);
    intakeTop.move(20);
    //intakeTop.move(127);
    //smallIntake.move(-127);
    drivePID(5,500,100);
    chassis.turnToHeading(0,100,{},false);
    drivePID(-47.3,1000);//go to loader //29.3
    loader.set_value(HIGH); 
    chassis.turnToHeading(270,500,{},false);
    drivePID(32,800,23);//get loader balls
    //chassis.turnToHeading(-270,400 ,{},false);
    drivePID(1.5,200);//shimmy 
    drivePID(-5,200,130);
    chassis.turnToHeading(272,300,{},false);

    //alignToLongGoal(-273,false);

    loader.set_value(LOW); 
    //delay(100);
    //outake(50);
    //intakeStop(); 
    drivePID(-28,600);
    


    //outake(50);

    intake.move(127);
    intakeTop.move(127);
    //smallIntake.move(-127);
    stopper.set_value(HIGH);
    drivePID(-10,300,160);
    chassis.turnToHeading(270,200,{},false); 
    drivePID(-10,300);
    // recalibrate pose after odom drift
    intakeStop();
    chassis.turnToHeading(11,900,{},false);
    
    chassis.setPose(-26.603, -39.53,chassis.getPose().theta); 
    //chassis.moveToPose(-34.936,-36.884,45,2500,{.forwards=true, .lead=.9, .minSpeed=50},false);
    stopper.set_value(LOW);
    intakeTop.brake();
    intake.move(127);
    pros::Task IntakeTask([]
                         { delay(1300);
                        loader.set_value(HIGH);});
    chassis.moveToPose(-18.942,30.97,0,1800,{.forwards=true, .lead=.1},false);
    loader.set_value(HIGH);
    //lift.set_value(LOW);
    intake.brake();
    //outake(500);
    //stopper.set_value(HIGH);
    chassis.moveToPose(-8.879,6.897,315,1500,{.forwards=false, .lead=.3},false);
   outake(100);
    intakeTop.move(-50);
    intake.move(80);
    


    chassis.turnToHeading(315,300,{},false);
    delay(150);
    lift.set_value(HIGH);
    chassis.setPose(-10.731,9.939,chassis.getPose().theta);
    delay(100);
    //stopperTwo.set_value(LOW);
    intake.brake();
    intakeTop.brake();
    stopper.set_value(LOW);
    

    //smallIntake.move(-127);
    //frontGate.set_value(HIGH);
    //drivePID(-5,300);
    
    chassis.moveToPose(-45.75,45.849,315,1200,{.forwards=true, .lead=.0},false);
    //drivePID(50,1200,50);
    intake.move(127); 
    intakeTop.move(10);
    chassis.turnToHeading(270,500,{},false);
    
    drivePID(25,800,23);//get loader balls
    //chassis.turnToHeading(-270,400,{},false);
    drivePID(1.5,300);//shimmy 
    drivePID(-5,200,130);
    chassis.turnToHeading(273,250,{},false);

    //alignToLongGoal(-273,false);

    loader.set_value(LOW); 
    
    //delay(100); 
    drivePID(-35,600,100);
    stopper.set_value(HIGH);
    //outake(50);

    intake.move(127);
    intakeTop.move(127);
    //smallIntake.move(-127);

    //stopper.set_value(LOW);
    intakeTop.move(127);

    drivePID(-10,500,150);
    chassis.turnToHeading(270,1000,{},false); 
    
    
    
}
/**
 * tylerAuton – Elimination 9-ball route starting at heading -67°.
 * Picks up 3 mid balls, 2 balls under the goal, loads from match-loader,
 * and scores in the high goal. Designed for elimination rounds.
 */
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
    outake(150);
    delay(50);
    drivePID(10,670);
    //delay(275); //drop 3 balls mid goal
    //chassis.turnToHeading(-139,300,{},false);
    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);
    drivePID(-51.5,1600);//drive to matchloader
    loader.set_value(HIGH);//matchloader down
    chassis.turnToHeading(-270,1000,{},false);//turn to matchloader
    drivePID(23,1200,25);//drive into matchloader
    chassis.turnToHeading(-271,200,{},false);
    drivePID(-30,800);//go to highgoal
    outake(20);
    drivePID(3,100);
    stopper.set_value(LOW);
    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);
    drivePID(-7,500);
}
/**
 * skills – Original 60-second skills run (older version).
 * Two full match-loader cycles + cross-field traversal.
 * Superseded by skillsNew.
 */
void skills(int i){
    
    chassis.setPose(0,0,0);
    
    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);
    drivePID(30.5,1500,30);//go to loader
    chassis.turnToHeading(270,1000,{},false);
    loader.set_value(HIGH); 
    delay(500);
    drivePID(20,1200,23);//get loader balls
    chassis.turnToHeading(270,400,{},false);
    drivePID(1.5,500);//shimmy
    drivePID(-3,500);
    drivePID(5,700);//shimmy
    drivePID(-5,500);
    chassis.turnToHeading(271,600,{},false);
    
    loader.set_value(LOW); 
    stopper.set_value(LOW);
    outake(50);
    intakeStop(); 
    drivePID(-25,1500);
    

    outake(200);

    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);

    drivePID(-5,500);
    chassis.turnToHeading(270,1000,{},false);
    delay(3000);
    drivePID(20,1000);
    stopper.set_value(HIGH);
    chassis.turnToHeading(135,1100,{},false);//go to other side of field
    drivePID(49,1300,30);
    drivePID(-10,1000);
    chassis.turnToHeading(180,1000,{},false);//go to other loader
    drivePID(50,2300,15);
    chassis.turnToHeading(223,1400,{},false);

    drivePID(32,1000);
    chassis.turnToHeading(270,1000,{},false);
    outake(150);
    intakeStop();
    loader.set_value(HIGH);
    stopper.set_value(LOW); 
    drivePID(-22,1000);
    


    outake(200);


    intake.move(127);
    intakeTop.move(127); 
    smallIntake.move(-127);


    drivePID(-5,500);
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
    outake(100);
    intakeStop();
    drivePID(-30,1500,30);//go to score
    intake.move(127);
    intakeTop.move(127);
    smallIntake.move(-127);
    chassis.turnToHeading(270,500,{},false);
    drivePID(-5,500);
    delay(4500);
    
    drivePID(17,1000);
    stopper.set_value(HIGH);
    chassis.turnToHeading(145,1000,{},false);//
    outake(1000);
    drivePID(-28.67,1000,30);
    chassis.turnToHeading(157,1000,{},false);//
    drivePID(-60,2000,150);
    drivePID(8,1000,100); 

}

/**
 * skillsNew – Current full-field 60-second skills run using odometry (moveToPose).
 * Starts at (-47.105, -12.737, 180°). Executes four match-loader cycles,
 * traverses the full field twice, and scores in all four goals.
 * Uses chassis.setPose() mid-route to re-anchor odometry after long straight drives.
 */
void skillsNew(int i){
    chassis.setPose(-47.105,-12.737,180);
    //stopperTwo.set_value(LOW);
    lift.set_value(HIGH);
    /*
    //chassis.moveToPose(-13.89,-20.233,90,2500,{.forwards=true,.lead=.5},false);
    //chassis.moveToPose(-49.761,-45.334,90,2500,{.forwards=false, .lead=.01},false);
    //chassis.turnToHeading(270,1000,{},false);
    intake.move(127);
    //intakeTop.move(127);
    smallIntake.move(-127);
    deScores.set_value(HIGH);
    
    frontGate.set_value(HIGH);

    chassis.moveToPose(-10.598,-26.964,120,2000,{.forwards=true,.lead=.2},false);
    chassis.moveToPose(-47.766,-48.166,270,2000,{.forwards=true,.lead=.1},false);
    drivePID(-25,800);
    intake.move(127);
    intakeTop.move(127);
    stopper.set_value(HIGH);
    //go to loader
    chassis.turnToHeading(270,800,{},false);
    drivePID(-10,500);
    stopper.set_value(LOW);
    intakeTop.brake();
    drivePID(20,1000);
    intakeAll(1);*/
    intake.move(127);
    deScores.set_value(HIGH);
    loader.set_value(HIGH); 
    intakeTop.move(10);
    drivePID(31.7,1000,40);
    chassis.turnToHeading(270,800,{},false);
    drivePID(20,1200,23);//get loader balls
    chassis.turnToHeading(270,400,{},false);
    drivePID(1.5,500);//shimmy
    drivePID(-3,500);
    drivePID(5,700);//shimmy
    drivePID(-5,500);
    //chassis.turnToHeading(271,600,{},false);
    
    
    //outake(100);
    //stopper.set_value(LOW);
    chassis.setPose(-58.761,-47.334,chassis.getPose().theta); //-47.334 , -45.334
    delay(100);

    chassis.moveToPose(-26.868,-58.222,270,2000,{.forwards=false},false);
    loader.set_value(LOW); 
    chassis.moveToPose(45.066,-59.958,270,2500,{.forwards=false, .lead=.5},false);
    chassis.moveToPose(25.57,-49.305,90,3000,{.forwards=false, .lead=.5},false);//-48.805
    loader.set_value(HIGH); 

    //outake(200);
    stopper.set_value(HIGH); 
    outake(120);
    intake.move(127);
    intakeTop.move(127);
    //intakeTop.move(127);
    drivePID(-10,200,100);
    chassis.turnToHeading(90,2000,{},false);
    delay(1000);
    loader.set_value(HIGH); 
    drivePID(5,1000);
    stopper.set_value(LOW);
    intakeTop.move(10);
    chassis.turnToHeading(90,500,{},false);
    //delay(500);
    intake.move(127);
    drivePID(32,1200,25);//get loader balls
    chassis.turnToHeading(90,400,{},false);
    drivePID(-1.5,700);//shimmy
    drivePID(5,500);
    drivePID(-1.5,500);
    drivePID(5,500);
    drivePID(-5,500);
    chassis.turnToHeading(90,500,{},false);
    loader.set_value(LOW);
    //outake(100);
    //intakeStop();
    drivePID(-30,1500,50);//go to score
    //outake(200);
    stopper.set_value(HIGH);
    //outake(200);
    intake.move(127);
    intakeTop.move(127);
    //smallIntake.move(-127);
    chassis.turnToHeading(90,500,{},false);
    drivePID(-5,500);
    delay(1300);
    //outake(5);
    //intakeStop();
    //drivePID(10,450);
    stopper.set_value(LOW);
    intakeTop.move(10);
    drivePID(10,400);
    drivePID(-15,500);
    delay(100);
    chassis.setPose(28.569,-48.805,chassis.getPose().theta);
    delay(100);
    chassis.moveToPose(43.5,-46.54,0,600,{.forwards=true, .lead=.5},false);
    chassis.moveToPose(40.913,-24.716,0,1500,{.forwards=true, .lead=.5},false);
    stopper.set_value(LOW);
    intakeTop.move(10);

    intake.move(127);
    loader.set_value(HIGH);
    chassis.moveToPose(41.637,49.143,0,1900,{.forwards=true, .lead=.0},false);//47.637
    chassis.turnToHeading(90,500,{},false); 
    /*intakeAll(1);
    chassis.moveToPose(17.046,-18.234,315,2400,{.forwards=true, .lead=.1},false);//47.637
    chassis.turnToHeading(135,800,{},false);
    chassis.moveToPose(8.301,-8.621,135,1500,{.forwards=false, .lead=.0},false);
    outake(200);
    intakeStop();
    stopperTwo.set_value(HIGH);
    intakeAll(1000);
    stopperTwo.set_value(LOW);

    chassis.moveToPose(22.073,26.045,0,2400,{.forwards=true, .lead=.5},false);//47.637
    chassis.moveToPose(42.178,46.843,90,2400,{.forwards=true, .lead=.1},false);//47.637
    drivePID(-25,1000);
    stopper.set_value(HIGH);
    loader.set_value(HIGH);
    delay(700);//47.637
    chassis.turnToHeading(90,200,{},false);
    stopper.set_value(LOW);
    drivePID(20,1000);*/
    
    drivePID(30,1200,25);//get loader balls ////////////////CUT FROM HERE 
    chassis.turnToHeading(90,400,{},false);
    drivePID(-1.5,700);//shimmy
    drivePID(5,500);
    drivePID(-1.5,500);
    drivePID(5,500);
    chassis.turnToHeading(90,500,{},false);
    chassis.setPose(56.761,48.76,chassis.getPose().theta); //46.711 , 47.211
    delay(100);
    chassis.moveToPose(26.868,60.222,90,2000,{.forwards=false},false);
    loader.set_value(LOW);
    chassis.moveToPose(-44.066,61.958,90,2000,{.forwards=false, .lead=.5},false);
    chassis.moveToPose(-24.57,49.805,270,3000,{.forwards=false, .lead=.5},false);
    //outake(200);
    stopper.set_value(HIGH); 
    outake(120);
    intake.move(127);
    intakeTop.move(127);
    drivePID(-10,500,100);
    chassis.turnToHeading(270,1000,{},false);
    delay(1000);
    loader.set_value(HIGH); 
    
    drivePID(15,1000);
    stopper.set_value(LOW);
    intakeTop.move(10);

    chassis.turnToHeading(272,500,{},false);
    //delay(500);
    drivePID(22,1000,30);//get loader balls
    chassis.turnToHeading(272,400,{},false);
    drivePID(-1.5,700);//shimmy
    drivePID(5,500);
    drivePID(-5,500);
    chassis.turnToHeading(271,500,{},false);
    loader.set_value(LOW);
    //outake(100);
    drivePID(-35,1200,50);//go to score
    //outake(200);
    stopper.set_value(HIGH);

    outake(100);

    //outake(200);
    intake.move(127);
    intakeTop.move(127);
    //smallIntake.move(-127);
    chassis.turnToHeading(270,500,{},false);
    drivePID(-10,500);
    delay(1500); 
    //outake(5);
    //intakeStop();
    //drivePID(10,450);
    stopper.set_value(LOW);
    intakeTop.brake();
    drivePID(10,400);
    drivePID(-15,500);
    delay(100);

    //drivePID(-15,500);
    //delay(100);
    chassis.setPose(-25.57,48.805,chassis.getPose().theta);
    //intakeAll(1);
    chassis.moveToPose(-58.127,29.633,190,1700,{.forwards=true, .lead=.5},false);
    stopper.set_value(HIGH);
    intakeTop.move(127);
    //outake(1);
    loader.set_value(HIGH);
    drivePID(67,2500,100);
    drivePID(-7,1500,100);

    /*
   chassis.setPose(-49.618,-17.441,180);
   loader.set_value(HIGH); 
    chassis.moveToPose(-58.761,-47.334,270,5000,{.earlyExitRange=0},false);
    delay(2000);
    //drivePID(10,2000,30);
    chassis.moveToPose(-28.587,-47.466,270,5000,{.forwards=false,.lead=.5,.minSpeed=60},false);*/


}
/**
 * leftPush – 7-ball match routine from the left side.
 * Loads balls, scores high goal, then pushes remaining balls via odometry moveToPose.
 */
void leftPush(int i){
    chassis.setPose(0,0,0);
    loader.set_value(HIGH); 
    lift.set_value(HIGH);
    intake.move(127);
    //intakeTop.move(127);
    //smallIntake.move(-127);
    //frontGate.set_value(HIGH);

    deScores.set_value(HIGH);
    drivePID(29.3,730,70);//go to loader //29.3
    chassis.turnToHeading(270,400,{},false);
    deScores.set_value(LOW);
    drivePID(20,830,26);//get loader balls
    //chassis.turnToHeading(-270,400,{},false);
    drivePID(1.5,200);//shimmy 
    drivePID(-5,200,130);
    chassis.turnToHeading(270,100,{},false);

    //alignToLongGoal(-273,false);

    loader.set_value(LOW); 
    //delay(100);
    //outake(50);
    //intakeStop(); 
    drivePID(-28,600);
    


    //outake(50);

    intake.move(127);
    intakeTop.move(127);
    //smallIntake.move(-127);
    stopper.set_value(HIGH);
    drivePID(-10,300,160);
    chassis.turnToHeading(270,200,{},false); 
    drivePID(-10,200);
    //intakeStop();// recalibrate pose after odom drift
    chassis.turnToHeading(180,900,{},false);
    stopper.set_value(LOW);
    intakeTop.brake();
    intakeAll(1);
    chassis.setPose(-26.603, 39.53,chassis.getPose().theta); 
    chassis.moveToPose(-23.296,11.262,180,1000,{.forwards=true, .lead=.1},false);
    lift.set_value(LOW);
    chassis.moveToPose(-22.899,22.902,180,700,{.forwards=false, .lead=.0},false);
    chassis.moveToPose(-9.143,8.088,315,1000,{.forwards=false, .lead=.2},false);
    //outake(60);
    //frontGate.set_value(LOW);
    stopper.set_value(HIGH);
    intakeTop.move(127);
    intake.move(100);
    smallIntake.move(80);
    chassis.turnToHeading(315,300,{},false);
    delay(200);
    
    lift.set_value(HIGH); 
      
    
    
    chassis.moveToPose(-23.458,31.758,45,1100,{.forwards=true, .lead=.5},false);
    intakeTop.brake();
    stopper.set_value(LOW); 
    intake.brake();
    chassis.moveToPose(-2.93,40.265,90,1100,{.forwards=true, .lead=.5},false);
    //chassis.turnToHeading(120,600,{},false);
    chassis.turnToHeading(150,700,{.minSpeed=110},false);


}
/**
 * rightPush – 7-ball match routine from the right side.
 * Mirror of leftPush; starts at heading 180°.
 */
void rightPush(int i){
    chassis.setPose(0,0,180);
    loader.set_value(HIGH); 
    lift.set_value(HIGH);
    intake.move(127);
    intakeTop.move(15);
    //intakeTop.move(127);
    //smallIntake.move(-127);
    //frontGate.set_value(HIGH);

    deScores.set_value(HIGH);

    drivePID(27.2,700);//go to loader //29.3
    deScores.set_value(LOW);
    chassis.turnToHeading(270,500,{},false);
    drivePID(20,700,23);//get loader balls
    //chassis.turnToHeading(-270,400,{},false);
    drivePID(1.5,200);//shimmy
    drivePID(-5,200,130);
    chassis.turnToHeading(270,200,{},false);

    //alignToLongGoal(-273,false);

    loader.set_value(LOW); 
    //delay(100);
    //outake(50);
    //intakeStop(); 
    drivePID(-28,600);
    


    //outake(50);

    intake.move(127);
    intakeTop.move(127);
    //smallIntake.move(-127);
    stopper.set_value(HIGH);
    drivePID(-10,300,160);
    chassis.turnToHeading(270,200,{},false); 
    drivePID(-10,200);
    // recalibrate pose after odom drift
    chassis.turnToHeading(8,900,{},false);
    chassis.setPose(-26.603, -39.53,chassis.getPose().theta); 
    intakeTop.move(10);
    intake.move(127);
    //chassis.moveToPose(-34.936,-36.884,45,2500,{.forwards=true, .lead=.9, .minSpeed=50},false);
    stopper.set_value(LOW);
    chassis.moveToPose(-22.899,-17.97,45,1000,{.forwards=true, .lead=.1},false);
    chassis.moveToPose(-9.54,-10.166,45,1000,{.forwards=true, .lead=.5},false);
    intake.move(-80);
    intakeTop.move(-127);
    delay(1000);
    chassis.moveToPose(-27.661,-36.207,315,1000,{.forwards=false, .lead=.5},false);
    intakeAll(1);
    chassis.moveToPose(-2.63,-38.965,270,1600,{.forwards=false, .lead=.5},false);
    chassis.turnToHeading(225,500,{},false);

}
/**
 * skillsFinal – Alternate full-field skills variant.
 * Starts at heading 270°. Performs multi-cycle loader routine with
 * carefully timed intake reversals to seat balls correctly, then
 * mirrors skillsNew's cross-field traversal.
 */
void skillsFinal(int i){
    chassis.setPose(0,0,270);
    lift.set_value(HIGH);
    intake.move(127);
    intakeTop.move(15);
    loader.set_value(HIGH);
    /*
    drivePID(-5,500);
    loader.set_value(HIGH);
    delay(300);
     pros::Task IntakeTask([]
                         { delay(300);
                        loader.set_value(LOW);});
    drivePID(80,2500,70);
    chassis.turnToHeading(180,800,{},false);
   
    drivePID(-45,1600,7);
    loader.set_value(LOW);
    chassis.setPose(-62.669,-18.087,chassis.getPose().theta);
    delay(300);
    chassis.turnToHeading(90,1000,{},false);
    chassis.setPose(-57.981,-20.233,chassis.getPose().theta);

    chassis.moveToPose(-28.411,-17.426,115,1500,{.forwards=true, .lead=.1},false);
    intake.brake();
    //chassis.moveToPose(-17.476,0,340,1500,{.forwards=true, .lead=.1},false);
    chassis.turnToHeading(0,1000,{},false);
    intake.move(127);
    chassis.moveToPose(-20.519,21.05,340,1500,{.forwards=true, .lead=.1},false);
    loader.set_value(HIGH);
    delay(200);
    intake.brake();
    lift.set_value(LOW);
    chassis.turnToHeading(315,500,{},false); 
    intake.move(127);*/
    delay(200);
    loader.set_value(LOW);

    delay(200);   
    drivePID(13,500);
    
    drivePID(-8,500);
    
    delay(200);
    
    intake.move(-127);
    delay(200);
    intake.move(127);

    
    drivePID(15,500);
    drivePID(-6,500);
    
    delay(200);
    delay(200);
    intake.move(-127);
    
    delay(120);
    intake.move(127);
    
    drivePID(15,500);
    drivePID(-6,500);
    
    delay(400);
    intake.move(-127);
    delay(200);
    intake.move(127);
    drivePID(10,500);
    chassis.turnToHeading(270,300,{},false);
    drivePID(-36,1400,25);
    chassis.turnToHeading(270,300,{},false);
    drivePID(30,1500,8);
    chassis.setPose(-44.592,0,chassis.getPose().theta); //46.711 , 47.211
    chassis.moveToPose(-17.476,0,340,1500,{.forwards=false, .lead=.1},false);
    //lift.set_value(LOW);
    chassis.moveToPose(-20.919,21.05,330,1500,{.forwards=true, .lead=.1},false);
    loader.set_value(HIGH);
    chassis.turnToHeading(315,400,{},false);
    chassis.moveToPose(-11.392,6.22,315,1500,{.forwards=false, .lead=.1},false);
    drivePID(5,300);
    intake.move(-80);
    intakeTop.move(-127);
    chassis.turnToHeading(315,300,{},false);
    //stopper.set_value(HIGH);
    intake.move(80);
    intakeTop.move(-30);
    delay(500);
    intake.move(-127);
    intakeTop.move(-127);
    delay(160);
    intake.move(60);
    intakeTop.move(-20);
    delay(600);
    intake.move(-127);
    intakeTop.move(-127);
    delay(160);
    intake.move(60);
    intakeTop.move(-20);
    delay(1500);
    lift.set_value(HIGH);
    drivePID(-5,400);
    stopper.set_value(LOW);

    /*drivePID(20,800);
    drivePID(-10,500);
    drivePID(40,700);
    delay(200);
    drivePID(-10,700);
    delay(200);
    drivePID(30,700);
    delay(200);
    drivePID(-30,900);
    delay(200);
    chassis.turnToHeading(90,1100,{},false);
    drivePID(-20,1000,10);
    chassis.setPose(-44.195,0,chassis.getPose().theta); //46.711 , 47.211
    chassis.moveToPose(-17.476,0,340,1500,{.forwards=true, .lead=.1},false);
    chassis.moveToPose(-20.519,21.05,340,1500,{.forwards=true, .lead=.1},false);
    chassis.turnToHeading(315,500,{},false);    
    chassis.moveToPose(-9.408,8.352,315,1500,{.forwards=false, .lead=.2},false);
    stopper.set_value(HIGH);
    intakeTop.move(60);
    delay(3000);
    stopper.set_value(LOW);*/


    chassis.setPose(-10.731,9.939,chassis.getPose().theta);
    loader.set_value(HIGH);
    lift.set_value(HIGH);
    chassis.moveToPose(-43.93,44.549,270,1550,{.forwards=true, .lead=.0},false);
    deScores.set_value(HIGH);
    chassis.turnToHeading(270,200,{},false);
    intakeTop.move(10);
    drivePID(25,800,23);
    drivePID(-1.5,700);//shimmy
    drivePID(5,500);
    drivePID(-1.5,500);
    chassis.setPose(-55.613,46.182, chassis.getPose().theta);
    chassis.moveToPose(-26.868,61.222,270,2000,{.forwards=false},false);
    loader.set_value(LOW);
    chassis.moveToPose(51.066,61.958,270,2000,{.forwards=false, .lead=.5},false);
    chassis.moveToPose(21.57,49.305,90,2000,{.forwards=false, .lead=.5},false);
    stopper.set_value(HIGH); 
    intake.move(127);
    intakeTop.move(127);
    //intakeTop.move(127);
    drivePID(-10,200,100);
    chassis.turnToHeading(90,1200,{},false);
    drivePID(-10,1000);
    loader.set_value(HIGH); 
    drivePID(5,1000);
    stopper.set_value(LOW);
    intakeTop.brake();
    chassis.turnToHeading(90,500,{},false);
    //delay(500);
    intake.move(127);
    drivePID(32,1200,20);//get loader balls
    chassis.turnToHeading(90,400,{},false);
    drivePID(-1.5,700);//shimmy
    drivePID(5,500);
    drivePID(-1.5,500);
    chassis.turnToHeading(90,500,{},false);
    loader.set_value(LOW);
    drivePID(-30,1500,50);//go to score
    stopper.set_value(HIGH);
    //outake(200);
    intake.move(127);
    intakeTop.move(127);
    chassis.turnToHeading(90,500,{},false);
    drivePID(-10,1100);

    
    drivePID(10,400);
    stopper.set_value(LOW);
    drivePID(-15,500);

    delay(100);
    chassis.setPose(28.569,48.805,chassis.getPose().theta);
    delay(100);
    chassis.moveToPose(43.5,46.54,180,600,{.forwards=true, .lead=.5},false);
    chassis.moveToPose(40.913,24.716,180,1300,{.forwards=true, .lead=.5},false);
    stopper.set_value(LOW);
    intakeTop.brake();

    intake.move(127);
    loader.set_value(HIGH);
    chassis.moveToPose(47.637,-47.043,180,2400,{.forwards=true, .lead=.0},false);
    chassis.turnToHeading(90,800,{},false);
    
    drivePID(30,1200,25);//get loader balls ////////////////CUT FROM HERE 
    drivePID(-1.5,700);//shimmy
    drivePID(5,500);
    drivePID(-1.5,500);
    chassis.turnToHeading(90,200,{},false);
    chassis.setPose(56.761,-48.76,chassis.getPose().theta); //46.711 , 47.211
    delay(100);
    chassis.moveToPose(26.868,-61.222,90,2000,{.forwards=false},false);
    loader.set_value(LOW);
    chassis.moveToPose(-51.066,-62.958,90,2000,{.forwards=false, .lead=.5},false);
    chassis.moveToPose(-24.57,-49.805,270,3000,{.forwards=false, .lead=.5},false);
    //outake(200);
    stopper.set_value(HIGH); 
    outake(75);
    intake.move(127);
    intakeTop.move(127);
    drivePID(-10,500,100);
    chassis.turnToHeading(270,1000,{},false);


    drivePID(15,1000);
    stopper.set_value(LOW);
    intakeTop.brake();
    loader.set_value(HIGH);

    chassis.turnToHeading(272,500,{},false);
    //delay(500);
    drivePID(22,1000,30);//get loader balls
    chassis.turnToHeading(272,400,{},false);
    drivePID(-1.5,700);//shimmy
    drivePID(5,500);
    drivePID(-5,500);
    chassis.turnToHeading(271,500,{},false);
    loader.set_value(LOW);
    //outake(100);
    intakeStop();
    drivePID(-35,1200,50);//go to score
    //outake(200);
    stopper.set_value(HIGH);

    //outake(100);

    //outake(200);
    intake.move(127);
    intakeTop.move(127);
    //smallIntake.move(-127);
    chassis.turnToHeading(270,500,{},false);
    drivePID(-10,500);
    delay(1000); 
    //outake(5);
    intakeStop();
    //drivePID(10,450);
    stopper.set_value(LOW);
    intakeTop.brake();
    drivePID(10,400);
    drivePID(-15,500);
    delay(100);

    //drivePID(-15,500);
    //delay(100);
    chassis.setPose(-25.57,-48.805,chassis.getPose().theta);
    intakeAll(1);
    chassis.moveToPose(-55.127,-29.633,27,1100,{.forwards=true, .lead=.5},false);
    stopper.set_value(HIGH);
    intakeTop.move(127);
    //outake(1);
    //loader.set_value(HIGH);
    drivePID(50,2500,100);
    drivePID(-7,1500,100);
}
/**
 * nineBallRight – 9-ball elimination route starting at (-45.385, -14.531, 90°).
 * Uses moveToPose to collect balls around the right side of the field,
 * scores in the goal, then loads from the match-loader.
 */
void nineBallRight(int i){
    chassis.setPose(-45.385,-14.531,90);
    //lift.set_value(HIGH);
    intake.move(127);
    intakeTop.move(10);
    chassis.moveToPose(-20.122,-23.525,115,1100,{.forwards=true, .lead=.3},false);
    chassis.turnToHeading(45,500,{},false);
    chassis.moveToPose(-11.524,-9.901,45,800,{.forwards=true, .lead=.1},false);
    intake.move(-127);
    intakeTop.move(-70);
    delay(1000);
    intake.move(127);
    intakeTop.move(10);
    chassis.moveToPose(-14.17,-29.345,0,1500,{.forwards=false, .lead=.1},false);
    chassis.turnToHeading(180,1000,{},false);
    chassis.moveToPose(-8.159,-47.466,180,1700,{.forwards=true, .lead=.1},false);
    chassis.moveToPose(-19.196,-19.16,160,1500,{.forwards=false, .lead=.1},false);
    loader.set_value(HIGH);

    chassis.turnToHeading(220,500,{},false);
    chassis.moveToPose(-55.306,-46.921,220,1300,{.forwards=true, .lead=.1},false);
    lift.set_value(HIGH);
    chassis.turnToHeading(270,500,{},false);
    drivePID(20,1200,23);//get loader balls
    drivePID(1.5,500);//shimmy
    drivePID(-3,500);
    chassis.turnToHeading(272,200,{},false);
    loader.set_value(LOW);
    drivePID(-40,1000);
    intake.move(127);
    intakeTop.move(127);
    stopper.set_value(HIGH);
    drivePID(-10,500,160);
    chassis.turnToHeading(270,200,{},false);
    delay(100);
    chassis.setPose(-29.777,-47.334,chassis.getPose().theta);
    chassis.moveToPose(-48.824,-35.429,270,1000,{.forwards=true, .lead=.1},false);
    stopper.set_value(LOW);
    intakeTop.move(10);
    deScores.set_value(LOW);
    chassis.moveToPose(-10.334,-40.456,270,2000,{.forwards=false, .lead=.1},false);
}
/**
 * nineBallLeft – 9-ball elimination route starting at (-45.65, 14.569, 90°).
 * Mirror of nineBallRight for the left side of the field.
 */
void nineBallLeft(int i){
    chassis.setPose(-45.65,14.569,90);
    intake.move(127);
    intakeTop.move(10);
    chassis.moveToPose(-20.122,23.525,65,1100,{.forwards=true, .lead=.3},false);
    chassis.moveToPose(-7.821,44.891,7,1100,{.forwards=true, .lead=.3},false);
    //loader.set_value(HIGH);
    //delay(500);
    chassis.moveToPose(-20.783,19.727,7,1000,{.forwards=false, .lead=.1},false);
    chassis.turnToHeading(315,700,{},false);
    chassis.moveToPose(-9.926,10.079,315,900,{.forwards=false, .lead=.1},false);
    stopper.set_value(HIGH);
    intake.move(-127);
    intakeTop.move(-70);
    delay(100);
    intake.move(127);
    intakeTop.move(70);
    chassis.turnToHeading(315,500,{},false);
    delay(100);
    stopper.set_value(LOW);
    intakeTop.move(10);
    lift.set_value(HIGH);
    drivePID(-4,100);
    chassis.setPose(-12.979,12.056,chassis.getPose().theta);
    
    chassis.moveToPose(-44.195,45.711,270,1700,{.forwards=true, .lead=.1},false);
    loader.set_value(HIGH);
    chassis.turnToHeading(270,300,{},false);
    drivePID(26,900,23);//get loader balls
    drivePID(5,500);//shimmy
    drivePID(-3,500);
    
    chassis.turnToHeading(271,200,{},false);
    loader.set_value(LOW);
    drivePID(-40,1000);
    intake.move(127);
    intakeTop.move(127);
    stopper.set_value(HIGH);
    drivePID(-10,400,160);
    chassis.turnToHeading(270,200,{},false);
    delay(500);
    chassis.setPose(-29.777,47.334,chassis.getPose().theta);
    chassis.moveToPose(-48.824,57.689,270,1000,{.forwards=true, .lead=.1},false);
    stopper.set_value(LOW);
    intakeTop.move(10);
    deScores.set_value(LOW);
    chassis.moveToPose(-17.334,52.836,270,1700,{.forwards=false, .lead=.1},false);
}
/**
 * rightPushFast – Fast right-side push route; starts at heading 90°.
 * Spawns a background task to drop the loader at the right moment during the
 * first drive so intake and loader happen simultaneously. Ends by driving
 * to the far corner alliance zone.
 */
void rightPushFast(int i){
    chassis.setPose(0,0,90);
    intake.move(127);
    intakeTop.move(10);
    lift.set_value(HIGH);
    pros::Task IntakeTask([]
                         { delay(480);
                        loader.set_value(HIGH);});
    drivePID(25,700);
    loader.set_value(HIGH);
    chassis.turnToHeading(230,700,{},false);
    drivePID(40,870);
    chassis.turnToHeading(270,300,{},false);
    drivePID(20,700,23);//get loader balls
    drivePID(1.5,200);//shimmy
    drivePID(-5,200,130);
    chassis.turnToHeading(270,200,{},false);
    drivePID(-38,800);
    loader.set_value(LOW);
    intake.move(127);
    intakeTop.move(127);
    stopper.set_value(HIGH);
    delay(950);
    chassis.setPose(-31.232,-47.334,chassis.getPose().theta);
    chassis.moveToPose(-48.692,-38.036,270,1000,{.forwards=true, .lead=.1},false);
    chassis.turnToHeading(270,200,{},false);
    drivePID(-28,2000,100);
}

