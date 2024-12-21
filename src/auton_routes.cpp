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

int autonSection = 0;
void endSection(int delay)
{

    // functions as normal delay
    if (inCompetition)
    {
        pros::delay(delay);
    }
    else
    {
        // handle updating timers
        int startTime = pros::millis();
        int deltaTime = startTime - prevTime;
        totalTime += deltaTime;
        prevTime = startTime;

        // print timer positions to console for permanent logging
        std::cout << std::setw(14) << autonSection << " | "
                  << std::setw(14) << deltaTime << " | "
                  << std::setw(14) << totalTime << " | "
                  << std::endl;

        // print timer positions on controller and screen for temporary logging
        // pros::lcd::print(5, "Auton Section: %f", autonSection);
        // pros::lcd::print(5, "Section Time: %f", deltaTime);
        // pros::lcd::print(5, "Total Time: %f", totalTime);

        // controller.set_text(2,1,std) // controller WIP bc set_text is bad

        // while button hasn't been pressed and hasn't timed out
        while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) && pros::millis() - startTime < delay)
        {
            pros::delay(20);
        }
        // updates controller screen with section information
        autonSection++;
        controller.clear_line(1);
        controller.set_text(1, 1, std::to_string(autonSection).c_str());
    }
}

// This file includes all of the routes coded in PROS for our robot
// The routes should have linked path.jerryio files for reference

void progSkills(int i)
{
    chassis.setPose(-54, 0, 270);
    setArmAlliance();
    delay(1000);
    clamp.set_value(HIGH);
    drivePID(-4, 1000);
    // endSection(50000);
    setArmBottom();
    chassis.turnToHeading(0, 2000);
    drivePID(-22);
    drivePID(-10, 1000, 42.5);
    clamp.set_value(LOW);
    delay(150);
    drivePID(5, 600);
    endSection(500000);
    chassis.turnToHeading(90, 2000);
    intake.move(127);
    drivePID(25);
    delay(300);
    chassis.turnToHeading(120, 2000);
    drivePID(37);
    endSection(50000);
    chassis.turnToHeading(180, 2000);
    setArmMid();
    drivePID(24);
    delay(3000);
    intake.brake();
    setArmTop();
    drivePID(10, 1000);
    endSection(50000);
    drivePID(-15);
    setArmBottom();
    chassis.turnToHeading(270, 2000);
    chassis.turnToHeading(270, 500);
    intake.move(127);
    drivePID(60, 6000, 25);
    delay(500);
    drivePID(-15);
    chassis.turnToHeading(180, 2000);
    drivePID(20);
    delay(150);
    drivePID(-20);

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
    doinker.set_value(LOW);
    endSection(50000);

    // back up with goal doinkered
    chassis.moveToPoint(-17, -55, 2000, {.forwards = false, .minSpeed = 40}, false);
    endSection(50000);

    // undoinker and turn to ring1
    doinker.set_value(HIGH);
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

void allianceRedRingSide(int i)
{

    clamp.set_value(HIGH);
    chassis.setPose(0, 0, 221);

    // arm functions
    setArmAlliance();
    delay(1000);
    drivePID(-6);
    setArmTop();
    endSection(700);

    // move to alliance ring and score it
    chassis.turnToHeading(161, 1000, {}, false);
    intake.move(127);
    drivePID(27, 1000, 45);
    setArmBottom();
    waitUntilRedIntake(3000);
    intake.brake();
    // chassis.turnToHeading(180 ,1000,{},false);

    drivePID(-15, 1000);
    delay(500);
    chassis.turnToHeading(252, 700, {}, false);
    intake.brake();

    // go to goal and clamp
    drivePID(-18, 1500);
    drivePID(-10, 1000, 42.5);
    clamp.set_value(LOW);
    delay(300);
    intake.move(127);
    endSection(500);
    chassis.turnToHeading(20, 1000, {}, false);

    // score ring 2
    drivePID(29, 1000, 45);
    endSection();

    // go to middle
    chassis.turnToHeading(165, 1000, {}, false);
    setArmMid();
    left_motors.move(42);
    right_motors.move(42);
    delay(5000);
    left_motors.brake();
    right_motors.brake();
}

void allianceBlueRingSide(int i)
{
    clamp.set_value(HIGH);
    chassis.setPose(0, 0, -221);

    // arm functions
    setArmAlliance();
    delay(1000);
    drivePID(-6);
    setArmTop();
    endSection(700);

    // move to alliance ring and score it
    chassis.turnToHeading(-161, 1000, {}, false); //
    intake.move(127);
    drivePID(20, 1000, 45);
    intake.brake();
    setArmBottom();
    delay(400);

    // back up
    intake.move(127);
    chassis.turnToHeading(-180, 1000, {}, true); //
    waitUntilRedIntake(500);
    intake.brake();
    chassis.waitUntilDone();

    drivePID(-8, 1000);
    delay(500);
    chassis.turnToHeading(-252, 700, {}, false); //
    intake.brake();

    // go to goal and clamp
    drivePID(-20);
    drivePID(-8, 1000, 42.5);
    clamp.set_value(LOW);
    intake.move(127);
    endSection(500);
    chassis.turnToHeading(-20, 1000, {}, false); //

    // score ring 2
    drivePID(29, 1000, 45);
    endSection();

    // go to middle
    chassis.turnToHeading(-165, 1000, {}, false); //
    setArmTop();
    left_motors.move(42);
    right_motors.move(42);
    delay(5000);
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
    setArmTop();
    chassis.moveToPoint(-5, -5, 4321);
    endSection(987654321);
}

void redRingRush(int i)
{

    // initial pose
    clamp.set_value(HIGH);

    setArmAlliance();
    endSection(987654321);

    // back up to goal and clamp
    chassis.setPose(0, 0, 248);
    drivePID(-6);
    chassis.turnToHeading(295, 4321);
    drivePID(-27);
    delay(200);
    clamp.set_value(LOW);
    setArmBottom();
    endSection(987654321);

    // turn to ring rush
    chassis.turnToHeading(60, 4321);

    // rush rings
    intake.move(127);
    chassis.moveToPose(-6.5, 35.027, 0, 4321, {.maxSpeed = 30}, false);
    chassis.moveToPoint(-6.5, 60.508, 4321, {.maxSpeed = 30}, false);
}

void oldRedRingSide(int i) // 4 ring, red ringside, ported from vexcode
{
 
// drive and clamp, also start running the intake 
  clamp.set_value(HIGH);
  drivePID(-27);
  //driveInchesClamp(-7, 30);
  drivePID(-4,1500,40);
  clamp.set_value(LOW);
  delay(500);
  intake.move(127);

  // turns the robot, moves toward ring1 to inake it
  chassis.turnToHeading(75,2000);
  drivePID(15);
  delay(500);
 // drivePID(-4);

// robot turns again, heads toward ring2, intakes
// then the robot turns, drives, and intakes the next ring, before turning again
  chassis.turnToHeading(145,2000);
  drivePID(12.9,3000,20);
  delay(500);
  drivePID(-25);
  chassis.turnToHeading(-123,2000);
  drivePID(10);
  drivePID(6.4, 20);
  delay(1000);
  drivePID(-48);
  chassis.turnToHeading(120,2000);

 //arm moves up, then turns toward the middle in order to touch the ladder
  setArmTop();
  drivePID(47);
  intake.move(63);;
  setArmBottom();
  delay(300);
  intake.brake();
  delay(200);
  drivePID(-10);
  intake.move(127);;
  chassis.turnToHeading(-60,2000);
}