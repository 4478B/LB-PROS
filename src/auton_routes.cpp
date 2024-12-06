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

// These our functions made for backwards-compatibility with VEXCode routes

void drivePIDOdom(double goalInches, bool clamping, double clampDistInches) {
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
    if (clamping) {
        // Move to point with clamping to prevent overshoot.
        chassis.MoveToPointClamp(poseGoal.x, poseGoal.y, 4000, clampDistInches, {.forwards = isForwards});
    } else {
        // Move to point without clamping.
        chassis.moveToPoint(poseGoal.x, poseGoal.y, 4000, {.forwards = isForwards});
    }

    // Step 6: Print debug information for testing pose calculations.
    // Output trimmed to 3 decimal places to fit the screen.
    pros::lcd::print(3, "Pose Init: X: %.3f, Y: %.3f, Th: %.3f", poseInit.x, poseInit.y, poseInit.theta);
    pros::lcd::print(4, "Pose Goal: X: %.3f, Y: %.3f, Th: %.3f", poseGoal.x, poseGoal.y, poseGoal.theta);
    pros::lcd::print(5, "Unit Angle: %.3f", unitCircleAngle);
}

void driveInchesClamp(double gDist, double cDist = .5){
    drivePID(gDist,true,cDist);
}

/*void inert(float theta)
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
        double startTime = pros::millis();
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

void progSkills()
{

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
    clamp.set_value(HIGH);

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
void blueGoalSide()
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
    chassis.turnToHeading(-180, 1000);
    endSection(100);

    // goto goal2 and clamp
    chassis.moveToPose(22, -36, -240, 2000, {.forwards = false, .minSpeed = 15}, false);
    clamp.set_value(LOW);
    intake.move(127);

    endSection(0);

    // grab ring1 for goal2
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
    //while(colorSens.)
    endSection(500);

    //
    //chassis.moveToPoint(-12, -12, 5000, {}, false);
    chassis.moveToPoint(64, 5, 4000, {.minSpeed = 40}, false);
    clamp.set_value(LOW);

    // score on alliance stake
    chassis.moveToPose(79,-13.5,-60,5000, {.forwards = false, .minSpeed = 72},false);
    intake.move(127);
    endSection(1000);

    // go to middle with arm up
    intake.brake();
    setArmTop();
    chassis.moveToPoint(5,0,2000,{.maxSpeed=70},false);

}

void redGoalSide() // edit this one and mirror it for the other part
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

    endSection(0);

    // grab ring1 for goal2
    //intake.move(127);
    setArmTop();
    chassis.moveToPoint(-52, -6, 4000, {.minSpeed = 40}, false);
    setArmBottom();
    delay(500);
    intake.move(127);
    delay(400);
    intake.brake();
    chassis.moveToPoint(-59, -40, 4000, {.forwards = false, .minSpeed = 40}, false);
    clamp.set_value(HIGH);
    //while(colorSens.)
    endSection(500);

    //
    //chassis.moveToPoint(-12, -12, 5000, {}, false);
    chassis.moveToPoint(-64, 5, 4000, {.minSpeed = 40}, false);
    clamp.set_value(LOW);

    // score on alliance stake
    chassis.moveToPose(-79,-13.5,60,5000, {.forwards = false, .minSpeed = 72},false);
    intake.move(127);
    endSection(1000);

    // go to middle with arm up
    intake.brake();
    setArmTop();
    chassis.moveToPoint(-5,0,2000,{.maxSpeed=70},false);

}
void blueRingSide()
{

    drivePID(-27);
    driveInchesClamp(-7, 30);
    delay(500);
    intake.move(127);
    endSection(10000);

    inert(-55);
    delay(500);
    drivePID(27); //+3; nvm
    delay(500);
    drivePID(-4);
    endSection(10000);

    inert(-145);
    drivePID(20);
    delay(500);
    endSection(10000);

    drivePID(-10);
    inert(-110);
    drivePID(11, 30);
    delay(500);
    drivePID(-6);
    inert(-145);
    endSection(10000);

    drivePID(-15);
    inert(-55);
    endSection(10000);

    drivePID(-11); //-3; nvm
    drivePID(-20);
    setArmTop();
    inert(-145);
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
void redRingSide()
{
}