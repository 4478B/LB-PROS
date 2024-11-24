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


// This file includes all of the routes coded in PROS for our robot
// The routes should have linked path.jerryio files for reference

void progSkills(){

    // PROG SKILLS ROUTE 1 (reference is skills_aio.txt)
    
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

    // May need to add an odom resetter here
    // look up how they work but tldr is they have robot go into wall and then reset position of odom based on knowing they are at wall
    // it prevents drift

}
void blueGoalSide(){



}
void redGoalSide(){


    
}
void blueRingSide(){


    
}
void redRingSide(){


    
}