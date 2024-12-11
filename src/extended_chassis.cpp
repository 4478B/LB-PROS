#include "extended_chassis.h"
#include "devices.h"


// This is a custom extension of the Chassis lemlib class to add clamping during MoveToPoint and MoveToPose

void ExtendedChassis::MoveToPointClamp(float x, float y, int timeout, float clampDist, lemlib::MoveToPointParams params) {
    
    this->moveToPoint(x, y, timeout, {.forwards=false});
    Pose poseGoal(x,y,0);
    bool clampState = HIGH;

    while(chassis.isInMotion()) {
        if(chassis.getPose().distance(poseGoal) < clampDist && clampState != LOW){
            clamp.set_value(LOW);
        }
        delay(20);
    }
}


void ExtendedChassis::MoveToPoseClamp(float x, float y, float theta, int timeout, float clampDist, lemlib::MoveToPoseParams params) {
    
    this->moveToPose(x, y, theta, timeout, {.forwards=false},true);
    Pose poseGoal(x,y,theta);
    bool clampState = HIGH;

    while(chassis.isInMotion()) {
        if(chassis.getPose().distance(poseGoal) < clampDist && clampState != LOW){
            clamp.set_value(LOW);
        }
        delay(20);
    }
}

void printPose(int line1, int line2, int line3){
    lemlib::Pose pose = chassis.getPose();
    pros::lcd::print(line1, "X: %f", pose.x);
    pros::lcd::print(line2, "Y: %f", pose.y);
    pros::lcd::print(line3, "t: %f", pose.theta);
}