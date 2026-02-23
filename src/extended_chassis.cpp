/**
 * @file extended_chassis.cpp
 * @brief Implementation of ExtendedChassis clamping motion methods.
 *
 * Both methods start an asynchronous backward motion, then poll the robot's
 * odometry pose every 20 ms. Once the robot is within `clampDist` inches of
 * the target, the clamp pneumatic is actuated. The methods return when LemLib
 * reports that the motion is complete.
 */

#include "extended_chassis.h"
#include "devices.h"

void ExtendedChassis::MoveToPointClamp(float x, float y, int timeout, float clampDist, lemlib::MoveToPointParams params) {
    // Start moving backward to the target point (async so we can monitor distance)
    this->moveToPoint(x, y, timeout, {.forwards=false});
    Pose poseGoal(x, y, 0);
    bool clampState = HIGH; // clamp starts retracted (HIGH = not yet fired)

    // Poll while in motion; fire clamp once close enough
    while(chassis.isInMotion()) {
        if(chassis.getPose().distance(poseGoal) < clampDist && clampState != LOW){
            clamp.set_value(LOW); // LOW = clamp extends / engages goal
        }
        delay(20);
    }
}

void ExtendedChassis::MoveToPoseClamp(float x, float y, float theta, int timeout, float clampDist, lemlib::MoveToPoseParams params) {
    // Start moving backward to the target pose (async)
    this->moveToPose(x, y, theta, timeout, {.forwards=false}, true);
    Pose poseGoal(x, y, theta);
    bool clampState = HIGH;

    // Poll while in motion; fire clamp once close enough
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