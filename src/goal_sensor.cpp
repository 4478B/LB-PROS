#include "goal_sensor.h"
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <cstdlib>
#include "devices.h"

const int MIN_GOAL_DETECTION = 3; // Amount of detections needed to quit loop
const int MAX_GOAL_DISTANCE = 10; // maximum distance goal is to be counted



void waitUntilClamp(int maxDist, int maxTime){
    int startTime = pros::millis(); // Record the start time of the function
    int goalDetected = 0; // Counter for consecutive ring detections

    // ensure clamp is up
    clamp.set_value(HIGH);

    // set up motor for distance tracking
    left_motors.tare_position(0);

    // convert maxDist from inches to rotations
    maxDist /= lemlib::Omniwheel::NEW_275 * M_PI;


    pros::lcd::print(1, "Max Distance (rotations): %f", maxDist);
    pros::lcd::print(2, "Max Time: %d ms", maxTime);
    
    // loop until goal is detected enough times or it times out
    while( pros::millis() - startTime < maxTime 
        && goalDetected < MIN_GOAL_DETECTION
        && left_motors.get_position(0) > -maxDist){
        
        // Get the current distance from the sensor
        int currentGoalDist = goalSens.get_distance(); 

        // Determine if goal is within proximity
        bool inRange = currentGoalDist <= MAX_GOAL_DISTANCE;

        if(inRange){
            // Increment the detection counter if conditions are met
            goalDetected++;
        }
        else {
            // Reset the detection counter if the conditions are not met
            goalDetected = 0;
        }


        pros::lcd::print(3, "Current Goal Distance: %d", currentGoalDist);
        pros::lcd::print(4, "Goal Detected Count: %d", goalDetected);
        pros::lcd::print(5, "Left Motor Position: %f", left_motors.get_position());

        pros::delay(20);

    }
    // clamp goal
    clamp.set_value(LOW);
}

bool isGoalClamped(){

    // if goal is detected and clamp is down
    if(goalSens.get_distance() <= MAX_GOAL_DISTANCE && clamp.get_value() == LOW) {
        return true;
    }
    else {
        return false;
    }
}

