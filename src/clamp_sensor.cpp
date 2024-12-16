#include "color_sort.h"
#include "auton_routes.h"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <cstdlib>
#include "devices.h"
#include "old_systems.h"
#include "testing.h"
#include <iomanip>

const int MIN_GOAL_DETECTION = 3; // Amount of detections needed to quit loop
const int MAX_GOAL_DISTANCE = 10; // maximum distance goal is to be counted



void driveClamp(int volts, int maxDist, int maxTime){
    int startTime = pros::millis(); // Record the start time of the function
    int goalDetected = 0; // Counter for consecutive ring detections

    // ensure clamp is up
    clamp.set_value(HIGH);
    
    // start moving at user specified speed
    all_motors.move(-volts);

    // set up motor for distance tracking
    left_motors.tare_position(0);

    // convert maxDist from inches to rotations
    maxDist /= lemlib::Omniwheel::NEW_275 * M_PI;

    pros::lcd::print(1, "Volts: %d", volts);
    pros::lcd::print(2, "Max Distance (rotations): %f", maxDist);
    pros::lcd::print(3, "Max Time: %d ms", maxTime);
    
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


        pros::lcd::print(4, "Current Goal Distance: %d", currentGoalDist);
        pros::lcd::print(5, "Goal Detected Count: %d", goalDetected);
        pros::lcd::print(6, "Left Motor Position: %f", left_motors.get_position());

        pros::delay(20);

    }
    
    // stop moving when goal is detected or timeouts are reached
    all_motors.brake();
    // clamp goal
    clamp.set_value(LOW);

}



