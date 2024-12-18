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


// exact values of hues to detect
const int RED_RING_HUE = 0;
const int BLUE_RING_HUE = 210;

// one sided hue range that is considered close enough
const int HUE_RANGE = 10;

const int MIN_RING_DETECTION = 1; // Amount of detections needed to quit loop
const int MAX_RING_DISTANCE = 10; // maximum distance ring is on intake from optical sensor


// global setter for color sort detector
bool isRedAlliance = true;

/*

AUTONOMOUS

*/

bool waitUntilRingDetected(int msecTimeout, bool getRed = isRedAlliance){
    int startTime = pros::millis(); // Record the start time of the function
    int ringDetected = 0; // Counter for consecutive ring detections

    // Define acceptable hue range based on the desired ring color
    int targetHue, hueMin, hueMax; 
    
    // set target hue to correct team
    targetHue = getRed ? RED_RING_HUE : BLUE_RING_HUE; 
    hueMin = (targetHue - HUE_RANGE + 360) % 360; // Wrap around to ensure valid range
    hueMax = (targetHue + HUE_RANGE) % 360; // Wrap around to ensure valid range

    ringSens.set_led_pwm(100); // Set the LED brightness to maximum for better detection

    while(pros::millis() - startTime < msecTimeout && ringDetected < MIN_RING_DETECTION){
        // Calculate elapsed time and check if detection target is met

        int currentHue = ringSens.get_hue(); // Get the current hue value from the sensor
        int currentDist = ringSens.get_proximity(); // Get the current proximity value from the sensor

        // Determine if the current hue falls within the valid range, considering wrapping around 360 degrees
        bool inHueRange = (hueMin <= hueMax) ? 
                        (currentHue >= hueMin && currentHue <= hueMax) : 
                        (currentHue >= hueMin || currentHue <= hueMax);

        if(inHueRange && currentDist > 255 - MAX_RING_DISTANCE) {
            // Increment the detection counter if the conditions are met
            ringDetected++;
        }
        else {
            // Reset the detection counter if the conditions are not met
            ringDetected = 0;
        }

        // Print debug information
        //pros::lcd::print(2, "Sensor hue %f", ringSens.get_hue());
        //pros::lcd::print(3, "Sensor dist: %i", ringSens.get_proximity());
        //pros::lcd::print(4, "Error: %s", strerror(ringSens.get_proximity())); 

        pros::delay(20); // Wait briefly before the next sensor reading to prevent excessive polling
    }
    ringSens.set_led_pwm(0);
    if(ringDetected>=MIN_RING_DETECTION){
        return true;
    }
    else{
        return false;
    }
}

bool waitUntilRedIntake(int timeout) {
    // Wait until red rings are detected using the specified timeout
    return waitUntilRingDetected(timeout, true);
}

bool waitUntilBlueIntake(int timeout) {
    // Wait until blue rings are detected using the specified timeout
    return waitUntilRingDetected(timeout, false);
}

/*

DRIVER CONTROL

*/
// Singleton instance
colorSortHandler& colorSortHandler::getInstance() {
    static colorSortHandler instance;
    return instance;
}

// Constructor
colorSortHandler::colorSortHandler() 
    : targetHue(0), hueMin(0), hueMax(0), isRedAlliance(true), isCurrentlySorting(false) {
    swapTeam();
    swapTeam(); // Initialize correct hue values
    killSwitch();
    killSwitch(); // Set LED brightness
}

// Toggle kill switch
void colorSortHandler::killSwitch() {
    isCurrentlySorting = !isCurrentlySorting;
    if (isCurrentlySorting) {
        ringSens.set_led_pwm(100);
    } else {
        ringSens.set_led_pwm(0);
    }
}

// Toss a ring
void colorSortHandler::tossRing() {
    intake.move(-127);
    delay(1000);
}

// Swap team
void colorSortHandler::swapTeam() {
    isRedAlliance = !isRedAlliance;
    targetHue = isRedAlliance ? RED_RING_HUE : BLUE_RING_HUE;
    hueMin = (targetHue - HUE_RANGE + 360) % 360;
    hueMax = (targetHue + HUE_RANGE) % 360;
}

// Accessor methods
bool colorSortHandler::getIsRedAlliance() const {
    return isRedAlliance;
}

bool colorSortHandler::getIsCurrentlySorting() const {
    return isCurrentlySorting;
}

colorSortHandler& sorter = colorSortHandler::getInstance();

void color_sort_task(void *param)
{
    int ringDetected = 0; // Counter for consecutive ring detections

    
    while(true){

        if(sorter.getIsCurrentlySorting()) {

            int currentHue = ringSens.get_hue(); // Get the current hue value from the sensor
            int currentDist = ringSens.get_proximity(); // Get the current proximity value from the sensor

            // Determine if the current hue falls within the valid range, considering wrapping around 360 degrees
            bool inHueRange = (sorter.hueMin <= sorter.hueMax) ? 
                           (currentHue >= sorter.hueMin && currentHue <= sorter.hueMax) : 
                           (currentHue >= sorter.hueMin || currentHue <= sorter.hueMax);

            if(inHueRange && currentDist < MAX_RING_DISTANCE) {
                // Increment the detection counter if the conditions are met
                ringDetected++;
            }
            else {
                // Reset the detection counter if the conditions are not met
                ringDetected = 0;
            }

            if(ringDetected > MIN_RING_DETECTION){
                sorter.tossRing();
                ringDetected = 0;
            }
        }
        else {
            delay(480);
        }
        delay(20);

        // print debug information

    }
    
}

/*void color_sort_task_2(void param*){

    waitUntilRingDetected(1000){

            if(inHueRange && currentDist < MAX_RING_DISTANCE) {
        if(ringSens.)
        tossRing();
    }

}*/

/*bool scoreDetectorActive = true;
void score_detect_thread(void param*){

    bool isIntakeMoving;
    bool isRingDetected;
    int lastScoreTime = pros::millis(); // for cooldown

    while(scoreDetectorActive){

        bool isIntakeMoving = true;
        
        
        if(isIntakeMoving || isRingDetected){

            delay(20);
        }
        else{
            delay(100);
        }

    }

}



*/