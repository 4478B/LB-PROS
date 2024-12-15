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
#include "testing.h"
#include <iomanip>


// exact values of hues to detect
const int RED_RING_HUE = 0;
const int BLUE_RING_HUE = 210;

// one sided hue range that is considered close enough
const int HUE_RANGE = 10;

/*

AUTONOMOUS

*/

const int ringDetectionRequirement = 3; // Amount of detections needed to quit loop

void waitUntilRingDetected(int msecTimeout, bool getRed){
    int startTime = pros::millis(); // Record the start time of the function
    int ringDetected = 0; // Counter for consecutive ring detections

    // Define acceptable hue range based on the desired ring color
    int targetHue, hueMin, hueMax; 
    
    // set target hue to correct team
    targetHue = getRed ? RED_RING_HUE : BLUE_RING_HUE; 
    hueMin = (targetHue - HUE_RANGE + 360) % 360; // Wrap around to ensure valid range
    hueMax = (targetHue + HUE_RANGE) % 360; // Wrap around to ensure valid range

    colorSens.set_led_pwm(100); // Set the LED brightness to maximum for better detection

    while(pros::millis() - startTime < msecTimeout && ringDetected < ringDetectionRequirement){
        // Calculate elapsed time and check if detection target is met

        int currentHue = colorSens.get_hue(); // Get the current hue value from the sensor
        int currentDist = colorSens.get_proximity(); // Get the current proximity value from the sensor

        // Determine if the current hue falls within the valid range, considering wrapping around 360 degrees
        bool inRange = (hueMin <= hueMax) ? 
                        (currentHue >= hueMin && currentHue <= hueMax) : 
                        (currentHue >= hueMin || currentHue <= hueMax);

        if(inRange && currentDist < 20) {
            // Increment the detection counter if the ring is within range and proximity threshold is met
            ringDetected++;
        }
        else {
            // Reset the detection counter if the conditions are not met
            ringDetected = 0;
        }

        // Print debug information
        std::cout << "Current Hue: " << currentHue << ", In Range: " << inRange 
                  << ", Current Distance: " << currentDist << ", Ring Detected: " << ringDetected << "\n";

        pros::delay(20); // Wait briefly before the next sensor reading to prevent excessive polling
    }
    colorSens.set_led_pwm(0);
}

/*
pros::lcd::print(1, "Detecting color %s", isRed ? "Red" : "Blue");
pros::lcd::print(2, "Sensor hue %f", currentHue);
pros::lcd::print(3, "Sensor dist: %f", currentDist);
pros::lcd::print(4, "Times detected: %f", ringDetected);
*/

void waitUntilRedIntake(int timeout) {
    // Wait until red rings are detected using the specified timeout
    waitUntilRingDetected(timeout, true);
}

void waitUntilBlueIntake(int timeout) {
    // Wait until blue rings are detected using the specified timeout
    waitUntilRingDetected(timeout, false);
}

/*

DRIVER CONTROL

*/

bool isRedAlliance = false;

// this function is called during the color_sort_task
// it is the actions that are taken to sort out a ring
void tossRing()
{

    intake.move(-127);
    delay(1000);
}

bool isColorSorting = false; // global variable for killswitch


void color_sort_task(void *param)
{
    int hueMin, hueMax; // these are endpoints for acceptable ring colors
    if (isRedAlliance)
    {
        hueMin = 330; // min < max b/c it loops around 360 degrees
        hueMax = 45;
    }
    else
    {
        hueMin = 0; // placeholder values
        hueMax = 0;
    }

    int currentHue = colorSens.get_hue(); // Get the current hue value from the sensor
    int currentDist = colorSens.get_proximity(); // Get the current proximity value from the sensor

        // Determine if the current hue falls within the valid range, considering wrapping around 360 degrees
        bool inRange = (hueMin < hueMax) ? (currentHue >= hueMin && currentHue <= hueMax) : (currentHue >= hueMin || currentHue <= hueMax);

        if(inRange && currentDist < 20) {
            // Increment the detection counter if the ring is within range and proximity threshold is met
            ringDetected++;
        }
        else {
            // Reset the detection counter if the conditions are not met
            ringDetected = 0;
        }
    delay(20);
}