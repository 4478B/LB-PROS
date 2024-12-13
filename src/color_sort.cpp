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

/*
 * This program uses a color sensor to detect rings based on their color and proximity.
 * The function "waitUntilRingDetected" waits until a specified number of rings with 
 * a certain color (red or blue) are detected within a given timeout period.
 * Supporting functions "waitUntilRedIntake" and "waitUntilBlueIntake" allow for 
 * easier invocation for red and blue rings, respectively.
 * 
 * How it works:
 * 1. The program sets a hue range for the desired ring color.
 * 2. Continuously checks the hue and proximity reported by the color sensor.
 * 3. Counts consecutive detections of the specified ring color.
 * 4. Terminates either when the count reaches the target (3 detections) or when the timeout occurs.
 */

void waitUntilRingDetected(int msecTimeout, bool isRed){
    int startTime = pros::millis(); // Record the start time of the function
    int ringDetected = 0; // Counter for consecutive ring detections

    int hueMin, hueMax; // Define acceptable hue range based on the desired ring color
    if(isRed) { 
        hueMin = 180; // Minimum hue value for red rings 
        hueMax = 240;  // Maximum hue value for red rings
    }
    else { 
        hueMin = 330; // Minimum hue value for blue rings (wrapping around 360 degrees)
        hueMax = 45; // Maximum hue value for blue rings
    }

    colorSens.set_led_pwm(100); // Set the LED brightness to maximum for better detection
    while(pros::millis() - startTime < msecTimeout && ringDetected < 3){
        // Calculate elapsed time and check if detection target is met
 
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
        
        // print debug information

        pros::lcd::print(1, "Detecting color %s", isRed ? "Red" : "Blue");
        pros::lcd::print(2, "Sensor hue %f", currentHue);
        pros::lcd::print(3, "Sensor dist: %f", currentDist);
        pros::lcd::print(4, "Times detected: %f", ringDetected);

        pros::delay(20); // Wait briefly before the next sensor reading to prevent excessive polling
    }
    colorSens.set_led_pwm(0);
}

void waitUntilRedIntake(int timeout) {
    // Wait until red rings are detected using the specified timeout
    waitUntilRingDetected(timeout, true);
}

void waitUntilBlueIntake(int timeout) {
    // Wait until blue rings are detected using the specified timeout
    waitUntilRingDetected(timeout, false);
}
