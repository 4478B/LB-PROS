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



// global setter for color sort detector
bool isRedAlliance = true;

/*

AUTONOMOUS

*/

bool isRingDetected(Hue targetHue){
    // Calculate elapsed time and check if detection target is met

        int currentHue = ringSens.get_hue();        // Get the current hue value from the sensor
        int currentDist = ringSens.get_proximity(); // Get the current proximity value from the sensor

        // Determine if the current hue falls within the valid range, considering wrapping around 360 degrees

        bool inDist = currentDist > 255 - MAX_RING_DISTANCE;
        if ((targetHue.inHueRange(currentHue)) && inDist) {
            return true;
        }
        else {
            return false;
        }
}

bool waitUntilRingDetected(int msecTimeout, Hue targetHue)
{
    int startTime = pros::millis(); // Record the start time of the function
    int ringDetected = 0;           // Counter for consecutive ring detections

    ringSens.set_led_pwm(100); // Set the LED brightness to maximum for better detection
    ringSens.set_integration_time(10);


    while (pros::millis() - startTime < msecTimeout && ringDetected < MIN_RING_DETECTION)
    {
        
        if (isRingDetected(targetHue))
        {
            // Increment the detection counter if the conditions are met
            ringDetected++;
        }
        else
        {
            // Reset the detection counter if the conditions are not met
            ringDetected = 0;
        }

        // Print debug information
        pros::lcd::print(2, "Sensor hue %f", ringSens.get_hue());
        pros::lcd::print(3, "Sensor dist: %i", ringSens.get_proximity());
        pros::lcd::print(4, "Detections: %i", ringDetected);

        // pros::lcd::print(4, "Error: %s", strerror(ringSens.get_proximity()));

        pros::delay(10); // Wait briefly before the next sensor reading to prevent excessive polling
    }
    ringSens.set_led_pwm(100);
    ringSens.set_integration_time(100);
    if (ringDetected >= MIN_RING_DETECTION)
    {
        return true;
    }
    else
    {
        return false;
    }
    
}



bool waitUntilRedIntake(int timeout)
{
    // Wait until red rings are detected using the specified timeout
    return waitUntilRingDetected(timeout, RED_RING_HUE);
}

bool waitUntilBlueIntake(int timeout)
{
    // Wait until blue rings are detected using the specified timeout
    return waitUntilRingDetected(timeout, BLUE_RING_HUE);
}

bool waitUntilAnyIntake(int timeout)
{
    // Wait until blue rings are detected using the specified timeout
    return waitUntilRingDetected(timeout, -1);
}
/*

DRIVER CONTROL

*/