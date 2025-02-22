#include "color_sort.h"
#include "auton_routes.h"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <cstdlib>
#include "devices.h"
#include "old_systems.h"
#include "pros/misc.hpp"
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
        if (isRingDetected(targetHue)){
            ringDetected++; // Increment the detection counter
        }
        else{
            ringDetected = 0; // Reset the detection counter
        }
        // Print debug information

        pros::delay(10); // Wait briefly before the next sensor reading to prevent excessive polling
    }
    ringSens.set_led_pwm(100);
    ringSens.set_integration_time(100);
    if (ringDetected >= MIN_RING_DETECTION){
        return true;
    }
    else{
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

// global intake and arm stuck detectors
bool intakeStuck = false;
int intakeStuckCount = 0;
bool armStuck = false;
int armStuckCount = 0;

// ONLY ENABLE DURING ROUTES WHEN WANTED
bool intakeOverride = false;
bool armOverride = false;

void stuck_task(void* param) {
    while (true) {
        // Check if the intake is stuck based on voltage and efficiency
        if (abs(intake.get_voltage()) > 4000 && intake.get_efficiency() < 5) {
            intakeStuckCount++;
        } else {
            intakeStuckCount = 0;
        }

        // Set the intakeStuck flag if the stuck count exceeds the threshold
        if (intakeStuckCount > 50) {
            intakeStuck = true;
            if (intakeOverride) {
                int initVelo = intake.get_target_velocity();
                intake.move(-127);
                pros::delay(400); // Stay stuck for 400ms
                intake.move(initVelo);
            } else {
                pros::delay(400); // Stay stuck for 400ms
            }
        } else {
            intakeStuck = false;
        }

        // Check if the arm is stuck based on voltage and efficiency
        if (abs(arm_motors.get_voltage()) > 4000 && arm_motors.get_efficiency() < 5) {
            armStuckCount++;
        } else {
            armStuckCount = 0;
        }

        // Set the armStuck flag if the stuck count exceeds the threshold
        if (armStuckCount > 50) {
            armStuck = true;
            if (armOverride) {
                setArmMid();
            } else {
                pros::delay(400); // Stay stuck for 400ms
            }
        } else {
            armStuck = false;
        }

        pros::delay(20); // Use pros::delay for consistency with the rest of the code
    }
}


/*

DRIVER CONTROL

*/