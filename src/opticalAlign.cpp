#include "devices.h"
#include "opticalAlign.h"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include <cmath>

using namespace pros;

// Constants for goal detection and alignment
const double minGoalDist = 680.0;  // Minimum valid distance to goal (mm)
const double maxGoalDist = 850.0;  // Maximum valid distance to goal (mm)
const double turnSpd = 70.0;       // Speed for turning during scan
const double backupSpd = 100.0;     // Speed for backing into goal
const double alignTolerance = 0.5; // Tolerance for heading alignment (degrees)
const double maxScanAngle = 40.0;  // Maximum angle to scan
const double scanStartAngle = 160.0; // Starting angle for scan
const double scanEndAngle = 210.0;   // Ending angle for scanu
const double extraTurnAngle = 0.5;   // Extra angle for final turn

/**
 * @brief Detects if a goal is in view based on distance and object size
 * @return true if goal is detected, false otherwise
 */
bool detectsGoal() {
    double dist = backDistance.get();
    int objSize = backDistance.get_object_size();
    
    // Object size range 0-400, where ~75 is an 18" x 30" grey card
    const int minMediumSize = 49;
    const int maxMediumSize = 100;
    
    return (objSize >= minMediumSize && objSize <= maxMediumSize && 
            dist >= minGoalDist && dist <= maxGoalDist);
}

/**
 * @brief Scans for a goal by turning through the scan angle range
 * @param targetHeading Reference to store the heading where goal was found
 * @return true if goal was found, false otherwise
 */
bool scanForGoal(double& targetHeading) {
    double currentHeading = imu.get_heading();
    
    // Determine sweep direction based on default scan angles
    double headingDiff = scanEndAngle - currentHeading;
    
    // Normalize to [-180, 180]
    while (headingDiff > 180) headingDiff -= 360;
    while (headingDiff < -180) headingDiff += 360;
    
    // Calculate sweep range starting from current heading
    double endHeading;
    
    if (headingDiff > 0) {
        // Goal is to the right, sweep right
        endHeading = currentHeading + maxScanAngle;
    } else {
        // Goal is to the left, sweep left
        endHeading = currentHeading - maxScanAngle;
    }
    
    // Normalize heading to [0, 360]
    while (endHeading < 0) endHeading += 360;
    while (endHeading >= 360) endHeading -= 360;
    
    // Begin async scan to end from current position (no initial turn away)
    chassis.turnToHeading(endHeading, 1000, {.maxSpeed = 25}, true);
    
    bool goalFound = false;
    
    while (chassis.isInMotion()) {
        double dist = backDistance.get();
        int objSize = backDistance.get_object_size();
        double currHeading = imu2.get_heading();
        bool isDetected = detectsGoal();
        
        pros::lcd::clear_line(0);
        pros::lcd::print(0, "Detected: %s", isDetected ? "YES" : "NO");
        pros::lcd::clear_line(1);
        pros::lcd::print(1, "Size: %d  Dist: %.0fmm", objSize, dist);
        pros::lcd::clear_line(2);
        pros::lcd::print(2, "Heading: %.1f -> %.1f", currHeading, endHeading);
        
        if (isDetected) {
            chassis.cancelAllMotions();
            targetHeading = currHeading;
            goalFound = true;
            left_motors.brake();
            right_motors.brake();
            pros::delay(150);
            return true;
        }
        
        pros::delay(20);
    }
    
    left_motors.brake();
    right_motors.brake();
    return goalFound;
}

/**
 * @brief Scans for a goal by turning through a scan angle range
 * @param targetHeading Reference to store the heading where goal was found
 * @param intendedHeading The heading the robot expects the goal to be at (determines sweep direction)
 * @return true if goal was found, false otherwise
 */
bool scanForGoal(double& targetHeading, double intendedHeading) {
    double currentHeading = imu.get_heading();
    
    // Normalize intended heading to [0, 360]
    double normalizedIntended = intendedHeading;
    while (normalizedIntended < 0) normalizedIntended += 360;
    while (normalizedIntended >= 360) normalizedIntended -= 360;
    
    // Begin async scan to intended heading from current position (no initial turn away)
    chassis.turnToHeading(normalizedIntended, 1000, {.maxSpeed = 25}, true);
    
    while (chassis.isInMotion()) {
        double dist = backDistance.get();
        int objSize = backDistance.get_object_size();
        double currHeading = imu2.get_heading();
        bool isDetected = detectsGoal();
        
        pros::lcd::clear_line(0);
        pros::lcd::print(0, "Detected: %s", isDetected ? "YES" : "NO");
        pros::lcd::clear_line(1);
        pros::lcd::print(1, "Size: %d  Dist: %.0fmm", objSize, dist);
        pros::lcd::clear_line(2);
        pros::lcd::print(2, "Heading: %.1f -> %.1f", currHeading, normalizedIntended);
        
        if (isDetected) {
            chassis.cancelAllMotions();
            targetHeading = currHeading;
            left_motors.brake();
            right_motors.brake();
            pros::delay(150);
            return true;
        }
        
        pros::delay(20);
    }
    
    left_motors.brake();
    right_motors.brake();
    return false;
}

/**
 * @brief Scans for a goal by sweeping past it and averaging detected headings
 * @param targetHeading Reference to store the averaged heading where goal was found
 * @param intendedHeading The heading the robot expects the goal to be at (determines sweep direction)
 * @return true if goal was found, false otherwise
 */
bool scanForGoalAverage(double& targetHeading, double intendedHeading) {
    double currentHeading = imu.get_heading();
    
    // Determine sweep direction based on intended heading
    double headingDiff = intendedHeading - currentHeading;
    
    // Normalize to [-180, 180]
    while (headingDiff > 180) headingDiff -= 360;
    while (headingDiff < -180) headingDiff += 360;
    
    // Calculate sweep range starting from current heading
    double endHeading;
    
    if (headingDiff > 0) {
        // Goal is to the right, sweep right
        endHeading = currentHeading + maxScanAngle;
    } else {
        // Goal is to the left, sweep left
        endHeading = currentHeading - maxScanAngle;
    }
    
    // Normalize heading to [0, 360]
    while (endHeading < 0) endHeading += 360;
    while (endHeading >= 360) endHeading -= 360;
    
    // Begin async sweep to end from current position (no initial turn away)
    chassis.turnToHeading(endHeading, 1000, {.maxSpeed = 25}, true);
    
    // Arrays to store detected headings (up to 50 samples)
    const int maxSamples = 50;
    double detectedHeadings[maxSamples];
    int detectedCount = 0;
    
    pros::lcd::clear_line(0);
    pros::lcd::print(0, "Sweep: %.1f -> %.1f", currentHeading, endHeading);
    
    while (chassis.isInMotion()) {
        double dist = backDistance.get();
        int objSize = backDistance.get_object_size();
        double currHeading = imu2.get_heading();
        bool isDetected = detectsGoal();
        
        pros::lcd::clear_line(1);
        pros::lcd::print(1, "Detected: %s  Samples: %d", isDetected ? "YES" : "NO", detectedCount);
        pros::lcd::clear_line(2);
        pros::lcd::print(2, "Size: %d  Dist: %.0fmm", objSize, dist);
        pros::lcd::clear_line(3);
        pros::lcd::print(3, "Heading: %.1f", currHeading);
        
        if (isDetected && detectedCount < maxSamples) {
            detectedHeadings[detectedCount] = currHeading;
            detectedCount++;
        }
        
        pros::delay(20);
    }
    
    // Stop the robot
    left_motors.brake();
    right_motors.brake();
    
    if (detectedCount == 0) {
        pros::lcd::print(4, "No detections found");
        return false;
    }
    
    // Calculate average of detected headings
    // Handle wrap-around cases properly
    double sumSin = 0.0;
    double sumCos = 0.0;
    
    for (int i = 0; i < detectedCount; i++) {
        double rad = detectedHeadings[i] * M_PI / 180.0;
        sumSin += sin(rad);
        sumCos += cos(rad);
    }
    
    double avgHeading = atan2(sumSin, sumCos) * 180.0 / M_PI;
    
    // Normalize to [0, 360]
    while (avgHeading < 0) avgHeading += 360;
    
    targetHeading = avgHeading;
    
    pros::lcd::clear_line(4);
    pros::lcd::print(4, "Avg Heading: %.1f (n=%d)", avgHeading, detectedCount);
    
    pros::delay(150);
    return true;
}

/**
 * @brief Backs the robot into the goal while maintaining alignment
 * @param targetHeading The heading to maintain while backing
 */
void backIntoGoal(double targetHeading) {
    const double backupTimeout = 1200;
    double startTime = pros::millis();
    int lostObjCount = 0;
    const int lostObjThreshold = 5;
    
    while (pros::millis() - startTime < backupTimeout) {
        int objDetected = backDistance.get_object_size();
        
        if (objDetected == 0) {
            lostObjCount++;
            
            if (lostObjCount >= lostObjThreshold) {
                left_motors.brake();
                right_motors.brake();
                pros::delay(100);
                
                left_motors.move(70);
                right_motors.move(70);
                pros::delay(300);
                left_motors.brake();
                right_motors.brake();
                pros::delay(100);
                
                if (scanForGoal(targetHeading)) {
                    startTime = pros::millis();
                    lostObjCount = 0;
                    continue;
                } else {
                    return;
                }
            }
        } else {
            lostObjCount = 0;
        }
        
        double currHeading = imu2.get_heading();
        double headingErr = targetHeading - currHeading;
        
        // Normalize heading error to [-180, 180]
        while (headingErr > 180) headingErr -= 360;
        while (headingErr < -180) headingErr += 360;
        
        double correction = headingErr * 1.5;
        
        // Limit correction speed
        if (correction > 30) correction = 30;
        if (correction < -30) correction = -30;
        
        double leftPwr = -backupSpd + correction;
        double rightPwr = -backupSpd - correction;
        
        left_motors.move(leftPwr);
        right_motors.move(rightPwr);
        
        pros::delay(20);
    }
    
    left_motors.brake();
    right_motors.brake();
}

/**
 * @brief Main function to align to long goal
 * Scans for the goal in both directions, then backs into it while maintaining alignment
 */
void alignToLongGoal() {
    double targetHeading = 0;
    double currentHeading = imu2.get_heading();
    
    // Try sweeping both directions to find the goal
    // First try sweeping right
    double endHeadingRight = currentHeading + maxScanAngle;
    while (endHeadingRight >= 360) endHeadingRight -= 360;
    
    chassis.turnToHeading(endHeadingRight, 1000, {.maxSpeed = 25}, true);
    
    bool goalFound = false;
    
    while (chassis.isInMotion()) {
        double dist = backDistance.get();
        int objSize = backDistance.get_object_size();
        double currHeading = imu2.get_heading();
        bool isDetected = detectsGoal();
        
        pros::lcd::clear_line(0);
        pros::lcd::print(0, "Right Sweep: %s", isDetected ? "YES" : "NO");
        pros::lcd::clear_line(1);
        pros::lcd::print(1, "Size: %d  Dist: %.0fmm", objSize, dist);
        pros::lcd::clear_line(2);
        pros::lcd::print(2, "Heading: %.1f -> %.1f", currHeading, endHeadingRight);
        
        if (isDetected) {
            chassis.cancelAllMotions();
            targetHeading = currHeading;
            goalFound = true;
            left_motors.brake();
            right_motors.brake();
            pros::delay(150);
            
            backIntoGoal(targetHeading);
            return;
        }
        
        pros::delay(20);
    }
    
    left_motors.brake();
    right_motors.brake();
    
    // If not found, try sweeping left
    if (!goalFound) {
        double endHeadingLeft = currentHeading - maxScanAngle;
        while (endHeadingLeft < 0) endHeadingLeft += 360;
        
        chassis.turnToHeading(endHeadingLeft, 1000, {.maxSpeed = 25}, true);
        
        while (chassis.isInMotion()) {
            double dist = backDistance.get();
            int objSize = backDistance.get_object_size();
            double currHeading = imu2.get_heading();
            bool isDetected = detectsGoal();
            
            pros::lcd::clear_line(0);
            pros::lcd::print(0, "Left Sweep: %s", isDetected ? "YES" : "NO");
            pros::lcd::clear_line(1);
            pros::lcd::print(1, "Size: %d  Dist: %.0fmm", objSize, dist);
            pros::lcd::clear_line(2);
            pros::lcd::print(2, "Heading: %.1f -> %.1f", currHeading, endHeadingLeft);
            
            if (isDetected) {
                chassis.cancelAllMotions();
                targetHeading = currHeading;
                goalFound = true;
                left_motors.brake();
                right_motors.brake();
                pros::delay(150);
                
                backIntoGoal(targetHeading);
                return;
            }
            
            pros::delay(20);
        }
        
        left_motors.brake();
        right_motors.brake();
    }
    
    // If still not found, give up
    if (!goalFound) {
        left_motors.brake();
        right_motors.brake();
    }
}

/**
 * @brief Align to long goal with intended heading
 * @param intendedHeading The expected heading of the goal (determines sweep direction)
 */
void alignToLongGoal(double intendedHeading) {
    alignToLongGoal(intendedHeading, true);
}

/**
 * @brief Align to long goal with intended heading and optional backup
 * @param intendedHeading The expected heading of the goal (determines sweep direction)
 * @param shouldBackup Whether to back into the goal after finding it
 */
void alignToLongGoal(double intendedHeading, bool shouldBackup) {
    double targetHeading = 0;
    
    if (scanForGoal(targetHeading, intendedHeading)) {
        if (shouldBackup) {
            backIntoGoal(targetHeading);
        } else {
            left_motors.brake();
            right_motors.brake();
        }
    } else {
        // scanForGoal already turns to intendedHeading if no goal found
        left_motors.brake();
        right_motors.brake();
    }
}

/**
 * @brief Align to long goal using averaged heading from sweep
 * @param intendedHeading The expected heading of the goal (determines sweep direction)
 */
void alignToLongGoalAverage(double intendedHeading) {
    alignToLongGoalAverage(intendedHeading, true);
}

/**
 * @brief Align to long goal using averaged heading from sweep with optional backup
 * @param intendedHeading The expected heading of the goal (determines sweep direction)
 * @param shouldBackup Whether to back into the goal after finding it
 */
void alignToLongGoalAverage(double intendedHeading, bool shouldBackup) {
    double targetHeading = 0;
    
    if (scanForGoalAverage(targetHeading, intendedHeading)) {
        if (shouldBackup) {
            backIntoGoal(targetHeading);
        } else {
            left_motors.brake();
            right_motors.brake();
        }
    } else {
        left_motors.brake();
        right_motors.brake();
    }
}

