#include "testing.h"
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
#include "auton_routes.h"

void drivePID(double inches, double kP, double kI, double kD, double goalThreshold)
{
  // Function to control robot movement using PID
  int inGoal = 0;                                      // Tracks robot's time in goal threshold
  double currentDelta;                                 // Error between target and current position
  double P = 0, I = 0, D = 0, totalPID;                // PID terms
  double pollingRate = 20;                             // Polling rate in ms
  double target = inches * (360 / (2 * M_PI * 1.375)); // Target position in degrees (1.375 is wheelRadius)

  inches *= 48.0 / 36; // Account for gear ratio

  double previousDelta = target; // Initialize previous error as target
  double integralSum = 0;        // Cumulative error for integral term

  double startTime = pros::millis();
  double timeout = 3000;                                 // max time before PID times out
  double goalsNeeded = (fabs(inches) / 5) * pollingRate; // makes time spent in goal proportional to distance
  if (goalsNeeded == 0)
  { // sets bounds (max & min) for goals needed to reach goal
    goalsNeeded = 1;
  }
  else if (goalsNeeded > 5)
  {
    goalsNeeded = 5;
  }

  // Reset motor encoder value to 0
  left_motors.set_zero_position(left_motors.get_position(0),0);

  while (inGoal < goalsNeeded) // CHECK IF IT SHOULD BE A < or <=
  {
    // Main PID loop; runs until target is reached
    // Read motor position (you can average left and right motor values for straight driving)
    double currentPosition = (left_motors.get_position(0));

    // Calculate the current error
    currentDelta = target - currentPosition;

    // Proportional: Larger error results in larger response
    P = (kP / 1000) * currentDelta;

    // Integral: Sum of all errors helps correct for small errors over time
    integralSum += currentDelta;
    I = kI * integralSum;

    // Derivative: React to the rate of error change
    D = kD * (currentDelta - previousDelta) / pollingRate;

    // Calculate total PID response
    totalPID = P + I + D;

    // Use totalPID to move motors proportionally
    left_motors.move_velocity(totalPID*6);
    right_motors.move_velocity(totalPID*6);

    // Check if the error is small enough to stop
    if (fabs(currentDelta) < goalThreshold)
    {
      inGoal++;
    }
    else
    {
      inGoal = 0;
    }
    // Check if should timeout
    if ((pros::millis() - startTime) >= timeout)
    {
      break;
    }

    // Update the previous error for the next loop
    previousDelta = currentDelta;

    // Wait for the polling rate before next iteration
    delay(pollingRate);
  }
  // Stop the motors once goal is met
  left_motors.brake();
  right_motors.brake();
}

void driveInches(double inches, int veloc, bool clamping)
{

  // adjusted inches based on gear ratios
  double adjustedInches = inches * (48/36);

  // conversion from inches to degrees
  double degrs = (adjustedInches * 180) / (1.375 * M_PI);

  double average = 0;
  left_motors.set_zero_position(left_motors.get_position(0),0);

  // Determine direction of movement
  int dirType = (degrs >= 0) ? 1 : -1;
  // Use absolute value for comparisons
  double targetDegrees = fabs(degrs);
  double slowdownThreshold = (degrs >= 0) ? 250 : 150;

  while ((dirType == 1 && average < degrs) ||
         (dirType == -1 && average > degrs))
  {
    // Calculate average position
    average = (left_motors.get_position(0));

    // Spin motors in appropriate direction
    left_motors.move_velocity(veloc*dirType*6);
    right_motors.move_velocity(veloc*dirType*6);

    // Handle slowdown and clamp
    double distanceRemaining = fabs(degrs - average);
    if (veloc > 15 && distanceRemaining < slowdownThreshold)
    {
      veloc = veloc * (distanceRemaining / targetDegrees);
      // Set clamp opposite to current value
      if (clamping)
      {
        clamp.set_value(clamp.get_value() == LOW ? HIGH : LOW);
      }
    }

    delay(10);
  }
}

// alias for clamping mode
void driveInchesClamp(double inches, int veloc) { driveInches(inches, veloc, true); }
