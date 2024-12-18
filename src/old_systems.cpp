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
#include <numeric>

// Define constants for conversions
const double WHEEL_RADIUS = 1.375;               // Inches
const double WHEEL_CIRCUMFERENCE = 2 * M_PI * WHEEL_RADIUS; // Circumference in inches
const double GEAR_RATIO = 48.0 / 36;            // Ratio for gear adjustment
const double CLAMP_DISTANCE = 1;



void drivePID(double inches, int timeout, double kP, double kI, double kD, double goalThreshold, bool clamping)
{
  all_motors.set_encoder_units_all(E_MOTOR_ENCODER_ROTATIONS);
  // Function to control robot movement using PID
  int inGoal = 0;                       // Tracks robot's time in goal threshold
  double currentDelta;                  // Error between target and current position
  double P = 0, I = 0, D = 0, totalPID; // PID terms
  double pollingRate = 20;              // Polling rate in ms
  // Convert inches into encoder rotations

  double target = inches * GEAR_RATIO / WHEEL_CIRCUMFERENCE; // Encoder rotations for target distance
  goalThreshold *= GEAR_RATIO / WHEEL_CIRCUMFERENCE; 

  double previousDelta = target; // Initialize previous error as target
  double integralSum = 0;        // Cumulative error for integral term

  // make sure clamp is up if clamping
  double clampState = clamp.get_value();
  if(clamping && clampState == LOW){
    clamp.set_value(HIGH);
    clampState = HIGH;
  }

  double startTime = pros::millis();                                // max time before PID times out
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
  all_motors.tare_position_all();

  while (inGoal < goalsNeeded) // CHECK IF IT SHOULD BE A < or <=
  {
    // Main PID loop; runs until target is reached
    // Read motor position (you can average left and right motor values for straight driving)
    
    // finds average motor position
    std::vector<double> positions = all_motors.get_position_all();
    double currentPosition = std::accumulate(positions.begin(), positions.end(), 0.0) / positions.size();
    
    // Calculate the current error
    currentDelta = target - currentPosition;

    // Proportional: Larger error results in larger response
    P = kP * currentDelta;

    // Integral: Sum of all errors helps correct for small errors over time
    integralSum += currentDelta;
    I = kI * integralSum;

    I = std::clamp(I,-50.0,50.0);

    // Derivative: React to the rate of error change
    D = kD * (currentDelta - previousDelta) / pollingRate;

    // Calculate total PID response
    totalPID = P + I + D;

    // Use totalPID to move motors proportionally
    totalPID = std::clamp(totalPID,-127.0,127.0);

    all_motors.move(totalPID);

    // Check if the error is small enough to stop
    if (fabs(currentDelta) < goalThreshold)
    {
      inGoal++;
    }
    else
    {
      inGoal = 0;
    }
    
    // Check if should clamp
    if (clamping && clampState == HIGH && fabs(currentDelta) < CLAMP_DISTANCE){

      clamp.set_value(LOW);
      clampState = LOW;

    }

    // Check if should timeout
    if ((pros::millis() - startTime) >= timeout)
    {
      break;
    }

    // Update the previous error for the next loop
    previousDelta = currentDelta;
    /*
    // Convert currentPosition back to inches
    double currentPositionInInches = currentPosition * WHEEL_CIRCUMFERENCE / GEAR_RATIO;
    pros::lcd::print(3, "Current Pos: %f inches", currentPositionInInches);

    // Convert currentDelta back to inches
    double currentDeltaInInches = currentDelta * WHEEL_CIRCUMFERENCE / GEAR_RATIO;
    pros::lcd::print(4, "Target Delta: %f inches", currentDeltaInInches);

    // Display the PID output (already in motor velocity units, no conversion needed)
    pros::lcd::print(6, "Next Movement: %f", totalPID);
    */

    // Wait for the polling rate before next iteration
    delay(pollingRate);
  }
  // Stop the motors once goal is met
  all_motors.brake();
}

// alias for clamping
void drivePIDClamp(double inches, int timeout, double kP, double kI, double kD, double goalThreshold){
  drivePID(inches, timeout, kP, kI, kD, goalThreshold, true);
}

/*void inert(double target, double kP, double kI, double kD)
{
  bool isComplete = false;
  double startTime = pros::millis(), prevTime = startTime, deltaTime, currentTime;
  double integral = 0, error = 0, derivative = 0, prevError = 0;
  double currentDeg = imu.get_heading();
  double output;
  int oscillation = 0;
  while (!isComplete)
  {
    //////////////////setting values
    currentTime = pros::millis();
    deltaTime = currentTime - prevTime;
    prevTime = currentTime;
    currentDeg = imu.get_heading();

    // calculations

    error = target - currentDeg;

    if (error > 180)
    {
      error = error - 360;
    }
    else if (error < -180)
    {
      error = 360 + error;
    }

    derivative = (error - prevError) / deltaTime;

    prevError = error;

    integral = +error * deltaTime;

    output = kP * error + kI * integral + kD * derivative;

    //  output = turnSlew(output);

    //////////////////////// motor output

    left_motors.move_velocity(output * 6);
    right_motors.move_velocity(-output * 6);

    //      Printing values

    //      Exiting loop
    if (((fabs(prevError) < 1.5)))
    {
      oscillation++;
      if (oscillation > 1)
      {
        isComplete = true;
        left_motors.brake();
        right_motors.brake();
        break;
      }
    }
    delay(20);
  }
}

void driveInches(double inches, int veloc, bool clamping)
{

  // adjusted inches based on gear ratios
  double adjustedInches = inches * (48 / 36);

  // conversion from inches to degrees
  double degrs = (adjustedInches * 180) / (1.375 * M_PI);

  double average = 0;
  left_motors.tare_position_all();

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
    left_motors.move_velocity(veloc * dirType * 6);
    right_motors.move_velocity(veloc * dirType * 6);

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
*/