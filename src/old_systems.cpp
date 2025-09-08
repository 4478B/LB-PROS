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
const double GEAR_RATIO = 36.0 / 48;            // Ratio for gear adjustment
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

  /*// make sure clamp is up if clamping
  double clampState = clamp.get_value();
  if(clamping && clampState == LOW){
    clamp.set_value(HIGH);
    clampState = HIGH;
  }*/

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
    double currentPosition = (all_motors.get_position(0) + all_motors.get_position(1) + all_motors.get_position(2) + all_motors.get_position(3) + all_motors.get_position(4) + all_motors.get_position(5)) / 6.0;
    pros::lcd::print(0, "LM1 Pos: %f", all_motors.get_position(0));
    pros::lcd::print(1, "LM2 Pos: %f", all_motors.get_position(1));
    pros::lcd::print(2, "LM3 Pos: %f", all_motors.get_position(2));
    pros::lcd::print(3, "RM1 Pos: %f", all_motors.get_position(3));
    pros::lcd::print(4, "RM2 Pos: %f", all_motors.get_position(4));
    pros::lcd::print(5, "RM3 Pos: %f", all_motors.get_position(5));
  

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

    // POTENTIAL FIX BELOW
    //totalPID *= 600/127.0;
    //all_motors.move_velocity(totalPID);
    
    left_motors.move(totalPID);
    right_motors.move(totalPID);
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
    if (clamping && clamp.get_value() == HIGH && fabs(currentDelta) < CLAMP_DISTANCE){

      clamp.set_value(LOW);

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
    pros::lcd::print(6, "Current Pos: %f inches", currentPositionInInches);

    // Convert currentDelta back to inches
    double currentDeltaInInches = currentDelta * WHEEL_CIRCUMFERENCE / GEAR_RATIO;
    pros::lcd::print(7, "Target Delta: %f inches", currentDeltaInInches);

    // Display the PID output (already in motor velocity units, no conversion needed)
    pros::lcd::print(8, "Next Movement: %f", totalPID);
    */

    // Wait for the polling rate before next iteration
    delay(pollingRate);
  }
  // Stop the motors once goal is met
  all_motors.brake();
}

void drivePIDCurve(double inches, int timeout, double kP, double leftPowerPCT, double rightPowerPCT) {
  double kI = 0.0;
  double kD = 0.0;
  // Function to control robot movement using PID with different power for left and right motors
  all_motors.set_encoder_units_all(E_MOTOR_ENCODER_ROTATIONS);
  double currentDeltaLeft, currentDeltaRight;   // Errors for left and right motors
  double P = 0, I = 0, D = 0, totalPID;        // PID terms
  double pollingRate = 20;                     // Polling rate in ms
  // Convert inches into encoder rotations

  double targetLeft = (inches * GEAR_RATIO / WHEEL_CIRCUMFERENCE) * (leftPowerPCT / 100.0);  // Adjust target for left motor based on power percentage
  double targetRight = (inches * GEAR_RATIO / WHEEL_CIRCUMFERENCE) * (rightPowerPCT / 100.0); // Adjust target for right motor based on power percentage

  double previousDeltaLeft = targetLeft;  // Initialize previous error for left motor
  double previousDeltaRight = targetRight; // Initialize previous error for right motor
  double integralSumLeft = 0;         // Cumulative error for left motor
  double integralSumRight = 0;        // Cumulative error for right motor

  // Reset motor encoder value to 0
  all_motors.tare_position_all();

  double startTime = pros::millis(); // Start time for timeout
  double goalsNeededLeft = (fabs(inches) / 5) * pollingRate * (leftPowerPCT / 100.0);  // Goals proportional to left motor power
  double goalsNeededRight = (fabs(inches) / 5) * pollingRate * (rightPowerPCT / 100.0); // Goals proportional to right motor power

  goalsNeededLeft = std::clamp(goalsNeededLeft, 1.0, 5.0);   // Clamp goals for left motor
  goalsNeededRight = std::clamp(goalsNeededRight, 1.0, 5.0); // Clamp goals for right motor

  int inGoalLeft = 0;  // Tracks time spent in goal threshold for left motor
  int inGoalRight = 0; // Tracks time spent in goal threshold for right motor

  while (inGoalLeft < goalsNeededLeft || inGoalRight < goalsNeededRight) {
    // Main PID loop; runs until target is reached for both motors
    double currentPositionLeft = (all_motors.get_position(0) + all_motors.get_position(1) + all_motors.get_position(2)) / 3.0;
    double currentPositionRight = (all_motors.get_position(3) + all_motors.get_position(4) + all_motors.get_position(5)) / 3.0;

    // Calculate the current errors
    currentDeltaLeft = targetLeft - currentPositionLeft;
    currentDeltaRight = targetRight - currentPositionRight;

    // Proportional: Larger error results in larger response
    double PLeft = kP * currentDeltaLeft;
    double PRight = kP * currentDeltaRight;

    // Integral: Sum of all errors helps correct for small errors over time
    integralSumLeft += currentDeltaLeft;
    integralSumRight += currentDeltaRight;
    double ILeft = kI * integralSumLeft;
    double IRight = kI * integralSumRight;
    ILeft = std::clamp(ILeft, -50.0, 50.0);
    IRight = std::clamp(IRight, -50.0, 50.0);

    // Derivative: React to the rate of error change
    double DLeft = kD * (currentDeltaLeft - previousDeltaLeft) / pollingRate;
    double DRight = kD * (currentDeltaRight - previousDeltaRight) / pollingRate;

    // Calculate total PID response
    double totalPIDLeft = PLeft + ILeft + DLeft;
    double totalPIDRight = PRight + IRight + DRight;
    totalPIDLeft = std::clamp(totalPIDLeft, -127.0, 127.0);
    totalPIDRight = std::clamp(totalPIDRight, -127.0, 127.0);

    // Apply power proportionally to left and right motors
    double leftPower = totalPIDLeft * (leftPowerPCT / 100.0);
    double rightPower = totalPIDRight * (rightPowerPCT / 100.0);

    left_motors.move(leftPower);
    right_motors.move(rightPower);

    // Check if the error is small enough to stop for left motor
    if (fabs(currentDeltaLeft) < (targetLeft)) {
      inGoalLeft++;
    } else {
      inGoalLeft = 0;
    }

    // Check if the error is small enough to stop for right motor
    if (fabs(currentDeltaRight) < (targetRight)) {
      inGoalRight++;
    } else {
      inGoalRight = 0;
    }

    // Check if timeout is reached
    if ((pros::millis() - startTime) >= timeout) {
      break;
    }

    // Update the previous errors for the next loop
    previousDeltaLeft = currentDeltaLeft;
    previousDeltaRight = currentDeltaRight;

    // Wait for the polling rate before next iteration
    delay(pollingRate);
  }

  // Stop the motors once goal is met
  all_motors.brake();
}
