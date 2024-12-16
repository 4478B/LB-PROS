#include "devices.h"
#include "main.h"

// function for drivePID using lemlib pids
void drivePIDLL(double goalInches)
{
  double currentPos;
  double error;
  double nextMovement;

  // account for gear ratio
  goalInches *= 48.0 / 36;

  // reset encoder before usage
  left_motors.set_zero_position(left_motors.get_position(0), 0);

  while (fabs(error) < 30)
  {
    // current position
    currentPos = left_motors.get_position(0);

    // calculate how far chassis is from target
    error = goalInches - currentPos;

    // determine how far to move based on PID
    nextMovement = lateralPID.update(error);

    // ensure values fit bounds of motor voltage
    nextMovement = std::clamp(nextMovement, -127.0, 127.0);

    // move arm motors based on PID
    all_motors.move(nextMovement);

    pros::delay(20);
  }

  // reset PID for next usage
  lateralPID.reset();

  // stop arm motors in place
  all_motors.brake();
}

// function for inert using lemlib pids
void inertLL(double degrees)
{
  double currentPos;
  double error;
  double nextMovement;

  int oscillation = 0;

  while (oscillation < 2)
  {
    // current position
    currentPos = imu.get_heading();

    // calculate how far chassis is from target
    error = degrees - currentPos;

    // determine how far to move based on PID
    nextMovement = angularPID.update(error);

    // ensure values fit bounds of motor voltage
    nextMovement = std::clamp(nextMovement, -127.0, 127.0);

    // move arm motors based on PID
    all_motors.move(nextMovement);

    pros::delay(20);

    if (fabs(error) < 1.5)
    {
      oscillation++;
    }
  }

  // reset PID for next usage
  angularPID.reset();

  // stop arm motors in place
  all_motors.brake();
}

// This is a really jenk implementation of drivePID with odometry
// It works relatively and then updates the global positions based on the change in movement
void drivePIDWTF(double goalInches, bool clamping, double clampDistInches)
{
  // Step 1: Get the current robot pose from odometry.
  Pose poseInit(chassis.getPose());

  // Step 2: Set dummy pose to run moveToPoint relatively
  chassis.setPose(0, 0, 0);
  chassis.setPose(0, 0, 0);
  chassis.setPose(0, 0, 0);
  chassis.setPose(0, 0, 0);

  // Step 3: Determine whether the movement is forward or backward
  bool isForwards = goalInches > 0;

  // Step 4: Move to the calculated point
  chassis.moveToPoint(0, goalInches, 4000, {.forwards = isForwards,.maxSpeed = 127,.minSpeed = 10,.earlyExitRange = 1});

  // Step 5: use change in pose to update global pose
  Pose poseDelta(chassis.getPose());
  Pose poseGlobal(poseInit.x + poseDelta.x, poseInit.y + poseDelta.y, poseInit.theta + poseDelta.theta);
  //chassis.setPose(poseGlobal);

  // Step 6: Print debug information for testing pose calculations.
  // Output trimmed to 3 decimal places to fit the screen.
  pros::lcd::print(3, "Pose Init: X: %.3f, Y: %.3f, Th: %.3f", poseInit.x, poseInit.y, poseInit.theta);
  pros::lcd::print(4, "Pose Delta: X: %.3f, Y: %.3f, Th: %.3f", poseDelta.x, poseDelta.y, poseDelta.theta);
  pros::lcd::print(5, "Pose Global: X: %.3f, Y: %.3f, Th: %.3f", poseGlobal.x, poseGlobal.y, poseGlobal.theta);
}