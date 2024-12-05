#include "devices.h"

// function for drivePID using lemlib pids
void DrivePIDLL(double goalInches)
{
  double currentPos;
  double error;
  double nextMovement;

  // account for gear ratio
  goalInches *= 48 / 36;

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
    left_motors.move(nextMovement);
    right_motors.move(nextMovement);

    pros::delay(20);
  }

  // reset PID for next usage
  lateralPID.reset();

  // stop arm motors in place
  left_motors.brake();
  right_motors.brake();
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
    left_motors.move(nextMovement);
    right_motors.move(-nextMovement);

    pros::delay(20);

    if (fabs(error) < 1.5)
    {
      oscillation++;
    }
  }

  // reset PID for next usage
  angularPID.reset();

  // stop arm motors in place
  left_motors.brake();
  right_motors.brake();
}
