/**
 * @file old_systems.h
 * @brief Custom PID drive function and legacy motion helpers.
 *
 * drivePID() is the workhorse straight-line drive used throughout all auton routes.
 * It drives all 6 drivetrain motors using a PID controller running on motor encoder
 * rotations rather than relying on LemLib's odometry, making it fast and deterministic
 * for short straight segments.
 *
 * "Old systems" refers to this code predating the switch to LemLib; the PID drive
 * is still actively used because it is simpler and more reliable for straight-line
 * moves than LemLib's moveToPoint on routes where heading correction isn't critical.
 */

#ifndef OLD_SYSTEMS_H
#define OLD_SYSTEMS_H

extern double slew(double, double);               // Rate-limiter (unused in current build)
extern void drivePIDTest(double, double=100, int=1000); // Legacy test variant (commented out)

/**
 * Drive forward or backward a specified number of inches using a PID controller.
 *
 * The controller reads all 6 drive-motor encoders, takes their median to reject
 * faulty readings, and outputs a proportional motor command each 20 ms cycle.
 *
 * @param inches         Target distance. Positive = forward, negative = backward.
 * @param timeout        Maximum time (ms) before the PID gives up and brakes.
 * @param kP             Proportional gain (default 40 – quite aggressive).
 * @param kI             Integral gain (default 0 – disabled; avoid unless needed).
 * @param kD             Derivative gain (default 7).
 * @param goalThreshold  Error threshold (inches) considered "close enough" to stop.
 * @param clamping       If true, fires the clamp pneumatic when within CLAMP_DISTANCE.
 */
void drivePID(double inches, int timeout = 1000, double kP = 40, double kI = 0, double kD = 7, double goalThreshold = .2, bool clamping = false);

/** Variant of drivePID with clamping enabled and conservative defaults. */
void drivePIDClamp(double inches, int timeout = 3000, double kP = 50, double kI = 0, double kD = 0, double goalThreshold = .5);

#endif // OLD_SYSTEMS_H