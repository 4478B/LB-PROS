#ifndef OLD_SYSTEMS_H
#define OLD_SYSTEMS_H

void drivePID(double inches, int timeout = 3000, double kP = 50, double kI = 0, double kD = 0, double goalThreshold = .5, bool clamping = false);
void drivePIDClamp(double inches, int timeout = 3000, double kP = 50, double kI = 0, double kD = 0, double goalThreshold = .5);
// Open-loop Driving
//void driveInches(double fwdVal, int veloc, bool clamping = false);
//void driveInchesClamp(double fwdVal, int veloc);
//void inert(double target, double kP = 0.499, double kI = 0, double kD = 0.002);
#endif // OLD_SYSTEMS_H