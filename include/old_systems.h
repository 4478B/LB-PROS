#ifndef OLD_SYSTEMS_H
#define OLD_SYSTEMS_H

void drivePID(double inches, double kP = 110, double kI = 0, double kD = 0.15, double goalThreshold = 30);
// Open-loop Driving
void driveInches(double fwdVal, int veloc, bool clamping = false);
void driveInchesClamp(double fwdVal, int veloc);
void inert(double target, double kP = 0.499, double kI = 0, double kD = 0.002);
#endif // OLD_SYSTEMS_H