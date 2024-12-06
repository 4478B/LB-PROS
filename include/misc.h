#ifndef MISC_H
#define MISC_H

void drivePIDLL(double goalInches);
void drivePIDWTF(double goalInches, bool clamping = false, double clampDistInches = 2);

#endif // MISC_H