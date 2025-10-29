#ifndef AUTON_ROUTES_H
#define AUTON_ROUTES_H

extern int autonSection;

void drivePIDOdom(double goalInches, bool clamping = false, double clampDistInches = 2);
bool endSection(int delay = 0);

void fullAWPLeft(int i);
void fullAWPRight(int i);
void fullLocalAWP(int i);
void tylerAuton(int i);
void skills(int i);
void skillsNew(int i);

#endif // AUTON_ROUTES_H