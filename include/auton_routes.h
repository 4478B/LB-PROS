#ifndef AUTON_ROUTES_H
#define AUTON_ROUTES_H

extern int autonSection;

void drivePIDOdom(double goalInches, bool clamping = false, double clampDistInches = 2);
void endSection(int delay = 0);
void progSkills();
void blueGoalSide();
void redGoalSide();
void blueRingSide();
void redRingSide();

#endif // AUTON_ROUTES_H