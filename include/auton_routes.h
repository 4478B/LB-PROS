#ifndef AUTON_ROUTES_H
#define AUTON_ROUTES_H

extern int autonSection;

void drivePIDOdom(double goalInches, bool clamping = false, double clampDistInches = 2);
bool endSection(int delay = 0);

void park();
void fullAWPLeft(int i);
void fullAWPRight(int i);
void fullLocalAWP(int i);
void newfullLocalAWP(int i);
void odomAWP(int i);
void odomAWPHigh(int i);
void tylerAuton(int i);
void testPID(int i);
void skills(int i);
void skillsNew(int i);
void leftPush(int i);
void rightPush(int i);
void skillsFinal(int i);
void nineBallRight(int i);
void nineBallLeft(int i);
#endif // AUTON_ROUTES_H