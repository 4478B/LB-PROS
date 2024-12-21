#ifndef AUTON_ROUTES_H
#define AUTON_ROUTES_H

extern int autonSection;

void drivePIDOdom(double goalInches, bool clamping = false, double clampDistInches = 2);
void endSection(int delay = 0);
void progSkills(int i);
void blueGoalSide(int i);
void redGoalSide(int i);
void redGoalSidePostWPI(int i);
void blueRingSide(int i);
void redRingSide(int i);
void WPIAWP(int i);
void allianceRedRingSide(int i);
void allianceBlueRingSide(int i);
void redRingRush(int i);
void oldRedRingSide(int i);

#endif // AUTON_ROUTES_H