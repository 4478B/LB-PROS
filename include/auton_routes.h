#ifndef AUTON_ROUTES_H
#define AUTON_ROUTES_H

extern int autonSection;

void drivePIDOdom(double goalInches, bool clamping = false, double clampDistInches = 2);
void endSection(int delay = 0);
void soloPushRight(int i);
void soloPushLeft(int i);
void ladyBrownRushRight(int i);
void newRingSideRight(int i);
void progSkills(int i);
void blueGoalSide(int i);
void redGoalSide(int i);
void redGoalSidePostWPI(int i);
void blueRingSide(int i);
void redRingSide(int i);
void WPIAWP(int i);
void safeAWPLeft(int i);
void safeAWPRight(int i);
void danburyRedRS(int i);
void danburyBlueRS(int i);
void redRingRush(int i);
void oldRedRingSide(int i);
void redGoalSideSugarRush(int i);
void blueGoalSideSugarRush(int i);

#endif // AUTON_ROUTES_H