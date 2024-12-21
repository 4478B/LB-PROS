#ifndef TESTING_H
#define TESTING_H

void testRingSens(int i);
void testGoalSens(int i);
void testOdometryStraight(int i);
void testOdometryTurn(int i);
void testOdometryBoth(int i);
void testAuton(bool inp = true);
void testRandom();
extern int totalTime;
extern int prevTime;

#endif // TESTING_H