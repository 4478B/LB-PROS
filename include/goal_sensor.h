#ifndef GOAL_SENSOR_H
#define GOAL_SENSOR_H


void driveClamp(int volts, int maxDist, int maxTime);
bool isGoalClamped();

#endif // GOAL_SENSOR_H