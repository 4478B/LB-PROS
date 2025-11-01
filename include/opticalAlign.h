#ifndef __OPTICAL_ALIGN__
#define __OPTICAL_ALIGN__

// Align to long goal using back optical sensor
// Slowly turns to find goal, then backs into it while maintaining alignment
extern void alignToLongGoal();

// Align to long goal using averaged heading from sweep
// @param intendedHeading The expected heading of the goal (determines sweep direction)
extern void alignToLongGoalAverage(double intendedHeading);

#endif