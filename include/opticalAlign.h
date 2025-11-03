#ifndef __OPTICAL_ALIGN__
#define __OPTICAL_ALIGN__

// Align to long goal using back optical sensor
// Sweeps in both directions to find goal, then backs into it while maintaining alignment
extern void alignToLongGoal();

// Align to long goal with intended heading
// @param intendedHeading The expected heading of the goal (determines sweep direction)
extern void alignToLongGoal(double intendedHeading);

// Align to long goal with intended heading and optional backup
// @param intendedHeading The expected heading of the goal (determines sweep direction)
// @param shouldBackup Whether to back into the goal after finding it
extern void alignToLongGoal(double intendedHeading, bool shouldBackup);

// Align to long goal using averaged heading from sweep
// @param intendedHeading The expected heading of the goal (determines sweep direction)
extern void alignToLongGoalAverage(double intendedHeading);

// Align to long goal using averaged heading from sweep with optional backup
// @param intendedHeading The expected heading of the goal (determines sweep direction)
// @param shouldBackup Whether to back into the goal after finding it
extern void alignToLongGoalAverage(double intendedHeading, bool shouldBackup);

#endif