/**
 * @file extended_chassis.h
 * @brief Extends LemLib's Chassis class with clamping motion primitives.
 *
 * LemLib's built-in moveToPoint / moveToPose don't support automatically
 * triggering a pneumatic clamp mid-motion. ExtendedChassis adds two variants
 * that fire the mobile-goal clamp when the robot gets within `clampDist` inches
 * of the target coordinates, so the clamp engages at the right moment without
 * separate timing logic in auton routes.
 *
 * All other Chassis functionality (turnToHeading, setPose, getPose, etc.) is
 * inherited unchanged from lemlib::Chassis.
 */

#ifndef EXTENDED_CHASSIS_H
#define EXTENDED_CHASSIS_H

#include "lemlib/chassis/chassis.hpp"

class ExtendedChassis : public lemlib::Chassis {
public:
    // Inherit all constructors from the base Chassis class
    using lemlib::Chassis::Chassis;

    /**
     * Move backwards to (x, y) and lower the clamp when within clampDist inches.
     * @param x, y        Target field coordinates in inches.
     * @param timeout     Maximum time in milliseconds before the motion is aborted.
     * @param clampDist   Distance (inches) from target at which the clamp fires.
     * @param params      Additional LemLib motion parameters.
     */
    void MoveToPointClamp(float x, float y, int timeout, float clampDist = .5, lemlib::MoveToPointParams params = {});

    /**
     * Move backwards to pose (x, y, theta) and lower the clamp when within clampDist inches.
     * @param theta   Target heading in degrees.
     * @param clampDist   Distance (inches) from target at which the clamp fires.
     */
    void MoveToPoseClamp(float x, float y, float theta, int timeout, float clampDist = .5, lemlib::MoveToPoseParams params = {});

    /** Prints the current odometry pose (X, Y, theta) to three brain LCD lines. */
    void printPose(int line1 = 1, int line2 = 2, int line3 = 3);
};
#endif // EXTENDED_CHASSIS_H