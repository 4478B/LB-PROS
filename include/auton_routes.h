/**
 * @file auton_routes.h
 * @brief Declarations for every autonomous routine and helper used during the 15-second auto period.
 *
 * All routines follow the signature `void routineName(int i)` where `i` is an
 * unused parameter kept for compatibility with AutonSelector's function pointer type.
 *
 * Coordinate system (LemLib):
 *   Origin = robot start position.  +X = right,  +Y = forward,  theta = heading in degrees.
 *   Positive drivePID distance = forward.
 */

#ifndef AUTON_ROUTES_H
#define AUTON_ROUTES_H

// Tracks the current section index during a split-section auton test (see endSection).
extern int autonSection;

// Variable delay (ms) inserted at the push timing point in leftPushFast / rightPushFast.
// Adjusted in 500 ms increments via selectPushDelay() before a match.
extern int pushDelay;

/**
 * Drives forward/backward `goalInches` inches using odometry-based motion.
 * Computes a goal coordinate from the current pose and calls LemLib moveToPoint.
 * @param goalInches   Target distance in inches (positive = forward).
 * @param clamping     If true, triggers the clamp pneumatic when close to the target.
 * @param clampDistInches  Distance threshold (inches) at which the clamp fires.
 */
void drivePIDOdom(double goalInches, bool clamping = false, double clampDistInches = 2);

/**
 * Marks the end of a named auton section for split testing.
 * - In competition: behaves as a simple delay.
 * - Out of competition: pauses and prints timing/pose info; waits for controller input
 *   (X = continue, A = alternate path, Y = heading correction, B = position correction).
 * @param delay  Milliseconds to wait (competition) or maximum wait time (testing).
 * @return true if the alternate path button (A) was pressed.
 */
bool endSection(int delay = 0);

// ─── Autonomous Routines ──────────────────────────────────────────────────────
// Each function represents one complete 15-second autonomous path.
// The `int i` parameter is unused but required by the AutonSelector callback type.

void park();                 // Drive into the elevated parking zone at end of skills
void fullAWPLeft(int i);     // Full Autonomous Win Point route starting on the left side
void fullAWPRight(int i);    // Full Autonomous Win Point route starting on the right side
void fullLocalAWP(int i);    // Local AWP (no odometry) starting left
void newfullLocalAWP(int i); // Updated local AWP left – current competition-ready route
void odomAWP(int i);         // Odometry-based AWP (uses moveToPose for accuracy)
void odomAWPHigh(int i);     // Odometry AWP that also scores in the high goal
void tylerAuton(int i);      // Elimination 9-ball route (right side start)
void testPID(int i);         // Simple PID drive + turn test (used for tuning)
void skills(int i);          // Full-field skills run (older version)
void skillsNew(int i);       // Full-field skills run (current, uses odometry)
void leftPush(int i);        // 7-ball left-side push route (match play)
void rightPush(int i);       // 7-ball right-side push route (match play)
void skillsFinal(int i);     // Final skills run variant
void testMid(int i);         // Mid-field test route for tuning and diagnostics
void nineBallRight(int i);   // 9-ball route from right alliance zone
void nineBallLeft(int i);    // 9-ball route from left alliance zone
void rightPushFast(int i);   // Fast variant of the right-side push route
void leftPushFast(int i);    // Fast variant of the left-side push route
void empty(int i);
#endif // AUTON_ROUTES_H