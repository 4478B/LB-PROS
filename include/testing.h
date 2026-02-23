/**
 * @file testing.h
 * @brief Utilities for testing autonomous routines during driver practice sessions.
 *
 * testAuton() allows the driver to trigger autonomous routines from the controller
 * during opcontrol (without a competition switch) so autons can be re-run and
 * debugged repeatedly without needing to reset the program.
 *
 * totalTime / prevTime are shared section-timer variables updated by endSection()
 * in auton_routes.cpp to track how long each auton section takes.
 */

#ifndef TESTING_H
#define TESTING_H

/**
 * Triggered inside opcontrol() when NOT connected to a competition switch.
 * If A + X + Y are all held simultaneously (and inputReq is true), runs the
 * autonomous routine hard-coded at the bottom of this function.
 * Also prints per-section timing tables to the console for post-run analysis.
 *
 * @param inp  If true (default), requires A+X+Y to start. If false, runs immediately.
 */
void testAuton(bool inp = true);

void testRandom(); // Placeholder / unused

extern int totalTime; // Accumulated auton time in ms across all sections
extern int prevTime;  // Timestamp of the start of the current section (ms)

#endif // TESTING_H