/**
 * @file auton_selector.h
 * @brief Brain-screen autonomous routine selector used during competition_initialize().
 *
 * During the pre-match period (when connected to a field control switch) the driver
 * navigates the selector with the brain's three LCD buttons:
 *   Left button  → prevSelection()   (cycles backward through routines)
 *   Right button → nextSelection()   (cycles forward through routines)
 *   Center button → toggle red/blue alliance color
 *
 * When autonomous starts, runSelection() executes whichever routine is currently shown.
 *
 * Usage:
 *   1. Define an array of AutonRoutine structs (see auton_selector.cpp for COMPETITION_ROUTINES).
 *   2. Construct an AutonSelector with that array.
 *   3. Call displaySelectionBrain() to show the current selection on the LCD.
 *   4. Call runSelection() inside autonomous() to run the chosen route.
 */

#ifndef AUTON_SELECTOR_H
#define AUTON_SELECTOR_H

#include <string>
#include <vector>
#include <functional>

// Forward declaration – full definition is in auton_selector.cpp
struct AutonRoutine;

/**
 * Manages a list of autonomous routines and tracks which one is currently selected.
 * The selection wraps around (circular), so prevSelection() from the first item
 * jumps to the last, and nextSelection() from the last jumps back to the first.
 */
class AutonSelector
{
private:
    std::vector<AutonRoutine> routines;  // All available routines (main + optional extra)
    int currentSelection;                // 1-based index of the currently selected routine

public:
    /**
     * @param routinesArray      Primary array of routines (always included).
     * @param routineCount       Number of entries in routinesArray.
     * @param combineTesting     If true, also appends extraRoutinesArray entries.
     * @param extraRoutinesArray Optional second array (e.g., testing-only routes).
     * @param extraCount         Number of entries in extraRoutinesArray.
     */
    AutonSelector(const AutonRoutine *routinesArray, size_t routineCount, bool combineTesting = false, const AutonRoutine *extraRoutinesArray = nullptr, size_t extraCount = 0);

    /** Prints the current alliance color and selected routine name to the brain LCD. */
    void displaySelectionBrain();

    /** (Unused) Would print the current selection to the controller screen. */
    void displaySelectionController();

    /** Cycles the selection one step backward (wraps around). */
    void prevSelection();

    /** Cycles the selection one step forward (wraps around). */
    void nextSelection();

    /** Executes the currently selected autonomous routine. */
    void runSelection();

    /** Returns the total number of routines available. */
    int getRoutineCount() const;
};

// Extern declarations for global objects

extern const AutonRoutine COMPETITION_ROUTINES[];
extern const AutonRoutine TESTING_ROUTINES[];
extern AutonSelector competitionSelector;
extern AutonSelector testingSelector;

// Function declarations
void on_left_button();
void on_right_button();

/**
 * Switches the brain LCD buttons into push-delay adjustment mode.
 * LEFT/RIGHT = ±500 ms, CENTER = confirm and return to routine selection.
 * Register as the CENTER button callback in competition_initialize().
 */
void enterPushDelayMode();

#endif // AUTON_SELECTOR_H
