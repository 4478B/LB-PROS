/**
 * @file auton_selector.cpp
 * @brief Implements the AutonSelector class and defines the competition routine list.
 *
 * To add a new routine to the selector:
 *   1. Declare and implement the function in auton_routes.h / auton_routes.cpp.
 *   2. Add a new entry to COMPETITION_ROUTINES below:
 *        {"Display Name", functionPointer, parameter}
 *   3. The selector will automatically include it in the cycle.
 */

#include "auton_selector.h"
#include "auton_routes.h"
#include "testing.h"
#include "devices.h"
#include <iostream>

/**
 * Represents one selectable autonomous routine.
 *   displayName – shown on the brain LCD during pre-match selection
 *   func        – pointer to the autonomous function (signature: void f(int))
 *   parameter   – integer argument forwarded to func() when it runs (usually 1)
 */
struct AutonRoutine {
    std::string displayName;
    std::function<void(int)> func;
    int parameter = 0;
};

// Constructor
AutonSelector::AutonSelector(const AutonRoutine* routinesArray, size_t routineCount, bool combineTesting, const AutonRoutine* extraRoutinesArray, size_t extraCount) {

    // Add main routines
    for (size_t i = 0; i < routineCount; i++) {
        if (routinesArray[i].func == nullptr) {
            pros::lcd::clear_line(3);
            pros::lcd::print(3, "Routine %d has null function", i);
        }
        routines.push_back(routinesArray[i]);
    }

    // Add extra routines if required
    if (combineTesting && extraRoutinesArray != nullptr) {
        for (size_t i = 0; i < extraCount; i++) {
            if (extraRoutinesArray[i].func == nullptr) {
                pros::lcd::clear_line(3);
                pros::lcd::print(3, "Extra routine %d has null function", i);
            }
            routines.push_back(extraRoutinesArray[i]);
        }
    }

}

// Method implementations
void AutonSelector::displaySelectionBrain() {
    if (currentSelection < 1 || currentSelection > routines.size()) {
        pros::lcd::clear_line(4);
        pros::lcd::print(4, "Invalid selection: %i", currentSelection);
        return;
    }
    pros::lcd::clear_line(1);
    pros::lcd::print(1, "Alliance: %s", red ? "Red" : "Blue");
    pros::lcd::clear_line(2);
    pros::lcd::print(2, "%s",routines[currentSelection - 1].displayName.c_str());
}

void AutonSelector::prevSelection() {
    currentSelection = (currentSelection - 2 + routines.size()) % routines.size() + 1;
}

void AutonSelector::nextSelection() {
    currentSelection = currentSelection % routines.size() + 1;
}

void AutonSelector::runSelection() {
    if (currentSelection < 1 || currentSelection > routines.size()) {
        pros::lcd::clear_line(4);
        pros::lcd::print(4, "Invalid selection: %d", currentSelection);
        return;
    }

    const AutonRoutine& selectedRoutine = routines[currentSelection - 1];

    if (selectedRoutine.func) {
        selectedRoutine.func(selectedRoutine.parameter);
    }
}

int AutonSelector::getRoutineCount() const {
    return routines.size();
}

// ─── Competition Routine List ──────────────────────────────────────────────────
// This array drives the brain-screen selector during pre-match.
// Order here = order shown on screen. Cycle with left/right LCD buttons.
const AutonRoutine COMPETITION_ROUTINES[] = {
    {"Full AWP LEFT SIDE",       newfullLocalAWP, 1}, // Primary match AWP (left start)
    {"Full FIELD SKILLS",        skillsNew,       1}, // 60-second skills run
    {"Elim 9 Ball",              tylerAuton,      1}, // 9-ball elimination route
    {"Odom AWP Right",           odomAWPHigh,     1}, // Right-side AWP using odometry
    {"Left 7 Ball Push",         leftPush,        1}, // Left-side push routine
    {"Right 7 Ball Push Fast",   rightPushFast,   1}, // Fast right-side push
};


const bool isTestingCombined = false;

AutonSelector competitionSelector(COMPETITION_ROUTINES, sizeof(COMPETITION_ROUTINES) / sizeof(COMPETITION_ROUTINES[0]));

void on_left_button() {
    competitionSelector.prevSelection();
    competitionSelector.displaySelectionBrain();
}

void on_right_button() {
    competitionSelector.nextSelection();
    competitionSelector.displaySelectionBrain();
}
