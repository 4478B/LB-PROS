#include "auton_selector.h"
#include "auton_routes.h"
#include "testing.h"
#include "devices.h"
#include <iostream>

// Define the AutonRoutine structure
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

// Global object definitions
const AutonRoutine COMPETITION_ROUTINES[] = {
    {"Prog Skills", progSkills, 0},
    {"Alliance Red Ringside AWP", allianceRedRingSide, 1},
    {"Alliance Blue Ringside AWP", allianceRedRingSide, -1},
    {"Blue Goal Side", redGoalSide}
};

const AutonRoutine TESTING_ROUTINES[] = {
    {"Test Ring Sensor", testRingSens}
};

const bool isTestingCombined = false;

AutonSelector competitionSelector(COMPETITION_ROUTINES, sizeof(COMPETITION_ROUTINES) / sizeof(COMPETITION_ROUTINES[0]));
AutonSelector testingSelector(TESTING_ROUTINES, sizeof(TESTING_ROUTINES) / sizeof(TESTING_ROUTINES[0]), isTestingCombined, COMPETITION_ROUTINES, sizeof(COMPETITION_ROUTINES) / sizeof(COMPETITION_ROUTINES[0]));

void on_left_button() {
    competitionSelector.prevSelection();
    competitionSelector.displaySelectionBrain();
}

void on_right_button() {
    competitionSelector.nextSelection();
    competitionSelector.displaySelectionBrain();
}
