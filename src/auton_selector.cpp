#include "auton_selector.h"
#include "main.h"
#include "devices.h"
#include "auton_routes.h"

/*
 * Framework for the autonomous selector.
 * Add/remove routines by editing the "routines" array below.
 */

// Define the routines array
static const AutonRoutine ROUTINES[] = {
    {"Prog Skills", progSkills},        // Route 1
    {"Blue Goal Side", blueGoalSide},   // Route 2
    {"Red Goal Side", redGoalSide},     // Route 3
    {"Blue Ring Side", blueRingSide},   // Route 4
    {"Red Ring Side", redRingSide},     // Route 5
    {"WPI AWP", WPIAWP}                 // Route 6
};

// Create static instance
static AutonSelector instance;

// Constructor implementation
AutonSelector::AutonSelector() : routines(ROUTINES),
                               currentSelection(3), // Start with Prog Skills (Route 1)
                               routineCount(sizeof(ROUTINES) / sizeof(ROUTINES[0]))
{
}

void AutonSelector::nextSelection()
{
    currentSelection = (currentSelection % routineCount) + 1;
    displayCurrentSelection();
}

void AutonSelector::prevSelection()
{
    // Branchless implementation
    currentSelection = ((currentSelection + routineCount - 2) % routineCount) + 1;
    displayCurrentSelection();
}

void AutonSelector::displayCurrentSelection()
{
    pros::lcd::clear_line(2);
    pros::lcd::print(2, routines[currentSelection - 1].displayName);
}

void AutonSelector::runSelectedAuton()
{
    const AutonRoutine &selected = routines[currentSelection - 1];
    selected.routine();  // Removed multiplier parameter
}

int AutonSelector::getCurrentSelection() const
{
    return currentSelection;
}

// Global instance getter implementation
AutonSelector &getAutonSelector()
{
    return instance;
}

void on_left_button() {
    getAutonSelector().prevSelection();
}

void on_right_button() {
    getAutonSelector().nextSelection();
}
