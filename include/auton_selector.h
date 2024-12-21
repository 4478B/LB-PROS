#ifndef AUTON_SELECTOR_H
#define AUTON_SELECTOR_H

#include <string>
#include <vector>
#include <functional>

// Forward declaration of AutonRoutine structure
struct AutonRoutine;

// Declaration of AutonSelector class
class AutonSelector {
private:
    std::vector<AutonRoutine> routines;
    int currentSelection;

public:
    // Constructor
    AutonSelector(const AutonRoutine* routinesArray, size_t routineCount, bool combineTesting = false, const AutonRoutine* extraRoutinesArray = nullptr, size_t extraCount = 0);

    // Display methods
    void displaySelectionBrain();
    void displaySelectionController();

    // Navigation methods
    void prevSelection();
    void nextSelection();

    // Execution method
    void runSelection();

    // Utility method
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

#endif // AUTON_SELECTOR_H
