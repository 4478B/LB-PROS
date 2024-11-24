#ifndef AUTON_SELECTOR_H
#define AUTON_SELECTOR_H

#include <functional>

// Forward declaration of internal structs/classes
struct AutonRoutine {
    const char* displayName;
    std::function<void()> routine;
};

class AutonSelector {
private:
    const AutonRoutine* routines;  // Pointer to array of routines
    int currentSelection;          // Current selected routine
    int routineCount;             // Total number of routines

public:
    AutonSelector();  // Constructor
    void nextSelection();
    void prevSelection();         // Added prevSelection declaration
    void displayCurrentSelection();
    void runSelectedAuton();
    int getCurrentSelection() const;
};

// Global instance getter
AutonSelector& getAutonSelector();

// button callback forward declarations for LCD screen
void on_left_button();
void on_right_button();

#endif // AUTON_SELECTOR_H