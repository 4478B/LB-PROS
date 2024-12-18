#ifndef COLOR_SORT_H
#define COLOR_SORT_H


// Waits until either red ring is in intake or it times out based on the timeout
bool waitUntilRedIntake(int timeout);
// Waits until either blue ring is in intake or it times out based on the timeout
bool waitUntilBlueIntake(int timeout);
// color sort object structure
// Singleton class for handling color sorting
class colorSortHandler {
public:
    int targetHue;
    int hueMin;
    int hueMax;

private:
    bool isRedAlliance;
    bool isCurrentlySorting;

    // Private constructor for singleton
    colorSortHandler();

public:
    // Static method to access the singleton instance
    static colorSortHandler& getInstance();

    // Prevent copying and assignment
    colorSortHandler(const colorSortHandler&) = delete;
    void operator=(const colorSortHandler&) = delete;

    // Public methods
    void killSwitch();
    void tossRing();
    void swapTeam();

    // Accessor methods
    bool getIsRedAlliance() const;
    bool getIsCurrentlySorting() const;
};

void color_sort_task(void *param);

#endif // COLOR_SORT_H