#ifndef COLOR_SORT_H
#define COLOR_SORT_H



// one sided hue range that is considered close enough
#include <type_traits>
const int HUE_RANGE = 10;

const int MIN_RING_DETECTION = 1; // Amount of detections needed to quit loop
const int MAX_RING_DISTANCE = 10; // maximum distance ring is on intake from optical sensor
class Hue{
    public:
    int hue;
    int hueMin;
    int hueMax;
    Hue(int hue){
        this->hue = hue;
        if (hue == -1)
        {
            // detecting any hue if no team is specified
            hueMin = 0;
            hueMax = 360;
        }
        else
        {
            // detecting specific hue
            hueMin = (hue - HUE_RANGE + 360) % 360; // Wrap around to ensure valid range
            hueMax = (hue + HUE_RANGE) % 360;       // Wrap around to ensure valid range
        }
    }
    bool inHueRange(int currentHue){
        return (hueMin <= hueMax) ? (currentHue >= hueMin && currentHue <= hueMax) : (currentHue >= hueMin || currentHue <= hueMax);
    }
};

// exact values of hues to detect
const Hue RED_RING_HUE(0);
const Hue BLUE_RING_HUE(210);

bool isRingDetected(Hue targetHue);
// Waits until either red ring is in intake or it times out based on the timeout
bool waitUntilRedIntake(int timeout);
// Waits until either blue ring is in intake or it times out based on the timeout
bool waitUntilBlueIntake(int timeout);
// Waits until either any ring is in intake or it times out based on the timeout
bool waitUntilAnyIntake(int timeout);

extern bool intakeStuck;
extern bool intakeOverride;
extern void intake_stuck_task(void* param);


#endif // COLOR_SORT_H