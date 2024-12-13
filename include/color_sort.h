#ifndef COLOR_SORT_H
#define COLOR_SORT_H

// Waits until either red ring is in intake or it times out based on the timeout
void waitUntilRedIntake(int timeout);
// Waits until either blue ring is in intake or it times out based on the timeout
void waitUntilBlueIntake(int timeout);

#endif // COLOR_SORT_H