#ifndef EXTENDED_CHASSIS_H
#define EXTENDED_CHASSIS_H

#include "lemlib/chassis/chassis.hpp"

class ExtendedChassis : public lemlib::Chassis {
public:
    // Inherit all constructors from the base Chassis class
    using lemlib::Chassis::Chassis;

    // Forward declare methods to add to Chassis class
    void MoveToPointClamp(float x, float y, int timeout, float clampDist = .5, lemlib::MoveToPointParams params = {});
    void MoveToPoseClamp(float x, float y, float theta, int timeout, float clampDist = .5, lemlib::MoveToPoseParams params = {});
    void printPose(int line1 = 1, int line2 = 2, int line3 = 3);
};
#endif // EXTENDED_CHASSIS_H