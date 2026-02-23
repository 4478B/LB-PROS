/**
 * @file devices.h
 * @brief Global extern declarations for every hardware device on the robot.
 *
 * Include this header in any file that needs to access motors, sensors, or
 * pneumatics. The actual object definitions live in src/devices.cpp.
 *
 * Overview of hardware:
 *   Motors    – 6 drivetrain (3L/3R), 2 bottom intake, 1 top intake, 1 small intake
 *   Pneumatics – 8 solenoids on ADI ports A-H (clamp, stopper, lift, wings, gates …)
 *   Sensors   – 2× IMU, 1 optical (ball color), 1 distance (goal alignment), 1 rotation
 *   Chassis   – LemLib ExtendedChassis with tank drivetrain + odometry
 */

#ifndef DEVICES_H
#define DEVICES_H

#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/pid.hpp"
#include "extended_chassis.h"
#include "averaged_imu.h"

using namespace pros;
using namespace lemlib;

// ─── Drive Motors ──────────────────────────────────────────────────────────────
extern MotorGroup left_motors;   // Three left-side drive motors
extern MotorGroup right_motors;  // Three right-side drive motors
extern MotorGroup all_motors;    // All six drive motors (used by drivePID)
extern MotorGroup arm_motors;    // Arm motors (reserved / unused in current build)

// ─── Controller ────────────────────────────────────────────────────────────────
extern Controller controller;   // Primary (master) game controller

// ─── Intake Motors ─────────────────────────────────────────────────────────────
extern MotorGroup intake;        // Bottom roller intake
extern Motor intakeTop;          // Upper roller that feeds into scorer
extern Motor smallIntake;        // Auxiliary intake roller

// ─── Pneumatic Solenoids (ADI Digital Outputs) ─────────────────────────────────
extern adi::Port clamp;          // ADI F – mobile-goal clamp arm
extern adi::Port intake_lift;    // ADI G – lifts intake mechanism
extern adi::Port stopper;        // ADI C – ball stopper (prevents rollback)
extern adi::Port stopperTwo;     // ADI H – secondary stopper
extern adi::Port lift;           // ADI A – ball-scoring lift
extern adi::Port deScores;       // ADI B – de-scoring wings
extern adi::Port loader;         // ADI D – match-loader gate
extern adi::Port frontGate;      // ADI E – front ball-retention gate

// ─── Standalone PID Objects ────────────────────────────────────────────────────
extern PID lateralPID;
extern PID angularPID;

// ─── Sensors ──────────────────────────────────────────────────────────────────
extern Rotation autoRot;         // Rotational encoder for mechanism feedback
extern Drivetrain drivetrain;    // LemLib drivetrain geometry descriptor
extern Optical ballSensor;       // Optical sensor – ball color & proximity detection
extern Distance backDistance;    // Distance sensor on rear – used for goal alignment

// ─── Odometry / Tracking ──────────────────────────────────────────────────────
extern lemlib::TrackingWheel left_tracking_wheel;
extern lemlib::TrackingWheel right_tracking_wheel;

// Individual IMUs (used directly in some contexts)
extern Imu imu;    // IMU on port 3
extern Imu imu2;   // IMU on port 16

// AveragedIMU combines both IMUs; use this for odometry and most heading queries.
// Falls back to whichever IMU is still connected if one disconnects.
extern AveragedIMU imu1;

// ─── LemLib Chassis Configuration ─────────────────────────────────────────────
extern OdomSensors sensors;                  // Odometry sensor bundle
extern ControllerSettings lateral_controller; // Lateral PID gains + tolerances
extern ControllerSettings angular_controller; // Angular PID gains + tolerances
extern ExpoDriveCurve throttle_curve;        // Expo input curve for driver control
extern ExtendedChassis chassis;              // Main chassis object (move, turn, pose …)

#endif // DEVICES_H