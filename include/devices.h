#ifndef DEVICES_H
#define DEVICES_H

// required files for devices
#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/pid.hpp"
#include "extended_chassis.h"
#include "averaged_imu.h"

// namespace for declarations
using namespace pros;
using namespace lemlib;

// External variable declarations
extern MotorGroup left_motors;
extern MotorGroup right_motors;
extern MotorGroup all_motors;
extern MotorGroup arm_motors;
extern Controller controller;
extern Motor intake;
extern Motor intakeTop;
extern Motor smallIntake;

extern adi::Port clamp;
extern adi::Port intake_lift;
extern adi::Port stopper; //
extern adi::Port deScores;
extern adi::Port loader;
extern adi::Port frontGate;


extern PID lateralPID;
extern PID angularPID;

extern Rotation autoRot;
extern Drivetrain drivetrain;
extern Optical ballSensor;
extern Distance backDistance;

extern lemlib::TrackingWheel left_tracking_wheel;
extern lemlib::TrackingWheel right_tracking_wheel;

// Individual IMUs
extern Imu imu;
extern Imu imu2;

// Averaged IMU (use this for odometry)
extern AveragedIMU imu1;
extern OdomSensors sensors;
extern ControllerSettings lateral_controller;
extern ControllerSettings angular_controller;
extern ExpoDriveCurve throttle_curve;
extern ExtendedChassis chassis;

#endif // DEVICES_H