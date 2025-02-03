#ifndef DEVICES_H
#define DEVICES_H

// required files for devices
#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/pid.hpp"
#include "extended_chassis.h"

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
extern adi::Port clamp;
extern pros::Optical ringSens;
extern pros::Distance goalSens;
extern adi::Port left_doinker;
extern adi::Port right_doinker;

extern PID armPID;
extern PID lateralPID;
extern PID angularPID;

extern Rotation armRot;
extern Drivetrain drivetrain;

extern pros::Rotation vertical_encoder;
extern pros::Rotation horizontal_encoder;

extern Imu imu;
extern OdomSensors sensors;
extern ControllerSettings lateral_controller;
extern ControllerSettings angular_controller;
extern ExpoDriveCurve throttle_curve;
extern ExtendedChassis chassis;

#endif // DEVICES_H