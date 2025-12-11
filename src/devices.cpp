#include "devices.h"
#include "extended_chassis.h"
#include "pros/distance.hpp"

// left motor group
MotorGroup left_motors({-7, 9, -8}, pros::MotorGearset::blue);
// right motor group
MotorGroup right_motors({1, -2, 3}, pros::MotorGearset::blue);

MotorGroup all_motors({-7, 9, -8, 1, -2, 3}, pros::MotorGearset::blue);

MotorGroup arm_motors({1, -1}, pros::MotorGearset::blue);

// controller definition
Controller controller(pros::E_CONTROLLER_MASTER);

Motor intake(19, pros::MotorGearset::blue);
Motor intakeTop(-13, pros::MotorGearset::blue);

Motor smallIntake(-12, pros::MotorGearset::blue);

adi::Port clamp('F', pros::E_ADI_DIGITAL_OUT);

adi::Port intake_lift('G', pros::E_ADI_DIGITAL_OUT);

adi::Port stopper('C', pros::E_ADI_DIGITAL_OUT);
adi::Port deScores('B', pros::E_ADI_DIGITAL_OUT);
adi::Port loader('A', pros::E_ADI_DIGITAL_OUT);
adi::Port frontGate('E', pros::E_ADI_DIGITAL_OUT);

PID lateralPID(.11, 0, 0.15);
PID angularPID(0.495, 0, 0.002);

Rotation autoRot(10);

Optical ballSensor(19);   // Optical sensor on port 19
Distance backDistance(2); // Back distance sensor on port 8

// drivetrain settings
Drivetrain drivetrain(&left_motors,  // left motor group
                      &right_motors, // right motor group
                      11.5,          // 11 inch track width
                      3.25,          // using new 2.75" omnis
                      450,           // drivetrain rpm is 450
                      1.5            // horizontal drift is 8 (center traction wheel drivebase)
);

// Individual IMUs
Imu imu(5);  // First IMU on port 7
Imu imu2(16); // Second IMU on port 8 (change this to your actual port)

// Averaged IMU that combines both sensors
AveragedIMU imu1(&imu, &imu2);

lemlib::TrackingWheel left_tracking_wheel(&left_motors, drivetrain.wheelDiameter, -drivetrain.trackWidth / 2.0f, drivetrain.rpm);
lemlib::TrackingWheel right_tracking_wheel(&right_motors, drivetrain.wheelDiameter, drivetrain.trackWidth / 2.0f, drivetrain.rpm);

OdomSensors sensors(&left_tracking_wheel,  // vertical tracking wheel 1 (left drive)
                    &right_tracking_wheel, // vertical tracking wheel 2 (right drive)
                    nullptr,               // horizontal tracking wheel 1
                    nullptr,               // horizontal tracking wheel 2
                    &imu                   // inertial sensor
);

// lateral PID controller
ControllerSettings lateral_controller(10,  // proportional gain (kP)
                                      0,   // integral gain (kI)
                                      3,   // derivative gain (kD)
                                      3,   // anti windup
                                      0.1, // small error range, in inches
                                      100, // small error range timeout, in milliseconds
                                      0.5, // large error range, in inches
                                      500, // large error range timeout, in milliseconds
                                      20   // maximum acceleration (slew)
);

// angular PID controller
ControllerSettings angular_controller(2.9, // proportional gain (kP)
                                      0.2, // integral gain (kI)
                                      24,  // derivative gain (kD)
                                      5,   // anti windup
                                      0.1, // small error range, in inches
                                      250, // small error range timeout, in milliseconds
                                      0.3, // large error range, in inches
                                      250, // large error range timeout, in milliseconds
                                      0    // maximum acceleration (slew)
);

// input curve for throttle input during driver control
ExpoDriveCurve throttle_curve(3,    // joystick deadband out of 127
                              0,    // minimum output where drivetrain will move out of 127
                              1.019 // expo curve gain
);

// create the chassis
ExtendedChassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors,            // odometry sensors
                        &throttle_curve     // log drive
);
