#include "devices.h"
#include "extended_chassis.h"
#include "pros/distance.hpp"

// left motor group
MotorGroup left_motors({-17, -16, -15}, pros::MotorGearset::blue);
// right motor group
MotorGroup right_motors({18, 19, 20}, pros::MotorGearset::blue);

MotorGroup all_motors({-17, -16, -15, 18, 19, 20}, pros::MotorGearset::blue);

MotorGroup arm_motors({12, -13}, pros::MotorGearset::blue);

// controller definition
Controller controller(pros::E_CONTROLLER_MASTER);

Motor intake(14);

adi::Port clamp('B', pros::E_ADI_DIGITAL_OUT);

adi::Port left_doinker('H', pros::E_ADI_DIGITAL_OUT);
adi::Port right_doinker('C', pros::E_ADI_DIGITAL_OUT);


PID armPID(2.9, 0, 2.5); // old 2.9 2.5
PID lateralPID(.11, 0, 0.15);
PID angularPID(0.499, 0, 0.002);

Rotation armRot(10);

Optical ringSens(0);

Distance goalSens(0);

// drivetrain settings
Drivetrain drivetrain(&left_motors,               // left motor group
                      &right_motors,              // right motor group
                      11,                         // 11 inch track width
                      lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                      450,                        // drivetrain rpm is 450
                      8                           // horizontal drift is 8 (center traction wheel drivebase)
);

Imu imu(4);


pros::Rotation vertical_encoder(-9);
pros::Rotation horizontal_encoder(-8);

lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -2.44);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, .375);

OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1
                    nullptr, // vertical tracking wheel 2
                    &horizontal_tracking_wheel, // horizontal tracking wheel 1
                    nullptr, // horizontal tracking wheel 2
                    &imu     // inertial sensor
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
ControllerSettings angular_controller(2.5, // proportional gain (kP)
                                      0.2,   // integral gain (kI)
                                      20,  // derivative gain (kD)
                                      5,   // anti windup
                                      0.2, // small error range, in inches
                                      250, // small error range timeout, in milliseconds
                                      0.4,   // large error range, in inches
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
