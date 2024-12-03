#include "devices.h"
#include "extended_chassis.h"


// left motor group
MotorGroup left_motors({-8, -9, -10}, pros::MotorGearset::blue);
// right motor group
MotorGroup right_motors({5, 6, 7}, pros::MotorGearset::blue);

MotorGroup arm_motors({-3, 4}, pros::MotorGearset::blue);

// controller definition
Controller controller(pros::E_CONTROLLER_MASTER);

Motor intake(1);

adi::Port clamp('B', pros::E_ADI_DIGITAL_OUT);

PID armPID(1.5, 0, 1);

Rotation armRot(2);

Optical colorSens(19);

// drivetrain settings
Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11, // 11 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
);

Imu imu(20);

OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
ExpoDriveCurve throttle_curve(6, // joystick deadband out of 127
                                     6, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// create the chassis
ExtendedChassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
						&throttle_curve // log drive
);
