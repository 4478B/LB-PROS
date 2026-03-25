/**
 * @file devices.cpp
 * @brief Hardware definitions for every sensor, motor, and pneumatic on the robot.
 *
 * All objects declared as extern in devices.h are constructed here.
 * Motor port numbers follow VEX Brain port numbering (1-21).
 * A negative port number means the motor is physically reversed (flipped in the
 * gearbox) so that calling move(+x) produces forward motion on both sides.
 *
 * ADI ports A-H are the 8 digital/analog expansion ports on the V5 Brain
 * and are used here exclusively as pneumatic solenoid outputs (DIGITAL_OUT).
 *
 * LemLib chassis:
 *   Drivetrain  – 3L / 3R blue-cartridge motors, 3.25" wheels, 11.5" track
 *   Odometry    – left & right drive encoders + single IMU (port 3)
 *   PIDs        – lateral (kP=10, kD=3) and angular (kP=2.9, kI=0.2, kD=24)
 */

#include "devices.h"
#include "extended_chassis.h"
#include "pros/distance.hpp"

// ─── Drive Motors ──────────────────────────────────────────────────────────────
// left motor group
MotorGroup left_motors({-11, 12, -13}, pros::MotorGearset::blue);
// right motor group
MotorGroup right_motors({20, -19, 18}, pros::MotorGearset::blue);

// Combined group used when commanding all drive motors at once (e.g., drivePID)
MotorGroup all_motors({-11, 12, -13, 20, -19, 18}, pros::MotorGearset::blue);

// Arm motors (unused in current build; kept for future expansion)
MotorGroup arm_motors({1, -1}, pros::MotorGearset::blue);

// ─── Controller ────────────────────────────────────────────────────────────────
Controller controller(pros::E_CONTROLLER_MASTER);

// ─── Intake Motors ─────────────────────────────────────────────────────────────
// intake      – bottom roller that pulls game objects into the robot
// intakeTop   – upper roller that feeds objects into the scoring mechanism
// smallIntake – auxiliary roller (e.g., for a small side intake or anti-jam)
MotorGroup intake({10, -9}, pros::MotorGearset::blue);
Motor intakeTop(-1, pros::MotorGearset::blue);
Motor smallIntake(-5, pros::MotorGearset::blue);

// ─── Pneumatic Actuators (ADI Digital Outputs) ─────────────────────────────────
// Each adi::Port controls one pneumatic solenoid via a digital signal.
// HIGH = solenoid energized (piston extended), LOW = retracted.
adi::Port clamp('F', pros::E_ADI_DIGITAL_OUT);       // Mobile-goal clamp arm
adi::Port intake_lift('G', pros::E_ADI_DIGITAL_OUT); // Lifts intake for climb
adi::Port stopper('A', pros::E_ADI_DIGITAL_OUT);     // Blocks ball from falling back out of intake
adi::Port stopperTwo('H', pros::E_ADI_DIGITAL_OUT);  // Secondary stopper
adi::Port lift('D', pros::E_ADI_DIGITAL_OUT);        // Ball-scoring lift piston
adi::Port deScores('C', pros::E_ADI_DIGITAL_OUT);    // De-scoring wings (pushes balls off goals)
adi::Port loader('B', pros::E_ADI_DIGITAL_OUT);      // Match-loader gate (drops balls from field wall)
adi::Port frontGate('E', pros::E_ADI_DIGITAL_OUT);   // Front gate to retain balls in intake

// ─── PID Objects (standalone; separate from LemLib ControllerSettings below) ───
PID lateralPID(.11, 0, 0.15);
PID angularPID(0.495, 0, 0.002);

// ─── Sensors ──────────────────────────────────────────────────────────────────
Rotation autoRot(10);               // Rotational sensor (port 10) for mechanisms

Optical ballSensor(19);             // Optical sensor (port 19) – detects ball color/proximity
Distance backDistance(2);           // Distance sensor (port 2) – used for goal alignment

// ─── LemLib Drivetrain Configuration ──────────────────────────────────────────
// Drivetrain describes the physical geometry and gearing to LemLib so it can
// correctly convert wheel revolutions into inches traveled.
Drivetrain drivetrain(&left_motors,  // left motor group
                      &right_motors, // right motor group
                      11.5,          // track width in inches (wheel-to-wheel)
                      3.25,          // wheel diameter in inches
                      450,           // drivetrain rpm
                      1.5            // horizontal drift factor (1.5 = slight center-traction bias)
);

// Individual IMUs
Imu imu(14);  // First IMU on port 7
Imu imu2(17); // Second IMU on port 8 (change this to your actual port)

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
