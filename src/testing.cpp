#include "testing.h"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/pid.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/adi.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include <cstdlib>
#include "devices.h"
#include "auton_routes.h"
#include "old_systems.h"
#include "misc.h"
#include <iomanip>

void testCombinedPID()
{
    double nextMovement = 0;

    while (true)
    {
        // Lateral PID accumulation (Left Joystick)
        int controllerOutput = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        if (abs(controllerOutput) >= 20)
        { // deadzone of 20 volts
            nextMovement += controllerOutput / 100.0;
        }

        // Trigger lateral movement with UP button
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
        {
            chassis.moveToPoint(0, nextMovement, 4000);
            nextMovement = 0;
        }

        // Angular PID triggering with X button (Right Joystick)
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
        {
            double x = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            double y = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

            // Calculate absolute angle (0-360 degrees)
            double theta;
            if (fabs(x) < 20 && fabs(y) < 20)
            {             // deadzone check
                continue; // skip if joystick is in deadzone
            }

            if (y == 0)
            {
                theta = (x >= 0) ? 90.0 : 270.0;
            }
            else
            {
                // Calculate absolute angle in degrees
                theta = atan2(x, y) * 180.0 / M_PI;

                // Convert to 0-360 range
                if (theta < 0)
                {
                    theta += 360.0;
                }
            }

            // Turn to absolute heading
            chassis.turnToHeading(theta, 4000); // Assuming turnToAngle uses absolute angles
        }

        // Display information
        controller.clear_line(1);
        controller.print(1, 1, "Dist: %f", nextMovement);

        double rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        double rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        double currentAngle = atan2(rightX, rightY) * 180.0 / M_PI;
        if (currentAngle < 0)
            currentAngle += 360.0;
        controller.clear_line(2);
        controller.print(2, 1, "Target Angle: %f", currentAngle);

        pros::delay(100);
    }
}


int totalTime;
int prevTime;
// This function runs in driver control WITHOUT COMM SWITCH, it is a better way of testing the
// autons since you can take inputs from the controller and test multiple times.
// NOTE: The arm is on a different task, so don't hit those buttons during auton
void testAuton(bool inputReq)
{

    // if the parameter inputReq is set to true (default), these buttons
    // will start the route when all pressed
    bool buttonsPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

    // it runs once automatically with inputReq, otherwise manually
    if ((!inputReq && autonSection == 0) || buttonsPressed)
    {

        // prints information about section to controller
        autonSection = 0;
        endSection();

        // sets motor brake type to hold (standard for auton)
        left_motors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
        right_motors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);

        // initalize timer variables
        totalTime = 0;
        prevTime = pros::millis();

        // set up permanent console logging for auton timers
        std::cout << std::fixed << std::setprecision(2);
        std::cout << std::setw(14) << "section" << " | "
                  << std::setw(14) << "time" << " | "
                  << std::setw(14) << "total" << " | "
                  << std::endl;
        std::cout << std::string(15 * 3 + 4, '-')  
                  << std::endl;

        // THIS IS WHERE YOU CHANGE THE ROUTE YOU'RE TESTING
        WPIAWP();


        // stops motors to prevent rogue movements after auton
        left_motors.brake();
        right_motors.brake();

        // small delay to make sure robot is still
        delay(2000);

        // sets motor brake type to coast (standard for usercontrol)
        intake.brake();
        left_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
        right_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
    }
}

void testDrivePID()
{

    int theta = 45;
    int type = 0;
    while (true)
    {

        chassis.setPose(0, 0, theta);
        if (type == 0)
        {
            pros::lcd::clear_line(1);
            pros::lcd::print(1, "drivePID");
            drivePID(24);
            endSection(10000);
            drivePID(-24);
        }
        else if (type == 1)
        {
            pros::lcd::clear_line(1);
            pros::lcd::print(1, "drivePIDOdom");
            drivePIDOdom(24);
            endSection(10000);
            drivePIDOdom(-24);
        }
        else if (type == 2)
        {
            pros::lcd::clear_line(1);
            pros::lcd::print(1, "drivePIDLL");
            drivePIDLL(24);
            endSection(10000);
            drivePIDLL(-24);
        }
        else if (type == 3)
        {
            pros::lcd::clear_line(1);
            pros::lcd::print(1, "drivePIDWTF");
            drivePIDWTF(24);
            endSection(10000);
            drivePIDWTF(-24);
        }
        endSection(10000);
        theta += 90;
        chassis.turnToHeading(theta, 4000);
        endSection(10000);
        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_A))
        {
            type = (type + 1) % 4;
        }
    }
}
