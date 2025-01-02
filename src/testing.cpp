#include "testing.h"
#include "goal_sensor.h"
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
#include "color_sort.h"

/*void testCombinedPID()
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
            endSection();
            chassis.moveToPoint(0, nextMovement, 4000);
            nextMovement = 0;
            endSection();
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

            endSection();
            // Turn to absolute heading
            chassis.turnToHeading(theta, 4000); // Assuming turnToAngle uses absolute angles
            endSection();
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
}*/

void testRingSens(int i)
{
    while (true)
    {

        intake.move(80);
        pros::lcd::clear_line(1);
        pros::lcd::print(1, "Waiting for red...");
        waitUntilRedIntake(100000);
        intake.brake();
        pros::lcd::clear_line(1);
        pros::lcd::print(1, "Got red!");
        endSection(1000000);

        intake.move(80);
        pros::lcd::clear_line(1);
        pros::lcd::print(1, "Waiting for blue...");
        waitUntilBlueIntake(100000);
        intake.brake();
        pros::lcd::clear_line(1);
        pros::lcd::print(1, "Got blue!");
        endSection(1000000);
    }
}

void testGoalSens(int i)
{
    while (true)
    {

        pros::lcd::clear_line(1);
        pros::lcd::print(1, "Waiting for goal...");
        all_motors.move(40);
        waitUntilClamp(48000000, 1000000);
        all_motors.brake();
        pros::lcd::clear_line(1);
        pros::lcd::print(1, "Got goal!");
        endSection(1000000);
    }
}

void testOdometryStraight(int i)
{

    chassis.setPose(0, 0, 0);
    while (true)
    {
        chassis.moveToPoint(0, 72, 3000, {.minSpeed = 1, .earlyExitRange = 1}, false);
        delay(500);
        chassis.printPose();
        endSection(1000000);
        chassis.moveToPoint(0, 0, 3000, {.minSpeed = 1, .earlyExitRange = 1}, false);
        delay(500);
        chassis.printPose();
        endSection(1000000);
    }
}

void testOdometryTurn(int i)
{

    chassis.setPose(0, 0, 0);
    while (true)
    {
        chassis.moveToPose(0, 0, 90, 3000, {}, false);
        delay(500);
        chassis.printPose();
        endSection(1000000);
        chassis.moveToPose(0, 0, 0, 3000, {}, false);
        delay(500);
        chassis.printPose();
        endSection(1000000);

        chassis.moveToPose(0, 0, 180, 3000, {}, false);
        delay(500);
        chassis.printPose();
        endSection(1000000);
        chassis.moveToPose(0, 0, 0, 3000, {}, false);
        delay(500);
        chassis.printPose();
        endSection(1000000);
    }
}

void testOdometryBoth(int i)
{
    chassis.setPose(0, 0, 0);
    while (true)
    {
        chassis.moveToPose(24, 24, 90, 3000, {}, false);
        delay(500);
        chassis.printPose();
        endSection(1000000);
        chassis.moveToPose(0, 0, 0, 3000, {}, false);
        delay(500);
        chassis.printPose();
        endSection(1000000);
    }
}

/*


        NON-RUNNER CODE



*/

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
        progSkills(1);

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

/*
The overloaded function updateController is optimized
to only change the values necessary for increased responsiveness.
1. Updates whole screen (most time-consuming)
2. Updates magnitude (least time-consuming)
3. Changes one line (somewhat time-consuming)
*/
// for changing whole screen
void updateController(int sel, double mag, ControllerSettings PID)
{

    controller.clear();
    controller.print(sel + 1, 1, "*"); // creates marker for current selected value
    controller.print(1, 1, "kP: %c %f", 0 == sel ? "*" : " ", PID.kP);
    controller.print(1, 1, "kI: %c %f", 1 == sel ? "*" : " ", PID.kI);
    controller.print(1, 1, "kD: %c %f", 2 == sel ? "*" : " ", PID.kD); // prints P I D on new lines

    controller.print(1, 14, "%f", mag);
}

void tunePID()
{
    using std::cout;
    using std::endl;
    const double resetVal = 0.1, resetMag = 0.1; // Defines what values to reset to
    double valMag = resetMag;                    // Sets initial magnitude
    int currentConst = 0;                        // Defines current constant (P,I, or D) to change
    ControllerSettings PID = lateral_controller;
    cout << "Now modifying P" << endl; // tells user default value modified
    updateController(currentConst, valMag, PID);
    while (true)
    {
        double bUP = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP);
        double bDOWN = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN);
        double bLEFT = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT);
        double bRIGHT = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT);

        if (bUP && bDOWN)
        {
            valMag = resetMag; // Resets magnitude of dynamic change
            cout << "magnitude RESET to " << resetMag << endl;
            updateController(currentConst, valMag, PID);
        }
        else if (bUP)
        {
            valMag *= 10; // Increments magnitude by a factor of 10
            cout << "magnitude set to " << valMag << endl;
            updateController(currentConst, valMag, PID);
        }
        else if (bDOWN)
        {
            valMag /= 10; // Decrements magnitude by a factor of 10
            cout << "magnitude set to " << valMag << endl;
            updateController(currentConst, valMag, PID);
        }

        if (bLEFT || bRIGHT)
        {
            double deltaVal;
            if (bLEFT)
            {
                deltaVal = -valMag; // Decreases temp. PID value by magnitude
            }
            else if (bRIGHT)
            {
                deltaVal = valMag; // Increases temp. PID value by magnitude
            }
            switch (currentConst)
            {
            case 1:
                PID.kP += deltaVal;
                cout << "kP: %f", PID.kP;
                break;
            case 2:
                PID.kI += deltaVal;
                break;
            case 3:
                PID.kD += deltaVal;
                break;
            }

            updateController(currentConst, valMag, PID);
        }

        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_Y))
        { // Controls value being changed
            currentConst += 1;
            currentConst %= 3; // variable has 3 states, modulus keeps within 3 states
            if (currentConst == 0)
            {
                cout << "Now modifying P" << endl;
            }
            else if (currentConst == 1)
            {
                cout << "Now modifying I" << endl;
            }
            else
            {
                cout << "Now modifying D" << endl;
            }
            updateController(currentConst, valMag, PID);
        }
    }
}