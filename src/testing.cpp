#include "testing.h"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/pid.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/adi.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include <cstdlib>
#include "devices.h"
#include "auton_routes.h"
#include "old_systems.h"
#include <iomanip>

int totalTime;
int prevTime;
// This function runs in driver control WITHOUT COMM SWITCH, it is a better way of testing the
// autons since you can take inputs from the controller and test multiple times.
// NOTE: The arm is on a different task, so don't hit those buttons during auton
void testAuton(bool inputReq){

    // if the parameter inputReq is set to true (default), these buttons
    // will start the route when all pressed
    bool buttonsPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

    // it runs once automatically with inputReq, otherwise manually
    if ((!inputReq && autonSection == 0) || buttonsPressed)
    {

        // prints information about section to controller
        autonSection = 1;

        // sets motor brake type to hold (standard for auton)
        left_motors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
        right_motors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);

        // initalize timer variables
        totalTime = 0;
        prevTime = pros::millis();

        // set up permanent console logging for auton timers
        std::cout << std::fixed << std::setprecision(2);
        std::cout << std::setw(10) << "section" << " | "
                  << std::setw(10) << "time" << " | "
                  << std::setw(10) << "total" << " | "
                  << std::setw(10) << "X" << " | "
                  << std::setw(10) << "Y" << " | "
                  << std::setw(10) << "theta" << " | "
                  << std::endl;
        std::cout << std::string(11 * 6 + 4, '-')
                  << std::endl;

    
        // ***********************************************
        // ************ CHANGE ROUTE HERE ***************
        // ***********************************************


        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {

        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
        }
        else
        {

           // aggressive P only trial
           /*all_motors.move_velocity(50);
           pros::delay(10000);
           all_motors.brake();*/
           //drivePID(48,10000);
           //fullAWPLeft(1);/////////////////////////////////////////////<-------- codes for 9/11
           skills(1);/////////////////////////////////////////////
           //intake.move(127);
//////////////////   TESTING AUTO HERE SWITCH WHICH ONE U WANNA RUN DOWN HERE   ///////////////////////////////
        }
        
        // ***********************************************


        // ***********************************************

        //  stops motors to prevent rogue movements after autonl
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