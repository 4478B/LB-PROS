#include "main.h"
#include "devices.h"

void testMotorVoltageWithBattery(){
    // Record the start time and initial battery percentage
    int startTime = pros::millis();
    int startBattery = pros::battery::get_capacity();

    // Print the header for the console output
    std::cout << "currentVoltage" << ", " << "elapsedTime" << ", " << "batteryPercentage" << std::endl;

    while (true) {
        // Move the intake motor at a velocity of 20
        intake.move(20);

        // Get the current battery percentage
        double currentBattery = pros::battery::get_capacity();
        // Calculate the elapsed time in seconds
        double elapsedTime = (pros::millis() - startTime) / 1000.0;
        // Calculate the rate of temperature rise
        double currentVoltage = intake.get_voltage();

        // Display the current battery percentage, elapsed time, and motor voltage on the LCD
        pros::lcd::print(1, "Battery: %f", currentBattery);
        pros::lcd::print(2, "Time: %f", elapsedTime);
        pros::lcd::print(3, "Voltage: %f", currentVoltage);

        // Print the current battery percentage, elapsed time, and motor voltage to the console
        std::cout << currentVoltage << ", " << elapsedTime << ", " << currentBattery << std::endl;

        // Delay for 1 second before the next iteration
        pros::delay(1000);
    }
}

void testMotorVoltageWithTemp(){
    // Record the start time and initial temperature of the intake motor
    int startTime = pros::millis();
    int startTemp = intake.get_temperature();

    // Print the header for the console output
    std::cout << "currentVoltage" << ", " << "elapsedTime" << ", " << "currentTemp" << std::endl;

    while (true) {
        // Move the intake motor at a velocity of 20
        intake.move(20);

        // Get the current temperature of the intake motor
        double currentTemp = intake.get_temperature();
        // Calculate the elapsed time in seconds
        double elapsedTime = (pros::millis() - startTime) / 1000.0;
        // Calculate the rate of temperature rise
        double currentVoltage = intake.get_voltage();

        // Display the current temperature, elapsed time, and motor voltage on the LCD
        pros::lcd::print(1, "Intake Temp: %f", currentTemp);
        pros::lcd::print(2, "Time: %f", elapsedTime);
        pros::lcd::print(3, "Voltage: %f", currentVoltage);

        // Print the current temperature, elapsed time, and motor voltage to the console
        std::cout << currentVoltage << ", " << elapsedTime << ", " << currentTemp << std::endl;

        // Delay for 1 second before the next iteration
        pros::delay(1000);
    }
}

void testTempRiseRate() {
    // Record the start time and initial temperature of the intake motor
    int startTime = pros::millis();
    int startTemp = intake.get_temperature();

    // Print the header for the console output
    std::cout << "currentTemp" << ", " << "elapsedTime" << ", " << "tempRiseRate" << std::endl;

    while (true) {
        // Move the intake motor at a velocity of 20
        intake.move(20);

        // Get the current temperature of the intake motor
        double currentTemp = intake.get_temperature();
        // Calculate the elapsed time in seconds
        double elapsedTime = (pros::millis() - startTime) / 1000.0;
        // Calculate the rate of temperature rise
        double tempRiseRate = (currentTemp - startTemp) / elapsedTime;

        // Display the current temperature, elapsed time, and temperature rise rate on the LCD
        pros::lcd::print(1, "Intake Temp: %f", currentTemp);
        pros::lcd::print(2, "Time: %f", elapsedTime);
        pros::lcd::print(3, "Temp Rise Rate: %f", tempRiseRate);

        // Print the current temperature, elapsed time, and temperature rise rate to the console
        std::cout << currentTemp << ", " << elapsedTime << ", " << tempRiseRate << std::endl;

        // Delay for 1 second before the next iteration
        pros::delay(1000);
    }
}

// EXPERIMENT 4:
// Test efficiency of intake motor
// run intake motor until temperature increases to 40C
// then record average efficiency values every 1 second until temp increases to 45C

void testEfficiency() {
    // Record the start time and initial temperature of the intake motor
    int startTime = pros::millis();
    int startTemp = intake.get_temperature();

    // Print the header for the console output
    std::cout << "averageEfficiency" << ", " << "elapsedTime" << ", " << "currentTemp" << std::endl;

    // Run the intake motor until the temperature reaches 40C
    if (startTemp >= 40) {
        intake.move(0);
        return;
    } else if (startTemp < 40) {
        intake.move(127);
        while (true) {
            double currentTemp = intake.get_temperature();
            if (currentTemp >= 40) {
                intake.move(0);
                break;
            }
        }
    }

    double totalEfficiency = 0;
    int count = 0;

    while (true) {
        // Move the intake motor at a velocity of 127
        intake.move(127);

        // Get the current temperature of the intake motor
        double currentTemp = intake.get_temperature();
        // Calculate the elapsed time in seconds
        double elapsedTime = (pros::millis() - startTime) / 1000.0;
        // Calculate the efficiency of the intake motor
        double currentEfficiency = intake.get_efficiency();

        // Accumulate efficiency values
        totalEfficiency += currentEfficiency;
        count++;

        // Display the current temperature, elapsed time, and efficiency on the LCD
        pros::lcd::print(1, "Intake Temp: %f", currentTemp);
        pros::lcd::print(2, "Time: %f", elapsedTime);
        pros::lcd::print(3, "Efficiency: %f", currentEfficiency);

        // Print the current temperature, elapsed time, and efficiency to the console
        std::cout << currentEfficiency << ", " << elapsedTime << ", " << currentTemp << std::endl;

        // Delay for 1 second before the next iteration
        pros::delay(1000);

        // Stop the intake motor if the temperature reaches 45C
        if (currentTemp >= 45) {
            intake.move(0);
            break;
        }
    }

    // Calculate and print the average efficiency
    double averageEfficiency = totalEfficiency / count;
    double elapsedTime = (pros::millis() - startTime) / 1000.0;
    double currentTemp = intake.get_temperature();

    // Display the average efficiency, elapsed time, and final temperature on the LCD
    pros::lcd::print(1, "Avg Efficiency: %f", averageEfficiency);
    pros::lcd::print(2, "Time: %f", elapsedTime);
    pros::lcd::print(3, "Final Temp: %f", currentTemp);

    // Print the average efficiency, elapsed time, and final temperature to the console
    std::cout << averageEfficiency << ", " << elapsedTime << ", " << currentTemp << std::endl;
}