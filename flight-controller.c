#include <stdio.h>
#include "imu.h"
#include "pid_controller.h"
#include "servo.h"

// Constants
#define DESIRED_ORIENTATION 0.0f // Desired orientation in degrees
#define DESIRED_ALTITUDE 0.0f // Desired altitude in meters
#define LANDING_THRESHOLD 0.1f // Threshold for considering the rocket landed

int main() {
    // Initialize IMU, PID controllers, and servo
    Imu imu;
    imu_init(&imu);

    PidController orientation_pid_controller;
    pid_controller_init(&orientation_pid_controller, 1.0f, 0.1f, 0.01f); // Example PID gains for orientation

    PidController altitude_pid_controller;
    pid_controller_init(&altitude_pid_controller, 1.0f, 0.1f, 0.01f); // Example PID gains for altitude

    Servo servo;
    servo_init(&servo);

    // Main control loop
    while (1) {
        // Read orientation and altitude from IMU
        float orientation = imu_read_orientation(&imu);
        float altitude = imu_read_altitude(&imu);

        // Check if the rocket has landed
        if (fabs(altitude - DESIRED_ALTITUDE) < LANDING_THRESHOLD && fabs(orientation - DESIRED_ORIENTATION) < LANDING_THRESHOLD) {
            // Cut off the engine and break the loop
            printf("Rocket has landed.\n");
            break;
        }

        // Update PID controllers with current orientation and altitude
        float orientation_control_output = pid_controller_update(&orientation_pid_controller, DESIRED_ORIENTATION, orientation);
        float altitude_control_output = pid_controller_update(&altitude_pid_controller, DESIRED_ALTITUDE, altitude);

        // Set servo position based on PID controller outputs
        float combined_control_output = orientation_control_output + altitude_control_output;
        servo_set_position(&servo, combined_control_output);
    }

    return 0;
}
