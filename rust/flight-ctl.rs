extern crate imu;
extern crate pid_controller;
extern crate servo;

const DESIRED_ORIENTATION: f32 = 0.0;
const DESIRED_ALTITUDE: f32 = 0.0;
const LANDING_THRESHOLD: f32 = 0.1;

fn main() {
    // Initialize IMU, PID controllers, and servo
    let imu = imu::Imu::new();
    let orientation_pid_controller = pid_controller::PidController::new(1.0, 0.1, 0.01); // Example PID gains for orientation
    let altitude_pid_controller = pid_controller::PidController::new(1.0, 0.1, 0.01); // Example PID gains for altitude
    let servo = servo::Servo::new();

    // Main control loop
    loop {
        // Read orientation and altitude from IMU
        let orientation = imu.read_orientation();
        let altitude = imu.read_altitude();

        // Check if the rocket has landed
        if (altitude - DESIRED_ALTITUDE).abs() < LANDING_THRESHOLD && (orientation - DESIRED_ORIENTATION).abs() < LANDING_THRESHOLD {
            // Cut off the engine and break the loop
            println!("Rocket has landed.");
            break;
        }

        // Update PID controllers with current orientation and altitude
        let orientation_control_output = orientation_pid_controller.update(DESIRED_ORIENTATION, orientation);
        let altitude_control_output = altitude_pid_controller.update(DESIRED_ALTITUDE, altitude);

        // Set servo position based on PID controller outputs
        let combined_control_output = orientation_control_output + altitude_control_output;
        servo.set_position(combined_control_output);
    }
}
