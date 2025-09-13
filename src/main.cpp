#include <iostream>
#include "main.h"
#include "lemlib/api.hpp"
#include "api.h"
#include "pid_tuning.hpp"
#include "config.hpp"

// #include "okapi/api.hpp" // <-- UNCOMMENT THIS LINE

using namespace pros;
using namespace lemlib;

Controller controller(E_CONTROLLER_MASTER);

Motor intake_motor(1, MotorGearset::blue); // Intake motor on port 4
Motor lift_motor(2, MotorGearset::red); // Flywheel motor on port

// Create motor groups for left and right sides
MotorGroup left_motors({11, 12, 13}, MotorGearset::blue); // Left motor group with blue gearset
MotorGroup right_motors({-18, -19, -20}, MotorGearset::blue); // Right motor group with blue gearset 

// Update drivetrain initialization to use motor groups and required parameters
Drivetrain drivetrain(
	&left_motors, // left motor group
	&right_motors, // right motor group
	11.5, // track width in inches
	lemlib::Omniwheel::NEW_4, // wheel diameter in inches (update as needed)
	450, // drivetrain rpm
	8 // horizontal drift
);

Imu imu(4);
Rotation horizontal_rotation_sensor(15);

TrackingWheel horizontal_tracking_wheel(&horizontal_rotation_sensor, lemlib::Omniwheel::NEW_275, -4.75);
TrackingWheel left_drivetrain_tracking_wheel(&left_motors, lemlib::Omniwheel::NEW_275, -5.75, 450);
TrackingWheel right_drivetrain_tracking_wheel(&right_motors, lemlib::Omniwheel::NEW_275, 5.75, 450);


OdomSensors sensors(&left_drivetrain_tracking_wheel, // vertical tracking wheel 1, set to null
                            &right_drivetrain_tracking_wheel, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
ControllerSettings lateral_controller(KP_LATERAL, // proportional gain (kP)
                                              KI_LATERAL, // integral gain (kI)
                                              KD_LATERAL, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
ControllerSettings angular_controller(KP_ANGULAR, // proportional gain (kP)
                                              KI_ANGULAR, // integral gain (kI)
                                              KD_ANGULAR, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0); // maximum acceleration (slew)

// input curve for throttle input during driver control
ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.25 // expo curve gain
);

// input curve for steer input during driver control
ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.25 // expo curve gain
);

// create the chassis
Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
						&throttle_curve, 
                        &steer_curve
);

PID lift_pid(0.5,
            0,
            0.1,
            0,
            false); // PID for lift
const int loop_frequency = 100; // milliseconds for each drive cycle

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() 
{
	using namespace pros::lcd;
	pros::delay(500);
    lcd::initialize();
    imu.reset(true);
    pros::delay(2000);
	set_text(1, "Hello PROS User!");

    pros::delay(1000);
    std::cout << "=== Program started ===" << std::endl;
    chassis.calibrate(); // optional: waits for IMU calibration
    chassis.setPose(0, 0, 0); // optional: initialize position
    autonomous(); // non-blocking
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() 
{

}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
ASSET(example_txt)
void autonomous() 
{
    
    using namespace pros;
    // set position to x:0, y:0, heading:0
    std::cout << KP_ANGULAR << ", " << KI_ANGULAR << ", "<< KD_ANGULAR << std::endl;
    // std::cout << "\033[2J\033[1;1H"; // clear terminal
    chassis.setPose(0, 0, 0);
    // while (true) std::cout << "X: " << chassis.getPose().x << ", Y: " << chassis.getPose().y << ", Theta: " << chassis.getPose().theta << std::endl;
    start_angular_pid_logging_task(&chassis, &imu, angular_controller, 90, 5000, 20); // Example usage of angular PID logging
    // turn to face heading 90 with a very long timeout
    float target = 90; // target heading in degrees
    
    
    // while (true)
    // {
    //     int start_t = pros::millis();
    //     lemlib::Pose pose = chassis.getPose();
    //     std::cout << "X: " << pose.x << ", Y: " << pose.y << ", Theta: " << pose.theta << std::endl;
    //     if (pros::millis() - start_t < loop_frequency) {
    //         pros::delay(loop_frequency - (pros::millis() - start_t));
    //     }
    //     
    // }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    using namespace pros;
    
    // autonomous();
    // return; // Uncomment this line to run autonomous instead of opcontrol

    // loop forever
    while (true) {
        int start_t = millis();

        // get left y and right y positions
        int leftY = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);
        int leftX = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
        int rightX = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
        
        bool intakeButtonForward = controller.get_digital(E_CONTROLLER_DIGITAL_R1);
        bool intakeButoonBackwards = controller.get_digital(E_CONTROLLER_DIGITAL_R2);

        bool liftButtonUp = controller.get_digital(E_CONTROLLER_DIGITAL_UP);
        bool liftButtonDown = controller.get_digital(E_CONTROLLER_DIGITAL_DOWN);

        // move the robot
        // if ((/*left_motors.is_over_current()  or*/ left_motors.is_over_temp()) or 
        //     (/*right_motors.is_over_current() or*/ right_motors.is_over_temp())) {
        //     left_motors.move(0); // stop left motors if they are overheating
        //     right_motors.move(0); // stop right motors if they are overheating
        //     controller.rumble("-"); // rumble controller to indicate overheating
        //     controller.clear_line(0); // clear the first line of the controller
        //     controller.print(0, 0, "Drivetrain Motors are overheating! Stopping movement.");
        //     delay(1000); // wait for a second to let the user see the message
        // } else {
        //     chassis.tank(leftY, rightY);
        // }
        
        // chassis.tank(leftY, rightY); // Switching to arcade drive for now
        
        // int leftY = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        // int rightX = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.tank(leftY, rightY); // Arcade drive with left and right y positions

        // move intake forward or backwards
        // if (intake_motor.is_over_temp() /*or intake_motor.is_over_current()*/) {
        //     intake_motor.move(0); // stdop intake if it is overheating
        //     controller.rumble("-"); // rumble controller to indicate overheating
        //     controller.clear_line(0); // clear the first line of the controller
        //     controller.print(0, 0, "Intake Motor is overheating! Stopping movement.");
        //     delay(1000); // wait for a second to let the user see the message
        /*} else*/ if (intakeButtonForward) {
            intake_motor.move_velocity(200); // move intake forward at 200 rpm
        } else if (intakeButoonBackwards) {
            intake_motor.move_velocity(-200); // move intake backwards at -200 rpm 
        } else {
            intake_motor.move_velocity(0); // stop intake
        }

        // move conveyor forward or backwards
        if (liftButtonUp) {
            lift_motor.move_velocity(200); // move conveyor forward at 200 rpm
        } else if (liftButtonDown) {
            lift_motor.move_velocity(-200); // move conveyor backwards at -200 rpm
        } else {
            lift_motor.move_velocity(0); // stop conveyor
        }
        // delay to save resources
        if (millis() - start_t <= loop_frequency) 
            delay(loop_frequency - (millis() - start_t));
    }
}