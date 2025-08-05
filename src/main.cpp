#include "main.h"
#include "lemlib/api.hpp"
#include "api.h"
// #include "okapi/api.hpp" // <-- UNCOMMENT THIS LINE

using namespace pros;

Controller controller(E_CONTROLLER_MASTER);

Motor intake_motor(-12, MotorGearset::green); // Intake motor on port 4
Motor conveyor_motor(3, MotorGearset::green); // Flywheel motor on port

// Create motor groups for left and right sides
MotorGroup left_motors({1}, MotorGearset::blue); // Left motor group with blue gearset
MotorGroup right_motors({-10}, MotorGearset::blue); // Right motor group with blue gearset 

// Update drivetrain initialization to use motor groups and required parameters
lemlib::Drivetrain drivetrain(
	&left_motors, // left motor group
	&right_motors, // right motor group
	11.5, // track width in inches
	lemlib::Omniwheel::NEW_4, // wheel diameter in inches (update as needed)
	200, // drivetrain rpm
	8 // horizontal drift
);

Imu imu(4);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
// lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0); // maximum acceleration (slew)

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
						&throttle_curve, 
                        &steer_curve
);

const int loop_frequency = 20; // milliseconds for each drive cycle

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() 
{
	using namespace pros::lcd;
	
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		lcd:: set_text(2, "I was pressed!");
	} else {
		lcd:: clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() 
{
	using namespace pros::lcd;
	lcd::initialize();
	set_text(1, "Hello PROS User!");

	register_btn1_cb(on_center_button);
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
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    const float target = 90; // target heading in degrees
    chassis.turnToHeading(target, 100000);
    double last_t = millis();
    std::cout << "Time, Error, P, I, D, Output" << std::endl; // header for output
    double i_error;
    while (chassis.isInMotion()) {
        int start_t = pros::millis();
        double theta = chassis.getPose().theta; // get the current error
        double err_ang = chassis.getPose().theta - target;  // old-style, may need to track your own error
        double d_error = -(imu.get_gyro_rate().z); 
        i_error += err_ang * (millis() - last_t); // integral term, can be used for anti-windup
        double last_t = millis();
        double u = angular_controller.kP * err_ang
                 + angular_controller.kI * i_error
                 + angular_controller.kD * d_error; // compute the control output
        std::cout << millis() << ", " << err_ang << ", " << angular_controller.kP * err_ang
         << ", " << angular_controller.kI * i_error << ", " << angular_controller.kD * d_error
         << ", " << u << std::endl;
               // delay to save resources
        if (pros::millis() - start_t < loop_frequency) {
            pros::delay(loop_frequency - (pros::millis() - start_t));
        }
}
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
	// using namespace pros;
  
    // loop forever
    while (true) {
        int start_t = pros::millis();

        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        bool intakeButtonForward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool intakeButoonBackwards = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

        bool conveyorButtonForward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool conveyorButtonBackwards = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

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
        
        // int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        // int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX); // Arcade drive with left and right y positions

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
        // if (conveyor_motor.is_over_temp() /*or conveyor_motor.is_over_current()*/) {
        //     conveyor_motor.move(0); // stop conveyor if it is overheating
        //     controller.rumble("-"); // rumble controller to indicate overheating
        //     controller.clear_line(0); // clear the first line of the controller  
        //     controller.print(0, 0, "Conveyor Motors are overheating! Stopping movement.");
        //     delay(1000); // wait for a second to let the user see the message
        /*} else*/ if (conveyorButtonForward) {
            conveyor_motor.move_velocity(200); // move conveyor forward at 200 rpm
        } else if (conveyorButtonBackwards) {
            conveyor_motor.move_velocity(-200); // move conveyor backwards at -200 rpm
        } else {
            conveyor_motor.move_velocity(0); // stop conveyor
        }
        // delay to save resources
        if (pros::millis() - start_t < loop_frequency) {
            pros::delay(loop_frequency - (pros::millis() - start_t));
        }
    }
}