#include "Lemlib_pid-logging.hpp"
#include <iostream>
#include <cmath>

void genericPidLoggerTask(void* param) {
    auto* args = static_cast<GenericPidLoggerParams*>(param);
    args->chassis->setPose(0, 0, 0);

    double last_t = pros::millis();
    double last_error = 0;
    double i_error = 0;

    std::cout << "time_ms, error, P_term, I_term, D_term, output" << std::endl;

    while (args->chassis->isInMotion()) {
        double now = pros::millis();
        double dt = (now - last_t) / 1000.0;

        double current;
        double error;
        double d_error;

        if (args->mode == PidMode::Angular) {
            current = args->chassis->getPose().theta;
            error = current - args->target;
            d_error = args->imu->get_gyro_rate().z; // deg/sec
        } else {
            current = args->chassis->getPose().x;
            error = args->target - current;
            d_error = (error - last_error) / dt; // inches/sec
        }

        i_error += error * dt;
        last_t = now;
        last_error = error;

        double P = args->controller.kP * error;
        double I = args->controller.kI * i_error;
        double D = args->controller.kD * d_error;
        double u = P + I + D;

        std::cout << now << ", "
                  << error << ", "
                  << P << ", "
                  << I << ", "
                  << D << ", "
                  << u << std::endl;

        pros::delay(20);
    }

    delete args;
}

void startGenericAsyncPidLogger(
    lemlib::Chassis* chassis,
    lemlib::ControllerSettings controller,
    pros::Imu* imu,
    PidMode mode,
    float target,
    int timeout_ms
) {
    auto* args = new GenericPidLoggerParams{
        chassis, controller, imu, mode, target, timeout_ms
    };
    // Initiate movement
    if (args->mode == PidMode::Angular) {
        args->chassis->turnToHeading(args->target, args->timeout_ms, {}, true);
    } else {
        args->chassis->moveToPoint(args->target, 0, args->timeout_ms, {}, true);
    }
    pros::Task task(genericPidLoggerTask, args);
}
