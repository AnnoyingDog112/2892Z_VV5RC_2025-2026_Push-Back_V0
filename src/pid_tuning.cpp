#include <iostream>
#include "api.h"  // ensure pros::millis()/delay are declared
#include "Lemlib_pid-logging.hpp"
#include "lemlib/api.hpp"
#include "pid_tuning.hpp"

static void angular_pid_logging_task(void* param) {
    auto* args = static_cast<AngularPidLogArgs*>(param);

    PIDLogger logger(args->controller.kP, args->controller.kI, args->controller.kD);
    logger.reset();

    std::cout << "time_ms, error, P, I, D, output" << std::endl;

    while (!args->stop.load(std::memory_order_acquire)) {
        int now = pros::millis();
        double current = args->chassis->getPose().theta;
        // PROS IMU returns Z gyro rate in deg/s
        double rate = args->imu->get_gyro_rate().z;

        double output = logger.update(args->target, current, rate, now);

        std::cout << now << ", "
                  << logger.getError() << ", "
                  << logger.getP() << ", "
                  << logger.getI() << ", "
                  << logger.getD() << ", "
                  << output << std::endl;

        int elapsed = pros::millis() - now;
        if (elapsed < args->loop_delay_ms) pros::delay(args->loop_delay_ms - elapsed);
    }

    delete args;  // Free heap-allocated struct (task owns it)
    return;
}

void start_angular_pid_logging_task(
    lemlib::Chassis* chassis,
    pros::Imu* imu,
    const lemlib::ControllerSettings& controller,
    float target,
    int timeout_ms,
    int loop_delay_ms  // keep default ONLY in the header
) {
    auto* args = new AngularPidLogArgs{
        .chassis       = chassis,
        .imu           = imu,
        .controller    = controller,
        .target        = target,
        .timeout_ms    = timeout_ms,
        .loop_delay_ms = loop_delay_ms,
    };
    args->stop.store(false, std::memory_order_release);

    // fire-and-forget logging task; it will free args when done
    pros::Task{angular_pid_logging_task, args};

    pros::delay(100);  // Give logging task time to start

    // Perform the motion while logging runs
    // Make this call BLOCKING so logging continues throughout the move
    chassis->turnToHeading(target, timeout_ms, {}, false);

    // Signal the logger to stop once the motion is done
    args->stop.store(true, std::memory_order_release);
}
