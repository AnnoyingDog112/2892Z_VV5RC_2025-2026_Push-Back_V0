#pragma once

#include "lemlib/api.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"

enum class PidMode {
    Angular,
    Lateral
};

struct GenericPidLoggerParams {
    lemlib::Chassis* chassis;
    lemlib::ControllerSettings controller;
    pros::Imu* imu; // used only for angular
    PidMode mode;
    float target; // angle in deg OR distance in inches
    int timeout_ms;
};

void startGenericAsyncPidLogger(
    lemlib::Chassis* chassis,
    lemlib::ControllerSettings controller,
    pros::Imu* imu,
    PidMode mode,
    float target,
    int timeout_ms = 5000
);
