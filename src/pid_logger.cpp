#include "Lemlib_pid-logging.hpp"

PIDLogger::PIDLogger(double kP, double kI, double kD)
    : kP(kP), kI(kI), kD(kD) {}

void PIDLogger::reset() {
    // clear internal state
    i_error   = 0.0;
    last_error = 0.0;
    last_time  = 0;
    P = I = D = output = 0.0;
}

double PIDLogger::update(double target, double current, double rate, int time_ms) {
    double error = target - current;

    int dt_ms = time_ms - last_time;
    double dt = (dt_ms > 0) ? dt_ms / 1000.0 : 1e-3;

    // Integral
    i_error += error * dt;

    // Derivative: if rate is supplied, it's -measured rate
    double d_error = (rate != 0.0) ? -rate : (error - last_error) / dt;

    // PID terms
    P = kP * error;
    I = kI * i_error;
    D = kD * d_error;
    output = P + I + D;

    // Save state
    last_error = error;
    last_time  = time_ms;

    return output;
}

double PIDLogger::getP() const { return P; }
double PIDLogger::getI() const { return I; }
double PIDLogger::getD() const { return D; }
double PIDLogger::getError() const { return last_error; }
double PIDLogger::getOutput() const { return output; }
