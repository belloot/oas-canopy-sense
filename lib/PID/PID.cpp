#include "PID.h"

const int STEP_PIN = 3;
const int DIR_PIN = 2;

long MAX_POS = 1750;
long MIN_POS = 0;

const double MAX_SPEED = 150.0;

// PID tuning values
double kp = 5.0;
double ki = 0.0;
double kd = 0.0;

double targetHeightInches = 8.0;

// Persistent PID state
double previousError = 0.0;
double integral = 0.0;

// Persistent motor command
double commandedSpeed = 0.0;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Apply software limits
double applySpeedLimit(double requestedSpeed) {
    long position = stepper.currentPosition();
    return ((requestedSpeed > 0.0 && position >= MAX_POS) || (requestedSpeed < 0.0 && position <= MIN_POS)) ? 0.0 : requestedSpeed;
}

// Position PID:
// Input: measured height
// Setpoint: desired height
// Output: desired motor speed (-100% to +100%)
double PID(double kp, double ki, double kd, double setpoint, double input, double* prev_err, double* integral, double dt) {
    double err = setpoint - input;

    *integral += err * dt;

    const double I_MAX = 50.0;

    if (*integral > I_MAX) *integral = I_MAX;
    if (*integral < -I_MAX) *integral = -I_MAX;

    double deriv = 0.0;

    if (dt > 0.0) deriv = (err - *prev_err) / dt;

    double output = kp * err + ki * (*integral) + kd * deriv;

    *prev_err = err;

    return (output < -100.0) ? -100.0 : ((output > 100.0) ? 100.0 : output);
}

void PID_setup() {
    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setMinPulseWidth(3);
    stepper.setCurrentPosition(0);
}