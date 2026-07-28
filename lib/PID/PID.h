#ifndef PID_HHH
#define PID_HHH

#include <Arduino.h>
#include <AccelStepper.h>

extern const int STEP_PIN;
extern const int DIR_PIN;

extern long MAX_POS;
extern long MIN_POS;

extern const double MAX_SPEED;

extern double kp;
extern double ki;
extern double kd;

extern double targetHeightInches;
extern double previousError;
extern double integral;
extern double commandedSpeed;

extern AccelStepper stepper;

double applySpeedLimit(double requestedSpeed);
double PID(double kp, double ki, double kd, double setpoint, double input, double* prev_err, double* integral, double dt);
void PID_setup();

#endif