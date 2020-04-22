#include <iostream>
#include "pid_controller.h"

PID::PID(double kp, double kd, double limit) {
   Kp = kp;
   Kd = kd;
   UPPER_LIM = limit;
};

double PID::cmd(double desired, double current) {
   double error = desired-current;
   double deriv_error = (error-previous_err)/dt;
   previous_err = error;
   double unlimited_cmd = Kp * error + Kd * deriv_error;
   double limited_cmd = limit_cmd(unlimited_cmd, error);

   return limited_cmd;
};

double PID::limit_cmd(double command, double err) {
    double abs_cmd = std::abs(command);
    if (abs_cmd >= UPPER_LIM) {
        abs_cmd = UPPER_LIM;
    };

    //if (err < 0) {
    //   abs_cmd *= -1;
    //};

    return abs_cmd;
};

