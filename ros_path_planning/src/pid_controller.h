#ifndef PID_H
#define PID_H

class PID {
   private:
      double Kp;
      double Kd;
      double UPPER_LIM;
      double dt=.1;
      double previous_err=0;

   public:
      PID(double kp, double kd, double limit);
      double cmd(double desired, double current);
      double limit_cmd(double command, double err);

};

#endif
