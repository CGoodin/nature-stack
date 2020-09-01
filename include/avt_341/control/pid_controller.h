#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

namespace avt_341 {
namespace control{

class PidController{
 public:
  PidController();

  double GetControlVariable(double measured_value, double dt);

  void SetSetpoint(double setpoint){setpoint_ = setpoint;}

  void SetKp(double kp){kp_=kp;}

  void SetKi(double ki){ki_ = ki;}

  void SetKd(double kd){kd_ = kd;}
  
 private:
  double kp_;
  double ki_;
  double kd_;
  double setpoint_;
  double previous_error_;
  double integral_;
};

} // namespace control
} // namespace avt_341
#endif
