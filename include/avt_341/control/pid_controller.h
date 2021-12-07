/**
* \class PidController
*
* A simple Proportional-Integral-Derivative (PID) controller.
* Controller is generic, but used for speed control in this application.
*
* \author Chris Goodin
*
* \date 8/31/2020
*/
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

  void SetOvershootLimiter(bool osl){ overshoot_limiter_ = osl; }
  
 private:
  double kp_;
  double ki_;
  double kd_;
  double setpoint_;
  double previous_error_;
  double integral_;
  bool overshoot_limiter_;
  bool crossed_setpoint_;
};

} // namespace control
} // namespace avt_341
#endif
