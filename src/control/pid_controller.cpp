
#include "avt_341/control/pid_controller.h"

namespace avt_341 {
namespace control{
  
PidController::PidController(){
  kp_ = 0.3;
  ki_ = 0.0;
  kd_ = 0.05;
  setpoint_ = 0.0;
  previous_error_ = 0.0;
  integral_ = 0.0;
}

// see: https://en.wikipedia.org/wiki/PID_controller
double PidController::GetControlVariable(double measured_value, double dt){
  double error = setpoint_ - measured_value;
  integral_ += error*dt;
  double derivative = (error - previous_error_)/dt;
  double output = kp_*error + ki_*integral_ + kd_*derivative;
  previous_error_ = error;
  return output;
}

} // namespace control
} // namespace avt_341