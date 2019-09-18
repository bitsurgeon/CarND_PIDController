#include "PID.h"


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = i_error = d_error = 0.;
  d_error_pre = 0.;
}

void PID::UpdateError(double cte) {
  p_error = cte;
  d_error = cte - d_error_pre;
  i_error += cte;
}

double PID::TotalError() {
  double error = - Kp * p_error - Kd * d_error - Ki * i_error;
  return error;
}
