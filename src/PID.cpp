#include "PID.h"

PID::PID(double Kp_, double Ki_, double Kd_):
Kp{Kp_}, Ki{Ki_}, Kd{Kd_} {}

PID::~PID() {}

void PID::UpdateError(double cte) {

  static double last_cte{cte};

  p_error = -Kp * cte;
  d_error = -Kd * (cte - last_cte);
  i_error = -Ki * cte_sum;

  last_cte = cte;
  cte_sum += cte;
}

double PID::TotalError() {
  return p_error + d_error + i_error;
}