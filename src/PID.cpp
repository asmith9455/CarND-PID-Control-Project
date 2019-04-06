#include "PID.h"

PID::PID(double Kp_, double Ki_, double Kd_):
Kp{Kp_}, Ki{Ki_}, Kd{Kd_} {}

PID::~PID() {}

double PID::CalcNewError(double cte) {

  static double last_cte{cte};
  static double cte_sum{0.0};

  p_error = -Kp * cte;
  d_error = -Kd * (cte - last_cte);
  i_error = -Ki * cte_sum;

  last_cte = cte;
  cte_sum += cte;

  return p_error + d_error + i_error;
}