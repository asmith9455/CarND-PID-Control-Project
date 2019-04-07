#include "PID.h"
#include <iostream>

PID::PID(double Kp_, double Ki_, double Kd_):
Kp{Kp_}, Ki{Ki_}, Kd{Kd_}, cte_sum{0.0}, first_time{true} {}

PID::~PID() {}

double PID::CalcNewError(double cte, const bool print_debug_info) {

  if (first_time)
  {
    last_cte = cte;
    first_time = false;
  }

  const double cte_diff{cte - last_cte};

  p_error = -Kp * cte;
  d_error = -Kd * cte_diff;
  i_error = -Ki * cte_sum;

  if (print_debug_info)
  {
    ::std::cout << "Kp: " << Kp << ", ";
    ::std::cout << "Ki: " << Ki << ", ";
    ::std::cout << "Kd: " << Kd << std::endl;
    ::std::cout << "cte: " << cte << ", ";
    ::std::cout << "last_cte: " << last_cte << ", ";
    ::std::cout << "cte_diff: " << cte_diff << ", ";
    ::std::cout << "cte_sum: " << cte_sum << std::endl;
    ::std::cout << "p_error: " << p_error << ", ";
    ::std::cout << "i_error: " << i_error << ", ";
    ::std::cout << "d_error: " << d_error << std::endl;
  }

  last_cte = cte;
  cte_sum += cte;

  return p_error + d_error + i_error;
}