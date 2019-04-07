#include "PID.h"
#include <iostream>
#include <cmath>

PID::PID(double Kp_, double Ki_, double Kd_):
Kp{Kp_}, Ki{Ki_}, Kd{Kd_}, cte_sum{0.0}, first_time{true} {}

PID::~PID() {}

double sgn(const double value)
{
  if (value > 0.0)
  {
    return 1.0;
  }
  else if (value < 0.0)
  {
    return -1.0;
  }
  else
  {
    return 0.0;
  }
}

double ModifyExp(const double x)
{
  using ::std::exp;
  using ::std::fabs;
  return exp(fabs(x)) * sgn(x);
}

double ModifyCub(const double x)
{
  return x*x*x;
}

double PID::CalcNewError(double cte, const bool print_debug_info) {

  if (first_time)
  {
    last_cte = cte;
    first_time = false;
  }

  const double cte_diff{cte - last_cte};

  double pseudo_cte = cte * cte;

  if (cte < 0)
  {
    pseudo_cte = -pseudo_cte;
  }

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

  // if (sgn(cte) != sgn(last_cte))
  // {
  //   cte_sum = 0.0;
  // }

  last_cte = cte;
  cte_sum += cte;

  return p_error + d_error + i_error;
}