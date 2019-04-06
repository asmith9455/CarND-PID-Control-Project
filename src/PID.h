#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   */
  PID(double Kp_, double Ki_, double Kd_);

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  double CalcNewError(double cte);

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
};

#endif  // PID_H