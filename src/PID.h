#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error = 0;
  double i_error = 0;
  double d_error = 0;

  double err = 0.0;
  double t_err = 0.0;

  int numSteps =1;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  double SteeringAngle();

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void Twiddle(double cte);
  void Twiddle2(double cte);
};

#endif /* PID_H */
