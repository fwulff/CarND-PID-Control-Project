#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double steer_value;
  double cte_curr;
  double cte_prev;
  double cte_diff;
  double cte_total;
  double cte_total_abs;


  // Twiddle
  double cte_best;

  double p[3] = {0.2, 0.004, 3.0};
  double dp[3] = {0.1, 0.001, 1.};
  int i_twiddle = 0 ;
  int i_runs = 0;
  int last_twiddle = 0;
  int init_twiddle = 0;

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

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void Print();
  /*
  * Returns the steering value
  */
  double Run();

  double Twiddle();
};

#endif /* PID_H */
